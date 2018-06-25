/*
 * Copyright 2018 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include <string.h>
#include <time.h>
#include <utils/Thread.h>

#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>

#include <cutils/log.h>

#include <sys/resource.h>

#include <system/graphics.h>

#include <hardware/hwcomposer2.h>

extern hw_module_t HAL_MODULE_INFO_SYM;

namespace {

const int32_t kNumColorModes = 9;

const hwc2_config_t kDummyConfig = 0;

// We're arbitrarily choosing these values to make the fake display not look
// suspicious.
const int32_t kDummyVSyncPeriod = 16666667;  // 60Hz
const int32_t kDummyDpiX = 160;
const int32_t kDummyDpiY = 160;

class DummyDisplay;

hwc2_display_t nextId = 1;
std::map<hwc2_display_t, DummyDisplay> displays;

HWC2_PFN_VSYNC vsync_callback = nullptr;
hwc2_callback_data_t vsync_data = nullptr;

HWC2_PFN_HOTPLUG hotplug_callback = nullptr;
hwc2_callback_data_t hotplug_data = nullptr;

class DummyDisplay;
DummyDisplay* physical_display = nullptr;

class DummyDisplay {
 public:
  DummyDisplay(hwc2_display_t id, uint32_t width, uint32_t height)
      : id_(id), width_(width), height_(height) {}

  hwc2_display_t GetId() { return id_; }
  bool IsPhysical() { return physical_display == this; }
  bool IsConfigured() { return configured_; }
  void Configure() { configured_ = true; }
  uint32_t GetWidth() { return width_; }
  uint32_t GetHeight() { return height_; }

  void MakePhysical() {
    if (physical_display != nullptr) {
      ALOGE("Dummy composer does not support multiple physical displays.");
    } else {
      physical_display = this;
    }
  }

  hwc2_layer_t CreateLayer() {
    hwc2_layer_t layer = nextLayer_++;
    layers_.insert(layer);
    return layer;
  }

  bool IsValidLayer(hwc2_layer_t layer) {
    return layers_.find(layer) != layers_.end();
  }

  void DestroyLayer(hwc2_layer_t layer) {
    isClientComposed_.erase(layer);
    layers_.erase(layer);
  }

  bool SetClientComposed(hwc2_layer_t layer, bool value) {
    if (layers_.find(layer) == layers_.end()) {
      return false;
    }

    isClientComposed_[layer] = value;
    return true;
  }

  uint32_t NumNotClientComposed() {
    uint32_t ret = 0;

    for (const auto& layer : layers_) {
      if (!isClientComposed_[layer]) {
        ret++;
      }
    }

    return ret;
  }

  void GetNonClientComposedIDs(hwc2_layer_t* layers, uint32_t size) {
    if (!layers) {
      return;
    }

    for (const auto& layer : layers_) {
      if (size == 0) {
        break;
      }

      if (!isClientComposed_[layer]) {
        *(layers++) = layer;
        size--;
      }
    }
  }

 private:

  hwc2_display_t id_;
  uint32_t width_;
  uint32_t height_;
  bool configured_ = false;
  hwc2_layer_t nextLayer_ = 1;
  std::set<hwc2_layer_t> layers_;
  std::map<hwc2_layer_t, bool> isClientComposed_;
};

class VSyncThread : public android::Thread {
 public:
  VSyncThread() : Thread(false) {}

 private:
  bool threadLoop() override {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    ts.tv_nsec += kDummyVSyncPeriod;

    if (ts.tv_nsec >= 1000000000) {
      ts.tv_nsec -= 1000000000;
      ts.tv_sec += 1;
    }

    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr))
      ;

    int64_t timestamp = ts.tv_sec * 1000000000 + ts.tv_nsec;

    if (vsync_callback == nullptr) {
      return true;
    }

    vsync_callback(vsync_data, physical_display->GetId(), timestamp);

    return true;
  }
};

android::sp<VSyncThread> vsyncThread;

const uint32_t kPhysicalDummyWidth = 640;
const uint32_t kPhysicalDummyHeight = 480;

int32_t Hwc2ImplCreateVirtualDisplay(hwc2_device_t* /*device*/, uint32_t width,
                                     uint32_t height, int32_t* /*format*/,
                                     hwc2_display_t* out_display) {
  hwc2_display_t id = nextId++;
  *out_display = id;

  displays.emplace(std::piecewise_construct, std::forward_as_tuple(id),
                   std::forward_as_tuple(id, width, height));

  if (hotplug_callback != nullptr) {
    hotplug_callback(hotplug_data, id, HWC2_CONNECTION_CONNECTED);
  }

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplDestroyVirtualDisplay(hwc2_device_t* /*device*/,
                                      hwc2_display_t display) {
  auto iter = displays.find(display);

  if (iter == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (iter->second.IsPhysical()) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (hotplug_callback != nullptr) {
    hotplug_callback(hotplug_data, display, HWC2_CONNECTION_DISCONNECTED);
  }

  displays.erase(iter);
  return HWC2_ERROR_NONE;
}

void Hwc2ImplDump(hwc2_device_t* /*device*/, uint32_t* out_size,
                  char* out_buffer) {
  const char* dump_data = u8"hwcomposer is a dummy";

  if (out_buffer) {
    strncpy(out_buffer, dump_data, *out_size);
  }

  *out_size = static_cast<uint32_t>(strlen(dump_data));
}

uint32_t Hwc2ImplGetMaxVirtualDisplayCount(hwc2_device_t* /*device*/) {
  return UINT32_MAX;
}

int32_t Hwc2ImplRegisterCallback(hwc2_device_t* /*device*/, int32_t descriptor,
                                 hwc2_callback_data_t callback_data,
                                 hwc2_function_pointer_t pointer) {
  switch (descriptor) {
    case HWC2_CALLBACK_HOTPLUG:
      hotplug_callback = reinterpret_cast<HWC2_PFN_HOTPLUG>(pointer);
      hotplug_data = callback_data;

      for (const auto& disp : displays) {
        hotplug_callback(hotplug_data, disp.first, HWC2_CONNECTION_CONNECTED);
      }

      return HWC2_ERROR_NONE;
    case HWC2_CALLBACK_VSYNC:
      vsync_callback = reinterpret_cast<HWC2_PFN_VSYNC>(pointer);
      vsync_data = callback_data;

      return HWC2_ERROR_NONE;
    case HWC2_CALLBACK_REFRESH:
      return HWC2_ERROR_NONE;
    default:
      return HWC2_ERROR_BAD_PARAMETER;
  }
}

int32_t Hwc2ImplAcceptDisplayChanges(hwc2_device_t* /*device*/,
                                     hwc2_display_t display) {
  if (displays.find(display) == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplCreateLayer(hwc2_device_t* /*device*/, hwc2_display_t display,
                            hwc2_layer_t* out_layer) {
  auto iter = displays.find(display);

  if (iter == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  *out_layer = iter->second.CreateLayer();

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplDestroyLayer(hwc2_device_t* /*device*/, hwc2_display_t display,
                             hwc2_layer_t layer) {
  auto iter = displays.find(display);

  if (iter == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  iter->second.DestroyLayer(layer);

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplGetActiveConfig(hwc2_device_t* /*device*/,
                                hwc2_display_t display,
                                hwc2_config_t* out_config) {
  auto iter = displays.find(display);

  if (iter == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!iter->second.IsConfigured()) {
    return HWC2_ERROR_BAD_CONFIG;
  }

  *out_config = kDummyConfig;
  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplGetChangedCompositionTypes(hwc2_device_t* /*device*/,
                                           hwc2_display_t display,
                                           uint32_t* out_num_elements,
                                           hwc2_layer_t* out_layers,
                                           int32_t* out_types) {
  auto iter = displays.find(display);

  if (iter == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  uint32_t out_size = *out_num_elements;
  uint32_t not_composed = iter->second.NumNotClientComposed();

  if (iter->second.IsPhysical()) {
    *out_num_elements = 0;
    return HWC2_ERROR_NONE;
  }


  if (out_layers == nullptr || out_types == nullptr) {
    *out_num_elements = not_composed;
    return HWC2_ERROR_NONE;
  }

  iter->second.GetNonClientComposedIDs(out_layers, out_size);

  for (uint32_t i = 0; i < out_size; i++) {
    out_types[i] = HWC2_COMPOSITION_CLIENT;
  }

  if (not_composed < out_size) {
    *out_num_elements = not_composed;
  }

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplGetClientTargetSupport(hwc2_device_t* /*device*/,
                                       hwc2_display_t /*display*/,
                                       uint32_t /*width*/, uint32_t /*height*/,
                                       int32_t /*format*/,
                                       int32_t /*dataspace*/) {
  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplGetColorModes(hwc2_device_t* /*device*/, hwc2_display_t display,
                              uint32_t* out_num_modes, int32_t* out_modes) {
  if (displays.find(display) == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (*out_num_modes > kNumColorModes) {
    *out_num_modes = kNumColorModes;
  }

  if (!out_modes) {
    return HWC2_ERROR_NONE;
  }

  for (uint32_t i = 0; i < *out_num_modes; i++) {
    *(out_modes++) = static_cast<int32_t>(i);
  }

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplGetDisplayAttribute(hwc2_device_t* /*device*/,
                                    hwc2_display_t display,
                                    hwc2_config_t config, int32_t attribute,
                                    int32_t* out_value) {
  auto iter = displays.find(display);

  if (iter == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (config != kDummyConfig) {
    return HWC2_ERROR_BAD_CONFIG;
  }

  switch (attribute) {
    case HWC2_ATTRIBUTE_WIDTH:
      *out_value =  720;//static_cast<int32_t>(iter->second.GetWidth());
      return HWC2_ERROR_NONE;
    case HWC2_ATTRIBUTE_HEIGHT:
      *out_value = 1280;//static_cast<int32_t>(iter->second.GetHeight());
      return HWC2_ERROR_NONE;
    case HWC2_ATTRIBUTE_VSYNC_PERIOD:
      *out_value = kDummyVSyncPeriod;
      return HWC2_ERROR_NONE;
    case HWC2_ATTRIBUTE_DPI_X:
      *out_value = kDummyDpiX;
      return HWC2_ERROR_NONE;
    case HWC2_ATTRIBUTE_DPI_Y:
      *out_value = kDummyDpiY;
      return HWC2_ERROR_NONE;
    default:
      *out_value = -1;
      return HWC2_ERROR_NONE;
  }
}

int32_t Hwc2ImplGetDisplayConfigs(hwc2_device_t* /*device*/,
                                  hwc2_display_t display,
                                  uint32_t* out_num_configs,
                                  hwc2_config_t* out_configs) {
  if (displays.find(display) == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (out_configs) {
    if (*out_num_configs >= 1) {
      out_configs[0] = kDummyConfig;
    } else {
      return HWC2_ERROR_NONE;
    }
  }

  *out_num_configs = 1;

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplGetDisplayName(hwc2_device_t* /*device*/,
                               hwc2_display_t display, uint32_t* out_size,
                               char* out_name) {
  if (displays.find(display) == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  auto str = std::to_string(display);

  if (out_name) {
    strncpy(out_name, str.c_str(), *out_size);
  }

  if (*out_size > (str.size() + 1)) {
    *out_size = static_cast<uint32_t>( str.size() + 1);
  }

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplGetDisplayRequests(hwc2_device_t* /*device*/,
                                   hwc2_display_t display,
                                   int32_t* out_display_requests,
                                   uint32_t* out_num_elements,
                                   hwc2_layer_t* /*out_layers*/,
                                   int32_t* /*out_layer_requests*/) {
  if (displays.find(display) == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  *out_num_elements = 0;
  *out_display_requests = 0;

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplGetDisplayType(hwc2_device_t* /*device*/,
                               hwc2_display_t display, int32_t* out_type) {
  auto iter = displays.find(display);

  if (iter == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (iter->second.IsPhysical()) {
    *out_type = HWC2_DISPLAY_TYPE_PHYSICAL;
  } else {
    *out_type = HWC2_DISPLAY_TYPE_VIRTUAL;
  }

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplGetDozeSupport(hwc2_device_t* /*device*/,
                               hwc2_display_t display, int32_t* out_support) {
  if (displays.find(display) == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  *out_support = 0;
  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplGetHdrCapabilities(
    hwc2_device_t* /*device*/, hwc2_display_t display, uint32_t* out_num_types,
    int32_t* /*out_types*/, float* /*out_max_luminance*/,
    float* /*out_max_average_luminance*/, float* /*out_min_luminance*/) {
  if (displays.find(display) == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  *out_num_types = 0;
  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplGetReleaseFences(hwc2_device_t* /*device*/,
                                 hwc2_display_t display,
                                 uint32_t* out_num_elements,
                                 hwc2_layer_t* /*out_layers*/,
                                 int32_t* /*out_fences*/) {
  if (displays.find(display) == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  *out_num_elements = 0;

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplPresentDisplay(hwc2_device_t* /*device*/,
                               hwc2_display_t display,
                               int32_t* out_retire_fence) {
  if (displays.find(display) == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  /* Hope this works... */
  *out_retire_fence = -1;

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplSetActiveConfig(hwc2_device_t* /*device*/,
                                hwc2_display_t display, hwc2_config_t config) {
  auto iter = displays.find(display);

  if (iter == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (config != kDummyConfig) {
    return HWC2_ERROR_BAD_CONFIG;
  }

  iter->second.Configure();

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplSetClientTarget(hwc2_device_t* /*device*/,
                                hwc2_display_t display,
                                buffer_handle_t /*target*/,
                                int32_t /*acquire_fence*/,
                                int32_t /*dataspace*/,
                                hwc_region_t /*damage*/) {
  if (displays.find(display) == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplSetColorMode(hwc2_device_t* /*device*/, hwc2_display_t display,
                             int32_t mode) {
  if (displays.find(display) == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (mode < 0 || mode >= kNumColorModes) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplSetColorTransform(hwc2_device_t* /*device*/,
                                  hwc2_display_t display,
                                  const float* /*matrix*/, int32_t /*hint*/) {
  if (displays.find(display) == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  /* A bad hint value should yield HWC2_ERROR_BAD_PARAMETER but the
   * documentation is incomplete and inaccurate as to what is and is not a
   * valid hint value.
   */

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplSetOutputBuffer(hwc2_device_t* /*device*/,
                                hwc2_display_t display,
                                buffer_handle_t /*buffer*/,
                                int32_t /*release_fence*/) {
  auto iter = displays.find(display);

  if (iter == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (iter->second.IsPhysical()) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplSetPowerMode(hwc2_device_t* /*device*/, hwc2_display_t display,
                             int32_t mode) {
  if (displays.find(display) == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  switch (mode) {
    case HWC2_POWER_MODE_OFF:
    case HWC2_POWER_MODE_ON:
      return HWC2_ERROR_NONE;
    case HWC2_POWER_MODE_DOZE:
    case HWC2_POWER_MODE_DOZE_SUSPEND:
      return HWC2_ERROR_UNSUPPORTED;
    default:
      return HWC2_ERROR_BAD_PARAMETER;
  }
}

int32_t Hwc2ImplSetVsyncEnabled(hwc2_device_t* /*device*/,
                                hwc2_display_t display, int32_t enabled) {
  auto iter = displays.find(display);

  if (iter == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!iter->second.IsPhysical()) {
    return HWC2_ERROR_NONE;
  }


  if (enabled == HWC2_VSYNC_ENABLE) {
    if (vsyncThread.get() != nullptr) {
      return HWC2_ERROR_NONE;
    }

    vsyncThread = new VSyncThread();

    android::status_t ret =
        vsyncThread->run("dummy_vsync", HAL_PRIORITY_URGENT_DISPLAY);

    if (ret != android::OK) {
      ALOGE("Could not create vsync thread (%d)", ret);
    }

    return HWC2_ERROR_NONE;
  }

  if (enabled != HWC2_VSYNC_DISABLE) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  vsyncThread->requestExit();
  vsyncThread.clear();

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplValidateDisplay(hwc2_device_t* /*device*/,
                                hwc2_display_t display, uint32_t* out_num_types,
                                uint32_t* out_num_requests) {
  auto iter = displays.find(display);

  if (iter == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  *out_num_requests = 0;
  if (iter->second.IsPhysical()) {
    *out_num_types = 0;
  } else {
    *out_num_types = iter->second.NumNotClientComposed();
  }

  return HWC2_ERROR_NONE;
}

int32_t validateLayer(hwc2_display_t display, hwc2_layer_t layer) {
  auto iter = displays.find(display);

  if (iter == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!iter->second.IsValidLayer(layer)) {
    return HWC2_ERROR_BAD_LAYER;
  }

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplSetCursorPosition(hwc2_device_t* /*device*/,
                                  hwc2_display_t display, hwc2_layer_t layer,
                                  int32_t /*x*/, int32_t /*y*/) {
  return validateLayer(display, layer);
}

int32_t Hwc2ImplSetLayerBuffer(hwc2_device_t* /*device*/,
                               hwc2_display_t display, hwc2_layer_t layer,
                               buffer_handle_t /*buffer*/,
                               int32_t /*acquireFence*/) {
  return validateLayer(display, layer);
}

int32_t Hwc2ImplSetLayerSurfaceDamage(hwc2_device_t* /*device*/,
                                      hwc2_display_t display,
                                      hwc2_layer_t layer,
                                      hwc_region_t /*damage*/) {
  return validateLayer(display, layer);
}

int32_t Hwc2ImplSetLayerBlendMode(hwc2_device_t* /*device*/,
                                  hwc2_display_t display, hwc2_layer_t layer,
                                  int32_t /*mode*/) {
  return validateLayer(display, layer);
}

int32_t Hwc2ImplSetLayerColor(hwc2_device_t* /*device*/, hwc2_display_t display,
                              hwc2_layer_t layer, hwc_color_t /*color*/) {
  return validateLayer(display, layer);
}

int32_t Hwc2ImplSetLayerCompositionType(hwc2_device_t* /*device*/,
                                        hwc2_display_t display,
                                        hwc2_layer_t layer, int32_t type) {
  auto iter = displays.find(display);

  if (iter == displays.end()) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (type == HWC2_COMPOSITION_SIDEBAND) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  if (!iter->second.SetClientComposed(layer, type == HWC2_COMPOSITION_CLIENT)) {
    return HWC2_ERROR_BAD_LAYER;
  }

  return HWC2_ERROR_NONE;
}

int32_t Hwc2ImplSetLayerDataspace(hwc2_device_t* /*device*/,
                                  hwc2_display_t display, hwc2_layer_t layer,
                                  int32_t /*dataspace*/) {
  return validateLayer(display, layer);
}

int32_t Hwc2ImplSetLayerDisplayFrame(hwc2_device_t* /*device*/,
                                     hwc2_display_t display, hwc2_layer_t layer,
                                     hwc_rect_t /*frame*/) {
  return validateLayer(display, layer);
}

int32_t Hwc2ImplSetLayerPlaneAlpha(hwc2_device_t* /*device*/,
                                   hwc2_display_t display, hwc2_layer_t layer,
                                   float /*alpha*/) {
  return validateLayer(display, layer);
}

int32_t Hwc2ImplSetLayerSidebandStream(hwc2_device_t* /*device*/,
                                       hwc2_display_t display,
                                       hwc2_layer_t layer,
                                       const native_handle_t* /*stream*/) {
  return validateLayer(display, layer);
}

int32_t Hwc2ImplSetLayerSourceCrop(hwc2_device_t* /*device*/,
                                   hwc2_display_t display, hwc2_layer_t layer,
                                   hwc_frect_t /*crop*/) {
  return validateLayer(display, layer);
}

int32_t Hwc2ImplSetLayerTransform(hwc2_device_t* /*device*/,
                                  hwc2_display_t display, hwc2_layer_t layer,
                                  int32_t /*transform*/) {
  return validateLayer(display, layer);
}

int32_t Hwc2ImplSetLayerVisibleRegion(hwc2_device_t* /*device*/,
                                      hwc2_display_t display,
                                      hwc2_layer_t layer,
                                      hwc_region_t /*visible*/) {
  return validateLayer(display, layer);
}

int32_t Hwc2ImplSetLayer_z_order(hwc2_device_t* /*device*/,
                                 hwc2_display_t display, hwc2_layer_t layer,
                                 uint32_t /*z*/) {
  return validateLayer(display, layer);
}

int Hwc2DeviceClose(struct hw_device_t* /*dev*/) { return 0; }

void Hwc2GetCapabilities(struct hwc2_device* /*device*/, uint32_t* out_count,
                         int32_t* /*out_capabilities*/) {
  *out_count = 0;
}

hwc2_function_pointer_t Hwc2GetFunction(struct hwc2_device* /*device*/,
                                        int32_t descriptor) {
  switch (descriptor) {
    case HWC2_FUNCTION_ACCEPT_DISPLAY_CHANGES:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplAcceptDisplayChanges);
    case HWC2_FUNCTION_CREATE_LAYER:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplCreateLayer);
    case HWC2_FUNCTION_CREATE_VIRTUAL_DISPLAY:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplCreateVirtualDisplay);
    case HWC2_FUNCTION_DESTROY_LAYER:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplDestroyLayer);
    case HWC2_FUNCTION_DESTROY_VIRTUAL_DISPLAY:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplDestroyVirtualDisplay);
    case HWC2_FUNCTION_DUMP:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplDump);
    case HWC2_FUNCTION_GET_ACTIVE_CONFIG:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplGetActiveConfig);
    case HWC2_FUNCTION_GET_CHANGED_COMPOSITION_TYPES:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplGetChangedCompositionTypes);
    case HWC2_FUNCTION_GET_CLIENT_TARGET_SUPPORT:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplGetClientTargetSupport);
    case HWC2_FUNCTION_GET_COLOR_MODES:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplGetColorModes);
    case HWC2_FUNCTION_GET_DISPLAY_ATTRIBUTE:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplGetDisplayAttribute);
    case HWC2_FUNCTION_GET_DISPLAY_CONFIGS:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplGetDisplayConfigs);
    case HWC2_FUNCTION_GET_DISPLAY_NAME:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplGetDisplayName);
    case HWC2_FUNCTION_GET_DISPLAY_REQUESTS:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplGetDisplayRequests);
    case HWC2_FUNCTION_GET_DISPLAY_TYPE:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplGetDisplayType);
    case HWC2_FUNCTION_GET_DOZE_SUPPORT:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplGetDozeSupport);
    case HWC2_FUNCTION_GET_HDR_CAPABILITIES:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplGetHdrCapabilities);
    case HWC2_FUNCTION_GET_MAX_VIRTUAL_DISPLAY_COUNT:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplGetMaxVirtualDisplayCount);
    case HWC2_FUNCTION_GET_RELEASE_FENCES:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplGetReleaseFences);
    case HWC2_FUNCTION_PRESENT_DISPLAY:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplPresentDisplay);
    case HWC2_FUNCTION_REGISTER_CALLBACK:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplRegisterCallback);
    case HWC2_FUNCTION_SET_ACTIVE_CONFIG:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplSetActiveConfig);
    case HWC2_FUNCTION_SET_CLIENT_TARGET:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplSetClientTarget);
    case HWC2_FUNCTION_SET_COLOR_MODE:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplSetColorMode);
    case HWC2_FUNCTION_SET_COLOR_TRANSFORM:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplSetColorTransform);
    case HWC2_FUNCTION_SET_CURSOR_POSITION:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplSetCursorPosition);
    case HWC2_FUNCTION_SET_LAYER_BLEND_MODE:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplSetLayerBlendMode);
    case HWC2_FUNCTION_SET_LAYER_BUFFER:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplSetLayerBuffer);
    case HWC2_FUNCTION_SET_LAYER_COLOR:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplSetLayerColor);
    case HWC2_FUNCTION_SET_LAYER_COMPOSITION_TYPE:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplSetLayerCompositionType);
    case HWC2_FUNCTION_SET_LAYER_DATASPACE:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplSetLayerDataspace);
    case HWC2_FUNCTION_SET_LAYER_DISPLAY_FRAME:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplSetLayerDisplayFrame);
    case HWC2_FUNCTION_SET_LAYER_PLANE_ALPHA:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplSetLayerPlaneAlpha);
    case HWC2_FUNCTION_SET_LAYER_SIDEBAND_STREAM:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplSetLayerSidebandStream);
    case HWC2_FUNCTION_SET_LAYER_SOURCE_CROP:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplSetLayerSourceCrop);
    case HWC2_FUNCTION_SET_LAYER_SURFACE_DAMAGE:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplSetLayerSurfaceDamage);
    case HWC2_FUNCTION_SET_LAYER_TRANSFORM:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplSetLayerTransform);
    case HWC2_FUNCTION_SET_LAYER_VISIBLE_REGION:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplSetLayerVisibleRegion);
    case HWC2_FUNCTION_SET_LAYER_Z_ORDER:
      return reinterpret_cast<hwc2_function_pointer_t>(
          Hwc2ImplSetLayer_z_order);
    case HWC2_FUNCTION_SET_OUTPUT_BUFFER:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplSetOutputBuffer);
    case HWC2_FUNCTION_SET_POWER_MODE:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplSetPowerMode);
    case HWC2_FUNCTION_SET_VSYNC_ENABLED:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplSetVsyncEnabled);
    case HWC2_FUNCTION_VALIDATE_DISPLAY:
      return reinterpret_cast<hwc2_function_pointer_t>(Hwc2ImplValidateDisplay);
    default:
      return nullptr;
  }
}

hwc2_device_t dummy_device = {
    .common = {.tag = HARDWARE_DEVICE_TAG,
               .version = HWC_DEVICE_API_VERSION_2_0,
               .module = &HAL_MODULE_INFO_SYM,
               .close = Hwc2DeviceClose},
    .getCapabilities = Hwc2GetCapabilities,
    .getFunction = Hwc2GetFunction,
};

int Hwc2DeviceOpen(const struct hw_module_t* /*module*/, const char* name,
                   struct hw_device_t** device) {
  if (strcmp(name, HWC_HARDWARE_COMPOSER)) {
    return -EINVAL;
  }

  if (physical_display == nullptr) {
    hwc2_display_t physical_dummy;

    Hwc2ImplCreateVirtualDisplay(&dummy_device, kPhysicalDummyWidth,
                                 kPhysicalDummyHeight, nullptr,
                                 &physical_dummy);

    displays.find(physical_dummy)->second.MakePhysical();

    Hwc2ImplSetActiveConfig(&dummy_device, physical_dummy, kDummyConfig);
  }

  *device = &dummy_device.common;
  return 0;
}

struct hw_module_methods_t Hwc2ModuleMethods = {
    .open = Hwc2DeviceOpen,
};

}  // namespace

hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 2,
    .version_minor = 0,
    .id = HWC_HARDWARE_MODULE_ID,
    .name = "Dummy hwcomposer module",
    .author = "The Android Open Source Project",
    .methods = &Hwc2ModuleMethods,
};
