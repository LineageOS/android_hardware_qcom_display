/*
 * Copyright (c) 2014-2019, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *
 * Copyright 2015 The Android Open Source Project
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

#include <QService.h>
#include <binder/Parcel.h>
#include <core/buffer_allocator.h>
#include <cutils/properties.h>
#include <display_config.h>
#include <hardware_legacy/uevent.h>
#include <private/color_params.h>
#include <qd_utils.h>
#include <sync/sync.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <utils/String16.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <QService.h>
#include <utils/utils.h>
#include <algorithm>
#include <utility>
#include <bitset>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hwc_buffer_allocator.h"
#include "hwc_session.h"
#include "hwc_debugger.h"

#define __CLASS__ "HWCSession"

#define HWC_UEVENT_SWITCH_HDMI "change@/devices/virtual/switch/hdmi"
#define HWC_UEVENT_DRM_EXT_HOTPLUG "mdss_mdp/drm/card"

static sdm::HWCSession::HWCModuleMethods g_hwc_module_methods;

hwc_module_t HAL_MODULE_INFO_SYM = {
  .common = {
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 3,
    .version_minor = 0,
    .id = HWC_HARDWARE_MODULE_ID,
    .name = "QTI Hardware Composer Module",
    .author = "CodeAurora Forum",
    .methods = &g_hwc_module_methods,
    .dso = 0,
    .reserved = {0},
  }
};

namespace sdm {

static HWCUEvent g_hwc_uevent_;
Locker HWCSession::locker_[HWCCallbacks::kNumDisplays];
bool HWCSession::power_on_pending_[HWCCallbacks::kNumDisplays];
Locker HWCSession::power_state_[HWCCallbacks::kNumDisplays];
Locker HWCSession::display_config_locker_;
static const int kSolidFillDelay = 100 * 1000;
int HWCSession::null_display_mode_ = 0;

// Map the known color modes to dataspace.
int32_t GetDataspaceFromColorMode(ColorMode mode) {
  switch (mode) {
    case ColorMode::SRGB:
    case ColorMode::NATIVE:
      return HAL_DATASPACE_V0_SRGB;
    case ColorMode::DCI_P3:
      return HAL_DATASPACE_DCI_P3;
    case ColorMode::DISPLAY_P3:
      return HAL_DATASPACE_DISPLAY_P3;
    case ColorMode::BT2100_PQ:
      return HAL_DATASPACE_BT2020_PQ;
    case ColorMode::BT2100_HLG:
      return HAL_DATASPACE_BT2020_HLG;
    default:
      return HAL_DATASPACE_UNKNOWN;
  }
}

void HWCUEvent::UEventThread(HWCUEvent *hwc_uevent) {
  const char *uevent_thread_name = "HWC_UeventThread";

  prctl(PR_SET_NAME, uevent_thread_name, 0, 0, 0);
  setpriority(PRIO_PROCESS, 0, HAL_PRIORITY_URGENT_DISPLAY);

  int status = uevent_init();
  if (!status) {
    std::unique_lock<std::mutex> caller_lock(hwc_uevent->mutex_);
    hwc_uevent->caller_cv_.notify_one();
    DLOGE("Failed to init uevent with err %d", status);
    return;
  }

  {
    // Signal caller thread that worker thread is ready to listen to events.
    std::unique_lock<std::mutex> caller_lock(hwc_uevent->mutex_);
    hwc_uevent->init_done_ = true;
    hwc_uevent->caller_cv_.notify_one();
  }

  while (1) {
    char uevent_data[PAGE_SIZE] = {};

    // keep last 2 zeros to ensure double 0 termination
    int length = uevent_next_event(uevent_data, INT32(sizeof(uevent_data)) - 2);

    // scope of lock to this block only, so that caller is free to set event handler to nullptr;
    {
      std::lock_guard<std::mutex> guard(hwc_uevent->mutex_);
      if (hwc_uevent->uevent_listener_) {
        hwc_uevent->uevent_listener_->UEventHandler(uevent_data, length);
      } else {
        DLOGW("UEvent dropped. No uevent listener.");
      }
    }
  }
}

HWCUEvent::HWCUEvent() {
  std::unique_lock<std::mutex> caller_lock(mutex_);
  std::thread thread(HWCUEvent::UEventThread, this);
  thread.detach();
  caller_cv_.wait(caller_lock);
}

void HWCUEvent::Register(HWCUEventListener *uevent_listener) {
  DLOGI("Set uevent listener = %p", uevent_listener);

  std::lock_guard<std::mutex> obj(mutex_);
  uevent_listener_ = uevent_listener;
}

HWCSession::HWCSession(const hw_module_t *module) {
  hwc2_device_t::common.tag = HARDWARE_DEVICE_TAG;
  hwc2_device_t::common.version = HWC_DEVICE_API_VERSION_2_0;
  hwc2_device_t::common.module = const_cast<hw_module_t *>(module);
  hwc2_device_t::common.close = Close;
  hwc2_device_t::getCapabilities = GetCapabilities;
  hwc2_device_t::getFunction = GetFunction;
}

int HWCSession::Init() {
  SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  int status = -EINVAL;
  const char *qservice_name = "display.qservice";

  if (!g_hwc_uevent_.InitDone()) {
    return status;
  }


#if defined(DISPLAY_CONFIG_1_2) && defined(CONFIG_BASEID_FROM_PROP)
  char indices[kPropertyMax];
  uint32_t index_start, index_size;
  if (Debug::Get()->GetProperty(BUILTIN_BASEID_AND_SIZE_PROP, indices) == kErrorNone) {
    if (std::sscanf(indices, "%d,%d", &index_start, &index_size) == 2) {
      setDisplayIndex(IDisplayConfig::DisplayTypeExt::DISPLAY_BUILTIN,
               index_start, index_size);
    }
  }
  if (Debug::Get()->GetProperty(PLUGGABLE_BASEID_AND_SIZE_PROP, indices) == kErrorNone) {
    if (std::sscanf(indices, "%d,%d", &index_start, &index_size) == 2) {
      setDisplayIndex(IDisplayConfig::DisplayTypeExt::DISPLAY_PLUGGABLE,
               index_start, index_size);
    }
  }
  if (Debug::Get()->GetProperty(VIRTUAL_BASEID_AND_SIZE_PROP, indices) == kErrorNone) {
    if (std::sscanf(indices, "%d,%d", &index_start, &index_size) == 2) {
      setDisplayIndex(IDisplayConfig::DisplayTypeExt::DISPLAY_VIRTUAL,
               index_start, index_size);
    }
  }
#endif

  // Start QService and connect to it.
  qService::QService::init();
  android::sp<qService::IQService> iqservice = android::interface_cast<qService::IQService>(
      android::defaultServiceManager()->getService(android::String16(qservice_name)));

  if (iqservice.get()) {
    iqservice->connect(android::sp<qClient::IQClient>(this));
    qservice_ = reinterpret_cast<qService::QService *>(iqservice.get());
  } else {
    DLOGE("Failed to acquire %s", qservice_name);
    return -EINVAL;
  }

  HWCDebugHandler::Get()->GetProperty(ENABLE_NULL_DISPLAY_PROP, &null_display_mode_);
  HWCDebugHandler::Get()->GetProperty(DISABLE_HOTPLUG_BWCHECK, &disable_hotplug_bwcheck_);
  HWCDebugHandler::Get()->GetProperty(DISABLE_MASK_LAYER_HINT, &disable_mask_layer_hint_);

  if (!null_display_mode_) {
    g_hwc_uevent_.Register(this);
  }

  int value = 0;
  Debug::Get()->GetProperty(ENABLE_ASYNC_POWERMODE, &value);
  async_powermode_ = (value == 1);
  DLOGI("builtin_powermode_override: %d", async_powermode_);

  InitSupportedDisplaySlots();
  // Create primary display here. Remaining builtin displays will be created after client has set
  // display indexes which may happen sometime before callback is registered.
  status = CreatePrimaryDisplay();
  if (status) {
    Deinit();
    return status;
  }

  is_composer_up_ = true;
  StartServices();

  return 0;
}

int HWCSession::Deinit() {
  // Destroy all connected displays
  DestroyDisplay(&map_info_primary_);

  for (auto &map_info : map_info_builtin_) {
    DestroyDisplay(&map_info);
  }

  for (auto &map_info : map_info_pluggable_) {
    DestroyDisplay(&map_info);
  }

  for (auto &map_info : map_info_virtual_) {
    DestroyDisplay(&map_info);
  }

  if (color_mgr_) {
    color_mgr_->DestroyColorManager();
  }

  if (!null_display_mode_) {
    g_hwc_uevent_.Register(nullptr);

    DisplayError error = CoreInterface::DestroyCore();
    if (error != kErrorNone) {
      DLOGE("Display core de-initialization failed. Error = %d", error);
    }
  }

  return 0;
}

void HWCSession::InitSupportedDisplaySlots() {
  // Default slots:
  //    Primary = 0, External = 1
  //    Additional external displays 2,3,...max_pluggable_count.
  //    Additional builtin displays max_pluggable_count + 1, max_pluggable_count + 2,...
  //    Last slots for virtual displays.
  // Virtual display id is only for SF <--> HWC communication.
  // It need not align with hwccomposer_defs

  map_info_primary_.client_id = qdutils::DISPLAY_PRIMARY;

  if (null_display_mode_) {
    // Skip display slot initialization.
    return;
  }

  DisplayError error = CoreInterface::CreateCore(&buffer_allocator_, &buffer_sync_handler_,
                                                 &socket_handler_, &core_intf_);
  if (error != kErrorNone) {
    DLOGE("Failed to create CoreInterface");
    return;
  }

  HWDisplayInterfaceInfo hw_disp_info = {};
  error = core_intf_->GetFirstDisplayInterfaceType(&hw_disp_info);
  if (error != kErrorNone) {
    CoreInterface::DestroyCore();
    DLOGE("Primary display type not recognized. Error = %d", error);
    return;
  }

  int max_builtin = 0;
  int max_pluggable = 0;
  int max_virtual = 0;

  error = core_intf_->GetMaxDisplaysSupported(kBuiltIn, &max_builtin);
  if (error != kErrorNone) {
    CoreInterface::DestroyCore();
    DLOGE("Could not find maximum built-in displays supported. Error = %d", error);
    return;
  }

  error = core_intf_->GetMaxDisplaysSupported(kPluggable, &max_pluggable);
  if (error != kErrorNone) {
    CoreInterface::DestroyCore();
    DLOGE("Could not find maximum pluggable displays supported. Error = %d", error);
    return;
  }

  error = core_intf_->GetMaxDisplaysSupported(kVirtual, &max_virtual);
  if (error != kErrorNone) {
    CoreInterface::DestroyCore();
    DLOGE("Could not find maximum virtual displays supported. Error = %d", error);
    return;
  }

  if (kPluggable == hw_disp_info.type && max_pluggable != 0) {
    // If primary is a pluggable display, we have already used one pluggable display interface.
    max_pluggable--;
  } else if (max_builtin != 0) {
    max_builtin--;
  }

  // Init slots in accordance to h/w capability.
  uint32_t disp_count = UINT32(std::min(max_pluggable, HWCCallbacks::kNumPluggable));
  hwc2_display_t base_id = qdutils::DISPLAY_EXTERNAL;
  map_info_pluggable_.resize(disp_count);
  for (auto &map_info : map_info_pluggable_) {
    map_info.client_id = base_id++;
  }

  disp_count = UINT32(std::min(max_builtin, HWCCallbacks::kNumBuiltIn));
  map_info_builtin_.resize(disp_count);
  for (auto &map_info : map_info_builtin_) {
    map_info.client_id = base_id++;
  }

  disp_count = UINT32(std::min(max_virtual, HWCCallbacks::kNumVirtual));
  map_info_virtual_.resize(disp_count);
  for (auto &map_info : map_info_virtual_) {
    map_info.client_id = base_id++;
  }

  // resize HDR supported map to total number of displays.
  is_hdr_display_.resize(UINT32(base_id));

  if (!async_powermode_) {
    return;
  }

  int start_index = HWCCallbacks::kNumRealDisplays;
  std::vector<DisplayMapInfo> map_info = {map_info_primary_};
  std::copy(map_info_builtin_.begin(), map_info_builtin_.end(), std::back_inserter(map_info));
  std::copy(map_info_pluggable_.begin(), map_info_pluggable_.end(), std::back_inserter(map_info));
  for (auto &map : map_info) {
    DLOGI("Display Pairs: map.client_id: %d, start_index: %d", map.client_id, start_index);
    map_hwc_display_.insert(std::make_pair(map.client_id, start_index++));
  }
}

int HWCSession::GetDisplayIndex(int dpy) {
  DisplayMapInfo *map_info = nullptr;
  switch (dpy) {
    case qdutils::DISPLAY_PRIMARY:
      map_info = &map_info_primary_;
      break;
    case qdutils::DISPLAY_EXTERNAL:
      map_info = map_info_pluggable_.size() ? &map_info_pluggable_[0] : nullptr;
      break;
    case qdutils::DISPLAY_EXTERNAL_2:
      map_info = (map_info_pluggable_.size() > 1) ? &map_info_pluggable_[1] : nullptr;
      break;
    case qdutils::DISPLAY_VIRTUAL:
      map_info = map_info_virtual_.size() ? &map_info_virtual_[0] : nullptr;
      break;
    case qdutils::DISPLAY_BUILTIN_2:
      map_info = map_info_builtin_.size() ? &map_info_builtin_[0] : nullptr;
      break;
    default:
      DLOGW("Unknown display %d.", dpy);
      break;
  }

  if (!map_info) {
    DLOGE("Display index not found for display %d.", dpy);
    return -1;
  }

  return INT(map_info->client_id);
}

int HWCSession::Open(const hw_module_t *module, const char *name, hw_device_t **device) {
  if (!module || !name || !device) {
    DLOGE("Invalid parameters.");
    return -EINVAL;
  }

  if (!strcmp(name, HWC_HARDWARE_COMPOSER)) {
    HWCSession *hwc_session = new HWCSession(module);
    if (!hwc_session) {
      return -ENOMEM;
    }

    int status = hwc_session->Init();
    if (status != 0) {
      delete hwc_session;
      hwc_session = NULL;
      return status;
    }

    hwc2_device_t *composer_device = hwc_session;
    *device = reinterpret_cast<hw_device_t *>(composer_device);
  }

  return 0;
}

int HWCSession::Close(hw_device_t *device) {
  if (!device) {
    return -EINVAL;
  }

  hwc2_device_t *composer_device = reinterpret_cast<hwc2_device_t *>(device);
  HWCSession *hwc_session = static_cast<HWCSession *>(composer_device);

  hwc_session->Deinit();

  return 0;
}

void HWCSession::GetCapabilities(struct hwc2_device *device, uint32_t *outCount,
                                 int32_t *outCapabilities) {
  if (!outCount) {
    return;
  }

  int value = 0;
  bool disable_skip_validate = false;
  if (Debug::Get()->GetProperty(DISABLE_SKIP_VALIDATE_PROP, &value) == kErrorNone) {
    disable_skip_validate = (value == 1);
  }
  uint32_t count = 1 + (disable_skip_validate ? 0 : 1);

  if (outCapabilities != nullptr && (*outCount >= count)) {
    outCapabilities[0] = HWC2_CAPABILITY_SKIP_CLIENT_COLOR_TRANSFORM;
    if (!disable_skip_validate) {
      outCapabilities[1] = HWC2_CAPABILITY_SKIP_VALIDATE;
    }
  }
  *outCount = count;
}

template <typename PFN, typename T>
static hwc2_function_pointer_t AsFP(T function) {
  static_assert(std::is_same<PFN, T>::value, "Incompatible function pointer");
  return reinterpret_cast<hwc2_function_pointer_t>(function);
}

// HWC2 functions returned in GetFunction
// Defined in the same order as in the HWC2 header

int32_t HWCSession::AcceptDisplayChanges(hwc2_device_t *device, hwc2_display_t display) {
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::AcceptDisplayChanges);
}

int32_t HWCSession::CreateLayer(hwc2_device_t *device, hwc2_display_t display,
                                hwc2_layer_t *out_layer_id) {
  if (!out_layer_id) {
    return  HWC2_ERROR_BAD_PARAMETER;
  }

  return CallDisplayFunction(device, display, &HWCDisplay::CreateLayer, out_layer_id);
}

int32_t HWCSession::CreateVirtualDisplay(hwc2_device_t *device, uint32_t width, uint32_t height,
                                         int32_t *format, hwc2_display_t *out_display_id) {
  // TODO(user): Handle concurrency with HDMI
  if (!device) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!out_display_id || !width || !height || !format) {
    return  HWC2_ERROR_BAD_PARAMETER;
  }

  HWCSession *hwc_session = static_cast<HWCSession *>(device);
  auto status = hwc_session->CreateVirtualDisplayObj(width, height, format, out_display_id);
  if (status == HWC2::Error::None) {
    DLOGI("Created virtual display id:% " PRIu64 ", res: %dx%d", *out_display_id, width, height);
  } else {
    DLOGE("Failed to create virtual display: %s", to_string(status).c_str());
  }
  return INT32(status);
}

int32_t HWCSession::DestroyLayer(hwc2_device_t *device, hwc2_display_t display,
                                 hwc2_layer_t layer) {
  return CallDisplayFunction(device, display, &HWCDisplay::DestroyLayer, layer);
}

int32_t HWCSession::DestroyVirtualDisplay(hwc2_device_t *device, hwc2_display_t display) {
  if (!device || display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  auto *hwc_session = static_cast<HWCSession *>(device);
  hwc2_display_t active_builtin_disp_id = hwc_session->GetActiveBuiltinDisplay();

  if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
    Locker::ScopeLock lock_a(locker_[active_builtin_disp_id]);
    std::bitset<kSecureMax> secure_sessions = 0;
    hwc_session->hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
    if (secure_sessions.any()) {
      DLOGW("Secure session is active, defer destruction of virtual display id:%" PRIu64, display);
      hwc_session->destroy_virtual_disp_pending_ = true;
      return HWC2_ERROR_NONE;
    }
  }

  for (auto &map_info : hwc_session->map_info_virtual_) {
    if (map_info.client_id == display) {
      DLOGI("Destroying virtual display id:%" PRIu64, display);
      hwc_session->DestroyDisplay(&map_info);
      break;
    }
  }

  return HWC2_ERROR_NONE;
}

void HWCSession::Dump(hwc2_device_t *device, uint32_t *out_size, char *out_buffer) {
  if (!device || !out_size) {
    return;
  }

  auto *hwc_session = static_cast<HWCSession *>(device);
  const size_t max_dump_size = 8192;

  if (out_buffer == nullptr) {
    *out_size = max_dump_size;
  } else {
    std::string s {};
    for (int id = 0; id < HWCCallbacks::kNumRealDisplays; id++) {
      SCOPE_LOCK(locker_[id]);
      if (hwc_session->hwc_display_[id]) {
        s += hwc_session->hwc_display_[id]->Dump();
      }
    }
    auto copied = s.copy(out_buffer, std::min(s.size(), max_dump_size), 0);
    *out_size = UINT32(copied);
  }
}

uint32_t HWCSession::GetMaxVirtualDisplayCount(hwc2_device_t *device) {
  if (device == nullptr) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  return 1;
}

static int32_t GetActiveConfig(hwc2_device_t *device, hwc2_display_t display,
                               hwc2_config_t *out_config) {
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::GetActiveConfig, out_config);
}

static int32_t GetChangedCompositionTypes(hwc2_device_t *device, hwc2_display_t display,
                                          uint32_t *out_num_elements, hwc2_layer_t *out_layers,
                                          int32_t *out_types) {
  // null_ptr check only for out_num_elements, as out_layers and out_types can be null.
  if (!out_num_elements) {
    return  HWC2_ERROR_BAD_PARAMETER;
  }
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::GetChangedCompositionTypes,
                                         out_num_elements, out_layers, out_types);
}

static int32_t GetClientTargetSupport(hwc2_device_t *device, hwc2_display_t display, uint32_t width,
                                      uint32_t height, int32_t format, int32_t dataspace) {
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::GetClientTargetSupport,
                                         width, height, format, dataspace);
}

static int32_t GetColorModes(hwc2_device_t *device, hwc2_display_t display, uint32_t *out_num_modes,
                             int32_t /*ColorMode*/ *int_out_modes) {
  auto out_modes = reinterpret_cast<ColorMode *>(int_out_modes);
  if (out_num_modes == nullptr) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::GetColorModes, out_num_modes,
                                         out_modes);
}

static int32_t GetRenderIntents(hwc2_device_t *device, hwc2_display_t display,
                                int32_t /*ColorMode*/ int_mode, uint32_t *out_num_intents,
                                int32_t /*RenderIntent*/ *int_out_intents) {
  auto mode = static_cast<ColorMode>(int_mode);
  auto out_intents = reinterpret_cast<RenderIntent *>(int_out_intents);
  if (out_num_intents == nullptr) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (mode < ColorMode::NATIVE || mode > ColorMode::BT2100_HLG) {
    DLOGE("Invalid ColorMode: %d", mode);
    return HWC2_ERROR_BAD_PARAMETER;
  }
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::GetRenderIntents, mode,
                                         out_num_intents, out_intents);
}

static int32_t GetDataspaceSaturationMatrix(hwc2_device_t *device,
                                            int32_t /*Dataspace*/ int_dataspace,
                                            float *out_matrix) {
  auto dataspace = static_cast<Dataspace>(int_dataspace);
  if (device == nullptr || out_matrix == nullptr || dataspace != Dataspace::SRGB_LINEAR) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  // We only have the matrix for sRGB
  float saturation_matrix[kDataspaceSaturationMatrixCount] = { 1.0, 0.0, 0.0, 0.0, \
                                                               0.0, 1.0, 0.0, 0.0, \
                                                               0.0, 0.0, 1.0, 0.0, \
                                                               0.0, 0.0, 0.0, 1.0 };

  // TODO(user): This value should ideally be retrieved from a QDCM configuration file
  char value[kPropertyMax] = {};
  if (Debug::Get()->GetProperty(DATASPACE_SATURATION_MATRIX_PROP, value) != kErrorNone) {
    DLOGW("Undefined saturation matrix");
    return HWC2_ERROR_BAD_CONFIG;
  }
  std::string value_string(value);
  std::size_t start = 0, end = 0;
  int index = 0;
  while ((end = value_string.find(",", start)) != std::string::npos) {
    saturation_matrix[index] = std::stof(value_string.substr(start, end - start));
    start = end + 1;
    index++;
    // We expect a 3x3, SF needs 4x4, keep the last row/column identity
    if ((index + 1) % 4 == 0) {
      index++;
    }
  }
  saturation_matrix[index] = std::stof(value_string.substr(start, end - start));
  if (index < kDataspaceSaturationPropertyElements - 1) {
    // The property must have kDataspaceSaturationPropertyElements delimited by commas
    DLOGW("Invalid saturation matrix defined");
    return HWC2_ERROR_BAD_CONFIG;
  }
  for (int32_t i = 0; i < kDataspaceSaturationMatrixCount; i += 4) {
    DLOGD("%f %f %f %f", saturation_matrix[i], saturation_matrix[i + 1], saturation_matrix[i + 2],
          saturation_matrix[i + 3]);
  }
  for (uint32_t i = 0; i < kDataspaceSaturationMatrixCount; i++) {
    out_matrix[i] = saturation_matrix[i];
  }
  return HWC2_ERROR_NONE;
}

static int32_t GetPerFrameMetadataKeys(hwc2_device_t *device, hwc2_display_t display,
                                       uint32_t *out_num_keys, int32_t *int_out_keys) {
  auto out_keys = reinterpret_cast<PerFrameMetadataKey *>(int_out_keys);
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::GetPerFrameMetadataKeys,
                                         out_num_keys, out_keys);
}

static int32_t SetLayerPerFrameMetadata(hwc2_device_t *device, hwc2_display_t display,
                                        hwc2_layer_t layer, uint32_t num_elements,
                                        const int32_t *int_keys, const float *metadata) {
  auto keys = reinterpret_cast<const PerFrameMetadataKey *>(int_keys);
  return HWCSession::CallLayerFunction(device, display, layer, &HWCLayer::SetLayerPerFrameMetadata,
                                       num_elements, keys, metadata);
}

static int32_t GetDisplayAttribute(hwc2_device_t *device, hwc2_display_t display,
                                   hwc2_config_t config, int32_t int_attribute,
                                   int32_t *out_value) {
  if (out_value == nullptr || int_attribute < HWC2_ATTRIBUTE_INVALID ||
      int_attribute > HWC2_ATTRIBUTE_DPI_Y) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  auto attribute = static_cast<HWC2::Attribute>(int_attribute);
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::GetDisplayAttribute, config,
                                         attribute, out_value);
}

static int32_t GetDisplayConfigs(hwc2_device_t *device, hwc2_display_t display,
                                 uint32_t *out_num_configs, hwc2_config_t *out_configs) {
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::GetDisplayConfigs,
                                         out_num_configs, out_configs);
}

static int32_t GetDisplayName(hwc2_device_t *device, hwc2_display_t display, uint32_t *out_size,
                              char *out_name) {
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::GetDisplayName, out_size,
                                         out_name);
}

static int32_t GetDisplayRequests(hwc2_device_t *device, hwc2_display_t display,
                                  int32_t *out_display_requests, uint32_t *out_num_elements,
                                  hwc2_layer_t *out_layers, int32_t *out_layer_requests) {
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::GetDisplayRequests,
                                         out_display_requests, out_num_elements, out_layers,
                                         out_layer_requests);
}

static int32_t GetDisplayType(hwc2_device_t *device, hwc2_display_t display, int32_t *out_type) {
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::GetDisplayType, out_type);
}


static int32_t GetHdrCapabilities(hwc2_device_t* device, hwc2_display_t display,
                                  uint32_t* out_num_types, int32_t* out_types,
                                  float* out_max_luminance, float* out_max_average_luminance,
                                  float* out_min_luminance) {
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::GetHdrCapabilities,
                                         out_num_types, out_types, out_max_luminance,
                                         out_max_average_luminance, out_min_luminance);
}


static int32_t GetReleaseFences(hwc2_device_t *device, hwc2_display_t display,
                                uint32_t *out_num_elements, hwc2_layer_t *out_layers,
                                int32_t *out_fences) {
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::GetReleaseFences,
                                         out_num_elements, out_layers, out_fences);
}

int32_t HWCSession::PresentDisplay(hwc2_device_t *device, hwc2_display_t display,
                                   int32_t *out_retire_fence) {
  HWCSession *hwc_session = static_cast<HWCSession *>(device);
  auto status = HWC2::Error::BadDisplay;
  DTRACE_SCOPED();

  if (!hwc_session || (display >= HWCCallbacks::kNumDisplays)) {
    DLOGW("Invalid Display : hwc session = %s display = %" PRIu64,
          hwc_session ? "Valid" : "NULL", display);
    return HWC2_ERROR_BAD_DISPLAY;
  }

  hwc_session->HandleSecureSession();
  {
    SCOPE_LOCK(power_state_[display]);
    if (hwc_session->power_state_transition_[display]) {
      // Route all interactions with client to dummy display.
      display = hwc_session->map_hwc_display_.find(display)->second;
    }
  }

  {
    SEQUENCE_EXIT_SCOPE_LOCK(locker_[display]);
    if (!hwc_session->hwc_display_[display]) {
      DLOGW("Removed Display : display = %" PRIu64, display);
      return HWC2_ERROR_BAD_DISPLAY;
    }

    if (out_retire_fence == nullptr) {
      return HWC2_ERROR_BAD_PARAMETER;
    }

    if (power_on_pending_[display]) {
      status = HWC2::Error::None;
    } else {
      status = hwc_session->PresentDisplayInternal(display, out_retire_fence);
      if (status == HWC2::Error::None) {
        // Check if hwc's refresh trigger is getting exercised.
        if (hwc_session->callbacks_.NeedsRefresh(display)) {
          hwc_session->hwc_display_[display]->SetPendingRefresh();
          hwc_session->callbacks_.ResetRefresh(display);
        }
        status = hwc_session->hwc_display_[display]->Present(out_retire_fence);
      }
    }
  }

  if (status != HWC2::Error::None && status != HWC2::Error::NotValidated) {
    SEQUENCE_CANCEL_SCOPE_LOCK(locker_[display]);
  }

  hwc_session->HandlePowerOnPending(display, *out_retire_fence);
  hwc_session->HandleHotplugPending(display, *out_retire_fence);
  hwc_session->display_ready_.set(UINT32(display));
  hwc_session->HandlePendingRefresh();

  return INT32(status);
}

void HWCSession::HandlePendingRefresh() {
  if (pending_refresh_.none()) {
    return;
  }

  for (size_t i = 0; i < pending_refresh_.size(); i++) {
    if (pending_refresh_.test(i)) {
      Refresh(i);
    }
    break;
  }

  pending_refresh_.reset();
}

int32_t HWCSession::RegisterCallback(hwc2_device_t *device, int32_t descriptor,
                                     hwc2_callback_data_t callback_data,
                                     hwc2_function_pointer_t pointer) {
  if (!device) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  HWCSession *hwc_session = static_cast<HWCSession *>(device);
  SCOPE_LOCK(hwc_session->callbacks_lock_);
  auto desc = static_cast<HWC2::Callback>(descriptor);
  auto error = hwc_session->callbacks_.Register(desc, callback_data, pointer);
  if (error != HWC2::Error::None) {
    return INT32(error);
  }

  DLOGD("%s callback: %s", pointer ? "Registering" : "Deregistering", to_string(desc).c_str());
  if (descriptor == HWC2_CALLBACK_HOTPLUG) {
    if (hwc_session->hwc_display_[HWC_DISPLAY_PRIMARY]) {
      DLOGI("Hotplugging primary...");
      hwc_session->callbacks_.Hotplug(HWC_DISPLAY_PRIMARY, HWC2::Connection::Connected);
    }

    std::vector<hwc2_display_t> pending_hotplugs;
    if (pointer) {
      for (auto &map_info : hwc_session->map_info_builtin_) {
        SCOPE_LOCK(locker_[map_info.client_id]);
        if (hwc_session->hwc_display_[map_info.client_id]) {
          pending_hotplugs.push_back(static_cast<hwc2_display_t>(map_info.client_id));
        }
      }
      for (auto &map_info : hwc_session->map_info_pluggable_) {
        SCOPE_LOCK(locker_[map_info.client_id]);
        if (hwc_session->hwc_display_[map_info.client_id]) {
          pending_hotplugs.push_back(static_cast<hwc2_display_t>(map_info.client_id));
        }
      }
    }

    // Create displays since they should now have their final display indices set.
    DLOGI("Handling built-in displays...");
    if (hwc_session->HandleBuiltInDisplays()) {
      DLOGW("Failed handling built-in displays.");
    }
    DLOGI("Handling pluggable displays...");
    int32_t err = hwc_session->HandlePluggableDisplays(false);
    if (err) {
      DLOGW("All displays could not be created. Error %d '%s'. Hotplug handling %s.", err,
            strerror(abs(err)), hwc_session->hotplug_pending_event_ == kHotPlugEvent ? "deferred" :
            "dropped");
    }

    // If previously registered, call hotplug for all connected displays to refresh
    if (pointer) {
      std::vector<hwc2_display_t> updated_pending_hotplugs;
      for (auto client_id : pending_hotplugs) {
        SCOPE_LOCK(locker_[client_id]);
        // check if the display is unregistered
        if (hwc_session->hwc_display_[client_id]) {
          updated_pending_hotplugs.push_back(client_id);
        }
      }
      for (auto client_id : updated_pending_hotplugs) {
        DLOGI("Re-hotplug display connected: client id = %d", client_id);
        hwc_session->callbacks_.Hotplug(client_id, HWC2::Connection::Connected);
      }
    }

    hwc_session->client_connected_ = !!pointer;
    // Notfify all displays.
    hwc_session->NotifyClientStatus(hwc_session->client_connected_);
  }
  hwc_session->need_invalidate_ = false;
  hwc_session->callbacks_lock_.Broadcast();
  return HWC2_ERROR_NONE;
}

static int32_t SetActiveConfig(hwc2_device_t *device, hwc2_display_t display,
                               hwc2_config_t config) {
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::SetActiveConfig, config);
}

static int32_t SetClientTarget(hwc2_device_t *device, hwc2_display_t display,
                               buffer_handle_t target, int32_t acquire_fence,
                               int32_t dataspace, hwc_region_t damage) {
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::SetClientTarget, target,
                                         acquire_fence, dataspace, damage);
}

int32_t HWCSession::SetColorMode(hwc2_device_t *device, hwc2_display_t display,
                                 int32_t /*ColorMode*/ int_mode) {
  auto mode = static_cast<ColorMode>(int_mode);
  if (mode < ColorMode::NATIVE || mode > ColorMode::BT2100_HLG) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::SetColorMode, mode);
}

int32_t HWCSession::SetColorModeWithRenderIntent(hwc2_device_t *device, hwc2_display_t display,
                                                 int32_t /*ColorMode*/ int_mode,
                                                 int32_t /*RenderIntent*/ int_render_intent) {
  auto mode = static_cast<ColorMode>(int_mode);
  if (mode < ColorMode::NATIVE || mode > ColorMode::BT2100_HLG) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  auto render_intent = static_cast<RenderIntent>(int_render_intent);
  if ((render_intent < RenderIntent::COLORIMETRIC) ||
      (render_intent > RenderIntent::TONE_MAP_ENHANCE)) {
    DLOGE("Invalid RenderIntent: %d", render_intent);
    return HWC2_ERROR_BAD_PARAMETER;
  }
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::SetColorModeWithRenderIntent,
                                         mode, render_intent);
}

int32_t HWCSession::SetColorTransform(hwc2_device_t *device, hwc2_display_t display,
                                      const float *matrix,
                                      int32_t /*android_color_transform_t*/ hint) {
  if (!matrix || hint < HAL_COLOR_TRANSFORM_IDENTITY ||
       hint > HAL_COLOR_TRANSFORM_CORRECT_TRITANOPIA) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  android_color_transform_t transform_hint = static_cast<android_color_transform_t>(hint);
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::SetColorTransform, matrix,
                                         transform_hint);
}

static int32_t SetCursorPosition(hwc2_device_t *device, hwc2_display_t display, hwc2_layer_t layer,
                                 int32_t x, int32_t y) {
  auto status = INT32(HWC2::Error::None);
  status = HWCSession::CallDisplayFunction(device, display, &HWCDisplay::SetCursorPosition,
                                           layer, x, y);
  if (status == INT32(HWC2::Error::None)) {
    // Update cursor position
    HWCSession::CallLayerFunction(device, display, layer, &HWCLayer::SetCursorPosition, x, y);
  }
  return status;
}

static int32_t SetLayerBlendMode(hwc2_device_t *device, hwc2_display_t display, hwc2_layer_t layer,
                                 int32_t int_mode) {
  if (int_mode < HWC2_BLEND_MODE_INVALID || int_mode > HWC2_BLEND_MODE_COVERAGE) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  auto mode = static_cast<HWC2::BlendMode>(int_mode);
  return HWCSession::CallLayerFunction(device, display, layer, &HWCLayer::SetLayerBlendMode, mode);
}

static int32_t SetLayerBuffer(hwc2_device_t *device, hwc2_display_t display, hwc2_layer_t layer,
                              buffer_handle_t buffer, int32_t acquire_fence) {
  return HWCSession::CallLayerFunction(device, display, layer, &HWCLayer::SetLayerBuffer, buffer,
                                       acquire_fence);
}

static int32_t SetLayerColor(hwc2_device_t *device, hwc2_display_t display, hwc2_layer_t layer,
                             hwc_color_t color) {
  return HWCSession::CallLayerFunction(device, display, layer, &HWCLayer::SetLayerColor, color);
}

static int32_t SetLayerCompositionType(hwc2_device_t *device, hwc2_display_t display,
                                       hwc2_layer_t layer, int32_t int_type) {
  auto type = static_cast<HWC2::Composition>(int_type);
  return HWCSession::CallLayerFunction(device, display, layer, &HWCLayer::SetLayerCompositionType,
                                       type);
}

static int32_t SetLayerDataspace(hwc2_device_t *device, hwc2_display_t display, hwc2_layer_t layer,
                                 int32_t dataspace) {
  return HWCSession::CallLayerFunction(device, display, layer, &HWCLayer::SetLayerDataspace,
                                       dataspace);
}

static int32_t SetLayerDisplayFrame(hwc2_device_t *device, hwc2_display_t display,
                                    hwc2_layer_t layer, hwc_rect_t frame) {
  return HWCSession::CallLayerFunction(device, display, layer, &HWCLayer::SetLayerDisplayFrame,
                                       frame);
}

static int32_t SetLayerPlaneAlpha(hwc2_device_t *device, hwc2_display_t display, hwc2_layer_t layer,
                                  float alpha) {
  return HWCSession::CallLayerFunction(device, display, layer, &HWCLayer::SetLayerPlaneAlpha,
                                       alpha);
}

static int32_t SetLayerSourceCrop(hwc2_device_t *device, hwc2_display_t display, hwc2_layer_t layer,
                                  hwc_frect_t crop) {
  return HWCSession::CallLayerFunction(device, display, layer, &HWCLayer::SetLayerSourceCrop, crop);
}

static int32_t SetLayerSurfaceDamage(hwc2_device_t *device, hwc2_display_t display,
                                     hwc2_layer_t layer, hwc_region_t damage) {
  return HWCSession::CallLayerFunction(device, display, layer, &HWCLayer::SetLayerSurfaceDamage,
                                       damage);
}

static int32_t SetLayerTransform(hwc2_device_t *device, hwc2_display_t display, hwc2_layer_t layer,
                                 int32_t int_transform) {
  auto transform = static_cast<HWC2::Transform>(int_transform);
  return HWCSession::CallLayerFunction(device, display, layer, &HWCLayer::SetLayerTransform,
                                       transform);
}

static int32_t SetLayerVisibleRegion(hwc2_device_t *device, hwc2_display_t display,
                                     hwc2_layer_t layer, hwc_region_t visible) {
  return HWCSession::CallLayerFunction(device, display, layer, &HWCLayer::SetLayerVisibleRegion,
                                       visible);
}

static int32_t SetLayerZOrder(hwc2_device_t *device, hwc2_display_t display, hwc2_layer_t layer,
                              uint32_t z) {
  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::SetLayerZOrder, layer, z);
}

int32_t HWCSession::SetOutputBuffer(hwc2_device_t *device, hwc2_display_t display,
                                    buffer_handle_t buffer, int32_t releaseFence) {
  if (!device) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  auto *hwc_session = static_cast<HWCSession *>(device);
  if (INT32(display) != hwc_session->GetDisplayIndex(qdutils::DISPLAY_VIRTUAL)) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  SCOPE_LOCK(locker_[display]);
  if (hwc_session->hwc_display_[display]) {
    auto vds = reinterpret_cast<HWCDisplayVirtual *>(hwc_session->hwc_display_[display]);
    auto status = vds->SetOutputBuffer(buffer, releaseFence);
    return INT32(status);
  } else {
    return HWC2_ERROR_BAD_DISPLAY;
  }
}

int32_t HWCSession::SetPowerMode(hwc2_device_t *device, hwc2_display_t display, int32_t int_mode) {
  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  //  validate device and also avoid undefined behavior in cast to HWC2::PowerMode
  if (!device || int_mode < HWC2_POWER_MODE_OFF || int_mode > HWC2_POWER_MODE_DOZE_SUSPEND) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  HWCSession *hwc_session = static_cast<HWCSession *>(device);
  if (hwc_session->power_on_pending_[display]) {
    DLOGW("Set power mode is not allowed during secure display session");
    return HWC2_ERROR_UNSUPPORTED;
  }

  auto mode = static_cast<HWC2::PowerMode>(int_mode);

  //  all displays support on/off. Check for doze modes
  int support = 0;
  auto status = hwc_session->GetDozeSupport(device, display, &support);
  if (status != HWC2_ERROR_NONE) {
    DLOGE("Failed to get doze support Error = %d", status);
    return INT32(status);
  }

  if (!support && (mode == HWC2::PowerMode::Doze || mode == HWC2::PowerMode::DozeSuspend)) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  bool override_mode = hwc_session->async_powermode_ &&
                       hwc_session->display_ready_.test(UINT32(display));
  if (!override_mode) {
    auto error = CallDisplayFunction(device, display, &HWCDisplay::SetPowerMode, mode,
                                   false /* teardown */);
    if (error != HWC2_ERROR_NONE) {
      return error;
    }
  } else {
    SCOPE_LOCK(locker_[display]);
    // Update hwc state for now. Actual poweron will handled through DisplayConfig.
    hwc_session->hwc_display_[display]->UpdatePowerMode(mode);
  }
  // Reset idle pc ref count on suspend, as we enable idle pc during suspend.
  if (mode == HWC2::PowerMode::Off) {
    hwc_session->idle_pc_ref_cnt_ = 0;
  }

  hwc_session->UpdateThrottlingRate();

  // Trigger refresh for doze mode to take effect.
  if (mode == HWC2::PowerMode::Doze) {
    hwc_session->Refresh(display);
    // Trigger one more refresh for PP features to take effect.
    hwc_session->pending_refresh_.set(UINT32(display));
  } else {
    // Reset the pending refresh bit
    hwc_session->pending_refresh_.reset(UINT32(display));
  }

  return HWC2_ERROR_NONE;
}

int32_t HWCSession::SetVsyncEnabled(hwc2_device_t *device, hwc2_display_t display,
                                    int32_t int_enabled) {
  //  avoid undefined behavior in cast to HWC2::Vsync
  if (int_enabled < HWC2_VSYNC_INVALID || int_enabled > HWC2_VSYNC_DISABLE) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  auto enabled = static_cast<HWC2::Vsync>(int_enabled);

  HWCSession *hwc_session = static_cast<HWCSession *>(device);

  if (int_enabled == HWC2_VSYNC_ENABLE) {
    hwc_session->callbacks_.UpdateVsyncSource(display);
  }

  return HWCSession::CallDisplayFunction(device, display, &HWCDisplay::SetVsyncEnabled, enabled);
}

int32_t HWCSession::GetDozeSupport(hwc2_device_t *device, hwc2_display_t display,
                                   int32_t *out_support) {
  if (!device || !out_support) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  HWCSession *hwc_session = static_cast<HWCSession *>(device);
  HWCDisplay *hwc_display = hwc_session->hwc_display_[display];;
  if (display >= HWCCallbacks::kNumDisplays || (hwc_display == nullptr)) {
    DLOGE("Invalid Display %d Handle %s ", display, hwc_display ?
          "Valid" : "NULL");
    return HWC2_ERROR_BAD_DISPLAY;
  }

  *out_support = 0;
  if (hwc_display->GetDisplayClass() == DISPLAY_CLASS_BUILTIN) {
    *out_support = 1;
  }

  return HWC2_ERROR_NONE;
}

int32_t HWCSession::ValidateDisplay(hwc2_device_t *device, hwc2_display_t display,
                                    uint32_t *out_num_types, uint32_t *out_num_requests) {
  //  out_num_types and out_num_requests will be non-NULL
  if (!device) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  HWCSession *hwc_session = static_cast<HWCSession *>(device);

  {
    SCOPE_LOCK(power_state_[display]);
    if (hwc_session->power_state_transition_[display]) {
      // Route all interactions with client to dummy display.
      display = hwc_session->map_hwc_display_.find(display)->second;
    }
  }
  DTRACE_SCOPED();
  // TODO(user): Handle secure session, handle QDCM solid fill
  auto status = HWC2::Error::BadDisplay;
  hwc_session->HandleSecureSession();
  {
    SEQUENCE_ENTRY_SCOPE_LOCK(locker_[display]);
    if (power_on_pending_[display]) {
      status = HWC2::Error::None;
    } else if (hwc_session->hwc_display_[display]) {
      hwc_session->hwc_display_[display]->SetFastPathComposition(false);
      status = hwc_session->ValidateDisplayInternal(display, out_num_types, out_num_requests);
    }
  }

  // Sequence locking currently begins on Validate, so cancel the sequence lock on failures
  if (status != HWC2::Error::None && status != HWC2::Error::HasChanges) {
    SEQUENCE_CANCEL_SCOPE_LOCK(locker_[display]);
  }

  return INT32(status);
}

hwc2_function_pointer_t HWCSession::GetFunction(struct hwc2_device *device,
                                                int32_t int_descriptor) {
  auto descriptor = static_cast<HWC2::FunctionDescriptor>(int_descriptor);

  switch (descriptor) {
    case HWC2::FunctionDescriptor::AcceptDisplayChanges:
      return AsFP<HWC2_PFN_ACCEPT_DISPLAY_CHANGES>(HWCSession::AcceptDisplayChanges);
    case HWC2::FunctionDescriptor::CreateLayer:
      return AsFP<HWC2_PFN_CREATE_LAYER>(CreateLayer);
    case HWC2::FunctionDescriptor::CreateVirtualDisplay:
      return AsFP<HWC2_PFN_CREATE_VIRTUAL_DISPLAY>(HWCSession::CreateVirtualDisplay);
    case HWC2::FunctionDescriptor::DestroyLayer:
      return AsFP<HWC2_PFN_DESTROY_LAYER>(DestroyLayer);
    case HWC2::FunctionDescriptor::DestroyVirtualDisplay:
      return AsFP<HWC2_PFN_DESTROY_VIRTUAL_DISPLAY>(HWCSession::DestroyVirtualDisplay);
    case HWC2::FunctionDescriptor::Dump:
      return AsFP<HWC2_PFN_DUMP>(HWCSession::Dump);
    case HWC2::FunctionDescriptor::GetActiveConfig:
      return AsFP<HWC2_PFN_GET_ACTIVE_CONFIG>(GetActiveConfig);
    case HWC2::FunctionDescriptor::GetChangedCompositionTypes:
      return AsFP<HWC2_PFN_GET_CHANGED_COMPOSITION_TYPES>(GetChangedCompositionTypes);
    case HWC2::FunctionDescriptor::GetClientTargetSupport:
      return AsFP<HWC2_PFN_GET_CLIENT_TARGET_SUPPORT>(GetClientTargetSupport);
    case HWC2::FunctionDescriptor::GetColorModes:
      return AsFP<HWC2_PFN_GET_COLOR_MODES>(GetColorModes);
    case HWC2::FunctionDescriptor::GetDisplayAttribute:
      return AsFP<HWC2_PFN_GET_DISPLAY_ATTRIBUTE>(GetDisplayAttribute);
    case HWC2::FunctionDescriptor::GetDisplayConfigs:
      return AsFP<HWC2_PFN_GET_DISPLAY_CONFIGS>(GetDisplayConfigs);
    case HWC2::FunctionDescriptor::GetDisplayName:
      return AsFP<HWC2_PFN_GET_DISPLAY_NAME>(GetDisplayName);
    case HWC2::FunctionDescriptor::GetDisplayRequests:
      return AsFP<HWC2_PFN_GET_DISPLAY_REQUESTS>(GetDisplayRequests);
    case HWC2::FunctionDescriptor::GetDisplayType:
      return AsFP<HWC2_PFN_GET_DISPLAY_TYPE>(GetDisplayType);
    case HWC2::FunctionDescriptor::GetHdrCapabilities:
      return AsFP<HWC2_PFN_GET_HDR_CAPABILITIES>(GetHdrCapabilities);
    case HWC2::FunctionDescriptor::GetDozeSupport:
      return AsFP<HWC2_PFN_GET_DOZE_SUPPORT>(GetDozeSupport);
    case HWC2::FunctionDescriptor::GetMaxVirtualDisplayCount:
      return AsFP<HWC2_PFN_GET_MAX_VIRTUAL_DISPLAY_COUNT>(HWCSession::GetMaxVirtualDisplayCount);
    case HWC2::FunctionDescriptor::GetReleaseFences:
      return AsFP<HWC2_PFN_GET_RELEASE_FENCES>(GetReleaseFences);
    case HWC2::FunctionDescriptor::PresentDisplay:
      return AsFP<HWC2_PFN_PRESENT_DISPLAY>(PresentDisplay);
    case HWC2::FunctionDescriptor::RegisterCallback:
      return AsFP<HWC2_PFN_REGISTER_CALLBACK>(RegisterCallback);
    case HWC2::FunctionDescriptor::SetActiveConfig:
      return AsFP<HWC2_PFN_SET_ACTIVE_CONFIG>(SetActiveConfig);
    case HWC2::FunctionDescriptor::SetClientTarget:
      return AsFP<HWC2_PFN_SET_CLIENT_TARGET>(SetClientTarget);
    case HWC2::FunctionDescriptor::SetColorMode:
      return AsFP<HWC2_PFN_SET_COLOR_MODE>(SetColorMode);
    case HWC2::FunctionDescriptor::SetColorTransform:
      return AsFP<HWC2_PFN_SET_COLOR_TRANSFORM>(SetColorTransform);
    case HWC2::FunctionDescriptor::SetCursorPosition:
      return AsFP<HWC2_PFN_SET_CURSOR_POSITION>(SetCursorPosition);
    case HWC2::FunctionDescriptor::SetLayerBlendMode:
      return AsFP<HWC2_PFN_SET_LAYER_BLEND_MODE>(SetLayerBlendMode);
    case HWC2::FunctionDescriptor::SetLayerBuffer:
      return AsFP<HWC2_PFN_SET_LAYER_BUFFER>(SetLayerBuffer);
    case HWC2::FunctionDescriptor::SetLayerColor:
      return AsFP<HWC2_PFN_SET_LAYER_COLOR>(SetLayerColor);
    case HWC2::FunctionDescriptor::SetLayerCompositionType:
      return AsFP<HWC2_PFN_SET_LAYER_COMPOSITION_TYPE>(SetLayerCompositionType);
    case HWC2::FunctionDescriptor::SetLayerDataspace:
      return AsFP<HWC2_PFN_SET_LAYER_DATASPACE>(SetLayerDataspace);
    case HWC2::FunctionDescriptor::SetLayerDisplayFrame:
      return AsFP<HWC2_PFN_SET_LAYER_DISPLAY_FRAME>(SetLayerDisplayFrame);
    case HWC2::FunctionDescriptor::SetLayerPlaneAlpha:
      return AsFP<HWC2_PFN_SET_LAYER_PLANE_ALPHA>(SetLayerPlaneAlpha);
    // Sideband stream is not supported
    // case HWC2::FunctionDescriptor::SetLayerSidebandStream:
    case HWC2::FunctionDescriptor::SetLayerSourceCrop:
      return AsFP<HWC2_PFN_SET_LAYER_SOURCE_CROP>(SetLayerSourceCrop);
    case HWC2::FunctionDescriptor::SetLayerSurfaceDamage:
      return AsFP<HWC2_PFN_SET_LAYER_SURFACE_DAMAGE>(SetLayerSurfaceDamage);
    case HWC2::FunctionDescriptor::SetLayerTransform:
      return AsFP<HWC2_PFN_SET_LAYER_TRANSFORM>(SetLayerTransform);
    case HWC2::FunctionDescriptor::SetLayerVisibleRegion:
      return AsFP<HWC2_PFN_SET_LAYER_VISIBLE_REGION>(SetLayerVisibleRegion);
    case HWC2::FunctionDescriptor::SetLayerZOrder:
      return AsFP<HWC2_PFN_SET_LAYER_Z_ORDER>(SetLayerZOrder);
    case HWC2::FunctionDescriptor::SetOutputBuffer:
      return AsFP<HWC2_PFN_SET_OUTPUT_BUFFER>(SetOutputBuffer);
    case HWC2::FunctionDescriptor::SetPowerMode:
      return AsFP<HWC2_PFN_SET_POWER_MODE>(SetPowerMode);
    case HWC2::FunctionDescriptor::SetVsyncEnabled:
      return AsFP<HWC2_PFN_SET_VSYNC_ENABLED>(SetVsyncEnabled);
    case HWC2::FunctionDescriptor::ValidateDisplay:
      return AsFP<HWC2_PFN_VALIDATE_DISPLAY>(HWCSession::ValidateDisplay);
    case HWC2::FunctionDescriptor::SetReadbackBuffer:
      return AsFP<HWC2_PFN_SET_READBACK_BUFFER>(HWCSession::SetReadbackBuffer);
    case HWC2::FunctionDescriptor::GetReadbackBufferAttributes:
      return AsFP<HWC2_PFN_GET_READBACK_BUFFER_ATTRIBUTES>(HWCSession::GetReadbackBufferAttributes);
    case HWC2::FunctionDescriptor::GetReadbackBufferFence:
      return AsFP<HWC2_PFN_GET_READBACK_BUFFER_FENCE>(HWCSession::GetReadbackBufferFence);
    case HWC2::FunctionDescriptor::GetRenderIntents:
      return AsFP<HWC2_PFN_GET_RENDER_INTENTS>(GetRenderIntents);
    case HWC2::FunctionDescriptor::SetColorModeWithRenderIntent:
      return AsFP<HWC2_PFN_SET_COLOR_MODE_WITH_RENDER_INTENT>
             (HWCSession::SetColorModeWithRenderIntent);
    case HWC2::FunctionDescriptor::GetDataspaceSaturationMatrix:
      return AsFP<HWC2_PFN_GET_DATASPACE_SATURATION_MATRIX>(GetDataspaceSaturationMatrix);
    case HWC2::FunctionDescriptor::GetPerFrameMetadataKeys:
      return AsFP<HWC2_PFN_GET_PER_FRAME_METADATA_KEYS>(GetPerFrameMetadataKeys);
    case HWC2::FunctionDescriptor::SetLayerPerFrameMetadata:
      return AsFP<HWC2_PFN_SET_LAYER_PER_FRAME_METADATA>(SetLayerPerFrameMetadata);
    case HWC2::FunctionDescriptor::GetDisplayIdentificationData:
      return AsFP<HWC2_PFN_GET_DISPLAY_IDENTIFICATION_DATA>
             (HWCSession::GetDisplayIdentificationData);
    case HWC2::FunctionDescriptor::GetDisplayCapabilities:
      return AsFP<HWC2_PFN_GET_DISPLAY_CAPABILITIES>(HWCSession::GetDisplayCapabilities);
    case HWC2::FunctionDescriptor::GetDisplayBrightnessSupport:
      return AsFP<HWC2_PFN_GET_DISPLAY_BRIGHTNESS_SUPPORT>(HWCSession::GetDisplayBrightnessSupport);
    default:
      DLOGD("Unknown/Unimplemented function descriptor: %d (%s)", int_descriptor,
            to_string(descriptor).c_str());
      return nullptr;
  }
  return nullptr;
}

HWC2::Error HWCSession::CreateVirtualDisplayObj(uint32_t width, uint32_t height, int32_t *format,
                                                hwc2_display_t *out_display_id) {
  if (!client_connected_) {
    DLOGE("Client is not ready yet.");
    return HWC2::Error::BadDisplay;
  }

  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[active_builtin_disp_id]);
    std::bitset<kSecureMax> secure_sessions = 0;
    if (hwc_display_[active_builtin_disp_id]) {
      hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
    }
    if (secure_sessions.any()) {
      DLOGE("Secure session is active, cannot create virtual display.");
      return HWC2::Error::Unsupported;
    } else if (IsPluggableDisplayConnected()) {
      DLOGE("External session is active, cannot create virtual display.");
      return HWC2::Error::Unsupported;
    }
  }

  if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
    DisplayError error = hwc_display_[HWC_DISPLAY_PRIMARY]->TeardownConcurrentWriteback();
    if (error) {
      return HWC2::Error::NoResources;
    }
  }

  HWDisplaysInfo hw_displays_info = {};
  DisplayError error = core_intf_->GetDisplaysStatus(&hw_displays_info);
  if (error != kErrorNone) {
    DLOGE("Failed to get connected display list. Error = %d", error);
    return HWC2::Error::BadDisplay;
  }

  // Lock confined to this scope
  int status = -EINVAL;
  for (auto &iter : hw_displays_info) {
    auto &info = iter.second;
    if (info.display_type != kVirtual) {
      continue;
    }

    for (auto &map_info : map_info_virtual_) {
      hwc2_display_t client_id = map_info.client_id;
      {
        SCOPE_LOCK(locker_[client_id]);
        auto &hwc_display = hwc_display_[client_id];
        if (hwc_display) {
          continue;
        }

        status = HWCDisplayVirtual::Create(core_intf_, &buffer_allocator_, &callbacks_, client_id,
                                           info.display_id, width, height, format, &hwc_display,
                                           set_min_lum_, set_max_lum_);
        // TODO(user): validate width and height support
        if (status) {
          return HWC2::Error::NoResources;
        }

        is_hdr_display_[UINT32(client_id)] = HasHDRSupport(hwc_display);
        DLOGI("Created virtual display id:% " PRIu64 " with res: %dx%d", client_id, width, height);

        *out_display_id = client_id;
        map_info.disp_type = info.display_type;
        map_info.sdm_id = info.display_id;
        break;
      }
    }
  }

  if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[active_builtin_disp_id]);
    hwc_display_[active_builtin_disp_id]->ResetValidation();
  }

  return HWC2::Error::None;
}

bool HWCSession::IsPluggableDisplayConnected() {
  for (auto &map_info : map_info_pluggable_) {
    if (hwc_display_[map_info.client_id]) {
      return true;
    }
  }
  return false;
}

// Qclient methods
android::status_t HWCSession::notifyCallback(uint32_t command, const android::Parcel *input_parcel,
                                             android::Parcel *output_parcel) {
  android::status_t status = -EINVAL;

  switch (command) {
    case qService::IQService::DYNAMIC_DEBUG:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = 0;
      DynamicDebug(input_parcel);
      break;

    case qService::IQService::SCREEN_REFRESH:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = RefreshScreen(input_parcel);
      break;

    case qService::IQService::SET_IDLE_TIMEOUT:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = setIdleTimeout(UINT32(input_parcel->readInt32()));
      break;

    case qService::IQService::SET_FRAME_DUMP_CONFIG:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetFrameDumpConfig(input_parcel);
      break;

    case qService::IQService::SET_MAX_PIPES_PER_MIXER:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetMaxMixerStages(input_parcel);
      break;

    case qService::IQService::SET_DISPLAY_MODE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetDisplayMode(input_parcel);
      break;

    case qService::IQService::SET_SECONDARY_DISPLAY_STATUS: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }
        int disp_id = INT(input_parcel->readInt32());
        HWCDisplay::DisplayStatus disp_status =
              static_cast<HWCDisplay::DisplayStatus>(input_parcel->readInt32());
        status = SetSecondaryDisplayStatus(disp_id, disp_status);
        output_parcel->writeInt32(status);
      }
      break;

    case qService::IQService::CONFIGURE_DYN_REFRESH_RATE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = ConfigureRefreshRate(input_parcel);
      break;

    case qService::IQService::SET_VIEW_FRAME:
      status = 0;
      break;

    case qService::IQService::TOGGLE_SCREEN_UPDATES: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }
        int32_t input = input_parcel->readInt32();
        status = toggleScreenUpdate(input == 1);
        output_parcel->writeInt32(status);
      }
      break;

    case qService::IQService::QDCM_SVC_CMDS:
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      status = QdcmCMDHandler(input_parcel, output_parcel);
      break;

    case qService::IQService::MIN_HDCP_ENCRYPTION_LEVEL_CHANGED: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }
        int disp_id = input_parcel->readInt32();
        uint32_t min_enc_level = UINT32(input_parcel->readInt32());
        status = MinHdcpEncryptionLevelChanged(disp_id, min_enc_level);
        output_parcel->writeInt32(status);
      }
      break;

    case qService::IQService::CONTROL_PARTIAL_UPDATE: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }
        int disp_id = input_parcel->readInt32();
        uint32_t enable = UINT32(input_parcel->readInt32());
        status = ControlPartialUpdate(disp_id, enable == 1);
        output_parcel->writeInt32(status);
      }
      break;

    case qService::IQService::SET_ACTIVE_CONFIG: {
        if (!input_parcel) {
          DLOGE("QService command = %d: input_parcel needed.", command);
          break;
        }
        uint32_t config = UINT32(input_parcel->readInt32());
        int disp_id = input_parcel->readInt32();
        status = SetActiveConfigIndex(disp_id, config);
      }
      break;

    case qService::IQService::GET_ACTIVE_CONFIG: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }
        int disp_id = input_parcel->readInt32();
        uint32_t config = 0;
        status = GetActiveConfigIndex(disp_id, &config);
        output_parcel->writeInt32(INT(config));
      }
      break;

    case qService::IQService::GET_CONFIG_COUNT: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }
        int disp_id = input_parcel->readInt32();
        uint32_t count = 0;
        status = GetConfigCount(disp_id, &count);
        output_parcel->writeInt32(INT(count));
      }
      break;

    case qService::IQService::GET_DISPLAY_ATTRIBUTES_FOR_CONFIG:
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      status = GetDisplayAttributesForConfig(input_parcel, output_parcel);
      break;

    case qService::IQService::GET_PANEL_BRIGHTNESS: {
        if (!output_parcel) {
          DLOGE("QService command = %d: output_parcel needed.", command);
          break;
        }
        int level = 0;
        status = GetPanelBrightness(&level);
        output_parcel->writeInt32(level);
      }
      break;

    case qService::IQService::SET_PANEL_BRIGHTNESS: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }
        uint32_t level = UINT32(input_parcel->readInt32());
        status = setPanelBrightness(level);
        output_parcel->writeInt32(status);
      }
      break;

    case qService::IQService::GET_DISPLAY_VISIBLE_REGION:
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      status = GetVisibleDisplayRect(input_parcel, output_parcel);
      break;

    case qService::IQService::SET_CAMERA_STATUS: {
        if (!input_parcel) {
          DLOGE("QService command = %d: input_parcel needed.", command);
          break;
        }
        uint32_t camera_status = UINT32(input_parcel->readInt32());
        status = setCameraLaunchStatus(camera_status);
      }
      break;

    case qService::IQService::GET_BW_TRANSACTION_STATUS: {
        if (!output_parcel) {
          DLOGE("QService command = %d: output_parcel needed.", command);
          break;
        }
        bool state = true;
        status = DisplayBWTransactionPending(&state);
        output_parcel->writeInt32(state);
      }
      break;

    case qService::IQService::SET_LAYER_MIXER_RESOLUTION:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetMixerResolution(input_parcel);
      break;

    case qService::IQService::SET_COLOR_MODE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetColorModeOverride(input_parcel);
      break;

    case qService::IQService::SET_COLOR_MODE_WITH_RENDER_INTENT:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetColorModeWithRenderIntentOverride(input_parcel);
      break;

    case qService::IQService::SET_COLOR_MODE_BY_ID:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetColorModeById(input_parcel);
      break;

    case qService::IQService::GET_COMPOSER_STATUS:
      if (!output_parcel) {
        DLOGE("QService command = %d: output_parcel needed.", command);
        break;
      }
      status = 0;
      output_parcel->writeInt32(getComposerStatus());
      break;

    case qService::IQService::SET_QSYNC_MODE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetQSyncMode(input_parcel);
      break;

    case qService::IQService::SET_IDLE_PC:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetIdlePC(input_parcel);
      break;

    case qService::IQService::SET_DPPS_AD4_ROI_CONFIG:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetAd4RoiConfig(input_parcel);
      break;

    case qService::IQService::SET_DSI_CLK:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetDsiClk(input_parcel);
      break;

    case qService::IQService::GET_DSI_CLK:
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      status = GetDsiClk(input_parcel, output_parcel);
      break;

    case qService::IQService::GET_SUPPORTED_DSI_CLK:
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      status = GetSupportedDsiClk(input_parcel, output_parcel);
      break;

    case qService::IQService::SET_PANEL_LUMINANCE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetPanelLuminanceAttributes(input_parcel);
      break;

    case qService::IQService::SET_COLOR_MODE_FROM_CLIENT:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetColorModeFromClient(input_parcel);
      break;

    default:
      DLOGW("QService command = %d is not supported.", command);
      break;
  }

  return status;
}

android::status_t HWCSession::getComposerStatus() {
  return is_composer_up_;
}

android::status_t HWCSession::GetDisplayAttributesForConfig(const android::Parcel *input_parcel,
                                                            android::Parcel *output_parcel) {
  int config = input_parcel->readInt32();
  int dpy = input_parcel->readInt32();
  int error = android::BAD_VALUE;
  DisplayConfigVariableInfo display_attributes;

  int disp_idx = GetDisplayIndex(dpy);
  if (disp_idx == -1 || config < 0) {
    DLOGE("Invalid display = %d, or config = %d", dpy, config);
    return android::BAD_VALUE;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
  if (hwc_display_[disp_idx]) {
    error = hwc_display_[disp_idx]->GetDisplayAttributesForConfig(config, &display_attributes);
    if (error == 0) {
      output_parcel->writeInt32(INT(display_attributes.vsync_period_ns));
      output_parcel->writeInt32(INT(display_attributes.x_pixels));
      output_parcel->writeInt32(INT(display_attributes.y_pixels));
      output_parcel->writeFloat(display_attributes.x_dpi);
      output_parcel->writeFloat(display_attributes.y_dpi);
      output_parcel->writeInt32(0);  // Panel type, unsupported.
    }
  }

  return error;
}

android::status_t HWCSession::ConfigureRefreshRate(const android::Parcel *input_parcel) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  uint32_t operation = UINT32(input_parcel->readInt32());
  HWCDisplay *hwc_display = hwc_display_[HWC_DISPLAY_PRIMARY];

  if (!hwc_display) {
    DLOGW("Display = %d is not connected.", HWC_DISPLAY_PRIMARY);
    return -ENODEV;
  }

  switch (operation) {
    case qdutils::DISABLE_METADATA_DYN_REFRESH_RATE:
      return hwc_display->Perform(HWCDisplayBuiltIn::SET_METADATA_DYN_REFRESH_RATE, false);

    case qdutils::ENABLE_METADATA_DYN_REFRESH_RATE:
      return hwc_display->Perform(HWCDisplayBuiltIn::SET_METADATA_DYN_REFRESH_RATE, true);

    case qdutils::SET_BINDER_DYN_REFRESH_RATE: {
      uint32_t refresh_rate = UINT32(input_parcel->readInt32());
      return hwc_display->Perform(HWCDisplayBuiltIn::SET_BINDER_DYN_REFRESH_RATE, refresh_rate);
    }

    default:
      DLOGW("Invalid operation %d", operation);
      return -EINVAL;
  }

  return 0;
}

android::status_t HWCSession::SetDisplayMode(const android::Parcel *input_parcel) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  if (!hwc_display_[HWC_DISPLAY_PRIMARY]) {
    DLOGW("Display = %d is not connected.", HWC_DISPLAY_PRIMARY);
    return -ENODEV;
  }

  uint32_t mode = UINT32(input_parcel->readInt32());
  return hwc_display_[HWC_DISPLAY_PRIMARY]->Perform(HWCDisplayBuiltIn::SET_DISPLAY_MODE, mode);
}

android::status_t HWCSession::SetMaxMixerStages(const android::Parcel *input_parcel) {
  DisplayError error = kErrorNone;
  std::bitset<32> bit_mask_display_type = UINT32(input_parcel->readInt32());
  uint32_t max_mixer_stages = UINT32(input_parcel->readInt32());
  android::status_t status = 0;

  for (uint32_t i = 0; i < 32 && bit_mask_display_type[i]; i++) {
    int disp_idx = GetDisplayIndex(INT(i));
    if (disp_idx == -1) {
      continue;
    }
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
    auto &hwc_display = hwc_display_[disp_idx];
    if (!hwc_display) {
      DLOGW("Display = %d is not connected.", disp_idx);
      status = (status)? status : -ENODEV;  // Return higher priority error.
      continue;
    }

    error = hwc_display->SetMaxMixerStages(max_mixer_stages);
    if (error != kErrorNone) {
      status = -EINVAL;
    }
  }

  return status;
}

android::status_t HWCSession::SetFrameDumpConfig(const android::Parcel *input_parcel) {
  uint32_t frame_dump_count = UINT32(input_parcel->readInt32());
  std::bitset<32> bit_mask_display_type = UINT32(input_parcel->readInt32());
  uint32_t bit_mask_layer_type = UINT32(input_parcel->readInt32());
  int32_t output_format = HAL_PIXEL_FORMAT_RGB_888;
  bool post_processed = true;

  // Read optional user preferences: output_format and post_processed.
  if (input_parcel->dataPosition() != input_parcel->dataSize()) {
    // HAL Pixel Format for output buffer
    output_format = input_parcel->readInt32();
  }
  if (input_parcel->dataPosition() != input_parcel->dataSize()) {
    // Option to dump Layer Mixer output (0) or DSPP output (1)
    post_processed = (input_parcel->readInt32() != 0);
  }

  android::status_t status = 0;

  for (uint32_t i = 0; i < bit_mask_display_type.size(); i++) {
    if (!bit_mask_display_type[i]) {
      continue;
    }
    int disp_idx = GetDisplayIndex(INT(i));
    if (disp_idx == -1) {
      continue;
    }
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
    auto &hwc_display = hwc_display_[disp_idx];
    if (!hwc_display) {
      DLOGW("Display = %d is not connected.", disp_idx);
      status = (status)? status : -ENODEV;  // Return higher priority error.
      continue;
    }

    HWC2::Error error = hwc_display->SetFrameDumpConfig(frame_dump_count, bit_mask_layer_type,
                                                        output_format, post_processed);
    if (error != HWC2::Error::None) {
      status = (HWC2::Error::NoResources == error) ? -ENOMEM : -EINVAL;
    }
  }

  return status;
}

android::status_t HWCSession::SetMixerResolution(const android::Parcel *input_parcel) {
  DisplayError error = kErrorNone;
  uint32_t dpy = UINT32(input_parcel->readInt32());

  if (dpy != HWC_DISPLAY_PRIMARY) {
    DLOGW("Resolution change not supported for this display = %d", dpy);
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);
  if (!hwc_display_[HWC_DISPLAY_PRIMARY]) {
    DLOGW("Primary display is not initialized");
    return -ENODEV;
  }

  uint32_t width = UINT32(input_parcel->readInt32());
  uint32_t height = UINT32(input_parcel->readInt32());

  error = hwc_display_[HWC_DISPLAY_PRIMARY]->SetMixerResolution(width, height);
  if (error != kErrorNone) {
    return -EINVAL;
  }

  return 0;
}

android::status_t HWCSession::SetColorModeOverride(const android::Parcel *input_parcel) {
  int display = static_cast<int>(input_parcel->readInt32());
  auto mode = static_cast<ColorMode>(input_parcel->readInt32());
  auto device = static_cast<hwc2_device_t *>(this);

  int disp_idx = GetDisplayIndex(display);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", display);
    return -EINVAL;
  }

  if (mode < ColorMode::NATIVE || mode > ColorMode::BT2100_HLG) {
    DLOGE("Invalid ColorMode: %d", mode);
    return HWC2_ERROR_BAD_PARAMETER;
  }
  auto err = CallDisplayFunction(device, static_cast<hwc2_display_t>(disp_idx),
                                 &HWCDisplay::SetColorMode, mode);
  if (err != HWC2_ERROR_NONE)
    return -EINVAL;

  return 0;
}

android::status_t HWCSession::SetAd4RoiConfig(const android::Parcel *input_parcel) {
  auto display_id = static_cast<uint32_t>(input_parcel->readInt32());
  auto h_s = static_cast<uint32_t>(input_parcel->readInt32());
  auto h_e = static_cast<uint32_t>(input_parcel->readInt32());
  auto v_s = static_cast<uint32_t>(input_parcel->readInt32());
  auto v_e = static_cast<uint32_t>(input_parcel->readInt32());
  auto f_in = static_cast<uint32_t>(input_parcel->readInt32());
  auto f_out = static_cast<uint32_t>(input_parcel->readInt32());

  return static_cast<android::status_t>(SetDisplayDppsAdROI(display_id, h_s, h_e, v_s,
                                                            v_e, f_in, f_out));
}

android::status_t HWCSession::SetColorModeWithRenderIntentOverride(
    const android::Parcel *input_parcel) {
  auto display = static_cast<hwc2_display_t>(input_parcel->readInt32());
  auto mode = static_cast<ColorMode>(input_parcel->readInt32());
  auto intent = static_cast<RenderIntent>(input_parcel->readInt32());
  auto device = static_cast<hwc2_device_t *>(this);

  if (mode < ColorMode::NATIVE || mode > ColorMode::BT2100_HLG) {
    DLOGE("Invalid ColorMode: %d", mode);
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (intent < RenderIntent::COLORIMETRIC || intent > RenderIntent::TONE_MAP_ENHANCE) {
    DLOGE("Invalid RenderIntent: %d", intent);
    return HWC2_ERROR_BAD_PARAMETER;
  }

  auto err =
      CallDisplayFunction(device, display, &HWCDisplay::SetColorModeWithRenderIntent, mode, intent);
  if (err != HWC2_ERROR_NONE)
    return -EINVAL;

  return 0;
}
android::status_t HWCSession::SetColorModeById(const android::Parcel *input_parcel) {
  int display = input_parcel->readInt32();
  auto mode = input_parcel->readInt32();
  auto device = static_cast<hwc2_device_t *>(this);

  int disp_idx = GetDisplayIndex(display);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", display);
    return -EINVAL;
  }

  auto err = CallDisplayFunction(device, static_cast<hwc2_display_t>(disp_idx),
                                 &HWCDisplay::SetColorModeById, mode);
  if (err != HWC2_ERROR_NONE)
    return -EINVAL;

  return 0;
}

android::status_t HWCSession::SetColorModeFromClient(const android::Parcel *input_parcel) {
  int display = input_parcel->readInt32();
  auto mode = input_parcel->readInt32();
  auto device = static_cast<hwc2_device_t *>(this);

  int disp_idx = GetDisplayIndex(display);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", display);
    return -EINVAL;
  }

  auto err = CallDisplayFunction(device, static_cast<hwc2_display_t>(disp_idx),
                                 &HWCDisplay::SetColorModeFromClientApi, mode);
  if (err != HWC2_ERROR_NONE)
    return -EINVAL;

  Refresh(static_cast<hwc2_display_t>(disp_idx));

  return 0;
}

android::status_t HWCSession::RefreshScreen(const android::Parcel *input_parcel) {
  int display = input_parcel->readInt32();

  int disp_idx = GetDisplayIndex(display);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", display);
    return -EINVAL;
  }

  Refresh(static_cast<hwc2_display_t>(disp_idx));

  return 0;
}

void HWCSession::DynamicDebug(const android::Parcel *input_parcel) {
  int type = input_parcel->readInt32();
  bool enable = (input_parcel->readInt32() > 0);
  DLOGI("type = %d enable = %d", type, enable);
  int verbose_level = input_parcel->readInt32();

  switch (type) {
    case qService::IQService::DEBUG_ALL:
      HWCDebugHandler::DebugAll(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_MDPCOMP:
      HWCDebugHandler::DebugStrategy(enable, verbose_level);
      HWCDebugHandler::DebugCompManager(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_PIPE_LIFECYCLE:
      HWCDebugHandler::DebugResources(enable, verbose_level);
      HWCDebugHandler::DebugQos(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_DRIVER_CONFIG:
      HWCDebugHandler::DebugDriverConfig(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_ROTATOR:
      HWCDebugHandler::DebugResources(enable, verbose_level);
      HWCDebugHandler::DebugDriverConfig(enable, verbose_level);
      HWCDebugHandler::DebugRotator(enable, verbose_level);
      HWCDebugHandler::DebugQos(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_QDCM:
      HWCDebugHandler::DebugQdcm(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_SCALAR:
      HWCDebugHandler::DebugScalar(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_CLIENT:
      HWCDebugHandler::DebugClient(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_DISPLAY:
      HWCDebugHandler::DebugDisplay(enable, verbose_level);
      break;

    default:
      DLOGW("type = %d is not supported", type);
  }
}

android::status_t HWCSession::QdcmCMDDispatch(uint32_t display_id,
                                              const PPDisplayAPIPayload &req_payload,
                                              PPDisplayAPIPayload *resp_payload,
                                              PPPendingParams *pending_action) {
  int ret = 0;
  bool is_physical_display = false;

  if (display_id >= HWCCallbacks::kNumDisplays || !hwc_display_[display_id]) {
      DLOGW("Invalid display id or display = %d is not connected.", display_id);
      return -ENODEV;
  }

  if (display_id == map_info_primary_.client_id) {
    is_physical_display = true;
  } else {
    for (auto &map_info : map_info_builtin_) {
      if (map_info.client_id == display_id) {
        is_physical_display = true;
        break;
     }
    }
  }

  if (!is_physical_display) {
    DLOGW("Skipping QDCM command dispatch on display = %d", display_id);
    return ret;
  }

  ret = hwc_display_[display_id]->ColorSVCRequestRoute(req_payload,
                                                       resp_payload,
                                                       pending_action);

  return ret;
}

android::status_t HWCSession::QdcmCMDHandler(const android::Parcel *input_parcel,
                                             android::Parcel *output_parcel) {
  int ret = 0;
  int32_t *brightness_value = NULL;
  uint32_t display_id(0);
  PPPendingParams pending_action;
  PPDisplayAPIPayload resp_payload, req_payload;
  uint8_t *disp_id = NULL;
  bool invalidate_needed = true;
  int32_t *mode_id = NULL;

  if (!color_mgr_) {
    DLOGW("color_mgr_ not initialized.");
    return -ENOENT;
  }

  pending_action.action = kNoAction;
  pending_action.params = NULL;

  // Read display_id, payload_size and payload from in_parcel.
  ret = HWCColorManager::CreatePayloadFromParcel(*input_parcel, &display_id, &req_payload);
  if (!ret) {
    ret = QdcmCMDDispatch(display_id, req_payload, &resp_payload, &pending_action);
  }

  if (ret) {
    output_parcel->writeInt32(ret);  // first field in out parcel indicates return code.
    req_payload.DestroyPayload();
    resp_payload.DestroyPayload();
    return ret;
  }

  if (kNoAction != pending_action.action) {
    int32_t action = pending_action.action;
    int count = -1;
    while (action > 0) {
      count++;
      int32_t bit = (action & 1);
      action = action >> 1;

      if (!bit)
        continue;

      DLOGV_IF(kTagQDCM, "pending action = %d, display_id = %d", BITMAP(count), display_id);
      switch (BITMAP(count)) {
        case kInvalidating:
          {
            invalidate_needed = false;
            Refresh(display_id);
          }
          break;
        case kEnterQDCMMode:
          ret = color_mgr_->EnableQDCMMode(true, hwc_display_[display_id]);
          break;
        case kExitQDCMMode:
          ret = color_mgr_->EnableQDCMMode(false, hwc_display_[display_id]);
          break;
        case kApplySolidFill:
          {
            SCOPE_LOCK(locker_[display_id]);
            ret = color_mgr_->SetSolidFill(pending_action.params,
                                           true, hwc_display_[display_id]);
          }
          Refresh(display_id);
          usleep(kSolidFillDelay);
          break;
        case kDisableSolidFill:
          {
            SCOPE_LOCK(locker_[display_id]);
            ret = color_mgr_->SetSolidFill(pending_action.params,
                                           false, hwc_display_[display_id]);
          }
          Refresh(display_id);
          usleep(kSolidFillDelay);
          break;
        case kSetPanelBrightness:
          brightness_value = reinterpret_cast<int32_t *>(resp_payload.payload);
          if (brightness_value == NULL) {
            DLOGE("Brightness value is Null");
            ret = -EINVAL;
          } else {
            ret = hwc_display_[display_id]->SetPanelBrightness(*brightness_value);
          }
          break;
        case kEnableFrameCapture:
          ret = color_mgr_->SetFrameCapture(pending_action.params, true,
                                            hwc_display_[display_id]);
          Refresh(display_id);
          break;
        case kDisableFrameCapture:
          ret = color_mgr_->SetFrameCapture(pending_action.params, false,
                                            hwc_display_[display_id]);
          break;
        case kConfigureDetailedEnhancer:
          ret = color_mgr_->SetDetailedEnhancer(pending_action.params,
                                                hwc_display_[display_id]);
          Refresh(display_id);
          break;
        case kModeSet:
          ret = static_cast<int>
                  (hwc_display_[display_id]->RestoreColorTransform());
          Refresh(display_id);
          break;
        case kNoAction:
          break;
        case kMultiDispProc:
          for (auto &map_info : map_info_builtin_) {
            uint32_t id = UINT32(map_info.client_id);
            if (id < HWCCallbacks::kNumDisplays && hwc_display_[id]) {
              int result = 0;
              resp_payload.DestroyPayload();
              result = hwc_display_[id]->ColorSVCRequestRoute(req_payload,
                                                              &resp_payload,
                                                              &pending_action);
              if (result) {
                DLOGW("Failed to dispatch action to disp %d ret %d", id, result);
                ret = result;
              }
            }
          }
          break;
        case kMultiDispGetId:
          ret = resp_payload.CreatePayloadBytes(HWCCallbacks::kNumDisplays, &disp_id);
          if (ret) {
            DLOGW("Unable to create response payload!");
          } else {
            for (int i = 0; i < HWCCallbacks::kNumDisplays; i++) {
              disp_id[i] = HWCCallbacks::kNumDisplays;
            }
            if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
              disp_id[HWC_DISPLAY_PRIMARY] = HWC_DISPLAY_PRIMARY;
            }
            for (auto &map_info : map_info_builtin_) {
              uint64_t id = map_info.client_id;
              if (id < HWCCallbacks::kNumDisplays && hwc_display_[id]) {
                disp_id[id] = (uint8_t)id;
              }
            }
          }
          break;
        case kSetModeFromClient:
          {
            SCOPE_LOCK(locker_[display_id]);
            mode_id = reinterpret_cast<int32_t *>(resp_payload.payload);
            if (mode_id) {
              ret = static_cast<int>(hwc_display_[display_id]->SetColorModeFromClientApi(*mode_id));
            } else {
              DLOGE("mode_id is Null");
              ret = -EINVAL;
            }
          }
          if (!ret) {
            Refresh(display_id);
          }
          break;
        default:
          DLOGW("Invalid pending action = %d!", pending_action.action);
          break;
      }
    }
  }
  // for display API getter case, marshall returned params into out_parcel.
  output_parcel->writeInt32(ret);
  HWCColorManager::MarshallStructIntoParcel(resp_payload, output_parcel);
  req_payload.DestroyPayload();
  resp_payload.DestroyPayload();

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[display_id]);
  if (invalidate_needed) {
    hwc_display_[display_id]->ResetValidation();
  }

  return ret;
}

int GetEventValue(const char *uevent_data, int length, const char *event_info) {
  const char *iterator_str = uevent_data;
  while (((iterator_str - uevent_data) <= length) && (*iterator_str)) {
    const char *pstr = strstr(iterator_str, event_info);
    if (pstr != NULL) {
      return (atoi(iterator_str + strlen(event_info)));
    }
    iterator_str += strlen(iterator_str) + 1;
  }

  return -1;
}

const char *GetTokenValue(const char *uevent_data, int length, const char *token) {
  const char *iterator_str = uevent_data;
  const char *pstr = NULL;
  while (((iterator_str - uevent_data) <= length) && (*iterator_str)) {
    pstr = strstr(iterator_str, token);
    if (pstr) {
      break;
    }
    iterator_str += strlen(iterator_str) + 1;
  }

  if (pstr)
    pstr = pstr+strlen(token);

  return pstr;
}

android::status_t HWCSession::SetDsiClk(const android::Parcel *input_parcel) {
  int disp_id = input_parcel->readInt32();
  uint64_t clk = UINT64(input_parcel->readInt64());
  if (disp_id < 0) {
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);
  if (!hwc_display_[disp_id]) {
    return -EINVAL;
  }

  return hwc_display_[disp_id]->SetDynamicDSIClock(clk);
}

android::status_t HWCSession::GetDsiClk(const android::Parcel *input_parcel,
                                        android::Parcel *output_parcel) {
  int disp_id = input_parcel->readInt32();
  if (disp_id < 0) {
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);
  if (!hwc_display_[disp_id]) {
    return -EINVAL;
  }

  uint64_t bitrate = 0;
  hwc_display_[disp_id]->GetDynamicDSIClock(&bitrate);
  output_parcel->writeUint64(bitrate);

  return 0;
}

android::status_t HWCSession::GetSupportedDsiClk(const android::Parcel *input_parcel,
                                                 android::Parcel *output_parcel) {
  int disp_id = input_parcel->readInt32();
  if (disp_id < 0) {
    return -EINVAL;
  }

  SCOPE_LOCK(locker_[disp_id]);
  if (!hwc_display_[disp_id]) {
    return -EINVAL;
  }

  std::vector<uint64_t> bit_rates;
  hwc_display_[disp_id]->GetSupportedDSIClock(&bit_rates);
  output_parcel->writeInt32(INT32(bit_rates.size()));
  for (auto &bit_rate : bit_rates) {
    output_parcel->writeUint64(bit_rate);
  }

  return 0;
}

android::status_t HWCSession::SetPanelLuminanceAttributes(const android::Parcel *input_parcel) {
  int disp_id = input_parcel->readInt32();

  // currently doing only for virtual display
  if (disp_id != qdutils::DISPLAY_VIRTUAL) {
    return -EINVAL;
  }

  std::lock_guard<std::mutex> obj(mutex_lum_);
  set_min_lum_ = input_parcel->readFloat();
  set_max_lum_ = input_parcel->readFloat();
  DLOGI("set max_lum %f, min_lum %f", set_max_lum_, set_min_lum_);

  return 0;
}

void HWCSession::UEventHandler(const char *uevent_data, int length) {
  // Drop hotplug uevents until SurfaceFlinger (the client) is connected. The equivalent of hotplug
  // uevent handling will be done once when SurfaceFlinger connects, at RegisterCallback(). Since
  // HandlePluggableDisplays() reads the latest connection states of all displays, no uevent is
  // lost.
  if (client_connected_ && strcasestr(uevent_data, HWC_UEVENT_DRM_EXT_HOTPLUG)) {
    // MST hotplug will not carry connection status/test pattern etc.
    // Pluggable display handler will check all connection status' and take action accordingly.
    const char *str_status = GetTokenValue(uevent_data, length, "status=");
    const char *str_mst = GetTokenValue(uevent_data, length, "MST_HOTPLUG=");
    if (!str_status && !str_mst) {
      return;
    }

    hpd_bpp_ = GetEventValue(uevent_data, length, "bpp=");
    hpd_pattern_ = GetEventValue(uevent_data, length, "pattern=");
    DLOGI("Uevent = %s, status = %s, MST_HOTPLUG = %s, bpp = %d, pattern = %d", uevent_data,
          str_status ? str_status : "NULL", str_mst ? str_mst : "NULL", hpd_bpp_, hpd_pattern_);

    int virtual_display_index =
        GetDisplayIndex(qdutils::DISPLAY_VIRTUAL);

    std::bitset<kSecureMax> secure_sessions = 0;
    hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
    if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
      Locker::ScopeLock lock_a(locker_[active_builtin_disp_id]);
      hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
    }
    if (secure_sessions[kSecureDisplay] ||
        ((virtual_display_index != -1) && (hwc_display_[virtual_display_index]))) {
      // Defer hotplug handling.
      SCOPE_LOCK(pluggable_handler_lock_);
      DLOGI("Marking hotplug pending...");
      hotplug_pending_event_ = kHotPlugEvent;
    } else {
      // Handle hotplug.
      int32_t err = HandlePluggableDisplays(true);
      if (err) {
        DLOGW("Hotplug handling failed. Error %d '%s'. Hotplug handling %s.", err,
              strerror(abs(err)), (hotplug_pending_event_ == kHotPlugEvent) ?
              "deferred" : "dropped");
      }
    }

    if (str_status) {
      bool connected = (strncmp(str_status, "connected", strlen("connected")) == 0);
      DLOGI("Connected = %d", connected);
      // Pass on legacy HDMI hot-plug event.
      qservice_->onHdmiHotplug(INT(connected));
    }
  }
}

int HWCSession::GetVsyncPeriod(int disp) {
  SCOPE_LOCK(locker_[disp]);
  // default value
  int32_t vsync_period = 1000000000l / 60;
  auto attribute = HWC2::Attribute::VsyncPeriod;

  if (hwc_display_[disp]) {
    hwc_display_[disp]->GetDisplayAttribute(0, attribute, &vsync_period);
  }

  return vsync_period;
}

android::status_t HWCSession::GetVisibleDisplayRect(const android::Parcel *input_parcel,
                                                    android::Parcel *output_parcel) {
  int disp_idx = GetDisplayIndex(input_parcel->readInt32());
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_idx);
    return android::BAD_VALUE;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
  if (!hwc_display_[disp_idx]) {
    return android::NO_INIT;
  }

  hwc_rect_t visible_rect = {0, 0, 0, 0};
  int error = hwc_display_[disp_idx]->GetVisibleDisplayRect(&visible_rect);
  if (error < 0) {
    return error;
  }

  output_parcel->writeInt32(visible_rect.left);
  output_parcel->writeInt32(visible_rect.top);
  output_parcel->writeInt32(visible_rect.right);
  output_parcel->writeInt32(visible_rect.bottom);

  return android::NO_ERROR;
}

void HWCSession::Refresh(hwc2_display_t display) {
  SCOPE_LOCK(callbacks_lock_);
  HWC2::Error err = callbacks_.Refresh(display);
  while (err != HWC2::Error::None) {
    callbacks_lock_.Wait();
    err = callbacks_.Refresh(display);
  }
}

void HWCSession::HotPlug(hwc2_display_t display, HWC2::Connection state) {
  SCOPE_LOCK(callbacks_lock_);
  HWC2::Error err = callbacks_.Hotplug(display, state);
  while (err != HWC2::Error::None) {
    callbacks_lock_.Wait();
    err = callbacks_.Hotplug(display, state);
  }
}

int HWCSession::CreatePrimaryDisplay() {
  int status = -EINVAL;
  HWDisplaysInfo hw_displays_info = {};

  if (null_display_mode_) {
    HWDisplayInfo hw_info = {};
    hw_info.display_type = kBuiltIn;
    hw_info.is_connected = 1;
    hw_info.is_primary = 1;
    hw_info.is_wb_ubwc_supported = 0;
    hw_info.display_id = 1;
    hw_displays_info[hw_info.display_id] = hw_info;
  } else {
    DisplayError error = core_intf_->GetDisplaysStatus(&hw_displays_info);
    if (error != kErrorNone) {
      DLOGE("Failed to get connected display list. Error = %d", error);
      return status;
    }
  }

  for (auto &iter : hw_displays_info) {
    auto &info = iter.second;
    if (!info.is_primary) {
      continue;
    }

    // todo (user): If primary display is not connected (e.g. hdmi as primary), a NULL display
    // need to be created. SF expects primary display hotplug during callback registration unlike
    // previous implementation where first hotplug could be notified anytime.
    if (!info.is_connected) {
      DLOGE("Primary display is not connected. Not supported at present.");
      break;
    }

    auto hwc_display = &hwc_display_[HWC_DISPLAY_PRIMARY];
    hwc2_display_t client_id = map_info_primary_.client_id;

    DLOGI("Create primary display type = %d, sdm id = %d, client id = %d", info.display_type,
                                                                    info.display_id, client_id);
    if (info.display_type == kBuiltIn) {
      status = HWCDisplayBuiltIn::Create(core_intf_, &buffer_allocator_, &callbacks_, this,
                                         qservice_, client_id, info.display_id, hwc_display);
    } else if (info.display_type == kPluggable) {
      status = HWCDisplayPluggable::Create(core_intf_, &buffer_allocator_, &callbacks_, this,
                                           qservice_, client_id, info.display_id, 0, 0, false,
                                           hwc_display);
    } else {
      DLOGE("Spurious primary display type = %d", info.display_type);
      break;
    }

    if (!status) {
      is_hdr_display_[UINT32(client_id)] = HasHDRSupport(*hwc_display);
      DLOGI("Primary display created.");
      map_info_primary_.disp_type = info.display_type;
      map_info_primary_.sdm_id = info.display_id;

      CreateDummyDisplay(HWC_DISPLAY_PRIMARY);
      color_mgr_ = HWCColorManager::CreateColorManager(&buffer_allocator_);
      if (!color_mgr_) {
        DLOGW("Failed to load HWCColorManager.");
      }
    } else {
      DLOGE("Primary display creation failed.");
    }

    // Primary display is found, no need to parse more.
    break;
  }

  return status;
}

void HWCSession::CreateDummyDisplay(hwc2_display_t client_id) {
  if (!async_powermode_) {
    return;
  }

  hwc2_display_t dummy_disp_id = map_hwc_display_.find(client_id)->second;
  auto hwc_display_dummy = &hwc_display_[dummy_disp_id];
  HWCDisplayDummy::Create(core_intf_, &buffer_allocator_, &callbacks_, this, qservice_,
                    0, 0, hwc_display_dummy);
  if (!*hwc_display_dummy) {
    DLOGE("Dummy display creation failed for %d display\n", client_id);
  }
}

int HWCSession::HandleBuiltInDisplays() {
  if (null_display_mode_) {
    DLOGW("Skipped BuiltIn display handling in null-display mode");
    return 0;
  }

  HWDisplaysInfo hw_displays_info = {};
  DisplayError error = core_intf_->GetDisplaysStatus(&hw_displays_info);
  if (error != kErrorNone) {
    DLOGE("Failed to get connected display list. Error = %d", error);
    return -EINVAL;
  }

  int status = 0;
  for (auto &iter : hw_displays_info) {
    auto &info = iter.second;

    // Do not recreate primary display.
    if (info.is_primary || info.display_type != kBuiltIn) {
      continue;
    }

    for (auto &map_info : map_info_builtin_) {
      hwc2_display_t client_id = map_info.client_id;

      {
        SCOPE_LOCK(locker_[client_id]);
        // Lock confined to this scope
        if (hwc_display_[client_id]) {
          continue;
        }

        DLOGI("Create builtin display, sdm id = %d, client id = %d", info.display_id, client_id);
        status = HWCDisplayBuiltIn::Create(core_intf_, &buffer_allocator_, &callbacks_, this,
                                           qservice_, client_id, info.display_id,
                                           &hwc_display_[client_id]);
        if (status) {
          DLOGE("Builtin display creation failed.");
          break;
        }
        is_hdr_display_[UINT32(client_id)] = HasHDRSupport(hwc_display_[client_id]);
        DLOGI("Builtin display created: sdm id = %d, client id = %d", info.display_id, client_id);
        map_info.disp_type = info.display_type;
        map_info.sdm_id = info.display_id;
        CreateDummyDisplay(client_id);
      }

      DLOGI("Hotplugging builtin display, sdm id = %d, client id = %d", info.display_id, client_id);
      callbacks_.Hotplug(client_id, HWC2::Connection::Connected);
      break;
    }
  }

  return status;
}

int HWCSession::HandlePluggableDisplays(bool delay_hotplug) {
  SCOPE_LOCK(pluggable_handler_lock_);
  if (null_display_mode_) {
    DLOGW("Skipped pluggable display handling in null-display mode");
    return 0;
  }

  DLOGI("Handling hotplug...");
  HWDisplaysInfo hw_displays_info = {};
  DisplayError error = core_intf_->GetDisplaysStatus(&hw_displays_info);
  if (error != kErrorNone) {
    DLOGE("Failed to get connected display list. Error = %d", error);
    return -EINVAL;
  }

  int status = HandleDisconnectedDisplays(&hw_displays_info);
  if (status) {
    DLOGE("All displays could not be disconnected.");
    return status;
  }

  status = HandleConnectedDisplays(&hw_displays_info, delay_hotplug);
  if (status) {
    switch (status) {
      case -EAGAIN:
      case -ENODEV:
        // Errors like device removal or deferral for which we want to try another hotplug handling.
        hotplug_pending_event_ = kHotPlugEvent;
        status = 0;
        break;
      default:
        // Real errors we want to flag and stop hotplug handling.
        hotplug_pending_event_ = kHotPlugNone;
        DLOGE("All displays could not be connected. Error %d '%s'.", status, strerror(abs(status)));
    }
    DLOGI("Handling hotplug... %s", (kHotPlugNone ==hotplug_pending_event_) ?
          "Stopped." : "Done. Hotplug events pending.");
    return status;
  }

  hotplug_pending_event_ = kHotPlugNone;

  DLOGI("Handling hotplug... Done.");
  return 0;
}

int HWCSession::HandleConnectedDisplays(HWDisplaysInfo *hw_displays_info, bool delay_hotplug) {
  int status = 0;
  std::vector<hwc2_display_t> pending_hotplugs = {};

  for (auto &iter : *hw_displays_info) {
    auto &info = iter.second;

    // Do not recreate primary display or if display is not connected.
    if (info.is_primary || info.display_type != kPluggable || !info.is_connected) {
      continue;
    }

    // Check if we are already using the display.
    auto display_used = std::find_if(map_info_pluggable_.begin(), map_info_pluggable_.end(),
                                     [&](auto &p) {
                                       return (p.sdm_id == info.display_id);
                                     });
    if (display_used != map_info_pluggable_.end()) {
      // Display is already used in a slot.
      continue;
    }

    // Count active pluggable display slots and slots with no commits.
    bool first_commit_pending = false;
    std::for_each(map_info_pluggable_.begin(), map_info_pluggable_.end(),
                   [&](auto &p) {
                     SCOPE_LOCK(locker_[p.client_id]);
                     if (hwc_display_[p.client_id]) {
                       if (!hwc_display_[p.client_id]->IsFirstCommitDone()) {
                         DLOGI("Display commit pending on display %d-1", p.sdm_id);
                         first_commit_pending = true;
                       }
                     }
                   });

    if (!disable_hotplug_bwcheck_ && first_commit_pending) {
      // Hotplug bandwidth check is accomplished by creating and hotplugging a new display after
      // a display commit has happened on previous hotplugged displays. This allows the driver to
      // return updated modes for the new display based on available link bandwidth.
      DLOGI("Pending display commit on one of the displays. Deferring display creation.");
      status = -EAGAIN;
      if (client_connected_) {
        // Trigger a display refresh since we depend on PresentDisplay() to handle pending hotplugs.
        hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
        if (active_builtin_disp_id >= HWCCallbacks::kNumDisplays) {
          active_builtin_disp_id = HWC_DISPLAY_PRIMARY;
        }
        Refresh(active_builtin_disp_id);
      }
      break;
    }

    // find an empty slot to create display.
    for (auto &map_info : map_info_pluggable_) {
      hwc2_display_t client_id = map_info.client_id;

      // Lock confined to this scope
      {
        SCOPE_LOCK(locker_[client_id]);
        auto &hwc_display = hwc_display_[client_id];
        if (hwc_display) {
          // Display slot is already used.
          continue;
        }

        DLOGI("Create pluggable display, sdm id = %d, client id = %d", info.display_id, client_id);

        // Test pattern generation ?
        map_info.test_pattern = (hpd_bpp_ > 0) && (hpd_pattern_ > 0);
        int err = 0;
        if (!map_info.test_pattern) {
          err = HWCDisplayPluggable::Create(core_intf_, &buffer_allocator_,
                                            &callbacks_, this, qservice_, client_id,
                                            info.display_id, 0, 0, false, &hwc_display);
        } else {
          err = HWCDisplayPluggableTest::Create(core_intf_, &buffer_allocator_,
                                                &callbacks_, this, qservice_, client_id,
                                                info.display_id, UINT32(hpd_bpp_),
                                                UINT32(hpd_pattern_), &hwc_display);
        }

        if (err) {
          DLOGW("Pluggable display creation failed/aborted. Error %d '%s'.", err,
                strerror(abs(err)));
          status = err;
          // Attempt creating remaining pluggable displays.
          break;
        }

        is_hdr_display_[UINT32(client_id)] = HasHDRSupport(hwc_display);
        DLOGI("Created pluggable display successfully: sdm id = %d, client id = %d",
              info.display_id, client_id);
        CreateDummyDisplay(client_id);
      }

      map_info.disp_type = info.display_type;
      map_info.sdm_id = info.display_id;

      pending_hotplugs.push_back((hwc2_display_t)client_id);

      // Display is created for this sdm id, move to next connected display.
      break;
    }
  }

  // No display was created.
  if (!pending_hotplugs.size()) {
    return status;
  }

  // Active builtin display needs revalidation
  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
    {
      SEQUENCE_WAIT_SCOPE_LOCK(locker_[active_builtin_disp_id]);
      hwc_display_[active_builtin_disp_id]->ResetValidation();
    }

    if (client_connected_) {
      Refresh(active_builtin_disp_id);
    }

    // Do not sleep if this method is called from client thread.
    if (delay_hotplug) {
      // wait sufficient time to ensure resources are available for new display connection.
      usleep(UINT32(GetVsyncPeriod(INT32(active_builtin_disp_id))) * 2 / 1000);
    }
  }

  for (auto client_id : pending_hotplugs) {
    DLOGI("Notify hotplug display connected: client id = %d", client_id);
    callbacks_.Hotplug(client_id, HWC2::Connection::Connected);
  }

  return status;
}

bool HWCSession::HasHDRSupport(HWCDisplay *hwc_display) {
  // query number of hdr types
  uint32_t out_num_types = 0;
  float out_max_luminance = 0.0f;
  float out_max_average_luminance = 0.0f;
  float out_min_luminance = 0.0f;
  if (hwc_display->GetHdrCapabilities(&out_num_types, nullptr, &out_max_luminance,
                                      &out_max_average_luminance, &out_min_luminance)
                                      != HWC2::Error::None) {
    return false;
  }

  return (out_num_types > 0);
}

int HWCSession::HandleDisconnectedDisplays(HWDisplaysInfo *hw_displays_info) {
  // Destroy pluggable displays which were connected earlier but got disconnected now.
  for (auto &map_info : map_info_pluggable_) {
    bool disconnect = true;   // disconnect in case display id is not found in list.

    for (auto &iter : *hw_displays_info) {
      auto &info = iter.second;
      if (info.display_id != map_info.sdm_id) {
        continue;
      }
      if (info.is_connected) {
        disconnect = false;
      }
      break;
    }

    if (disconnect) {
      DestroyDisplay(&map_info);
    }
  }

  return 0;
}

void HWCSession::DestroyDisplay(DisplayMapInfo *map_info) {
  switch (map_info->disp_type) {
    case kPluggable:
      DestroyPluggableDisplay(map_info);
      break;
    default:
      DestroyNonPluggableDisplay(map_info);
      break;
    }
}

void HWCSession::DestroyPluggableDisplay(DisplayMapInfo *map_info) {
  hwc2_display_t client_id = map_info->client_id;

  DLOGI("Notify hotplug display disconnected: client id = %d", client_id);
  callbacks_.Hotplug(client_id, HWC2::Connection::Disconnected);

  // Trigger refresh to make sure disconnect event received/updated properly by SurfaceFlinger.
  Refresh(HWC_DISPLAY_PRIMARY);

  // wait for sufficient time to ensure sufficient resources are available to process
  // connection.
  usleep(UINT32(GetVsyncPeriod(HWC_DISPLAY_PRIMARY)) * 2 / 1000);

  {
    SCOPE_LOCK(locker_[client_id]);
    auto &hwc_display = hwc_display_[client_id];
    if (!hwc_display) {
      return;
    }
    DLOGI("Destroy display %d-%d, client id = %d", map_info->sdm_id, map_info->disp_type,
         client_id);

    is_hdr_display_[UINT32(client_id)] = false;
    if (!map_info->test_pattern) {
      HWCDisplayPluggable::Destroy(hwc_display);
    } else {
      HWCDisplayPluggableTest::Destroy(hwc_display);
    }

    if (async_powermode_) {
      hwc2_display_t dummy_disp_id = map_hwc_display_.find(client_id)->second;
      auto &hwc_display_dummy = hwc_display_[dummy_disp_id];
      display_ready_.reset(UINT32(dummy_disp_id));
      if (hwc_display_dummy) {
        HWCDisplayDummy::Destroy(hwc_display_dummy);
        hwc_display_dummy = nullptr;
      }
    }
    display_ready_.reset(UINT32(client_id));
    hwc_display = nullptr;
    map_info->Reset();
  }
}

void HWCSession::DestroyNonPluggableDisplay(DisplayMapInfo *map_info) {
  hwc2_display_t client_id = map_info->client_id;

  SCOPE_LOCK(locker_[client_id]);
  auto &hwc_display = hwc_display_[client_id];
  if (!hwc_display) {
    return;
  }
  DLOGI("Destroy display %d-%d, client id = %d", map_info->sdm_id, map_info->disp_type,
        client_id);
  is_hdr_display_[UINT32(client_id)] = false;
  switch (map_info->disp_type) {
    case kBuiltIn:
      HWCDisplayBuiltIn::Destroy(hwc_display);
      break;
    default:
      HWCDisplayVirtual::Destroy(hwc_display);
      break;
    }

    if (async_powermode_ && map_info->disp_type == kBuiltIn) {
      hwc2_display_t dummy_disp_id = map_hwc_display_.find(client_id)->second;
      auto &hwc_display_dummy = hwc_display_[dummy_disp_id];
      display_ready_.reset(UINT32(dummy_disp_id));
      if (hwc_display_dummy) {
        HWCDisplayDummy::Destroy(hwc_display_dummy);
        hwc_display_dummy = nullptr;
      }
    }
    hwc_display = nullptr;
    display_ready_.reset(UINT32(client_id));
    map_info->Reset();
}

HWC2::Error HWCSession::ValidateDisplayInternal(hwc2_display_t display, uint32_t *out_num_types,
                                                uint32_t *out_num_requests) {
  HWCDisplay *hwc_display = hwc_display_[display];

  DTRACE_SCOPED();
  if (hwc_display->IsInternalValidateState()) {
    // Internal Validation has already been done on display, get the Output params.
    return hwc_display->GetValidateDisplayOutput(out_num_types, out_num_requests);
  }

  if (display == HWC_DISPLAY_PRIMARY) {
    // TODO(user): This can be moved to HWCDisplayPrimary
    if (need_invalidate_) {
      Refresh(display);
      need_invalidate_ = false;
    }

    if (color_mgr_) {
      color_mgr_->SetColorModeDetailEnhancer(hwc_display_[display]);
    }
  }

  return hwc_display->Validate(out_num_types, out_num_requests);
}

HWC2::Error HWCSession::PresentDisplayInternal(hwc2_display_t display, int32_t *out_retire_fence) {
  HWCDisplay *hwc_display = hwc_display_[display];

  DTRACE_SCOPED();
  // If display is in Skip-Validate state and Validate cannot be skipped, do Internal
  // Validation to optimize for the frames which don't require the Client composition.
  if (hwc_display->IsSkipValidateState() && !hwc_display->CanSkipValidate()) {
    uint32_t out_num_types = 0, out_num_requests = 0;
    hwc_display->SetFastPathComposition(true);
    HWC2::Error error = ValidateDisplayInternal(display, &out_num_types, &out_num_requests);
    if ((error != HWC2::Error::None) || hwc_display->HWCClientNeedsValidate()) {
      hwc_display->SetValidationState(HWCDisplay::kInternalValidate);
      hwc_display->SetFastPathComposition(false);
      return HWC2::Error::NotValidated;
    }
  }
  return HWC2::Error::None;
}

void HWCSession::DisplayPowerReset() {
  // Acquire lock on all displays.
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
    display < HWCCallbacks::kNumDisplays; display++) {
    locker_[display].Lock();
  }

  HWC2::Error status = HWC2::Error::None;
  HWC2::PowerMode last_power_mode[HWCCallbacks::kNumDisplays] = {};

  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
    display < HWCCallbacks::kNumDisplays; display++) {
    if (hwc_display_[display] != NULL) {
      last_power_mode[display] = hwc_display_[display]->GetCurrentPowerMode();
      DLOGI("Powering off display = %d", display);
      status = hwc_display_[display]->SetPowerMode(HWC2::PowerMode::Off,
                                                   true /* teardown */);
      if (status != HWC2::Error::None) {
        DLOGE("Power off for display = %d failed with error = %d", display, status);
      }
    }
  }
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
    display < HWCCallbacks::kNumDisplays; display++) {
    if (hwc_display_[display] != NULL) {
      HWC2::PowerMode mode = last_power_mode[display];
      DLOGI("Setting display %d to mode = %d", display, mode);
      status = hwc_display_[display]->SetPowerMode(mode, false /* teardown */);
      if (status != HWC2::Error::None) {
        DLOGE("%d mode for display = %d failed with error = %d", mode, display, status);
      }
      ColorMode color_mode = hwc_display_[display]->GetCurrentColorMode();
      RenderIntent intent = hwc_display_[display]->GetCurrentRenderIntent();
      status = hwc_display_[display]->SetColorModeWithRenderIntent(color_mode, intent);
      if (status != HWC2::Error::None) {
        DLOGE("SetColorMode failed for display = %d error = %d", display, status);
      }
    }
  }

  hwc2_display_t vsync_source = callbacks_.GetVsyncSource();
  status = hwc_display_[vsync_source]->SetVsyncEnabled(HWC2::Vsync::Enable);
  if (status != HWC2::Error::None) {
    DLOGE("Enabling vsync failed for disp: %" PRIu64 " with error = %d", vsync_source, status);
  }

  // Release lock on all displays.
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
    display < HWCCallbacks::kNumDisplays; display++) {
    locker_[display].Unlock();
  }

  Refresh(vsync_source);
}

void HWCSession::HandleSecureSession() {
  std::bitset<kSecureMax> secure_sessions = 0;
  {
    hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
    if (active_builtin_disp_id >= HWCCallbacks::kNumDisplays) {
      return;
    }
    Locker::ScopeLock lock_a(locker_[active_builtin_disp_id]);
    hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
  }

  // If it is called during primary prepare/commit, we need to pause any ongoing commit on
  // external/virtual display.
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
    display < HWCCallbacks::kNumDisplays; display++) {
    Locker::ScopeLock lock_d(locker_[display]);
    if (hwc_display_[display]) {
      hwc_display_[display]->HandleSecureSession(secure_sessions, &power_on_pending_[display]);
    }
  }
}

void HWCSession::HandlePowerOnPending(hwc2_display_t disp_id, int retire_fence) {
  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  if (disp_id != active_builtin_disp_id) {
    return;
  }

  Locker::ScopeLock lock_a(locker_[active_builtin_disp_id]);
  bool power_on_pending = false;
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
    display < HWCCallbacks::kNumDisplays; display++) {
    if (display != active_builtin_disp_id) {
      Locker::ScopeLock lock_d(locker_[display]);
      if (power_on_pending_[display]) {
        power_on_pending = true;
        break;
      }
    }
  }
  if (power_on_pending) {
    // retire fence is set only after successful primary commit, So check for retire fence to know
    // non secure commit went through to notify driver to change the CRTC mode to non secure.
    // Otherwise any commit to non-primary display would fail.
    if (retire_fence < 0) {
      return;
    }
    int error = sync_wait(retire_fence, 1000);
    if (error < 0) {
      DLOGE("sync_wait error errno = %d, desc = %s", errno, strerror(errno));
    }
  } else {
    return;
  }

  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
    display < HWCCallbacks::kNumDisplays; display++) {
    if (display != active_builtin_disp_id) {
      Locker::ScopeLock lock_d(locker_[display]);
      if (power_on_pending_[display] && hwc_display_[display]) {
        HWC2::Error status =
          hwc_display_[display]->SetPowerMode(HWC2::PowerMode::On, false /* teardown */);
        if (status == HWC2::Error::None) {
          power_on_pending_[display] = false;
        }
      }
    }
  }
}

void HWCSession::HandleHotplugPending(hwc2_display_t disp_id, int retire_fence) {
  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  if (disp_id != active_builtin_disp_id ||
      (kHotPlugNone == hotplug_pending_event_ && !destroy_virtual_disp_pending_)) {
    return;
  }

  std :: bitset < kSecureMax > secure_sessions = 0;
  if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
    Locker::ScopeLock lock_a(locker_[active_builtin_disp_id]);
    hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
  }

  if (secure_sessions.any() || active_builtin_disp_id >= HWCCallbacks::kNumDisplays) {
    return;
  }

  if (destroy_virtual_disp_pending_ || kHotPlugEvent == hotplug_pending_event_) {
    if (retire_fence >= 0) {
      int error = sync_wait(retire_fence, 1000);
      if (error < 0) {
        DLOGE("sync_wait error errno = %d, desc = %s", errno,  strerror(errno));
      }
    }
    // Destroy the pending virtual display if secure session not present.
    if (destroy_virtual_disp_pending_) {
      for (auto &map_info : map_info_virtual_) {
        DestroyDisplay(&map_info);
        destroy_virtual_disp_pending_ = false;
      }
    }
    // Handle connect/disconnect hotplugs if secure session is not present.
    int virtual_display_idx = GetDisplayIndex(qdutils::DISPLAY_VIRTUAL);
    if (!(virtual_display_idx != -1 && hwc_display_[virtual_display_idx]) &&
        kHotPlugEvent == hotplug_pending_event_) {
      // Handle deferred hotplug event.
      int32_t err = pluggable_handler_lock_.TryLock();
      if (!err) {
        // Do hotplug handling in a different thread to avoid blocking PresentDisplay.
        std::thread(&HWCSession::HandlePluggableDisplays, this, true).detach();
        pluggable_handler_lock_.Unlock();
      } else {
        // EBUSY means another thread is already handling hotplug. Skip deferred hotplug handling.
        if (EBUSY != err) {
          DLOGW("Failed to acquire pluggable display handler lock. Error %d '%s'.", err,
                strerror(abs(err)));
        }
      }
    }
  }
}

int32_t HWCSession::GetReadbackBufferAttributes(hwc2_device_t *device, hwc2_display_t display,
                                                int32_t *format, int32_t *dataspace) {
  if (!device || !format || !dataspace) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display != HWC_DISPLAY_PRIMARY) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  HWCSession *hwc_session = static_cast<HWCSession *>(device);
  HWCDisplay *hwc_display = hwc_session->hwc_display_[display];

  if (hwc_display) {
    if (!hwc_display->IsDisplayCommandMode()) {
      return HWC2_ERROR_UNSUPPORTED;
    }
    *format = HAL_PIXEL_FORMAT_RGB_888;
    *dataspace = GetDataspaceFromColorMode(hwc_display->GetCurrentColorMode());
    return HWC2_ERROR_NONE;
  }

  return HWC2_ERROR_BAD_DISPLAY;
}

int32_t HWCSession::SetReadbackBuffer(hwc2_device_t *device, hwc2_display_t display,
                                      const native_handle_t *buffer, int32_t acquire_fence) {
  if (!buffer) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display != HWC_DISPLAY_PRIMARY) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  HWCSession *hwc_session = static_cast<HWCSession *>(device);

  int external_display_index = hwc_session->GetDisplayIndex(qdutils::DISPLAY_EXTERNAL);
  if ((external_display_index >=0) && (hwc_session->hwc_display_[external_display_index])) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  int virtual_display_index = hwc_session->GetDisplayIndex(qdutils::DISPLAY_VIRTUAL);
  if ((virtual_display_index >=0) && (hwc_session->hwc_display_[virtual_display_index])) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  return CallDisplayFunction(device, display, &HWCDisplay::SetReadbackBuffer,
                             buffer, acquire_fence, false);
}

int32_t HWCSession::GetReadbackBufferFence(hwc2_device_t *device, hwc2_display_t display,
                                           int32_t *release_fence) {
  if (!release_fence) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display != HWC_DISPLAY_PRIMARY) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  return CallDisplayFunction(device, display, &HWCDisplay::GetReadbackBufferFence, release_fence);
}

int32_t HWCSession::GetDisplayIdentificationData(hwc2_device_t *device, hwc2_display_t display,
                                                 uint8_t *outPort, uint32_t *outDataSize,
                                                 uint8_t *outData) {
  if (!outPort || !outDataSize) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  return CallDisplayFunction(device, display, &HWCDisplay::GetDisplayIdentificationData, outPort,
                             outDataSize, outData);
}

int32_t HWCSession::GetDisplayCapabilities(hwc2_device_t *device, hwc2_display_t display,
                                           uint32_t *outNumCapabilities,
                                           uint32_t *outCapabilities) {
  if (!outNumCapabilities || !device) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  HWCSession *hwc_session = static_cast<HWCSession *>(device);
  HWCDisplay *hwc_display = hwc_session->hwc_display_[display];
  if (!hwc_display) {
    DLOGE("Expected valid hwc_display");
    return HWC2_ERROR_BAD_PARAMETER;
  }
  bool isBuiltin = (hwc_display->GetDisplayClass() == DISPLAY_CLASS_BUILTIN);
  if (!outCapabilities) {
    *outNumCapabilities = 0;
    if (isBuiltin) {
      *outNumCapabilities = 2;
    }
    return HWC2_ERROR_NONE;
  } else {
    if (isBuiltin) {
      // TODO(user): Handle SKIP_CLIENT_COLOR_TRANSFORM based on DSPP availability
      outCapabilities[0] = HWC2_DISPLAY_CAPABILITY_SKIP_CLIENT_COLOR_TRANSFORM;
      outCapabilities[1] = HWC2_DISPLAY_CAPABILITY_DOZE;
      *outNumCapabilities = 2;
    }
    return HWC2_ERROR_NONE;
  }
}

int32_t HWCSession::GetDisplayBrightnessSupport(hwc2_device_t *device, hwc2_display_t display,
                                                bool *outSupport) {
  if (!device || !outSupport) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  HWCSession *hwc_session = static_cast<HWCSession *>(device);
  HWCDisplay *hwc_display = hwc_session->hwc_display_[display];
  if (!hwc_display) {
    DLOGE("Expected valid hwc_display");
    return HWC2_ERROR_BAD_PARAMETER;
  }
  // This function isn't actually used in the framework
  // The capability is used instead
  *outSupport = false;
  return HWC2_ERROR_NONE;
}

android::status_t HWCSession::SetQSyncMode(const android::Parcel *input_parcel) {
  auto mode = input_parcel->readInt32();
  auto device = static_cast<hwc2_device_t *>(this);

  QSyncMode qsync_mode = kQSyncModeNone;
  switch (mode) {
    case qService::IQService::QSYNC_MODE_NONE:
      qsync_mode = kQSyncModeNone;
      break;
    case qService::IQService::QSYNC_MODE_CONTINUOUS:
      qsync_mode = kQSyncModeContinuous;
      break;
    case qService::IQService::QSYNC_MODE_ONESHOT:
      qsync_mode = kQsyncModeOneShot;
      break;
    default:
      DLOGE("Qsync mode not supported %d", mode);
      return -EINVAL;
  }
  return CallDisplayFunction(device, HWC_DISPLAY_PRIMARY, &HWCDisplay::SetQSyncMode, qsync_mode);
}

void HWCSession::UpdateThrottlingRate() {
  uint32_t new_min = 0;

  for (int i=0; i < HWCCallbacks::kNumDisplays; i++) {
    auto &display = hwc_display_[i];
    if (!display)
      continue;
    if (display->GetCurrentPowerMode() != HWC2::PowerMode::Off)
      new_min = (new_min == 0) ? display->GetMaxRefreshRate() :
        std::min(new_min, display->GetMaxRefreshRate());
  }

  SetNewThrottlingRate(new_min);
}

void HWCSession::SetNewThrottlingRate(const uint32_t new_rate) {
  if (new_rate !=0 && throttling_refresh_rate_ != new_rate) {
    HWCDisplay::SetThrottlingRefreshRate(new_rate);
    throttling_refresh_rate_ = new_rate;
  }
}

android::status_t HWCSession::SetIdlePC(const android::Parcel *input_parcel) {
  auto enable = input_parcel->readInt32();
  auto synchronous = input_parcel->readInt32();

  return static_cast<android::status_t>(controlIdlePowerCollapse(enable, synchronous));
}

hwc2_display_t HWCSession::GetActiveBuiltinDisplay() {
  hwc2_display_t disp_id = HWCCallbacks::kNumDisplays;
  // Get first active display among primary and built-in displays.
  std::vector<DisplayMapInfo> map_info = {map_info_primary_};
  std::copy(map_info_builtin_.begin(), map_info_builtin_.end(), std::back_inserter(map_info));

  for (auto &info : map_info) {
    SCOPE_LOCK(locker_[info.client_id]);
    auto &hwc_display = hwc_display_[info.client_id];
    if (hwc_display && hwc_display->GetCurrentPowerMode() != HWC2::PowerMode::Off) {
      disp_id = info.client_id;
      break;
    }
  }

  return disp_id;
}

void HWCSession::NotifyClientStatus(bool connected) {
  for (uint32_t i = 0; i < HWCCallbacks::kNumDisplays; i++) {
    if (!hwc_display_[i]) {
      continue;
    }
    SCOPE_LOCK(locker_[i]);
    hwc_display_[i]->NotifyClientStatus(connected);
  }
}

}  // namespace sdm
