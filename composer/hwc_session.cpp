/*
 * Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
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

/*
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
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
#include "ipc_impl.h"

#define __CLASS__ "HWCSession"

#define HWC_UEVENT_SWITCH_HDMI "change@/devices/virtual/switch/hdmi"
#define HWC_UEVENT_DRM_EXT_HOTPLUG "mdss_mdp/drm/card"

using HwcAttribute = composer_V2_4::IComposerClient::Attribute;

namespace sdm {

static HWCUEvent g_hwc_uevent_;
Locker HWCSession::locker_[HWCCallbacks::kNumDisplays];
bool HWCSession::pending_power_mode_[HWCCallbacks::kNumDisplays];
Locker HWCSession::hdr_locker_[HWCCallbacks::kNumDisplays];
std::bitset<HWCSession::kClientMax>
    HWCSession::clients_waiting_for_commit_[HWCCallbacks::kNumDisplays];
shared_ptr<Fence> HWCSession::retire_fence_[HWCCallbacks::kNumDisplays];
int HWCSession::commit_error_[HWCCallbacks::kNumDisplays] = { 0 };
Locker HWCSession::display_config_locker_;
std::mutex HWCSession::command_seq_mutex_;
static const int kSolidFillDelay = 100 * 1000;
static const uint32_t kBrightnessScaleMax = 100;
static const uint32_t kSvBlScaleMax = 65535;
Locker HWCSession::vm_release_locker_[HWCCallbacks::kNumDisplays];
std::bitset<HWCCallbacks::kNumDisplays> HWCSession::clients_waiting_for_vm_release_;
std::set<hwc2_display_t> HWCSession::active_displays_;


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
    case ColorMode::DISPLAY_BT2020:
      return HAL_DATASPACE_DISPLAY_BT2020;
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

HWCSession::HWCSession() : cwb_(this) {}

HWCSession *HWCSession::GetInstance() {
  // executed only once for the very first call.
  // GetInstance called multiple times from Composer and ComposerClient
  static HWCSession *hwc_session = ::new HWCSession();
  return hwc_session;
}

int HWCSession::Init() {
  SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);
  DLOGI("Initializing HWCSession");

  int status = -EINVAL;
  const char *qservice_name = "display.qservice";

  if (!g_hwc_uevent_.InitDone()) {
    DLOGE("HWCUEvent initialization is not done!");
    return status;
  } else {
    DLOGI("HWCUEvent initialization confirmed to be completed");
  }


  // Start QService and connect to it.
  DLOGI("Initializing QService");
  qService::QService::init();
  DLOGI("Initializing QService...done!");

  DLOGI("Getting IQService");
  android::sp<qService::IQService> iqservice = android::interface_cast<qService::IQService>(
      android::defaultServiceManager()->getService(android::String16(qservice_name)));
  DLOGI("Getting IQService...done!");

  if (iqservice.get()) {
    iqservice->connect(android::sp<qClient::IQClient>(this));
    qservice_ = reinterpret_cast<qService::QService *>(iqservice.get());
    DLOGI("Acquired %s", qservice_name);
  } else {
    DLOGE("Failed to acquire %s", qservice_name);
    return -EINVAL;
  }

  int value = 0;  // Default value when property is not present.
  HWCDebugHandler::Get()->GetProperty(ENABLE_VERBOSE_LOG, &value);
  if (value == 1) {
    HWCDebugHandler::DebugAll(value, value);
  }

  HWCDebugHandler::Get()->GetProperty(DISABLE_HOTPLUG_BWCHECK, &disable_hotplug_bwcheck_);
  DLOGI("disable_hotplug_bwcheck_: %d", disable_hotplug_bwcheck_);
  HWCDebugHandler::Get()->GetProperty(DISABLE_MASK_LAYER_HINT, &disable_mask_layer_hint_);
  DLOGI("disable_mask_layer_hint_: %d", disable_mask_layer_hint_);
  HWCDebugHandler::Get()->GetProperty(ENABLE_PRIMARY_RECONFIG_REQUEST,
                                      &enable_primary_reconfig_req_);
  DLOGI("enable_primary_reconfig_req_: %d", enable_primary_reconfig_req_);

  g_hwc_uevent_.Register(this);
  DLOGI("Registered HWCSession as the HWCUEvent handler");

  value = 0;
  Debug::Get()->GetProperty(ENABLE_ASYNC_VDS_CREATION, &value);
  async_vds_creation_ = (value == 1);
  DLOGI("async_vds_creation: %d", async_vds_creation_);

  value = 0;
  Debug::Get()->GetProperty(DISABLE_NON_WFD_VDS, &value);
  disable_non_wfd_vds_ = (value == 1);
  DLOGI("disable_non_wfd_vds: %d", disable_non_wfd_vds_);

  char prop_str[64] = {};
  Debug::Get()->GetProperty(ENABLE_HWC_VDS, prop_str);
  debug_enable_hwc_vds_ = (strcmp(prop_str, "true") == 0);
  DLOGI("debug_enable_hwc_vds: %d", debug_enable_hwc_vds_);

  DLOGI("Initializing supported display slots");
  InitSupportedDisplaySlots();
  DLOGI("Initializing supported display slots...done!");

  // Create primary display here. Remaining builtin displays will be created after client has set
  // display indexes which may happen sometime before callback is registered.
  DLOGI("Creating the Primary display");
  status = CreatePrimaryDisplay();
  if (status) {
    DLOGE("Creating the Primary display...failed!");
    Deinit();
    return status;
  } else {
    DLOGI("Creating the Primary display...done!");
  }

  is_composer_up_ = true;
  StartServices();

  PostInit();

  DLOGI("Initializing HWCSession...done!");
  return 0;
}

void HWCSession::PostInit() {
  // Start services which need IDisplayConfig to be up.
  // This avoids deadlock between composer and its clients.
  auto hwc_display = hwc_display_[HWC_DISPLAY_PRIMARY];
  hwc_display->PostInit();
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

  g_hwc_uevent_.Register(nullptr);

  DisplayError error = CoreInterface::DestroyCore();
  if (error != kErrorNone) {
    DLOGE("Display core de-initialization failed. Error = %d", error);
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

  ipc_intf_ = std::make_shared<IPCImpl>(IPCImpl());
  ipc_intf_->Init();

  DisplayError error = CoreInterface::CreateCore(&buffer_allocator_, nullptr,
                                                 &socket_handler_, ipc_intf_, &core_intf_);
  if (error != kErrorNone) {
    DLOGE("Failed to create CoreInterface");
    return;
  }

  HWDisplayInterfaceInfo hw_disp_info = {};
  error = WaitForPrimaryHotplug(&hw_disp_info);
  if (error != kErrorNone) {
    CoreInterface::DestroyCore();
    DLOGE("Primary display is not plugged. Error = %d", error);
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

  if (max_virtual == 0) {
    // Check if WB using GPU is supported.
    max_virtual += virtual_display_factory_.IsGPUColorConvertSupported() ? 1 : 0;
  }

  if (kPluggable == hw_disp_info.type) {
    // If primary is a pluggable display, we have already used one pluggable display interface.
    max_pluggable--;
  } else {
    max_builtin--;
  }

  // Init slots in accordance to h/w capability.
  uint32_t disp_count = UINT32(std::min(max_pluggable, HWCCallbacks::kNumPluggable));
  hwc2_display_t base_id = qdutils::DISPLAY_EXTERNAL;
  // if external displays are ZERO, then initialize it to 2 as 1 is reserved for external.
  if (!disp_count) {
    base_id = qdutils::DISPLAY_BUILTIN_2 - 1;
  }

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
    DLOGW("Display index not found for display %d.", dpy);
    return -1;
  }

  return INT(map_info->client_id);
}

void HWCSession::GetCapabilities(uint32_t *outCount, int32_t *outCapabilities) {
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

int32_t HWCSession::AcceptDisplayChanges(hwc2_display_t display) {
  return CallDisplayFunction(display, &HWCDisplay::AcceptDisplayChanges);
}

int32_t HWCSession::CreateLayer(hwc2_display_t display,
                                hwc2_layer_t *out_layer_id) {
  if (!out_layer_id) {
    return  HWC2_ERROR_BAD_PARAMETER;
  }

  return CallDisplayFunction(display, &HWCDisplay::CreateLayer, out_layer_id);
}

int32_t HWCSession::CreateVirtualDisplay(uint32_t width, uint32_t height, int32_t *format,
                                         hwc2_display_t *out_display_id) {
  // TODO(user): Handle concurrency with HDMI

  if (!out_display_id || !width || !height || !format) {
    return  HWC2_ERROR_BAD_PARAMETER;
  }

  if (async_vds_creation_ && virtual_id_ != HWCCallbacks::kNumDisplays) {
    *out_display_id = virtual_id_;
    return HWC2_ERROR_NONE;
  }

  auto status = CreateVirtualDisplayObj(width, height, format, out_display_id);
  if (status == HWC2::Error::None) {
    DLOGI("Created virtual display id:%" PRIu64 ", res: %dx%d", *out_display_id, width, height);
  } else {
    DLOGE("Failed to create virtual display: %s", to_string(status).c_str());
  }
  return INT32(status);
}

int32_t HWCSession::DestroyLayer(hwc2_display_t display, hwc2_layer_t layer) {
  return CallDisplayFunction(display, &HWCDisplay::DestroyLayer, layer);
}

int32_t HWCSession::DestroyVirtualDisplay(hwc2_display_t display) {
  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  for (auto &map_info : map_info_virtual_) {
    if (map_info.client_id == display) {
      DLOGI("Destroying virtual display id:%" PRIu64, display);
      DestroyDisplay(&map_info);
      break;
    }
  }
  virtual_id_ = HWCCallbacks::kNumDisplays;

  return HWC2_ERROR_NONE;
}

int32_t HWCSession::GetVirtualDisplayId() {
  HWDisplaysInfo hw_displays_info = {};
  core_intf_->GetDisplaysStatus(&hw_displays_info);
  for (auto &iter : hw_displays_info) {
    auto &info = iter.second;
    if (info.display_type != kVirtual) {
      continue;
    }

    return info.display_id;
  }

  return -1;
}

void HWCSession::Dump(uint32_t *out_size, char *out_buffer) {
  if (!out_size) {
    return;
  }

  const size_t max_dump_size = 16384;  // 16 kB

  if (out_buffer == nullptr) {
    *out_size = max_dump_size;
  } else {
    std::ostringstream os;
    for (int id = 0; id < HWCCallbacks::kNumRealDisplays; id++) {
      SCOPE_LOCK(locker_[id]);
      if (hwc_display_[id]) {
        hwc_display_[id]->Dump(&os);
      }
    }
    Fence::Dump(&os);

    std::string s = os.str();
    auto copied = s.copy(out_buffer, std::min(s.size(), max_dump_size), 0);
    *out_size = UINT32(copied);
  }
}

uint32_t HWCSession::GetMaxVirtualDisplayCount() {
  if (!disable_non_wfd_vds_) {
    return map_info_virtual_.size();
  }

  return is_client_up_ && !debug_enable_hwc_vds_ ? map_info_virtual_.size() : 0;
}

int32_t HWCSession::GetActiveConfig(hwc2_display_t display, hwc2_config_t *out_config) {
  return CallDisplayFunction(display, &HWCDisplay::GetActiveConfig, out_config);
}

int32_t HWCSession::GetChangedCompositionTypes(hwc2_display_t display, uint32_t *out_num_elements,
                                               hwc2_layer_t *out_layers, int32_t *out_types) {
  // null_ptr check only for out_num_elements, as out_layers and out_types can be null.
  if (!out_num_elements) {
    return  HWC2_ERROR_BAD_PARAMETER;
  }
  return CallDisplayFunction(display, &HWCDisplay::GetChangedCompositionTypes, out_num_elements,
                             out_layers, out_types);
}

int32_t HWCSession::GetClientTargetSupport(hwc2_display_t display, uint32_t width, uint32_t height,
                                           int32_t format, int32_t dataspace) {
  return CallDisplayFunction(display, &HWCDisplay::GetClientTargetSupport, width, height, format,
                             dataspace);
}

int32_t HWCSession::GetColorModes(hwc2_display_t display, uint32_t *out_num_modes,
                                  int32_t /*ColorMode*/ *int_out_modes) {
  auto out_modes = reinterpret_cast<ColorMode *>(int_out_modes);
  if (out_num_modes == nullptr) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  return CallDisplayFunction(display, &HWCDisplay::GetColorModes, out_num_modes, out_modes);
}

int32_t HWCSession::GetRenderIntents(hwc2_display_t display, int32_t /*ColorMode*/ int_mode,
                                     uint32_t *out_num_intents,
                                     int32_t /*RenderIntent*/ *int_out_intents) {
  auto mode = static_cast<ColorMode>(int_mode);
  auto out_intents = reinterpret_cast<RenderIntent *>(int_out_intents);
  if (out_num_intents == nullptr) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (mode < ColorMode::NATIVE || mode > ColorMode::DISPLAY_BT2020) {
    DLOGE("Invalid ColorMode: %d", mode);
    return HWC2_ERROR_BAD_PARAMETER;
  }
  return CallDisplayFunction(display, &HWCDisplay::GetRenderIntents, mode, out_num_intents,
                             out_intents);
}

int32_t HWCSession::GetDataspaceSaturationMatrix(int32_t /*Dataspace*/ int_dataspace,
                                                 float *out_matrix) {
  auto dataspace = static_cast<Dataspace>(int_dataspace);
  if (out_matrix == nullptr || dataspace != Dataspace::SRGB_LINEAR) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  // We only have the matrix for sRGB
  float saturation_matrix[kDataspaceSaturationMatrixCount] = { 1.0, 0.0, 0.0, 0.0, \
                                                               0.0, 1.0, 0.0, 0.0, \
                                                               0.0, 0.0, 1.0, 0.0, \
                                                               0.0, 0.0, 0.0, 1.0 };

  for (int32_t i = 0; i < kDataspaceSaturationMatrixCount; i += 4) {
    DLOGD("%f %f %f %f", saturation_matrix[i], saturation_matrix[i + 1], saturation_matrix[i + 2],
          saturation_matrix[i + 3]);
  }
  for (uint32_t i = 0; i < kDataspaceSaturationMatrixCount; i++) {
    out_matrix[i] = saturation_matrix[i];
  }
  return HWC2_ERROR_NONE;
}

int32_t HWCSession::GetPerFrameMetadataKeys(hwc2_display_t display, uint32_t *out_num_keys,
                                            int32_t *int_out_keys) {
  auto out_keys = reinterpret_cast<PerFrameMetadataKey *>(int_out_keys);
  return CallDisplayFunction(display, &HWCDisplay::GetPerFrameMetadataKeys, out_num_keys,
                             out_keys);
}

int32_t HWCSession::SetLayerPerFrameMetadata(hwc2_display_t display, hwc2_layer_t layer,
                                             uint32_t num_elements, const int32_t *int_keys,
                                             const float *metadata) {
  auto keys = reinterpret_cast<const PerFrameMetadataKey *>(int_keys);
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerPerFrameMetadata, num_elements,
                           keys, metadata);
}

int32_t HWCSession:: SetLayerPerFrameMetadataBlobs(hwc2_display_t display,
                                                   hwc2_layer_t layer, uint32_t num_elements,
                                                   const int32_t *int_keys, const uint32_t *sizes,
                                                   const uint8_t *metadata) {
  auto keys = reinterpret_cast<const PerFrameMetadataKey *>(int_keys);
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerPerFrameMetadataBlobs,
                           num_elements, keys, sizes, metadata);
}

int32_t HWCSession::SetDisplayedContentSamplingEnabled(hwc2_display_t display, int32_t enabled,
                                                       uint8_t component_mask,
                                                       uint64_t max_frames) {
  static constexpr int32_t validComponentMask = HWC2_FORMAT_COMPONENT_0 | HWC2_FORMAT_COMPONENT_1 |
                                                HWC2_FORMAT_COMPONENT_2 | HWC2_FORMAT_COMPONENT_3;
  if (component_mask & ~validComponentMask)
    return HWC2_ERROR_BAD_PARAMETER;
  return CallDisplayFunction(display, &HWCDisplay::SetDisplayedContentSamplingEnabled, enabled,
                             component_mask, max_frames);
}

int32_t HWCSession::GetDisplayedContentSamplingAttributes(hwc2_display_t display, int32_t *format,
                                                          int32_t *dataspace,
                                                          uint8_t *supported_components) {
  return CallDisplayFunction(display, &HWCDisplay::GetDisplayedContentSamplingAttributes, format,
                             dataspace, supported_components);
}

int32_t HWCSession::GetDisplayedContentSample(hwc2_display_t display, uint64_t max_frames,
                                              uint64_t timestamp, uint64_t *numFrames,
                                              int32_t samples_size[NUM_HISTOGRAM_COLOR_COMPONENTS],
                                              uint64_t *samples[NUM_HISTOGRAM_COLOR_COMPONENTS]) {
  return CallDisplayFunction(display, &HWCDisplay::GetDisplayedContentSample, max_frames, timestamp,
                             numFrames, samples_size, samples);
}

int32_t HWCSession::GetDisplayAttribute(hwc2_display_t display, hwc2_config_t config,
                                        HwcAttribute attribute, int32_t *out_value) {
  if (out_value == nullptr) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  return CallDisplayFunction(display, &HWCDisplay::GetDisplayAttribute, config, attribute,
                             out_value);
}

int32_t HWCSession::GetDisplayConfigs(hwc2_display_t display, uint32_t *out_num_configs,
                                      hwc2_config_t *out_configs) {
  return CallDisplayFunction(display, &HWCDisplay::GetDisplayConfigs, out_num_configs,
                             out_configs);
}

int32_t HWCSession::GetDisplayName(hwc2_display_t display, uint32_t *out_size, char *out_name) {
  return CallDisplayFunction(display, &HWCDisplay::GetDisplayName, out_size, out_name);
}

int32_t HWCSession::GetDisplayRequests(hwc2_display_t display, int32_t *out_display_requests,
                                       uint32_t *out_num_elements, hwc2_layer_t *out_layers,
                                       int32_t *out_layer_requests) {
  return CallDisplayFunction(display, &HWCDisplay::GetDisplayRequests, out_display_requests,
                             out_num_elements, out_layers, out_layer_requests);
}

int32_t HWCSession::GetDisplayType(hwc2_display_t display, int32_t *out_type) {
  return CallDisplayFunction(display, &HWCDisplay::GetDisplayType, out_type);
}


int32_t HWCSession::GetHdrCapabilities(hwc2_display_t display, uint32_t* out_num_types,
                                       int32_t* out_types, float* out_max_luminance,
                                       float* out_max_average_luminance,
                                       float* out_min_luminance) {
  return CallDisplayFunction(display, &HWCDisplay::GetHdrCapabilities, out_num_types, out_types,
                             out_max_luminance, out_max_average_luminance, out_min_luminance);
}


int32_t HWCSession::GetReleaseFences(hwc2_display_t display, uint32_t *out_num_elements,
                                     hwc2_layer_t *out_layers,
                                     std::vector<shared_ptr<Fence>> *out_fences) {
  return CallDisplayFunction(display, &HWCDisplay::GetReleaseFences, out_num_elements, out_layers,
                             out_fences);
}

void HWCSession::PerformQsyncCallback(hwc2_display_t display, bool qsync_enabled,
                                      uint32_t refresh_rate, uint32_t qsync_refresh_rate) {
  // AIDL callback
  if (!callback_clients_.empty()) {
    std::lock_guard<decltype(callbacks_lock_)> lock_guard(callbacks_lock_);
    for (auto const& [id, callback] : callback_clients_) {
      if (callback) {
        callback->notifyQsyncChange(qsync_enabled, refresh_rate, qsync_refresh_rate);
      }
    }
  }

  // HIDL callback
  std::shared_ptr<DisplayConfig::ConfigCallback> callback = qsync_callback_.lock();
  if (!callback) {
    return;
  }

  callback->NotifyQsyncChange(qsync_enabled, refresh_rate, qsync_refresh_rate);
}


void HWCSession::NotifyConcurrencyFps(const float fps, DisplayConcurrencyType type,
                                      bool concurrency_begin) {
  DisplayConfigVariableInfo var_info;
  Attributes attributes;
  hwc2_display_t target_display = GetActiveBuiltinDisplay();
  hwc2_config_t current_config = 0;

  if (!is_client_up_ || !async_vds_creation_requested_) {
    return;
  }

  if(target_display >= HWCCallbacks::kNumDisplays) {
    return;
  }

  hwc_display_[target_display]->GetActiveConfig(&current_config);
  int error = hwc_display_[target_display]->GetDisplayAttributesForConfig(INT(current_config),
                                                                          &var_info);
  if(error) {
    return;
  }

  Concurrency concurrency;
  attributes.vsyncPeriod = (1 / fps) * 1000000;
  attributes.xRes = var_info.x_pixels;
  attributes.yRes = var_info.y_pixels;
  attributes.xDpi = var_info.x_dpi;
  attributes.yDpi = var_info.y_dpi;
  attributes.panelType = DisplayPortType::DEFAULT;
  attributes.isYuv = var_info.is_yuv;
  switch (type) {
    case DisplayConcurrencyType::kConcurrencyWfd:
      concurrency = Concurrency::WFD;
    break;
      case DisplayConcurrencyType::kConcurrencyDp:
      concurrency = Concurrency::DP;
    break;
      case DisplayConcurrencyType::kConcurrencyCamera:
      concurrency = Concurrency::CAMERA;
    break;
      case DisplayConcurrencyType::kConcurrencyCwb:
      concurrency = Concurrency::CWB;
    break;
    case DisplayConcurrencyType::kConcurrencyNone:
    case DisplayConcurrencyType::kConcurrencyMax:
      return;
  }

  if(concurrency_begin) {
    NotifyFpsMitigation(target_display, attributes, concurrency);
    int ret = WaitForCommitDone(target_display, kClientConcurrency);
    if (ret != 0) {
      DLOGE("WaitForCommitDone failed with error %d for %d", ret, kClientConcurrency);
    }
    return;
  }

  NotifyFpsMitigation(target_display, attributes, concurrency);
}

void HWCSession::PerformIdleStatusCallback(hwc2_display_t display) {
  std::shared_ptr<DisplayConfig::ConfigCallback> callback = idle_callback_.lock();
  if (!callback) {
    return;
  }

  if (hwc_display_[display]->IsDisplayIdle()) {
    DTRACE_SCOPED();
    callback->NotifyIdleStatus(true);
  }
}

int32_t HWCSession::PresentDisplay(hwc2_display_t display, shared_ptr<Fence> *out_retire_fence) {
  auto status = HWC2::Error::BadDisplay;
  DTRACE_SCOPED();

  if (display >= HWCCallbacks::kNumDisplays) {
    DLOGW("Invalid Display : display = %" PRIu64, display);
    return HWC2_ERROR_BAD_DISPLAY;
  }

  HandleSecureSession();

  {
    SEQUENCE_EXIT_SCOPE_LOCK(locker_[display]);
    if (!hwc_display_[display]) {
      DLOGW("Removed Display : display = %" PRIu64, display);
      return HWC2_ERROR_BAD_DISPLAY;
    }

    if (out_retire_fence == nullptr) {
      return HWC2_ERROR_BAD_PARAMETER;
    }

    if (pending_power_mode_[display]) {
      status = HWC2::Error::None;
    } else {
      hwc_display_[display]->ProcessActiveConfigChange();
      status = hwc_display_[display]->Present(out_retire_fence);
      if (status == HWC2::Error::None) {
        PostCommitLocked(display, *out_retire_fence);
      }
    }
  }

  if (status != HWC2::Error::None && status != HWC2::Error::NotValidated) {
    if (clients_waiting_for_commit_[display].any()) {
      retire_fence_[display] = nullptr;
      commit_error_[display] = -EINVAL;
      clients_waiting_for_commit_[display].reset();
    }
    SEQUENCE_CANCEL_SCOPE_LOCK(locker_[display]);
  }

  PostCommitUnlocked(display, *out_retire_fence, status);

  return INT32(status);
}

void HWCSession::PostCommitLocked(hwc2_display_t display, shared_ptr<Fence> &retire_fence) {
  PerformIdleStatusCallback(display);

  if (clients_waiting_for_commit_[display].any()) {
    retire_fence_[display] = retire_fence;
    commit_error_[display] = 0;
    clients_waiting_for_commit_[display].reset();
  }
}

void HWCSession::PostCommitUnlocked(hwc2_display_t display, const shared_ptr<Fence> &retire_fence,
                                    HWC2::Error status) {
  HandlePendingPowerMode(display, retire_fence);
  HandlePendingHotplug(display, retire_fence);
  HandlePendingRefresh();
  if (status != HWC2::Error::NotValidated) {
    cwb_.PresentDisplayDone(display);
  }
  display_ready_.set(UINT32(display));
  std::unique_lock<std::mutex> caller_lock(hotplug_mutex_);
  if (!resource_ready_) {
    resource_ready_ = true;
    active_display_id_ = display;
    cached_retire_fence_ = retire_fence;
    hotplug_cv_.notify_one();
  }
}

void HWCSession::HandlePendingRefresh() {
  if (pending_refresh_.none()) {
    return;
  }

  bool refresh_triggered = false;

  for (size_t i = 0; i < pending_refresh_.size(); i++) {
    if (pending_refresh_.test(i)) {
      hwc_display_[i]->SetPendingRefresh();
      if(!refresh_triggered) {
        callbacks_.Refresh(i);
        refresh_triggered = true;
      }
    }
  }

  pending_refresh_.reset();
}

void HWCSession::RegisterCallback(int32_t descriptor, hwc2_callback_data_t callback_data,
                                  hwc2_function_pointer_t pointer) {
  auto desc = static_cast<HWC2::Callback>(descriptor);

  // Detect if client died and now is back
  bool already_connected = false;
  vector<hwc2_display_t> pending_hotplugs;
  if (descriptor == HWC2_CALLBACK_HOTPLUG && pointer) {
    already_connected = callbacks_.IsClientConnected();
    if (already_connected) {
      for (auto& map_info : map_info_builtin_) {
        SCOPE_LOCK(locker_[map_info.client_id]);
        if (hwc_display_[map_info.client_id]) {
          pending_hotplugs.push_back(static_cast<hwc2_display_t>(map_info.client_id));
        }
      }
      for (auto& map_info : map_info_pluggable_) {
        SCOPE_LOCK(locker_[map_info.client_id]);
        if (hwc_display_[map_info.client_id]) {
          pending_hotplugs.push_back(static_cast<hwc2_display_t>(map_info.client_id));
        }
      }
    }
  }

  auto error = callbacks_.Register(desc, callback_data, pointer);
  if (error != HWC2::Error::None) {
    return;
  }

  DLOGI("%s callback: %s", pointer ? "Registering" : "Deregistering", to_string(desc).c_str());
  if (descriptor == HWC2_CALLBACK_HOTPLUG && pointer) {
    if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
      DLOGI("Hotplugging primary...");
      callbacks_.Hotplug(HWC_DISPLAY_PRIMARY, HWC2::Connection::Connected);
    }
    // Create displays since they should now have their final display indices set.
    DLOGI("Handling built-in displays...");
    if (HandleBuiltInDisplays()) {
      DLOGW("Failed handling built-in displays.");
    }
    DLOGI("Handling pluggable displays...");
    int32_t err = HandlePluggableDisplays(false);
    if (err) {
      DLOGW("All displays could not be created. Error %d '%s'. Hotplug handling %s.", err,
            strerror(abs(err)), pending_hotplug_event_ == kHotPlugEvent ? "deferred" :
            "dropped");
    }

    // If previously registered, call hotplug for all connected displays to refresh
    if (already_connected) {
      std::vector<hwc2_display_t> updated_pending_hotplugs;
      for (auto client_id : pending_hotplugs) {
        SCOPE_LOCK(locker_[client_id]);
        // check if the display is unregistered
        if (hwc_display_[client_id]) {
          updated_pending_hotplugs.push_back(client_id);
        }
      }
      for (auto client_id : updated_pending_hotplugs) {
        DLOGI("Re-hotplug display connected: client id = %d", UINT32(client_id));
        callbacks_.Hotplug(client_id, HWC2::Connection::Connected);
      }
    }
  }

  if (descriptor == HWC2_CALLBACK_HOTPLUG) {
    client_connected_ = !!pointer;
    // Notfify all displays.
    NotifyClientStatus(client_connected_);
  }

  // On SF stop, disable the idle time.
  if (!pointer && is_client_up_ && hwc_display_[HWC_DISPLAY_PRIMARY]) { // De-registering…
    DLOGI("disable idle time");
    hwc_display_[HWC_DISPLAY_PRIMARY]->SetIdleTimeoutMs(0,0);
    is_client_up_ = false;
    hwc_display_[HWC_DISPLAY_PRIMARY]->MarkClientActive(false);
    async_vds_creation_requested_ = false;
  }
}

int32_t HWCSession::SetActiveConfig(hwc2_display_t display, hwc2_config_t config) {
  return CallDisplayFunction(display, &HWCDisplay::SetActiveConfig, config);
}

int32_t HWCSession::SetClientTarget(hwc2_display_t display, buffer_handle_t target,
                                    const shared_ptr<Fence> acquire_fence, int32_t dataspace,
                                    hwc_region_t damage) {
  DTRACE_SCOPED();
  return CallDisplayFunction(display, &HWCDisplay::SetClientTarget, target, acquire_fence,
                             dataspace, damage);
}

int32_t HWCSession::SetClientTarget_3_1(hwc2_display_t display, buffer_handle_t target,
                                        const shared_ptr<Fence> acquire_fence, int32_t dataspace,
                                        hwc_region_t damage) {
  DTRACE_SCOPED();
  return CallDisplayFunction(display, &HWCDisplay::SetClientTarget_3_1, target, acquire_fence,
                             dataspace, damage);
}

int32_t HWCSession::SetColorMode(hwc2_display_t display, int32_t /*ColorMode*/ int_mode) {
  auto mode = static_cast<ColorMode>(int_mode);
  if (mode < ColorMode::NATIVE || mode > ColorMode::DISPLAY_BT2020) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  return CallDisplayFunction(display, &HWCDisplay::SetColorMode, mode);
}

int32_t HWCSession::SetColorModeWithRenderIntent(hwc2_display_t display,
                                                 int32_t /*ColorMode*/ int_mode,
                                                 int32_t /*RenderIntent*/ int_render_intent) {
  auto mode = static_cast<ColorMode>(int_mode);
  if (mode < ColorMode::NATIVE || mode > ColorMode::DISPLAY_BT2020) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if ((int_render_intent < 0) || (int_render_intent > MAX_EXTENDED_RENDER_INTENT)) {
    DLOGE("Invalid RenderIntent: %d", int_render_intent);
    return HWC2_ERROR_BAD_PARAMETER;
  }

  auto render_intent = static_cast<RenderIntent>(int_render_intent);
  return CallDisplayFunction(display, &HWCDisplay::SetColorModeWithRenderIntent, mode,
                             render_intent);
}

int32_t HWCSession::SetColorTransform(hwc2_display_t display, const float *matrix,
                                      int32_t /*android_color_transform_t*/ hint) {
  if (!matrix || hint < HAL_COLOR_TRANSFORM_IDENTITY ||
       hint > HAL_COLOR_TRANSFORM_CORRECT_TRITANOPIA) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  android_color_transform_t transform_hint = static_cast<android_color_transform_t>(hint);
  return CallDisplayFunction(display, &HWCDisplay::SetColorTransform, matrix, transform_hint);
}

int32_t HWCSession::SetCursorPosition(hwc2_display_t display, hwc2_layer_t layer, int32_t x,
                                      int32_t y) {
  auto status = INT32(HWC2::Error::None);
  status = CallDisplayFunction(display, &HWCDisplay::SetCursorPosition, layer, x, y);
  if (status == INT32(HWC2::Error::None)) {
    // Update cursor position
    CallLayerFunction(display, layer, &HWCLayer::SetCursorPosition, x, y);
  }
  return status;
}

int32_t HWCSession::SetLayerBlendMode(hwc2_display_t display, hwc2_layer_t layer,
                                      int32_t int_mode) {
  if (int_mode < HWC2_BLEND_MODE_INVALID || int_mode > HWC2_BLEND_MODE_COVERAGE) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  auto mode = static_cast<HWC2::BlendMode>(int_mode);
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerBlendMode, mode);
}

int32_t HWCSession::SetLayerBuffer(hwc2_display_t display, hwc2_layer_t layer,
                                   buffer_handle_t buffer,
                                   const shared_ptr<Fence> &acquire_fence) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerBuffer, buffer, acquire_fence);
}

int32_t HWCSession::SetLayerColor(hwc2_display_t display, hwc2_layer_t layer, hwc_color_t color) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerColor, color);
}

int32_t HWCSession::SetLayerCompositionType(hwc2_display_t display, hwc2_layer_t layer,
                                            int32_t int_type) {
  auto type = static_cast<HWC2::Composition>(int_type);
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerCompositionType, type);
}

int32_t HWCSession::SetLayerDataspace(hwc2_display_t display, hwc2_layer_t layer,
                                      int32_t dataspace) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerDataspace, dataspace);
}

int32_t HWCSession::SetLayerDisplayFrame(hwc2_display_t display, hwc2_layer_t layer,
                                         hwc_rect_t frame) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerDisplayFrame, frame);
}

int32_t HWCSession::SetLayerPlaneAlpha(hwc2_display_t display, hwc2_layer_t layer, float alpha) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerPlaneAlpha, alpha);
}

int32_t HWCSession::SetLayerSourceCrop(hwc2_display_t display, hwc2_layer_t layer,
                                       hwc_frect_t crop) {
  return HWCSession::CallLayerFunction(display, layer, &HWCLayer::SetLayerSourceCrop, crop);
}

int32_t HWCSession::SetLayerSurfaceDamage(hwc2_display_t display, hwc2_layer_t layer,
                                          hwc_region_t damage) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerSurfaceDamage, damage);
}

int32_t HWCSession::SetLayerTransform(hwc2_display_t display, hwc2_layer_t layer,
                                      int32_t int_transform) {
  auto transform = static_cast<HWC2::Transform>(int_transform);
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerTransform, transform);
}

int32_t HWCSession::SetLayerVisibleRegion(hwc2_display_t display, hwc2_layer_t layer,
                                          hwc_region_t visible) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerVisibleRegion, visible);
}

int32_t HWCSession::SetLayerZOrder(hwc2_display_t display, hwc2_layer_t layer, uint32_t z) {
  return CallDisplayFunction(display, &HWCDisplay::SetLayerZOrder, layer, z);
}

int32_t HWCSession::SetLayerType(hwc2_display_t display, hwc2_layer_t layer,
                                 IQtiComposerClient::LayerType type) {
  return CallDisplayFunction(display, &HWCDisplay::SetLayerType, layer, type);
}

int32_t HWCSession::SetLayerFlag(hwc2_display_t display, hwc2_layer_t layer,
                                 IQtiComposerClient::LayerFlag flag) {
   return CallLayerFunction(display, layer, &HWCLayer::SetLayerFlag, flag);
}

int32_t HWCSession::SetLayerColorTransform(hwc2_display_t display, hwc2_layer_t layer,
                                           const float *matrix) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerColorTransform, matrix);
}

int32_t HWCSession::SetDisplayElapseTime(hwc2_display_t display, uint64_t time) {
  return CallDisplayFunction(display, &HWCDisplay::SetDisplayElapseTime, time);
}

int32_t HWCSession::SetOutputBuffer(hwc2_display_t display, buffer_handle_t buffer,
                                    const shared_ptr<Fence> &release_fence) {
  if (INT32(display) != GetDisplayIndex(qdutils::DISPLAY_VIRTUAL)) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  SCOPE_LOCK(locker_[display]);
  if (hwc_display_[display]) {
    auto vds = reinterpret_cast<HWCDisplayVirtual *>(hwc_display_[display]);
    auto status = vds->SetOutputBuffer(buffer, release_fence);
    return INT32(status);
  } else {
    return HWC2_ERROR_BAD_DISPLAY;
  }
}

int32_t HWCSession::SetPowerMode(hwc2_display_t display, int32_t int_mode) {
  if (display >= HWCCallbacks::kNumDisplays || !hwc_display_[display]) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  //  validate device and also avoid undefined behavior in cast to HWC2::PowerMode
  if (int_mode < HWC2_POWER_MODE_OFF || int_mode > HWC2_POWER_MODE_DOZE_SUSPEND) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  auto mode = static_cast<HWC2::PowerMode>(int_mode);

  // When secure session going on primary, if power request comes on second built-in, cache it and
  // process once secure session ends.
  // Allow power off transition during secure session.
  {
    hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
    SCOPE_LOCK(locker_[display]);
    if (hwc_display_[display]) {
      bool is_builtin = (hwc_display_[display]->GetDisplayClass() == DISPLAY_CLASS_BUILTIN);
      bool is_power_off = (hwc_display_[display]->GetCurrentPowerMode() == HWC2::PowerMode::Off);
      if (secure_session_active_ && is_builtin && is_power_off) {
        if (active_builtin_disp_id != HWCCallbacks::kNumDisplays) {
          DLOGI("Secure session in progress, defer power state change");
          hwc_display_[display]->SetPendingPowerMode(mode);
          return HWC2_ERROR_NONE;
        }
      }
    }
  }
  if (pending_power_mode_[display]) {
    DLOGW("Set power mode is not allowed during secure display session");
    return HWC2_ERROR_UNSUPPORTED;
  }

  //  all displays support on/off. Check for doze modes
  int support = 0;
  auto status = GetDozeSupport(display, &support);
  if (status != HWC2_ERROR_NONE) {
    DLOGW("Failed to get doze support Error = %d", status);
    return INT32(status);
  }

  if (!support && (mode == HWC2::PowerMode::Doze || mode == HWC2::PowerMode::DozeSuspend)) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  HWC2::PowerMode last_power_mode = hwc_display_[display]->GetCurrentPowerMode();

  if (last_power_mode == mode) {
    return HWC2_ERROR_NONE;
  }

  auto error = CallDisplayFunction(display, &HWCDisplay::SetPowerMode, mode,
                                   false /* teardown */);
  if (INT32(error) != HWC2_ERROR_NONE) {
    return INT32(error);
  }

  // Reset idle pc ref count on suspend, as we enable idle pc during suspend.
  if (mode == HWC2::PowerMode::Off) {
    idle_pc_ref_cnt_ = 0;
  }

  UpdateThrottlingRate();

  if (mode == HWC2::PowerMode::Doze) {
    // Trigger one more refresh for PP features to take effect.
    pending_refresh_.set(UINT32(display));
  }

  return HWC2_ERROR_NONE;
}

int32_t HWCSession::SetVsyncEnabled(hwc2_display_t display, int32_t int_enabled) {
  //  avoid undefined behavior in cast to HWC2::Vsync
  if (int_enabled < HWC2_VSYNC_INVALID || int_enabled > HWC2_VSYNC_DISABLE) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  auto enabled = static_cast<HWC2::Vsync>(int_enabled);

  if (int_enabled == HWC2_VSYNC_ENABLE) {
    callbacks_.UpdateVsyncSource(display);
  }

  return CallDisplayFunction(display, &HWCDisplay::SetVsyncEnabled, enabled);
}

int32_t HWCSession::SetDimmingEnable(hwc2_display_t display, int32_t int_enabled) {
  return CallDisplayFunction(display, &HWCDisplay::SetDimmingEnable, int_enabled);
}

int32_t HWCSession::SetDimmingMinBl(hwc2_display_t display, int32_t min_bl) {
  return CallDisplayFunction(display, &HWCDisplay::SetDimmingMinBl, min_bl);
}

int32_t HWCSession::GetDozeSupport(hwc2_display_t display, int32_t *out_support) {
  if (!out_support) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays || (hwc_display_[display] == nullptr)) {
    // display may come as -1  from VTS test case
    DLOGW("Invalid Display %d ", UINT32(display));
    return HWC2_ERROR_BAD_DISPLAY;
  }

  *out_support = 0;

  SCOPE_LOCK(locker_[display]);
  if (!hwc_display_[display]) {
    DLOGW("Display %d is not created yet.", INT32(display));
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (hwc_display_[display]->GetDisplayClass() != DISPLAY_CLASS_BUILTIN) {
    return HWC2_ERROR_NONE;
  }

  *out_support = hwc_display_[display]->HasSmartPanelConfig() ? 1 : 0;

  return HWC2_ERROR_NONE;
}

HWC2::Error HWCSession::CreateVirtualDisplayObj(uint32_t width, uint32_t height, int32_t *format,
                                                hwc2_display_t *out_display_id) {
  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  hwc2_display_t client_id = HWCCallbacks::kNumDisplays;
  if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[active_builtin_disp_id]);
    std::bitset<kSecureMax> secure_sessions = 0;
    if (hwc_display_[active_builtin_disp_id]) {
      hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
    }
    if (secure_sessions.any()) {
      DLOGE("Secure session is active, cannot create virtual display.");
      return HWC2::Error::Unsupported;
    } else if (IsVirtualDisplayConnected()) {
      DLOGE("Previous virtual session is active, cannot create virtual display.");
      return HWC2::Error::Unsupported;
    } else if (IsPluggableDisplayConnected()) {
      DLOGE("External session is active, cannot create virtual display.");
      return HWC2::Error::Unsupported;
    }
  }

  HWC2::Error error = TeardownConcurrentWriteback(HWC_DISPLAY_PRIMARY);
  if (error != HWC2::Error::None) {
    return error;
  }

  core_intf_->ReserveDisplay(kVirtual);
  // Trigger Refresh.
  callbacks_.Refresh(HWC_DISPLAY_PRIMARY);
  // Ideally we need to wait on all connected displays.
  WaitForCommitDoneAsync(HWC_DISPLAY_PRIMARY, kClientVirtualDisplay);
  // Lock confined to this scope
  int status = -EINVAL;
  for (auto &map_info : map_info_virtual_) {
    client_id = map_info.client_id;
    {
      SCOPE_LOCK(locker_[client_id]);
      auto &hwc_display = hwc_display_[client_id];
      if (hwc_display) {
        continue;
      }

      int32_t display_id = GetVirtualDisplayId();
      status = virtual_display_factory_.Create(core_intf_, &buffer_allocator_, &callbacks_,
                                               this, client_id, display_id, width, height,
                                               format, set_min_lum_, set_max_lum_, &hwc_display);
      // TODO(user): validate width and height support
      if (status) {
        return HWC2::Error::NoResources;
      }

      {
        SCOPE_LOCK(hdr_locker_[client_id]);
        is_hdr_display_[UINT32(client_id)] = HasHDRSupport(hwc_display);
      }

      DLOGI("Created virtual display id:%" PRIu64 " with res: %dx%d", client_id, width, height);

      *out_display_id = client_id;
      map_info.disp_type = kVirtual;
      map_info.sdm_id = display_id;
      break;
    }
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

bool HWCSession::IsVirtualDisplayConnected() {
  for (auto &map_info : map_info_virtual_) {
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
      status = SetIdleTimeout(UINT32(input_parcel->readInt32()));
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
        status = SetDisplayStatus(disp_id, disp_status);
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

    case qService::IQService::TOGGLE_SCREEN_UPDATES: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }
        int32_t input = input_parcel->readInt32();
        status = ToggleScreenUpdate(input == 1);
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

    case qService::IQService::SET_NOISE_PLUGIN_OVERRIDE: {
        if (!input_parcel) {
          DLOGE("QService command = %d: input_parcel needed.", command);
          break;
        }

        int32_t disp_id = input_parcel->readInt32();

        bool override_en = ((input_parcel->readInt32()) == 1);

        int32_t attn = -1;
        if (input_parcel->dataPosition() != input_parcel->dataSize()) {
          attn = input_parcel->readInt32();
        }

        int32_t noise_zpos = -1;
        if (input_parcel->dataPosition() != input_parcel->dataSize()) {
          noise_zpos = input_parcel->readInt32();
        }

        status = SetNoisePlugInOverride(disp_id, override_en, attn, noise_zpos);
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

        uint32_t display = input_parcel->readUint32();
        uint32_t max_brightness_level = 0;
        status = getDisplayMaxBrightness(display, &max_brightness_level);
        if (status || !max_brightness_level) {
          output_parcel->writeInt32(max_brightness_level);
          DLOGE("Failed to get max brightness %u,  status %d", max_brightness_level, status);
          break;
        }
        DLOGV("Panel Max brightness is %u", max_brightness_level);

        float brightness_precent = -1.0f;
        status = getDisplayBrightness(display, &brightness_precent);
        if (brightness_precent == -1.0f) {
          output_parcel->writeInt32(0);
        } else {
          output_parcel->writeInt32(INT32(brightness_precent*(max_brightness_level - 1) + 1));
        }
      }
      break;

    case qService::IQService::SET_PANEL_BRIGHTNESS: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }

        uint32_t max_brightness_level = 0;
        uint32_t display = HWC_DISPLAY_PRIMARY;
        status = getDisplayMaxBrightness(display, &max_brightness_level);
        if (status || max_brightness_level <= 1) {
          output_parcel->writeInt32(max_brightness_level);
          DLOGE("Failed to get max brightness %u, status %d", max_brightness_level, status);
          break;
        }
        DLOGV("Panel Max brightness is %u", max_brightness_level);

        int level = input_parcel->readInt32();
        if (level == 0) {
          status = SetDisplayBrightness(display, -1.0f);
        } else {
          status = SetDisplayBrightness(display,
                    (level - 1)/(static_cast<float>(max_brightness_level - 1)));
        }
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
        status = SetCameraLaunchStatus(camera_status);
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

    case qService::IQService::SET_COLOR_SAMPLING_ENABLED:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = setColorSamplingEnabled(input_parcel);
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

    case qService::IQService::SET_FRAME_TRIGGER_MODE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetFrameTriggerMode(input_parcel);
      break;

    case qService::IQService::SET_BRIGHTNESS_SCALE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetDisplayBrightnessScale(input_parcel);
      break;

    case qService::IQService::SET_VSYNC_STATE: {
      if (!input_parcel || !output_parcel) {
        DLOGE("Qservice command = %d: input_parcel needed.", command);
        break;
      }
      auto display = input_parcel->readInt32();
      int32_t enable = input_parcel->readInt32();
      int32_t vsync_state = HWC2_VSYNC_INVALID;
      if (enable == 0) {
        vsync_state = HWC2_VSYNC_DISABLE;
      } else if (enable == 1) {
        vsync_state = HWC2_VSYNC_ENABLE;
      }
      status = SetVsyncEnabled(display, vsync_state);
      output_parcel->writeInt32(status);
    }
    break;

    case qService::IQService::NOTIFY_TUI_TRANSITION: {
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      int disp_id = input_parcel->readInt32();
      int event = input_parcel->readInt32();
      status = HandleTUITransition(disp_id, event);
      output_parcel->writeInt32(status);
    }
    break;

    case qService::IQService::GET_DISPLAY_PORT_ID: {
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      int disp_id = input_parcel->readInt32();
      int port_id = 0;
      status = GetDisplayPortId(disp_id, &port_id);
      output_parcel->writeInt32(port_id);
    }
    break;

    case qService::IQService::SET_DIMMING_ENABLE: {
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      int disp_id = input_parcel->readInt32();
      int enable = input_parcel->readInt32();
      status = SetDimmingEnable(disp_id, enable);
      output_parcel->writeInt32(status);
    }
    break;

    case qService::IQService::SET_DIMMING_MIN_BL: {
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      int disp_id = input_parcel->readInt32();
      int min_bl = input_parcel->readInt32();
      status = SetDimmingMinBl(disp_id, min_bl);
      output_parcel->writeInt32(status);
    }
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

android::status_t HWCSession::setColorSamplingEnabled(const android::Parcel *input_parcel) {
  int dpy = input_parcel->readInt32();
  int enabled_cmd = input_parcel->readInt32();
  if (dpy < HWC_DISPLAY_PRIMARY || dpy >= HWC_NUM_DISPLAY_TYPES || enabled_cmd < 0 ||
      enabled_cmd > 1) {
    return android::BAD_VALUE;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[dpy]);
  if (!hwc_display_[dpy]) {
    DLOGW("No display id %i active to enable histogram event", dpy);
    return android::BAD_VALUE;
  }

  auto error = hwc_display_[dpy]->SetDisplayedContentSamplingEnabledVndService(enabled_cmd);
  return (error == HWC2::Error::None) ? android::OK : android::BAD_VALUE;
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

  hwc2_config_t current_config = 0, new_config = 0;
  android::status_t status = -EINVAL;
  hwc_display_[HWC_DISPLAY_PRIMARY]->GetActiveConfig(&current_config);
  uint32_t mode = UINT32(input_parcel->readInt32());
  status = hwc_display_[HWC_DISPLAY_PRIMARY]->Perform(HWCDisplayBuiltIn::SET_DISPLAY_MODE, mode);
  hwc_display_[HWC_DISPLAY_PRIMARY]->GetActiveConfig(&new_config);

  //In case of config change, notify client with the new configuration
  if (new_config != current_config) {
    NotifyDisplayAttributes(HWC_DISPLAY_PRIMARY, new_config);
  }

  return status;
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

  // Output buffer dump is not supported, if Virtual display is present.
  bool output_buffer_dump = bit_mask_layer_type & (1 << OUTPUT_LAYER_DUMP);
  if (output_buffer_dump) {
    int virtual_dpy_index = GetDisplayIndex(qdutils::DISPLAY_VIRTUAL);
    if ((virtual_dpy_index != -1) && hwc_display_[virtual_dpy_index]) {
      DLOGW("Output buffer dump is not supported with Virtual display!");
      return -EINVAL;
    }
  }

  // Read optional user preferences: output_format, tap_point, pu_in_cwb_roi, cwb_roi.
  int32_t output_format = HAL_PIXEL_FORMAT_RGB_888;
  CwbConfig cwb_config = {};

  if (input_parcel->dataPosition() != input_parcel->dataSize()) {
    // HAL Pixel Format for output buffer
    output_format = input_parcel->readInt32();
  }

  LayerBufferFormat sdm_format = HWCLayer::GetSDMFormat(output_format, 0);
  if (sdm_format == kFormatInvalid) {
    DLOGW("Format %d is not supported by SDM", output_format);
    return -EINVAL;
  }

  if (input_parcel->dataPosition() != input_parcel->dataSize()) {
    // Option to dump Layer Mixer output (0) or DSPP output (1) or Demura  output (2)
    cwb_config.tap_point = static_cast<CwbTapPoint>(input_parcel->readInt32());
  }
  if (input_parcel->dataPosition() != input_parcel->dataSize()) {
    // Option to include PU ROI in CWB ROI
    cwb_config.pu_as_cwb_roi = static_cast<bool>(input_parcel->readInt32());
  }

  LayerRect &cwb_roi = cwb_config.cwb_roi;
  if (input_parcel->dataPosition() != input_parcel->dataSize()) {
    cwb_roi.left = static_cast<float>(input_parcel->readInt32());
  }
  if (input_parcel->dataPosition() != input_parcel->dataSize()) {
    cwb_roi.top = static_cast<float>(input_parcel->readInt32());
  }
  if (input_parcel->dataPosition() != input_parcel->dataSize()) {
    cwb_roi.right = static_cast<float>(input_parcel->readInt32());
  }
  if (input_parcel->dataPosition() != input_parcel->dataSize()) {
    cwb_roi.bottom = static_cast<float>(input_parcel->readInt32());
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
                                                        output_format, cwb_config);
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

  int disp_idx = GetDisplayIndex(display);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", display);
    return -EINVAL;
  }

  if (mode < ColorMode::NATIVE || mode > ColorMode::DISPLAY_BT2020) {
    DLOGE("Invalid ColorMode: %d", mode);
    return HWC2_ERROR_BAD_PARAMETER;
  }
  auto err = CallDisplayFunction(static_cast<hwc2_display_t>(disp_idx), &HWCDisplay::SetColorMode,
                                 mode);
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

android::status_t HWCSession::SetFrameTriggerMode(const android::Parcel *input_parcel) {
  auto display_id = static_cast<int>(input_parcel->readInt32());
  auto mode = static_cast<uint32_t>(input_parcel->readInt32());

  int disp_idx = GetDisplayIndex(display_id);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", display_id);
    return -EINVAL;
  }

  auto err = CallDisplayFunction(static_cast<hwc2_display_t>(disp_idx),
                                 &HWCDisplay::SetFrameTriggerMode, mode);
  if (err != HWC2_ERROR_NONE)
    return -EINVAL;

  return 0;
}

android::status_t HWCSession::SetColorModeWithRenderIntentOverride(
    const android::Parcel *input_parcel) {
  auto display = static_cast<hwc2_display_t>(input_parcel->readInt32());
  auto mode = static_cast<ColorMode>(input_parcel->readInt32());
  auto int_intent = static_cast<int>(input_parcel->readInt32());

  if (mode < ColorMode::NATIVE || mode > ColorMode::DISPLAY_BT2020) {
    DLOGE("Invalid ColorMode: %d", mode);
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if ((int_intent < 0) || (int_intent > MAX_EXTENDED_RENDER_INTENT)) {
    DLOGE("Invalid RenderIntent: %d", int_intent);
    return HWC2_ERROR_BAD_PARAMETER;
  }

  auto intent = static_cast<RenderIntent>(int_intent);
  auto err =
      CallDisplayFunction(display, &HWCDisplay::SetColorModeWithRenderIntent, mode, intent);
  if (err != HWC2_ERROR_NONE)
    return -EINVAL;

  return 0;
}
android::status_t HWCSession::SetColorModeById(const android::Parcel *input_parcel) {
  int display = input_parcel->readInt32();
  auto mode = input_parcel->readInt32();

  int disp_idx = GetDisplayIndex(display);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", display);
    return -EINVAL;
  }

  auto err = CallDisplayFunction(static_cast<hwc2_display_t>(disp_idx),
                                 &HWCDisplay::SetColorModeById, mode);
  if (err != HWC2_ERROR_NONE)
    return -EINVAL;

  return 0;
}

android::status_t HWCSession::SetColorModeFromClient(const android::Parcel *input_parcel) {
  int display = input_parcel->readInt32();
  auto mode = input_parcel->readInt32();

  int disp_idx = GetDisplayIndex(display);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", display);
    return -EINVAL;
  }

  auto err = CallDisplayFunction(static_cast<hwc2_display_t>(disp_idx),
                                 &HWCDisplay::SetColorModeFromClientApi, mode);
  if (err != HWC2_ERROR_NONE)
    return -EINVAL;

  callbacks_.Refresh(static_cast<hwc2_display_t>(disp_idx));

  return 0;
}

android::status_t HWCSession::RefreshScreen(const android::Parcel *input_parcel) {
  int display = input_parcel->readInt32();

  int disp_idx = GetDisplayIndex(display);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", display);
    return -EINVAL;
  }

  callbacks_.Refresh(static_cast<hwc2_display_t>(disp_idx));

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

  ret = hwc_display_[display_id]->ColorSVCRequestRoute(req_payload, resp_payload, pending_action);

  return ret;
}

android::status_t HWCSession::QdcmCMDHandler(const android::Parcel *input_parcel,
                                             android::Parcel *output_parcel) {
  int ret = 0;
  float *brightness = NULL;
  uint32_t display_id(0);
  PPPendingParams pending_action;
  PPDisplayAPIPayload resp_payload, req_payload;
  uint8_t *disp_id = NULL;
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
          callbacks_.Refresh(display_id);
          break;
        case kEnterQDCMMode:
          ret = color_mgr_->EnableQDCMMode(true, hwc_display_[display_id]);
          hwc_display_[display_id]->NotifyDisplayCalibrationMode(true);
          break;
        case kExitQDCMMode:
          ret = color_mgr_->EnableQDCMMode(false, hwc_display_[display_id]);
          hwc_display_[display_id]->NotifyDisplayCalibrationMode(false);
          break;
        case kApplySolidFill:
          {
            SCOPE_LOCK(locker_[display_id]);
            ret = color_mgr_->SetSolidFill(pending_action.params,
                                           true, hwc_display_[display_id]);
          }
          callbacks_.Refresh(display_id);
          usleep(kSolidFillDelay);
          break;
        case kDisableSolidFill:
          {
            SCOPE_LOCK(locker_[display_id]);
            ret = color_mgr_->SetSolidFill(pending_action.params,
                                           false, hwc_display_[display_id]);
          }
          callbacks_.Refresh(display_id);
          usleep(kSolidFillDelay);
          break;
        case kSetPanelBrightness:
          ret = -EINVAL;
          brightness = reinterpret_cast<float *>(resp_payload.payload);
          if (brightness == NULL) {
            DLOGE("Brightness payload is Null");
          } else {
            ret = INT(SetDisplayBrightness(static_cast<hwc2_display_t>(display_id), *brightness));
          }
          break;
        case kEnableFrameCapture: {
          int external_dpy_index = GetDisplayIndex(qdutils::DISPLAY_EXTERNAL);
          int virtual_dpy_index = GetDisplayIndex(qdutils::DISPLAY_VIRTUAL);
          if (((external_dpy_index != -1) && hwc_display_[external_dpy_index]) ||
              ((virtual_dpy_index != -1) && hwc_display_[virtual_dpy_index])) {
            return -ENODEV;
          }
          ret = color_mgr_->SetFrameCapture(pending_action.params, true, hwc_display_[display_id]);
          callbacks_.Refresh(display_id);
        } break;
        case kDisableFrameCapture:
          ret = color_mgr_->SetFrameCapture(pending_action.params, false,
                                            hwc_display_[display_id]);
          break;
        case kConfigureDetailedEnhancer:
          ret = color_mgr_->SetDetailedEnhancer(pending_action.params, hwc_display_[display_id]);
          callbacks_.Refresh(display_id);
          break;
        case kModeSet:
          ret = static_cast<int>
                  (hwc_display_[display_id]->RestoreColorTransform());
          callbacks_.Refresh(display_id);
          break;
        case kNoAction:
          break;
        case kMultiDispProc:
          for (auto &map_info : map_info_builtin_) {
            uint32_t id = UINT32(map_info.client_id);
            if (id < HWCCallbacks::kNumDisplays && hwc_display_[id]) {
              int result = 0;
              resp_payload.DestroyPayload();
              result = hwc_display_[id]->ColorSVCRequestRoute(req_payload, &resp_payload,
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
            callbacks_.Refresh(display_id);
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
  if (disp_id != HWC_DISPLAY_PRIMARY) {
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
  if (disp_id != HWC_DISPLAY_PRIMARY) {
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
  if (disp_id != HWC_DISPLAY_PRIMARY) {
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

  float min_lum = input_parcel->readFloat();
  float max_lum = input_parcel->readFloat();

  // check for out of range luminance values
  if (min_lum <= 0.0f || min_lum >= 1.0f || max_lum <= 100.0f || max_lum >= 1000.0f) {
    return -EINVAL;
  }

  std::lock_guard<std::mutex> obj(mutex_lum_);
  set_min_lum_ = min_lum;
  set_max_lum_ = max_lum;
  DLOGI("set max_lum %f, min_lum %f", set_max_lum_, set_min_lum_);

  return 0;
}

void HWCSession::UEventHandler(const char *uevent_data, int length) {
  // Drop hotplug uevents until SurfaceFlinger (the client) is connected. The equivalent of hotplug
  // uevent handling will be done once when SurfaceFlinger connects, at RegisterCallback(). Since
  // HandlePluggableDisplays() reads the latest connection states of all displays, no uevent is
  // lost.
  if (callbacks_.IsClientConnected() && strcasestr(uevent_data, HWC_UEVENT_DRM_EXT_HOTPLUG)) {
    // MST hotplug will not carry connection status/test pattern etc.
    // Pluggable display handler will check all connection status' and take action accordingly.
    const char *str_status = GetTokenValue(uevent_data, length, "status=");
    const char *str_sstmst = GetTokenValue(uevent_data, length, "HOTPLUG=");
    // Check MST_HOTPLUG for backward compatibility.
    const char *str_mst = GetTokenValue(uevent_data, length, "MST_HOTPLUG=");
    if (!str_status && !str_mst  && !str_sstmst) {
      return;
    }

    hpd_bpp_ = GetEventValue(uevent_data, length, "bpp=");
    hpd_pattern_ = GetEventValue(uevent_data, length, "pattern=");
    DLOGI("UEvent = %s, status = %s, HOTPLUG = %s (SST/MST)%s%s, bpp = %d, pattern = %d",
          uevent_data, str_status ? str_status : "NULL", str_sstmst ? str_sstmst : "NULL",
          str_mst ? ", MST_HOTPLUG = " : "", str_mst ? str_mst : "",
          hpd_bpp_, hpd_pattern_);

    // Handle hotplug.
    int32_t err = HandlePluggableDisplays(true);
    if (err) {
      DLOGW("Hotplug handling failed. Error %d '%s'. Hotplug handling %s.", err,
            strerror(abs(err)), (pending_hotplug_event_ == kHotPlugEvent) ?
            "deferred" : "dropped");
    }

    if (str_status) {
      bool connected = (strncmp(str_status, "connected", strlen("connected")) == 0);
      DLOGI("Connected = %d", connected);
      // Pass on legacy HDMI hot-plug event.
      qservice_->onHdmiHotplug(INT(connected));
    }
  }
}

int32_t HWCSession::GetVsyncPeriod(hwc2_display_t disp, uint32_t *vsync_period) {
  if (disp >= HWCCallbacks::kNumDisplays) {
    DLOGW("Invalid Display : display = %" PRIu64, disp);
    return HWC2_ERROR_BAD_DISPLAY;
  }

  SCOPE_LOCK(locker_[(int)disp]);
  // default value
  *vsync_period = 1000000000ul / 60;

  if (hwc_display_[disp]) {
    hwc_display_[disp]->GetDisplayAttribute(0, HwcAttribute::VSYNC_PERIOD, (int32_t *)vsync_period);
  }

  return HWC2_ERROR_NONE;
}

void HWCSession::Refresh(hwc2_display_t display) {
  callbacks_.Refresh(display);
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

int HWCSession::CreatePrimaryDisplay() {
  int status = -EINVAL;
  HWDisplaysInfo hw_displays_info = {};

  DisplayError error = core_intf_->GetDisplaysStatus(&hw_displays_info);
  if (error != kErrorNone) {
    DLOGE("Failed to get connected display list. Error = %d", error);
    return status;
  }

  SCOPE_LOCK(primary_display_lock_);

  while (primary_pending_) {
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
        DLOGI("Created primary display type = %d, sdm id = %d, client id = %d", info.display_type,
              info.display_id, UINT32(client_id));
        {
          SCOPE_LOCK(hdr_locker_[client_id]);
          is_hdr_display_[UINT32(client_id)] = HasHDRSupport(*hwc_display);
        }

        map_info_primary_.disp_type = info.display_type;
        map_info_primary_.sdm_id = info.display_id;
        color_mgr_ = HWCColorManager::CreateColorManager(&buffer_allocator_);
        if (!color_mgr_) {
          DLOGW("Failed to load HWCColorManager.");
        }

      } else {
        DLOGE("Primary display creation has failed! status = %d", status);
        return status;
      }

      primary_pending_ = false;
      primary_display_lock_.Signal();

      // Primary display is found, no need to parse more.
      break;
    }

    if (primary_pending_) {
      DisplayError error = core_intf_->GetDisplaysStatus(&hw_displays_info);
      if (error != kErrorNone) {
        DLOGE("Failed to get connected display list. Error = %d", error);
        return status;
      }
    }
  }
  return status;
}

int HWCSession::HandleBuiltInDisplays() {
  SCOPE_LOCK(primary_display_lock_);
  while (primary_pending_) {
    primary_display_lock_.Wait();
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

        DLOGI("Create builtin display, sdm id = %d, client id = %d", info.display_id,
              UINT32(client_id));
        status = HWCDisplayBuiltIn::Create(core_intf_, &buffer_allocator_, &callbacks_, this,
                                           qservice_, client_id, info.display_id,
                                           &hwc_display_[client_id]);
        if (status) {
          DLOGE("Builtin display creation failed.");
          break;
        }

        {
          SCOPE_LOCK(hdr_locker_[client_id]);
          is_hdr_display_[UINT32(client_id)] = HasHDRSupport(hwc_display_[client_id]);
        }

        DLOGI("Builtin display created: sdm id = %d, client id = %d", info.display_id,
              UINT32(client_id));
        map_info.disp_type = info.display_type;
        map_info.sdm_id = info.display_id;

      }

      DLOGI("Hotplugging builtin display, sdm id = %d, client id = %d", info.display_id,
            UINT32(client_id));
      // Free lock before the callback
      primary_display_lock_.Unlock();
      callbacks_.Hotplug(client_id, HWC2::Connection::Connected);
      primary_display_lock_.Lock();
      break;
    }
  }

  return status;
}

int HWCSession::HandlePluggableDisplays(bool delay_hotplug) {
  HWDisplaysInfo hw_displays_info = {};
  {
    SCOPE_LOCK(pluggable_handler_lock_);

    hwc2_display_t virtual_display_index =
        (hwc2_display_t)GetDisplayIndex(qdutils::DISPLAY_VIRTUAL);
    std::bitset<kSecureMax> secure_sessions = 0;

    hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
    if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
      Locker::ScopeLock lock_a(locker_[active_builtin_disp_id]);
      hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
    }

    if (secure_sessions.any() || hwc_display_[virtual_display_index]) {
      // Defer hotplug handling.
      DLOGI("Marking hotplug pending...");
      pending_hotplug_event_ = kHotPlugEvent;
      return -EAGAIN;
    }

    DLOGI("Handling hotplug...");
    DisplayError error = core_intf_->GetDisplaysStatus(&hw_displays_info);
    if (error != kErrorNone) {
      DLOGE("Failed to get connected display list. Error = %d", error);
      return -EINVAL;
    }
  }

  HandlePluggablePrimaryDisplay(&hw_displays_info);

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
        pending_hotplug_event_ = kHotPlugEvent;
        status = 0;
        break;
      default:
        // Real errors we want to flag and stop hotplug handling.
        pending_hotplug_event_ = kHotPlugNone;
        DLOGE("All displays could not be connected. Error %d '%s'.", status, strerror(abs(status)));
    }
    DLOGI("Handling hotplug... %s", (kHotPlugNone == pending_hotplug_event_) ?
          "Stopped." : "Done. Hotplug events pending.");
    return status;
  }

  pending_hotplug_event_ = kHotPlugNone;

  DLOGI("Handling hotplug... Done.");
  return 0;
}

int HWCSession::HandleConnectedDisplays(HWDisplaysInfo *hw_displays_info, bool delay_hotplug) {
  int status = 0;
  std::vector<hwc2_display_t> pending_hotplugs = {};
  hwc2_display_t client_id = 0;

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
      if (callbacks_.IsClientConnected()) {
        // Trigger a display refresh since we depend on PresentDisplay() to handle pending hotplugs.
        hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
        if (active_builtin_disp_id >= HWCCallbacks::kNumDisplays) {
          active_builtin_disp_id = HWC_DISPLAY_PRIMARY;
        }
        callbacks_.Refresh(active_builtin_disp_id);
      }
      break;
    }

    // find an empty slot to create display.
    for (auto &map_info : map_info_pluggable_) {
      client_id = map_info.client_id;

      // Lock confined to this scope
      {
        SCOPE_LOCK(locker_[client_id]);
        auto &hwc_display = hwc_display_[client_id];
        if (hwc_display) {
          // Display slot is already used.
          continue;
        }

        DLOGI("Create pluggable display, sdm id = %d, client id = %d", info.display_id,
              UINT32(client_id));

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

        {
          SCOPE_LOCK(hdr_locker_[client_id]);
          is_hdr_display_[UINT32(client_id)] = HasHDRSupport(hwc_display);
        }

        DLOGI("Created pluggable display successfully: sdm id = %d, client id = %d",
              info.display_id, UINT32(client_id));

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
    status = WaitForResources(delay_hotplug, active_builtin_disp_id, client_id);
    if (status) {
      return status;
    }
  }

  for (auto client_id : pending_hotplugs) {
    DLOGI("Notify hotplug display connected: client id = %d", UINT32(client_id));
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
      hwc2_display_t client_id = map_info.client_id;
      bool is_valid_pluggable_display = false;
      auto &hwc_display = hwc_display_[client_id];
      if (hwc_display) {
        is_valid_pluggable_display = true;
        hwc_display->Abort();
      }

      DestroyDisplay(&map_info);
      if (enable_primary_reconfig_req_ && is_valid_pluggable_display) {
        hwc2_display_t active_builtin_id = GetActiveBuiltinDisplay();
        if (active_builtin_id < HWCCallbacks::kNumDisplays) {
          SCOPE_LOCK(locker_[active_builtin_id]);
          hwc2_config_t current_config = 0, new_config = 0;
          hwc_display_[active_builtin_id]->GetActiveConfig(&current_config);
          hwc_display_[active_builtin_id]->SetAlternateDisplayConfig(false);
          hwc_display_[active_builtin_id]->GetActiveConfig(&new_config);
          if (new_config != current_config) {
            NotifyDisplayAttributes(active_builtin_id, new_config);
          }
        }
      }
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

  DLOGI("Notify hotplug display disconnected: client id = %d", UINT32(client_id));
  callbacks_.Hotplug(client_id, HWC2::Connection::Disconnected);

  // Wait until all commands are flushed.
  std::lock_guard<std::mutex> hwc_lock(command_seq_mutex_);
  {
    SCOPE_LOCK(locker_[client_id]);
    auto &hwc_display = hwc_display_[client_id];
    if (!hwc_display) {
      return;
    }
    DLOGI("Destroy display %d-%d, client id = %d", map_info->sdm_id, map_info->disp_type,
         UINT32(client_id));
    {
      SCOPE_LOCK(hdr_locker_[client_id]);
      is_hdr_display_[UINT32(client_id)] = false;
    }

    if (!map_info->test_pattern) {
      HWCDisplayPluggable::Destroy(hwc_display);
    } else {
      HWCDisplayPluggableTest::Destroy(hwc_display);
    }

    active_displays_.erase(client_id);
    display_ready_.reset(UINT32(client_id));
    pending_power_mode_[client_id] = false;
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
        UINT32(client_id));
  {
    SCOPE_LOCK(hdr_locker_[client_id]);
    is_hdr_display_[UINT32(client_id)] = false;
  }

  switch (map_info->disp_type) {
    case kBuiltIn:
      HWCDisplayBuiltIn::Destroy(hwc_display);
      break;
    default:
      virtual_display_factory_.Destroy(hwc_display);
      break;
    }

    active_displays_.erase(client_id);

    pending_power_mode_[client_id] = false;
    hwc_display = nullptr;
    display_ready_.reset(UINT32(client_id));
    map_info->Reset();
}

void HWCSession::PerformDisplayPowerReset() {
  // Wait until all commands are flushed.
  std::lock_guard<std::mutex> lock(command_seq_mutex_);
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
      DLOGI("Powering off display = %d", INT32(display));
      status = hwc_display_[display]->SetPowerMode(HWC2::PowerMode::Off,
                                                   true /* teardown */);
      if (status != HWC2::Error::None) {
        DLOGE("Power off for display = %d failed with error = %d", INT32(display), status);
      } else if (cwb_.IsCwbActiveOnDisplay(display)) {
        cwb_.PresentDisplayDone(display);
      }
    }
  }
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
    display < HWCCallbacks::kNumDisplays; display++) {
    if (hwc_display_[display] != NULL) {
      HWC2::PowerMode mode = last_power_mode[display];
      DLOGI("Setting display %d to mode = %d", INT32(display), mode);
      status = hwc_display_[display]->SetPowerMode(mode, false /* teardown */);
      if (status != HWC2::Error::None) {
        DLOGE("%d mode for display = %d failed with error = %d", mode, INT32(display), status);
      }
      ColorMode color_mode = hwc_display_[display]->GetCurrentColorMode();
      RenderIntent render_intent = hwc_display_[display]->GetCurrentRenderIntent();
      status = hwc_display_[display]->SetColorModeWithRenderIntent(color_mode, render_intent);
      if (status != HWC2::Error::None) {
        DLOGE("SetColorMode failed for display = %d error = %d", INT32(display), status);
      }
    }
  }

  hwc2_display_t vsync_source = callbacks_.GetVsyncSource();
  // adb shell stop sets vsync source as max display
  if (vsync_source != HWCCallbacks::kNumDisplays && hwc_display_[vsync_source]) {
    status = hwc_display_[vsync_source]->SetVsyncEnabled(HWC2::Vsync::Enable);
    if (status != HWC2::Error::None) {
      DLOGE("Enabling vsync failed for disp: %" PRIu64 " with error = %d", vsync_source, status);
    }
  }

  // Release lock on all displays.
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
    display < HWCCallbacks::kNumDisplays; display++) {
    locker_[display].Unlock();
  }

  callbacks_.Refresh(vsync_source);
}

void HWCSession::DisplayPowerReset() {
  // Do Power Reset in a different thread to avoid blocking of SDM event thread
  // when disconnecting display.
  std::thread(&HWCSession::PerformDisplayPowerReset, this).detach();
}

void HWCSession::VmReleaseDone(hwc2_display_t display) {
  SCOPE_LOCK(vm_release_locker_[display]);
  if (clients_waiting_for_vm_release_.test(display)) {
    vm_release_locker_[display].Signal();
    DLOGI("Signal vm release done!! for display %d", display);
    clients_waiting_for_vm_release_.reset(display);
  }
}

void HWCSession::HandleSecureSession() {
  std::bitset<kSecureMax> secure_sessions = 0;
  hwc2_display_t client_id = HWCCallbacks::kNumDisplays;
  {
    // TODO(user): Revisit if supporting secure display on non-primary.
    hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
    if (active_builtin_disp_id >= HWCCallbacks::kNumDisplays) {
      return;
    }
    Locker::ScopeLock lock_d(locker_[active_builtin_disp_id]);
    hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
  }

  if (secure_sessions[kSecureDisplay] || secure_sessions[kSecureCamera]) {
    secure_session_active_ = true;
  } else if (!secure_session_active_) {
    // No secure session active. No secure session transition to handle. Skip remaining steps.
    return;
  }

  // If there are any ongoing non-secure virtual displays, we need to destroy them.
  bool is_active_virtual_display = false;
  for (auto &map_info : map_info_virtual_) {
    if (map_info.disp_type == kVirtual) {
      is_active_virtual_display = true;
      client_id = map_info.client_id;
    }
  }
  if (is_active_virtual_display) {
    [[maybe_unused]] auto error = DestroyVirtualDisplay(client_id);
  }

  // If it is called during primary prepare/commit, we need to pause any ongoing commit on
  // external/virtual display.
  bool found_active_secure_display = false;
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
       display < HWCCallbacks::kNumRealDisplays; display++) {
    Locker::ScopeLock lock_d(locker_[display]);
    HWCDisplay *hwc_display = hwc_display_[display];
    if (!hwc_display) {
      continue;
    }

    bool is_active_secure_display = false;
    // The first On/Doze/DozeSuspend built-in display is taken as the secure display.
    if (!found_active_secure_display &&
        hwc_display->GetDisplayClass() == DISPLAY_CLASS_BUILTIN &&
        hwc_display->GetCurrentPowerMode() != HWC2::PowerMode::Off) {
      is_active_secure_display = true;
      found_active_secure_display = true;
    }
    hwc_display->HandleSecureSession(secure_sessions, &pending_power_mode_[display],
                                     is_active_secure_display);
  }
}

void HWCSession::HandlePendingPowerMode(hwc2_display_t disp_id,
                                        const shared_ptr<Fence> &retire_fence) {
  if (!secure_session_active_) {
    // No secure session active. Skip remaining steps.
    return;
  }

  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  if (disp_id != active_builtin_disp_id) {
    return;
  }

  Locker::ScopeLock lock_d(locker_[active_builtin_disp_id]);
  bool pending_power_mode = false;
  std::bitset<kSecureMax> secure_sessions = 0;
  hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY + 1;
    display < HWCCallbacks::kNumDisplays; display++) {
    if (display != active_builtin_disp_id) {
      Locker::ScopeLock lock_d(locker_[display]);
      if (pending_power_mode_[display]) {
        pending_power_mode = true;
        break;
      }
    }
  }

  if (!pending_power_mode) {
    if (!secure_sessions.any()) {
      secure_session_active_ = false;
    }
    return;
  }

  // retire fence is set only after successful primary commit, So check for retire fence to know
  // non secure commit went through to notify driver to change the CRTC mode to non secure.
  // Otherwise any commit to non-primary display would fail.
  if (retire_fence == nullptr) {
    return;
  }

  Fence::Wait(retire_fence);

  SCOPE_LOCK(pluggable_handler_lock_);
  HWDisplaysInfo hw_displays_info = {};
  DisplayError error = core_intf_->GetDisplaysStatus(&hw_displays_info);
  if (error != kErrorNone) {
    DLOGE("Failed to get connected display list. Error = %d", error);
    return;
  }

  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY + 1;
    display < HWCCallbacks::kNumDisplays; display++) {
    if (display == active_builtin_disp_id) {
      continue;
    }

    Locker::ScopeLock lock_d(locker_[display]);
    if (!pending_power_mode_[display] || !hwc_display_[display]) {
      continue;
    }

    // check if a pluggable display which is in pending power state is already disconnected.
    // In such cases, avoid powering up the display. It will be disconnected as part of
    // HandlePendingHotplug.
    bool disconnected = false;
    for (auto &map_info : map_info_pluggable_) {
      if (display != map_info.client_id) {
        continue;
      }

      for (auto &iter : hw_displays_info) {
        auto &info = iter.second;
        if (info.display_id != map_info.sdm_id) {
          continue;
        }
        if (!info.is_connected) {
          disconnected = true;
        }
        break;
      }
      break;
    }

    if (disconnected) {
      continue;
    }

    HWC2::PowerMode pending_mode = hwc_display_[display]->GetPendingPowerMode();

    if (pending_mode == HWC2::PowerMode::Off || pending_mode == HWC2::PowerMode::DozeSuspend) {
      active_displays_.erase(display);
    } else {
      active_displays_.insert(display);
    }
    HWC2::Error error = hwc_display_[display]->SetPowerMode(pending_mode, false);
    if (HWC2::Error::None == error) {
      pending_power_mode_[display] = false;
      hwc_display_[display]->ClearPendingPowerMode();
      pending_refresh_.set(UINT32(HWC_DISPLAY_PRIMARY));
    } else {
      DLOGE("SetDisplayStatus error = %d (%s)", error, to_string(error).c_str());
    }
  }

  secure_session_active_ = false;
}

void HWCSession::HandlePendingHotplug(hwc2_display_t disp_id,
                                      const shared_ptr<Fence> &retire_fence) {
  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  if (disp_id != active_builtin_disp_id ||
      (kHotPlugNone == pending_hotplug_event_)) {
    return;
  }

  std :: bitset < kSecureMax > secure_sessions = 0;
  if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
    Locker::ScopeLock lock_d(locker_[active_builtin_disp_id]);
    hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
  }

  if (secure_sessions.any() || active_builtin_disp_id >= HWCCallbacks::kNumDisplays) {
    return;
  }

  if (kHotPlugEvent == pending_hotplug_event_) {
    Fence::Wait(retire_fence);

    // Handle connect/disconnect hotplugs if secure session is not present.
    hwc2_display_t virtual_display_idx = (hwc2_display_t)GetDisplayIndex(qdutils::DISPLAY_VIRTUAL);
    if (!hwc_display_[virtual_display_idx] && kHotPlugEvent == pending_hotplug_event_) {
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

int32_t HWCSession::GetReadbackBufferAttributes(hwc2_display_t display, int32_t *format,
                                                int32_t *dataspace) {
  if (!format || !dataspace) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (display != HWC_DISPLAY_PRIMARY) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  HWCDisplay *hwc_display = hwc_display_[display];
  if (hwc_display == nullptr) {
    return HWC2_ERROR_BAD_DISPLAY;
  } else if (!hwc_display->HasReadBackBufferSupport()) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  *format = HAL_PIXEL_FORMAT_RGB_888;
  *dataspace = GetDataspaceFromColorMode(hwc_display->GetCurrentColorMode());

  return HWC2_ERROR_NONE;
}

int32_t HWCSession::SetReadbackBuffer(hwc2_display_t display, const native_handle_t *buffer,
                                      const shared_ptr<Fence> &acquire_fence) {
  if (!buffer) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (display != HWC_DISPLAY_PRIMARY) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  int virtual_dpy_index = GetDisplayIndex(qdutils::DISPLAY_VIRTUAL);
  if ((virtual_dpy_index != -1) && hwc_display_[virtual_dpy_index]) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  CwbConfig cwb_config = {}; /* SF uses LM tappoint*/

  return CallDisplayFunction(display, &HWCDisplay::SetReadbackBuffer, buffer, acquire_fence,
                             cwb_config, kCWBClientComposer);
}

int32_t HWCSession::GetReadbackBufferFence(hwc2_display_t display,
                                           shared_ptr<Fence> *release_fence) {
  if (!release_fence) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (display != HWC_DISPLAY_PRIMARY) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  return CallDisplayFunction(display, &HWCDisplay::GetReadbackBufferFence, release_fence);
}

int32_t HWCSession::GetDisplayIdentificationData(hwc2_display_t display, uint8_t *outPort,
                                                 uint32_t *outDataSize, uint8_t *outData) {
  if (!outPort || !outDataSize) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  return CallDisplayFunction(display, &HWCDisplay::GetDisplayIdentificationData, outPort,
                             outDataSize, outData);
}

int32_t HWCSession::GetDisplayCapabilities(hwc2_display_t display,
                                           hidl_vec<HwcDisplayCapability> *capabilities) {
  if (!capabilities) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!hwc_display_[display]) {
    DLOGE("Expected valid hwc_display");
    return HWC2_ERROR_BAD_PARAMETER;
  }

  bool isBuiltin = (hwc_display_[display]->GetDisplayClass() == DISPLAY_CLASS_BUILTIN);
  if (isBuiltin) {
    int32_t has_doze_support = 0;
    GetDozeSupport(display, &has_doze_support);

    // TODO(user): Handle SKIP_CLIENT_COLOR_TRANSFORM based on DSPP availability
    if (has_doze_support) {
      *capabilities = {HwcDisplayCapability::SKIP_CLIENT_COLOR_TRANSFORM,
                       HwcDisplayCapability::DOZE,
                       HwcDisplayCapability::BRIGHTNESS,
                       HwcDisplayCapability::PROTECTED_CONTENTS};
    } else {
      *capabilities = {HwcDisplayCapability::SKIP_CLIENT_COLOR_TRANSFORM,
                       HwcDisplayCapability::BRIGHTNESS,
                       HwcDisplayCapability::PROTECTED_CONTENTS};
    }
  }

  return HWC2_ERROR_NONE;
}

int32_t HWCSession::GetDisplayConnectionType(hwc2_display_t display,
                                             HwcDisplayConnectionType *type) {
  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!type) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (!hwc_display_[display]) {
    DLOGW("Expected valid hwc_display");
    return HWC2_ERROR_BAD_DISPLAY;
  }
  *type = HwcDisplayConnectionType::EXTERNAL;
  if (display == HWC_DISPLAY_PRIMARY ||
      hwc_display_[display]->GetDisplayClass() == DISPLAY_CLASS_BUILTIN) {
    *type = HwcDisplayConnectionType::INTERNAL;
  }

  return HWC2_ERROR_NONE;
}

int32_t HWCSession::GetClientTargetProperty(hwc2_display_t display,
                                            HwcClientTargetProperty *outClientTargetProperty) {
  if (!outClientTargetProperty) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  return CallDisplayFunction(display, &HWCDisplay::GetClientTargetProperty,
                             outClientTargetProperty);
}

int32_t HWCSession::GetDisplayBrightnessSupport(hwc2_display_t display, bool *outSupport) {
  if (!outSupport) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!hwc_display_[display]) {
    DLOGE("Expected valid hwc_display");
    return HWC2_ERROR_BAD_PARAMETER;
  }
  *outSupport = (hwc_display_[display]->GetDisplayClass() == DISPLAY_CLASS_BUILTIN);
  return HWC2_ERROR_NONE;
}

int32_t HWCSession::SetDisplayBrightness(hwc2_display_t display, float brightness) {
  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!hwc_display_[display]) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  return INT32(hwc_display_[display]->SetPanelBrightness(brightness));
}

android::status_t HWCSession::SetQSyncMode(const android::Parcel *input_parcel) {
  auto mode = input_parcel->readInt32();

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
  hwc_display_qsync_[HWC_DISPLAY_PRIMARY] = qsync_mode;
  return CallDisplayFunction(HWC_DISPLAY_PRIMARY, &HWCDisplay::SetQSyncMode, qsync_mode);
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

  return static_cast<android::status_t>(ControlIdlePowerCollapse(enable, synchronous));
}

hwc2_display_t HWCSession::GetActiveBuiltinDisplay() {
  hwc2_display_t active_display = HWCCallbacks::kNumDisplays;
  // Get first active display among primary and built-in displays.
  std::vector<DisplayMapInfo> map_info = {map_info_primary_};
  std::copy(map_info_builtin_.begin(), map_info_builtin_.end(), std::back_inserter(map_info));

  for (auto &info : map_info) {
    hwc2_display_t target_display = info.client_id;
    Locker::ScopeLock lock_d(locker_[target_display]);
    auto &hwc_display = hwc_display_[target_display];
    if (hwc_display && hwc_display->GetCurrentPowerMode() != HWC2::PowerMode::Off) {
      active_display = info.client_id;
      break;
    }
  }

  return active_display;
}

int32_t HWCSession::SetDisplayBrightnessScale(const android::Parcel *input_parcel) {
  auto display = input_parcel->readInt32();
  auto level = input_parcel->readInt32();

  if (level < 0) {
    DLOGE("Invalid backlight scale level %d", level);
    return -EINVAL;
  }

  // DPPS DRE case
  int32_t dre_case = 0;
  if (input_parcel->dataPosition() != input_parcel->dataSize()) {
    dre_case = input_parcel->readInt32();
  }

  // Non-Dre case to check max backlight scale
  if (!dre_case && level > kBrightnessScaleMax) {
    DLOGE("Invalid backlight scale level %d, max scale %d, dre_case %d",
      level, kBrightnessScaleMax, dre_case);
    return -EINVAL;
  }

  auto bl_scale = level * kSvBlScaleMax / kBrightnessScaleMax;
  auto error = CallDisplayFunction(display, &HWCDisplay::SetBLScale, (uint32_t)bl_scale);
  if (INT32(error) == HWC2_ERROR_NONE) {
    callbacks_.Refresh(display);
  }

  return INT32(error);
}

void HWCSession::NotifyClientStatus(bool connected) {
  for (uint32_t i = 0; i < HWCCallbacks::kNumDisplays; i++) {
    if (!hwc_display_[i]) {
      continue;
    }
    SCOPE_LOCK(locker_[i]);
    hwc_display_[i]->NotifyClientStatus(connected);
    hwc_display_[i]->SetVsyncEnabled(HWC2::Vsync::Disable);
  }
  callbacks_.UpdateVsyncSource(HWCCallbacks::kNumDisplays);
}

int HWCSession::WaitForResources(bool wait_for_resources, hwc2_display_t active_builtin_id,
                                  hwc2_display_t display_id) {
  std::vector<DisplayMapInfo> map_info = {map_info_primary_};
  std::copy(map_info_builtin_.begin(), map_info_builtin_.end(), std::back_inserter(map_info));

  if (wait_for_resources) {
    bool res_wait = true;
    bool needs_active_builtin_reconfig = false;
    if (enable_primary_reconfig_req_) {
      // todo (user): move this logic to wait for MDP resource reallocation/reconfiguration
      // to SDM module.
      {
        SCOPE_LOCK(locker_[display_id]);
        if (hwc_display_[display_id]) {
          res_wait = hwc_display_[display_id]->CheckResourceState(&needs_active_builtin_reconfig);
        }
        else {
          DLOGW("Display %" PRIu64 "no longer available.", display_id);
          return HWC2_ERROR_BAD_DISPLAY;
        }
      }
      if (needs_active_builtin_reconfig) {
        SCOPE_LOCK(locker_[active_builtin_id]);
        if (hwc_display_[active_builtin_id]) {
          hwc2_config_t current_config = 0, new_config = 0;
          hwc_display_[active_builtin_id]->GetActiveConfig(&current_config);
          int status = INT32(hwc_display_[active_builtin_id]->SetAlternateDisplayConfig(true));
          if (status) {
            DLOGE("Active built-in %" PRIu64 " cannot switch to lower resource configuration",
                  active_builtin_id);
            return status;
          }
          hwc_display_[active_builtin_id]->GetActiveConfig(&new_config);

          // In case of config change, notify client with the new configuration
          if (new_config != current_config) {
            NotifyDisplayAttributes(active_builtin_id, new_config);
          }
        }
        else {
          DLOGW("Display %" PRIu64 "no longer available.", active_builtin_id);
          return HWC2_ERROR_BAD_DISPLAY;
        }
      }
    }
    do {
      if (client_connected_) {
        Refresh(active_builtin_id);
      }
      {
        std::unique_lock<std::mutex> caller_lock(hotplug_mutex_);
        resource_ready_ = false;

        const uint32_t min_vsync_period_ms = 100;
        auto timeout = std::chrono::system_clock::now() +
                         std::chrono::milliseconds(min_vsync_period_ms);
        if (hotplug_cv_.wait_until(caller_lock, timeout) == std::cv_status::timeout) {
          DLOGW("hotplug timeout");
        }

        if (active_display_id_ == active_builtin_id && needs_active_builtin_reconfig &&
            cached_retire_fence_) {
          Fence::Wait(cached_retire_fence_);
        }
        cached_retire_fence_ = nullptr;
      }
      {
        SCOPE_LOCK(locker_[display_id]);
        if (hwc_display_[display_id]) {
          res_wait = hwc_display_[display_id]->CheckResourceState(&needs_active_builtin_reconfig);
          if (!enable_primary_reconfig_req_) {
            needs_active_builtin_reconfig = false;
          }
        }
        else {
          DLOGW("Display %" PRIu64 "no longer available.", display_id);
          return HWC2_ERROR_BAD_DISPLAY;
        }
      }
    } while (res_wait || needs_active_builtin_reconfig);
  }

  return 0;
}

int32_t HWCSession::GetDisplayVsyncPeriod(hwc2_display_t disp, VsyncPeriodNanos *vsync_period) {
  if (vsync_period == nullptr) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  return CallDisplayFunction(disp, &HWCDisplay::GetDisplayVsyncPeriod, vsync_period);
}

int32_t HWCSession::SetActiveConfigWithConstraints(
    hwc2_display_t display, hwc2_config_t config,
    const VsyncPeriodChangeConstraints *vsync_period_change_constraints,
    VsyncPeriodChangeTimeline *out_timeline) {
  if ((vsync_period_change_constraints == nullptr) || (out_timeline == nullptr)) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  return CallDisplayFunction(display, &HWCDisplay::SetActiveConfigWithConstraints, config,
                             vsync_period_change_constraints, out_timeline);
}

int HWCSession::WaitForCommitDoneAsync(hwc2_display_t display, int client_id) {
  std::chrono::milliseconds span(5000);
  auto &future = client_id == kClientVirtualDisplay ? wfd_refresh_future_ : commit_done_future_[display];
  if (future.valid()) {
    if (client_id == kClientVirtualDisplay) {
      return 0;	
    }
    std::future_status status = future.wait_for(std::chrono::milliseconds(0));
    if (status != std::future_status::ready) {
      // Previous task is stuck. Bail out early.
      return -ETIMEDOUT;
    }
  }

  future = std::async([](HWCSession* session, hwc2_display_t display, int client_id) {
                         return session->WaitForCommitDone(display, client_id);
                         }, this, display, client_id);
  if (future.wait_for(span) == std::future_status::timeout) {
    DLOGW("WaitForCommitDoneAsync timed out");
    return -ETIMEDOUT;
  }
  return future.get();
}

int HWCSession::WaitForCommitDone(hwc2_display_t display, int client_id) {
  shared_ptr<Fence> retire_fence = nullptr;
  int timeout_ms = -1;
  {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[display]);
    callbacks_.Refresh(display);
    clients_waiting_for_commit_[display].set(client_id);
    locker_[display].Wait();
    if (commit_error_[display] != 0) {
      DLOGE("Commit done failed with error %d for client %d display %" PRIu64,
            commit_error_[display], client_id, display);
      commit_error_[display] = 0;
      return -EINVAL;
    }
    retire_fence = retire_fence_[display];
    retire_fence_[display] = nullptr;
    if (hwc_display_[display]) {
      uint32_t config = 0;
      hwc_display_[display]->GetActiveDisplayConfig(&config);
      DisplayConfigVariableInfo display_attributes = {};
      hwc_display_[display]->GetDisplayAttributesForConfig(config, &display_attributes);
      timeout_ms = kNumDrawCycles * (display_attributes.vsync_period_ns / kDenomNstoMs);
      DLOGI("timeout in ms %d", timeout_ms);
    }
  }

  int ret = Fence::Wait(retire_fence, timeout_ms + kCommitDoneTimeoutMs);
  if (ret != 0) {
    DLOGE("Retire fence wait failed with error %d for client %d display %" PRIu64, ret,
          client_id, display);
  }
  return ret;
}

int HWCSession::WaitForVmRelease(hwc2_display_t display, int timeout_ms) {
  SCOPE_LOCK(vm_release_locker_[display]);
  clients_waiting_for_vm_release_.set(display);
  int re_try = kVmReleaseRetry;
  int ret = 0;
  do {
    ret = vm_release_locker_[display].WaitFinite(timeout_ms + kVmReleaseTimeoutMs);
    if (!ret) {
      break;
    }
  } while(re_try--);
  if (ret != 0) {
    DLOGE("Timed out with error %d for display %" PRIu64, ret, display);
  }
  return ret;
}

android::status_t HWCSession::HandleTUITransition(int disp_id, int event) {
  switch(event) {
    case qService::IQService::TUI_TRANSITION_PREPARE:
      return TUITransitionPrepare(disp_id);
    case qService::IQService::TUI_TRANSITION_START:
      return TUITransitionStart(disp_id);
    case qService::IQService::TUI_TRANSITION_END:
      return TUITransitionEnd(disp_id);
    default:
      DLOGE("Invalid event %d", event);
      return -EINVAL;
  }
}

android::status_t HWCSession::TUITransitionPrepare(int disp_id) {
  // Hold this lock to until on going hotplug handling is complete before we start TUI session
  SCOPE_LOCK(pluggable_handler_lock_);

  bool needs_refresh = false;
  hwc2_display_t target_display = GetDisplayIndex(disp_id);
  if (target_display == -1) {
    target_display = GetActiveBuiltinDisplay();
  }

  if (target_display != qdutils::DISPLAY_PRIMARY && target_display != qdutils::DISPLAY_BUILTIN_2) {
    DLOGE("Display %" PRIu64 " not supported", target_display);
    return -ENOTSUP;
  }

  std::vector<DisplayMapInfo> map_info = {map_info_primary_};
  std::copy(map_info_builtin_.begin(), map_info_builtin_.end(), std::back_inserter(map_info));
  std::copy(map_info_pluggable_.begin(), map_info_pluggable_.end(), std::back_inserter(map_info));
  std::copy(map_info_virtual_.begin(), map_info_virtual_.end(), std::back_inserter(map_info));

  for (auto &info : map_info) {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[info.client_id]);
    if (hwc_display_[info.client_id]) {
      if (hwc_display_[info.client_id]->HandleSecureEvent(kTUITransitionPrepare, &needs_refresh,
                                                           info.client_id == target_display) !=
                                                           kErrorNone) {
        return -EINVAL;
      }
    }
  }

  return 0;
}

android::status_t HWCSession::TUITransitionStart(int disp_id) {
  if (TUITransitionPrepare(disp_id) != 0) {
    return -EINVAL;
  }

  hwc2_display_t target_display = GetDisplayIndex(disp_id);
  bool needs_refresh = false;
  if (target_display == -1) {
    target_display = GetActiveBuiltinDisplay();
  }

  if (target_display != qdutils::DISPLAY_PRIMARY && target_display != qdutils::DISPLAY_BUILTIN_2) {
    DLOGE("Display %" PRIu64 " not supported", target_display);
    return -ENOTSUP;
  }

  HWC2::Error error = TeardownConcurrentWriteback(target_display);
  if (error != HWC2::Error::None) {
    return -ENODEV;
  }

  {
    // disable idle time out for video mode
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[target_display]);
    hwc_display_[target_display]->SetIdleTimeoutMs(0, 0);

    // disable qsync
    hwc_display_[target_display]->SetQSyncMode(kQSyncModeNone);
  }

  int ret = WaitForCommitDoneAsync(target_display, kClientTrustedUI);
  if (ret != 0) {
    if (ret != -ETIMEDOUT) {
      DLOGE("WaitForCommitDone failed with error = %d", ret);
    }
    return -EINVAL;
  }

  int timeout_ms = -1;
  {
    std::lock_guard<std::mutex> command_seq_lock(command_seq_mutex_);
    SCOPE_LOCK(locker_[target_display]);
    if (hwc_display_[target_display]) {
      if (hwc_display_[target_display]->HandleSecureEvent(kTUITransitionStart, &needs_refresh,
                                                          false) != kErrorNone) {
        return -EINVAL;
      }
      uint32_t config = 0;
      hwc_display_[target_display]->GetActiveDisplayConfig(&config);
      DisplayConfigVariableInfo display_attributes = {};
      hwc_display_[target_display]->GetDisplayAttributesForConfig(config, &display_attributes);
      timeout_ms = kNumDrawCycles * (display_attributes.vsync_period_ns / kDenomNstoMs);
      DLOGI("timeout in ms %d", timeout_ms);
    } else {
      DLOGW("Target display %d is not ready", disp_id);
      return -ENODEV;
    }
  }

  if (needs_refresh) {
    callbacks_.Refresh(target_display);

    DLOGI("Waiting for device assign");
    int ret = WaitForVmRelease(target_display, timeout_ms);
    if (ret != 0) {
      DLOGE("Device assign failed with error %d", ret);
      return -EINVAL;
    }
  }

  {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[target_display]);
    if (hwc_display_[target_display]) {
      if (hwc_display_[target_display]->PostHandleSecureEvent(kTUITransitionStart) != kErrorNone) {
        return -EINVAL;
      }
    } else {
      DLOGW("Target display %d is not ready", disp_id);
      return -ENODEV;
    }
  }

  tui_start_success_ = true;
  return 0;
}
android::status_t HWCSession::TUITransitionEnd(int disp_id) {
  // Hold this lock so that any deferred hotplug events will not be handled during the commit
  // and will be handled at the end of TUITransitionPrepare.
  if (!tui_start_success_) {
    DLOGI("Bailing out TUI end");
    return -EINVAL;
  }
  SCOPE_LOCK(pluggable_handler_lock_);
  hwc2_display_t target_display = GetDisplayIndex(disp_id);
  bool needs_refresh = false;
  if (target_display == -1) {
    target_display = GetActiveBuiltinDisplay();
  }

  if (target_display != qdutils::DISPLAY_PRIMARY && target_display != qdutils::DISPLAY_BUILTIN_2) {
    DLOGE("Display %" PRIu64 " not supported", target_display);
    return -ENOTSUP;
  }

  {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[target_display]);
    hwc_display_[target_display]->SetIdleTimeoutMs(idle_time_active_ms_, idle_time_inactive_ms_);
    hwc_display_[target_display]->SetQSyncMode(hwc_display_qsync_[target_display]);
    if (hwc_display_[target_display]) {
      if (hwc_display_[target_display]->HandleSecureEvent(kTUITransitionEnd, &needs_refresh,
                                                          false) != kErrorNone) {
        return -EINVAL;
      }
    } else {
      DLOGW("Target display %d is not ready", disp_id);
      return -ENODEV;
    }
  }

  if (needs_refresh) {
    DLOGI("Waiting for device unassign");
    int ret = WaitForCommitDoneAsync(target_display, kClientTrustedUI);
    if (ret != 0) {
      if (ret != -ETIMEDOUT) {
        DLOGE("Device unassign failed with error %d", ret);
      }
      TUITransitionUnPrepare(disp_id);
      tui_start_success_ = false;
      return -EINVAL;
    }
  }

  {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[target_display]);
    if (hwc_display_[target_display]) {
      if (hwc_display_[target_display]->PostHandleSecureEvent(kTUITransitionEnd) != kErrorNone) {
        return -EINVAL;
      }
    } else {
      DLOGW("Target display %d is not ready", disp_id);
      return -ENODEV;
    }
  }

  return TUITransitionUnPrepare(disp_id);
}

android::status_t HWCSession::TUITransitionUnPrepare(int disp_id) {
  bool trigger_refresh = false;
  hwc2_display_t target_display = GetDisplayIndex(disp_id);
  if (target_display == -1) {
    target_display = GetActiveBuiltinDisplay();
  }

  if (target_display != qdutils::DISPLAY_PRIMARY && target_display != qdutils::DISPLAY_BUILTIN_2) {
    DLOGE("Display %" PRIu64 " not supported", target_display);
    return -ENOTSUP;
  }

  std::vector<DisplayMapInfo> map_info = {map_info_primary_};
  std::copy(map_info_builtin_.begin(), map_info_builtin_.end(), std::back_inserter(map_info));
  std::copy(map_info_pluggable_.begin(), map_info_pluggable_.end(), std::back_inserter(map_info));
  std::copy(map_info_virtual_.begin(), map_info_virtual_.end(), std::back_inserter(map_info));

  for (auto &info : map_info) {
    bool needs_refresh = false;
    {
      SEQUENCE_WAIT_SCOPE_LOCK(locker_[info.client_id]);
      if (hwc_display_[info.client_id]) {
        if (info.disp_type == kPluggable && pending_hotplug_event_ == kHotPlugEvent) {
          continue;
        }
        if (hwc_display_[info.client_id]->HandleSecureEvent(kTUITransitionUnPrepare, &needs_refresh,
                                                            info.client_id == target_display) !=
                                                            kErrorNone) {
          return -EINVAL;
        }
      }
      trigger_refresh |= needs_refresh;
    }
  }
  if (trigger_refresh) {
    for (int i = 0; i < 2; i++) {
      int ret = WaitForCommitDoneAsync(target_display, kClientTrustedUI);
      if (ret != 0) {
        if (ret != -ETIMEDOUT) {
          DLOGE("WaitForCommitDone failed with error %d", ret);
        }
        tui_start_success_ = false;
        return -EINVAL;
      }
    }
  }

  if (pending_hotplug_event_ == kHotPlugEvent) {
    // Do hotplug handling in a different thread to avoid blocking TUI thread.
    std::thread(&HWCSession::HandlePluggableDisplays, this, true).detach();
  }
  // Reset tui session state variable.
  tui_start_success_ = false;
  DLOGI("End of TUI session on display %d", disp_id);
  return 0;
}

DispType HWCSession::GetDisplayConfigDisplayType(int qdutils_disp_type) {
  switch (qdutils_disp_type) {
    case qdutils::DISPLAY_PRIMARY:
      return DispType::kPrimary;

    case qdutils::DISPLAY_EXTERNAL:
      return DispType::kExternal;

    case qdutils::DISPLAY_VIRTUAL:
      return DispType::kVirtual;

    case qdutils::DISPLAY_BUILTIN_2:
      return DispType::kBuiltIn2;

    default:
      return DispType::kInvalid;
  }
}

int HWCSession::GetDispTypeFromPhysicalId(uint64_t physical_disp_id, DispType *disp_type) {
  // TODO(user): Least significant 8 bit is port id based on the SF current implementaion. Need to
  // revisit this if there is a change in logic to create physical display id in SF.
  int port_id = (physical_disp_id & 0xFF);
  int out_port = 0;
  for (int dpy = qdutils::DISPLAY_PRIMARY; dpy <= qdutils::DISPLAY_EXTERNAL_2; dpy++) {
    int ret = GetDisplayPortId(dpy, &out_port);
    if (ret != 0){
      return ret;
    }
    if (port_id == out_port) {
      *disp_type = GetDisplayConfigDisplayType(dpy);
      return 0;
    }
  }
  return -ENODEV;
}

android::status_t HWCSession::GetDisplayPortId(uint32_t disp_id, int *port_id) {
  hwc2_display_t target_display = GetDisplayIndex(disp_id);
  if (target_display == -1) {
    return -ENOTSUP;
  }
  uint8_t out_port = 0;
  uint32_t out_data_size = 0;
  Locker::ScopeLock lock_d(locker_[target_display]);
  if (hwc_display_[target_display]) {
    if (hwc_display_[target_display]->GetDisplayIdentificationData(&out_port, &out_data_size,
                                                                   NULL) == HWC2::Error::None) {
      *port_id = INT(out_port);
    }
  }
  return 0;
}

HWC2::Error HWCSession::TeardownConcurrentWriteback(hwc2_display_t display) {
  bool needs_refresh = false;
  {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[display]);
    if (hwc_display_[display]) {
      DisplayError error = hwc_display_[display]->TeardownConcurrentWriteback(&needs_refresh);
      if (error != kErrorNone) {
        return HWC2::Error::BadParameter;
      }
    }
  }
  if (!needs_refresh) {
    return HWC2::Error::None;
  }

  // Wait until concurrent WB teardown is complete
  int error = WaitForCommitDone(display, kClientTeardownCWB);
  if (error != 0) {
    DLOGE("concurrent WB teardown failed with error %d", error);
    return HWC2::Error::NoResources;
  }
  return HWC2::Error::None;
}

HWC2::Error HWCSession::CommitOrPrepare(hwc2_display_t display, bool validate_only,
                                        shared_ptr<Fence> *out_retire_fence,
                                        uint32_t *out_num_types, uint32_t *out_num_requests,
                                        bool *needs_commit) {
  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2::Error::BadDisplay;
  }

  {
    // ToDo: add support for async power mode.
    Locker::ScopeLock lock_d(locker_[display]);
    if (!hwc_display_[display]) {
      return HWC2::Error::BadDisplay;
    }
    if (pending_power_mode_[display]) {
      return HWC2::Error::None;
    }
  }

  HandleSecureSession();
  auto status = HWC2::Error::None;
  {
    SEQUENCE_ENTRY_SCOPE_LOCK(locker_[display]);
    hwc_display_[display]->ProcessActiveConfigChange();
    hwc_display_[display]->IsMultiDisplay((active_displays_.size() > 1) ? true : false);
    // Check if hwc's refresh trigger is getting exercised.
    if (callbacks_.NeedsRefresh(display)) {
      hwc_display_[display]->SetPendingRefresh();
      callbacks_.ResetRefresh(display);
    }
    status = hwc_display_[display]->CommitOrPrepare(validate_only, out_retire_fence, out_num_types,
                                                    out_num_requests, needs_commit);
  }
  if(!(*needs_commit)) {
    {
      SEQUENCE_EXIT_SCOPE_LOCK(locker_[display]);
      PostCommitLocked(display, *out_retire_fence);
    }
    PostCommitUnlocked(display, *out_retire_fence, status);
  }

  return status;
}

HWC2::Error HWCSession::TryDrawMethod(hwc2_display_t display,
                                      IQtiComposerClient::DrawMethod drawMethod) {
  Locker::ScopeLock lock_d(locker_[display]);
  if (!hwc_display_[display]) {
    return HWC2::Error::BadDisplay;
  }

  return hwc_display_[display]->TryDrawMethod(drawMethod);
}

void HWCSession::NotifyDisplayAttributes(hwc2_display_t display, hwc2_config_t config) {
  DisplayConfigVariableInfo var_info;
  Attributes attributes;
  int error = hwc_display_[display]->GetDisplayAttributesForConfig(INT(config), &var_info);
  if (!error) {
    attributes.vsyncPeriod = var_info.vsync_period_ns;
    attributes.xRes = var_info.x_pixels;
    attributes.yRes = var_info.y_pixels;
    attributes.xDpi = var_info.x_dpi;
    attributes.yDpi = var_info.y_dpi;
    attributes.panelType = DisplayPortType::DEFAULT;
    attributes.isYuv = var_info.is_yuv;
    NotifyResolutionChange(display, attributes);
  }
}

DisplayError HWCSession::WaitForPrimaryHotplug(HWDisplayInterfaceInfo *hw_disp_info) {
  int wait_for_hotplug = 0;  // Default value when property is not present.
  HWCDebugHandler::Get()->GetProperty(WAIT_FOR_PRIMARY_DISPLAY, &wait_for_hotplug);
  DLOGI("wait_for_primary_display :%d", wait_for_hotplug);

  const uint32_t kMaxAttempts = wait_for_hotplug ? 60 : 1;
  uint32_t attempts;
  for (attempts = 0; attempts < kMaxAttempts; attempts++) {
    DisplayError error = core_intf_->GetFirstDisplayInterfaceType(hw_disp_info);
    if (error != kErrorNone) {
      return error;
    }
    if (hw_disp_info->is_connected) {
      DLOGI("Primary display of type-%d is connected after %u %s",
            hw_disp_info->type, (attempts + 1), attempts ? "attempts" : "attempt");
      break;
    }
    usleep(1000000);    // sleep for 1 second
  }

  if (attempts == kMaxAttempts) {
    LOG_ALWAYS_FATAL("Primary display of type-%d is not yet connected after %u %s",
                     hw_disp_info->type, kMaxAttempts, kMaxAttempts == 1 ? "attempt" : "attempts");
  }

  return kErrorNone;
}

void HWCSession::HandlePluggablePrimaryDisplay(HWDisplaysInfo *hw_displays_info) {
  for (auto &iter : *hw_displays_info) {
    auto &info = iter.second;
    if (info.is_primary && (info.display_type == kPluggable) && !info.is_connected) {
      LOG_ALWAYS_FATAL("Primary pluggable display is disconnected");
    }
  }
}
}  // namespace sdm
