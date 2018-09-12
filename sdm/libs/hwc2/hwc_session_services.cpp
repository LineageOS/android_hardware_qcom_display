/*
* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation. nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <core/buffer_allocator.h>
#include <utils/debug.h>
#include <sync/sync.h>
#include <vector>

#include "hwc_buffer_sync_handler.h"
#include "hwc_session.h"

#define __CLASS__ "HWCSession"

namespace sdm {

using ::android::hardware::Void;

void HWCSession::StartServices() {
  android::status_t status = IDisplayConfig::registerAsService();
  if (status != android::OK) {
    DLOGW("Could not register IDisplayConfig as service (%d).", status);
  } else {
    DLOGI("IDisplayConfig service registration completed.");
  }
}

int MapDisplayType(IDisplayConfig::DisplayType dpy) {
  switch (dpy) {
    case IDisplayConfig::DisplayType::DISPLAY_PRIMARY:
      return qdutils::DISPLAY_PRIMARY;

    case IDisplayConfig::DisplayType::DISPLAY_EXTERNAL:
      return qdutils::DISPLAY_EXTERNAL;

    case IDisplayConfig::DisplayType::DISPLAY_VIRTUAL:
      return qdutils::DISPLAY_VIRTUAL;

    default:
      break;
  }

  return -EINVAL;
}

HWCDisplay::DisplayStatus MapExternalStatus(IDisplayConfig::DisplayExternalStatus status) {
  switch (status) {
    case IDisplayConfig::DisplayExternalStatus::EXTERNAL_OFFLINE:
      return HWCDisplay::kDisplayStatusOffline;

    case IDisplayConfig::DisplayExternalStatus::EXTERNAL_ONLINE:
      return HWCDisplay::kDisplayStatusOnline;

    case IDisplayConfig::DisplayExternalStatus::EXTERNAL_PAUSE:
      return HWCDisplay::kDisplayStatusPause;

    case IDisplayConfig::DisplayExternalStatus::EXTERNAL_RESUME:
      return HWCDisplay::kDisplayStatusResume;

    default:
      break;
  }

  return HWCDisplay::kDisplayStatusInvalid;
}

// Methods from ::vendor::hardware::display::config::V1_0::IDisplayConfig follow.
Return<void> HWCSession::isDisplayConnected(IDisplayConfig::DisplayType dpy,
                                            isDisplayConnected_cb _hidl_cb) {
  int32_t error = -EINVAL;
  bool connected = false;
  int disp_id = MapDisplayType(dpy);
  int disp_idx = GetDisplayIndex(disp_id);

  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_id);
  } else {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
    connected = hwc_display_[disp_idx];
    error = 0;
  }
  _hidl_cb(error, connected);

  return Void();
}

int32_t HWCSession::SetSecondaryDisplayStatus(int disp_id, HWCDisplay::DisplayStatus status) {
  int disp_idx = GetDisplayIndex(disp_id);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_id);
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
  DLOGI("Display = %d, Status = %d", disp_idx, status);

  if (disp_idx == qdutils::DISPLAY_PRIMARY) {
    DLOGE("Not supported for this display");
  } else if (!hwc_display_[disp_idx]) {
    DLOGW("Display is not connected");
  } else {
    return hwc_display_[disp_idx]->SetDisplayStatus(status);
  }

  return -EINVAL;
}

Return<int32_t> HWCSession::setSecondayDisplayStatus(IDisplayConfig::DisplayType dpy,
                                                  IDisplayConfig::DisplayExternalStatus status) {
  return SetSecondaryDisplayStatus(MapDisplayType(dpy), MapExternalStatus(status));
}

Return<int32_t> HWCSession::configureDynRefeshRate(IDisplayConfig::DisplayDynRefreshRateOp op,
                                                   uint32_t refreshRate) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);
  HWCDisplay *hwc_display = hwc_display_[HWC_DISPLAY_PRIMARY];

  if (!hwc_display) {
    DLOGW("Display = %d is not connected.", HWC_DISPLAY_PRIMARY);
    return -EINVAL;
  }

  switch (op) {
    case IDisplayConfig::DisplayDynRefreshRateOp::DISABLE_METADATA_DYN_REFRESH_RATE:
      return hwc_display->Perform(HWCDisplayBuiltIn::SET_METADATA_DYN_REFRESH_RATE, false);

    case IDisplayConfig::DisplayDynRefreshRateOp::ENABLE_METADATA_DYN_REFRESH_RATE:
      return hwc_display->Perform(HWCDisplayBuiltIn::SET_METADATA_DYN_REFRESH_RATE, true);

    case IDisplayConfig::DisplayDynRefreshRateOp::SET_BINDER_DYN_REFRESH_RATE:
      return hwc_display->Perform(HWCDisplayBuiltIn::SET_BINDER_DYN_REFRESH_RATE, refreshRate);

    default:
      DLOGW("Invalid operation %d", op);
      return -EINVAL;
  }

  return 0;
}

int32_t HWCSession::GetConfigCount(int disp_id, uint32_t *count) {
  int disp_idx = GetDisplayIndex(disp_id);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_id);
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);

  if (hwc_display_[disp_idx]) {
    return hwc_display_[disp_idx]->GetDisplayConfigCount(count);
  }

  return -EINVAL;
}

Return<void> HWCSession::getConfigCount(IDisplayConfig::DisplayType dpy,
                                        getConfigCount_cb _hidl_cb) {
  uint32_t count = 0;
  int32_t error = GetConfigCount(MapDisplayType(dpy), &count);

  _hidl_cb(error, count);

  return Void();
}

int32_t HWCSession::GetActiveConfigIndex(int disp_id, uint32_t *config) {
  int disp_idx = GetDisplayIndex(disp_id);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_id);
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);

  if (hwc_display_[disp_idx]) {
    return hwc_display_[disp_idx]->GetActiveDisplayConfig(config);
  }

  return -EINVAL;
}

Return<void> HWCSession::getActiveConfig(IDisplayConfig::DisplayType dpy,
                                         getActiveConfig_cb _hidl_cb) {
  uint32_t config = 0;
  int32_t error = GetActiveConfigIndex(MapDisplayType(dpy), &config);

  _hidl_cb(error, config);

  return Void();
}

int32_t HWCSession::SetActiveConfigIndex(int disp_id, uint32_t config) {
  int disp_idx = GetDisplayIndex(disp_id);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_id);
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
  int32_t error = -EINVAL;
  if (hwc_display_[disp_idx]) {
    error = hwc_display_[disp_idx]->SetActiveDisplayConfig(config);
    if (!error) {
      Refresh(0);
    }
  }

  return error;
}

Return<int32_t> HWCSession::setActiveConfig(IDisplayConfig::DisplayType dpy, uint32_t config) {
  return SetActiveConfigIndex(MapDisplayType(dpy), config);
}

Return<void> HWCSession::getDisplayAttributes(uint32_t configIndex,
                                              IDisplayConfig::DisplayType dpy,
                                              getDisplayAttributes_cb _hidl_cb) {
  int32_t error = -EINVAL;
  IDisplayConfig::DisplayAttributes display_attributes = {};
  int disp_id = MapDisplayType(dpy);
  int disp_idx = GetDisplayIndex(disp_id);

  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_id);
  } else {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
    if (hwc_display_[disp_idx]) {
      DisplayConfigVariableInfo var_info;
      error = hwc_display_[disp_idx]->GetDisplayAttributesForConfig(INT(configIndex), &var_info);
      if (!error) {
        display_attributes.vsyncPeriod = var_info.vsync_period_ns;
        display_attributes.xRes = var_info.x_pixels;
        display_attributes.yRes = var_info.y_pixels;
        display_attributes.xDpi = var_info.x_dpi;
        display_attributes.yDpi = var_info.y_dpi;
        display_attributes.panelType = IDisplayConfig::DisplayPortType::DISPLAY_PORT_DEFAULT;
        display_attributes.isYuv = var_info.is_yuv;
      }
    }
  }
  _hidl_cb(error, display_attributes);

  return Void();
}

Return<int32_t> HWCSession::setPanelBrightness(uint32_t level) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);
  int32_t error = -EINVAL;

  if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
    error = hwc_display_[HWC_DISPLAY_PRIMARY]->SetPanelBrightness(INT(level));
    if (error) {
      DLOGE("Failed to set the panel brightness = %d. Error = %d", level, error);
    }
  }

  return error;
}

int32_t HWCSession::GetPanelBrightness(int *level) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);
  int32_t error = -EINVAL;

  if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
    error = hwc_display_[HWC_DISPLAY_PRIMARY]->GetPanelBrightness(level);
    if (error) {
      DLOGE("Failed to get the panel brightness. Error = %d", error);
    }
  }

  return error;
}

Return<void> HWCSession::getPanelBrightness(getPanelBrightness_cb _hidl_cb) {
  int level = 0;
  int32_t error = GetPanelBrightness(&level);

  _hidl_cb(error, static_cast<uint32_t>(level));

  return Void();
}

int32_t HWCSession::MinHdcpEncryptionLevelChanged(int disp_id, uint32_t min_enc_level) {
  DLOGI("Display %d", disp_id);

  if (disp_id < 0) {
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);
  if (disp_id != HWC_DISPLAY_EXTERNAL) {
    DLOGE("Not supported for display");
  } else if (!hwc_display_[disp_id]) {
    DLOGW("Display is not connected");
  } else {
    return hwc_display_[disp_id]->OnMinHdcpEncryptionLevelChange(min_enc_level);
  }

  return -EINVAL;
}

Return<int32_t> HWCSession::minHdcpEncryptionLevelChanged(IDisplayConfig::DisplayType dpy,
                                                          uint32_t min_enc_level) {
  return MinHdcpEncryptionLevelChanged(MapDisplayType(dpy), min_enc_level);
}

Return<int32_t> HWCSession::refreshScreen() {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);
  Refresh(HWC_DISPLAY_PRIMARY);

  return 0;
}

int32_t HWCSession::ControlPartialUpdate(int disp_id, bool enable) {
  if (disp_id < 0) {
    return -EINVAL;
  }

  if (disp_id != HWC_DISPLAY_PRIMARY) {
    DLOGW("CONTROL_PARTIAL_UPDATE is not applicable for display = %d", disp_id);
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);
  HWCDisplay *hwc_display = hwc_display_[HWC_DISPLAY_PRIMARY];
  if (!hwc_display) {
    DLOGE("primary display object is not instantiated");
    return -EINVAL;
  }

  uint32_t pending = 0;
  DisplayError hwc_error = hwc_display->ControlPartialUpdate(enable, &pending);

  if (hwc_error == kErrorNone) {
    if (!pending) {
      return 0;
    }
  } else if (hwc_error == kErrorNotSupported) {
    return 0;
  } else {
    return -EINVAL;
  }

  // Todo(user): Unlock it before sending events to client. It may cause deadlocks in future.
  Refresh(HWC_DISPLAY_PRIMARY);

  // Wait until partial update control is complete
  int32_t error = locker_[disp_id].WaitFinite(kCommitDoneTimeoutMs);

  return error;
}

Return<int32_t> HWCSession::controlPartialUpdate(IDisplayConfig::DisplayType dpy, bool enable) {
  return ControlPartialUpdate(MapDisplayType(dpy), enable);
}

Return<int32_t> HWCSession::toggleScreenUpdate(bool on) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  int32_t error = -EINVAL;
  if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
    error = hwc_display_[HWC_DISPLAY_PRIMARY]->ToggleScreenUpdates(on);
    if (error) {
      DLOGE("Failed to toggle screen updates = %d. Error = %d", on, error);
    }
  }

  return error;
}

Return<int32_t> HWCSession::setIdleTimeout(uint32_t value) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
    hwc_display_[HWC_DISPLAY_PRIMARY]->SetIdleTimeoutMs(value);
    return 0;
  }

  DLOGW("Display = %d is not connected.", HWC_DISPLAY_PRIMARY);
  return -ENODEV;
}

Return<void> HWCSession::getHDRCapabilities(IDisplayConfig::DisplayType dpy,
                                            getHDRCapabilities_cb _hidl_cb) {
  int32_t error = -EINVAL;
  IDisplayConfig::DisplayHDRCapabilities hdr_caps = {};

  do {
    if (!_hidl_cb) {
      DLOGE("_hidl_cb callback not provided.");
      break;
    }

    int disp_id = MapDisplayType(dpy);
    int disp_idx = GetDisplayIndex(disp_id);
    if (disp_idx == -1) {
      DLOGE("Invalid display = %d", disp_id);
      break;
    }

    SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
    HWCDisplay *hwc_display = hwc_display_[disp_idx];
    if (!hwc_display) {
      DLOGW("Display = %d is not connected.", disp_idx);
      error = -ENODEV;
      break;
    }

    // query number of hdr types
    uint32_t out_num_types = 0;
    if (hwc_display->GetHdrCapabilities(&out_num_types, nullptr, nullptr, nullptr, nullptr)
        != HWC2::Error::None) {
      break;
    }

    if (!out_num_types) {
      error = 0;
      break;
    }

    // query hdr caps
    hdr_caps.supportedHdrTypes.resize(out_num_types);

    float out_max_luminance = 0.0f;
    float out_max_average_luminance = 0.0f;
    float out_min_luminance = 0.0f;
    if (hwc_display->GetHdrCapabilities(&out_num_types, hdr_caps.supportedHdrTypes.data(),
                                        &out_max_luminance, &out_max_average_luminance,
                                        &out_min_luminance) == HWC2::Error::None) {
      error = 0;
    }
  } while (false);

  _hidl_cb(error, hdr_caps);

  return Void();
}

Return<int32_t> HWCSession::setCameraLaunchStatus(uint32_t on) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  if (null_display_mode_) {
    return 0;
  }

  if (!core_intf_) {
    DLOGW("core_intf_ not initialized.");
    return -ENOENT;
  }

  if (!hwc_display_[HWC_DISPLAY_PRIMARY]) {
    DLOGW("Display = %d is not connected.", HWC_DISPLAY_PRIMARY);
    return -ENODEV;
  }

  HWBwModes mode = on > 0 ? kBwCamera : kBwDefault;

  // trigger invalidate to apply new bw caps.
  Refresh(HWC_DISPLAY_PRIMARY);

  if (core_intf_->SetMaxBandwidthMode(mode) != kErrorNone) {
    return -EINVAL;
  }

  new_bw_mode_ = true;
  need_invalidate_ = true;
  hwc_display_[HWC_DISPLAY_PRIMARY]->ResetValidation();

  return 0;
}

int32_t HWCSession::DisplayBWTransactionPending(bool *status) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
    if (sync_wait(bw_mode_release_fd_, 0) < 0) {
      DLOGI("bw_transaction_release_fd is not yet signaled: err= %s", strerror(errno));
      *status = false;
    }

    return 0;
  }

  DLOGW("Display = %d is not connected.", HWC_DISPLAY_PRIMARY);
  return -ENODEV;
}

Return<void> HWCSession::displayBWTransactionPending(displayBWTransactionPending_cb _hidl_cb) {
  bool status = true;

  if (!_hidl_cb) {
      DLOGE("_hidl_cb callback not provided.");
      return Void();
  }

  int32_t error = DisplayBWTransactionPending(&status);

  _hidl_cb(error, status);

  return Void();
}

#ifdef DISPLAY_CONFIG_1_1
Return<int32_t> HWCSession::setDisplayAnimating(uint64_t display_id, bool animating ) {
  return CallDisplayFunction(static_cast<hwc2_device_t *>(this), display_id,
                             &HWCDisplay::SetDisplayAnimating, animating);
}
#endif

#ifdef DISPLAY_CONFIG_1_2
static const char * GetDisplayTypeName(IDisplayConfig::DisplayTypeExt disp_type) {
  switch (disp_type) {
    case IDisplayConfig::DisplayTypeExt::DISPLAY_BUILTIN:   return "Built-in";
    case IDisplayConfig::DisplayTypeExt::DISPLAY_PLUGGABLE: return "Pluggable";
    case IDisplayConfig::DisplayTypeExt::DISPLAY_VIRTUAL:   return "Virtual";
    case IDisplayConfig::DisplayTypeExt::DISPLAY_PRIMARY:   return "Primary";
    default:                                                return "Unknown";
  }
}

Return<int32_t> HWCSession::setDisplayIndex(IDisplayConfig::DisplayTypeExt disp_type,
                                            uint32_t base, uint32_t count) {
  if (client_connected_) {
    DLOGW("Not supported after client connection is completed.");
    return -1;
  }

  DLOGI("%s display: base = %d, count = %d", GetDisplayTypeName(disp_type), base, count);

  // Is display slots capacity smaller than what client can support?
  if ((base + count) > kNumDisplays) {
    DLOGE("Exceeds max supported display slots = %d", kNumDisplays);
    return -1;
  }

  std::vector<DisplayMapInfo> *map_info_v = nullptr;
  switch (disp_type) {
    case IDisplayConfig::DisplayTypeExt::DISPLAY_BUILTIN:
      map_info_v = &map_info_builtin_;
      break;
    case IDisplayConfig::DisplayTypeExt::DISPLAY_PLUGGABLE:
      map_info_v = &map_info_pluggable_;
      break;
    case IDisplayConfig::DisplayTypeExt::DISPLAY_VIRTUAL:
      map_info_v = &map_info_virtual_;
      break;
    case IDisplayConfig::DisplayTypeExt::DISPLAY_PRIMARY:
      // nothing to do
      return 0;
    default:
      return -1;
  }

  // Reset default client id for each display with a new client id in given range.
  // Remove remaining elements as client as client can not handle these displays.
  DLOGI("Change %s display capacity to %d", GetDisplayTypeName(disp_type), count);
  map_info_v->resize(count);

  uint32_t idx = 0;
  for (auto &map_info : *map_info_v) {
    map_info.client_id = base++;
    DLOGI("Override: %s display: Index = %d, Client ID = %d",
                  GetDisplayTypeName(disp_type), idx++, map_info.client_id);
  }

  return 0;
}
#endif  // DISPLAY_CONFIG_1_2

#ifdef DISPLAY_CONFIG_1_3
Return<int32_t> HWCSession::controlIdlePowerCollapse(bool enable, bool synchronous) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
    if (!enable) {
      if (!idle_pc_ref_cnt_) {
        HWC2::Error err =
            hwc_display_[HWC_DISPLAY_PRIMARY]->ControlIdlePowerCollapse(enable, synchronous);
        if (err != HWC2::Error::None) {
          return -EINVAL;
        }
        Refresh(HWC_DISPLAY_PRIMARY);
        int32_t error = locker_[HWC_DISPLAY_PRIMARY].WaitFinite(kCommitDoneTimeoutMs);
        if (error == ETIMEDOUT) {
          DLOGE("Timed out!! Next frame commit done event not received!!");
          return error;
        }
        DLOGI("Idle PC disabled!!");
      }
      idle_pc_ref_cnt_++;
    } else if (idle_pc_ref_cnt_ > 0) {
      if (!(idle_pc_ref_cnt_ - 1)) {
        HWC2::Error err =
            hwc_display_[HWC_DISPLAY_PRIMARY]->ControlIdlePowerCollapse(enable, synchronous);
        if (err != HWC2::Error::None) {
          return -EINVAL;
        }
        DLOGI("Idle PC enabled!!");
      }
      idle_pc_ref_cnt_--;
    }
    return 0;
  }

  DLOGW("Display = %d is not connected.", HWC_DISPLAY_PRIMARY);
  return -ENODEV;
}
#endif  // DISPLAY_CONFIG_1_3


int32_t HWCSession::IsWbUbwcSupported(int *value) {
  HWDisplaysInfo hw_displays_info = {};
  DisplayError error = core_intf_->GetDisplaysStatus(&hw_displays_info);
  if (error != kErrorNone) {
    return -EINVAL;
  }

  for (auto &iter : hw_displays_info) {
    auto &info = iter.second;
    if (info.display_type == kVirtual && info.is_wb_ubwc_supported) {
      *value = 1;
    }
  }

  return error;
}

#ifdef DISPLAY_CONFIG_1_4
Return<void> HWCSession::getWriteBackCapabilities(getWriteBackCapabilities_cb _hidl_cb) {
  int value = 0;
  IDisplayConfig::WriteBackCapabilities wb_caps = {};
  int32_t error = IsWbUbwcSupported(&value);
  wb_caps.isWbUbwcSupported = value;
  _hidl_cb(error, wb_caps);

  return Void();
}
#endif  // DISPLAY_CONFIG_1_4

}  // namespace sdm
