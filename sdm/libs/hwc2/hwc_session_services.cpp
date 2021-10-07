/*
* Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
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
#include <string>
#include <errno.h>
#include <math.h>

#include "hwc_buffer_sync_handler.h"
#include "hwc_session.h"
#include "hwc_debugger.h"

#define __CLASS__ "HWCSession"

namespace sdm {

void HWCSession::StartServices() {
  int error = DisplayConfig::DeviceInterface::RegisterDevice(this);
  if (error) {
    DLOGW("Could not register IDisplayConfig as service (%d).", error);
  } else {
    DLOGI("IDisplayConfig service registration completed.");
  }
}

int MapDisplayType(DispType dpy) {
  switch (dpy) {
    case DispType::kPrimary:
      return qdutils::DISPLAY_PRIMARY;

    case DispType::kExternal:
      return qdutils::DISPLAY_EXTERNAL;

    case DispType::kVirtual:
      return qdutils::DISPLAY_VIRTUAL;

    default:
      break;
  }

  return -EINVAL;
}

HWCDisplay::DisplayStatus MapExternalStatus(DisplayConfig::ExternalStatus status) {
  switch (status) {
    case DisplayConfig::ExternalStatus::kOffline:
      return HWCDisplay::kDisplayStatusOffline;

    case DisplayConfig::ExternalStatus::kOnline:
      return HWCDisplay::kDisplayStatusOnline;

    case DisplayConfig::ExternalStatus::kPause:
      return HWCDisplay::kDisplayStatusPause;

    case DisplayConfig::ExternalStatus::kResume:
      return HWCDisplay::kDisplayStatusResume;

    default:
      break;
  }

  return HWCDisplay::kDisplayStatusInvalid;
}

int HWCSession::RegisterClientContext(std::shared_ptr<DisplayConfig::ConfigCallback> callback,
                                      DisplayConfig::ConfigInterface **intf) {
  if (!intf) {
    DLOGE("Invalid DisplayConfigIntf location");
    return -EINVAL;
  }

  std::weak_ptr<DisplayConfig::ConfigCallback> wp_callback = callback;
  DisplayConfigImpl *impl = new DisplayConfigImpl(wp_callback, this);
  *intf = impl;

  return 0;
}

void HWCSession::UnRegisterClientContext(DisplayConfig::ConfigInterface *intf) {
  delete static_cast<DisplayConfigImpl *>(intf);
}

HWCSession::DisplayConfigImpl::DisplayConfigImpl(
                               std::weak_ptr<DisplayConfig::ConfigCallback> callback,
                               HWCSession *hwc_session) {
  callback_ = callback;
  hwc_session_ = hwc_session;
}

int HWCSession::DisplayConfigImpl::IsDisplayConnected(DispType dpy, bool *connected) {
  int disp_id = MapDisplayType(dpy);
  int disp_idx = hwc_session_->GetDisplayIndex(disp_id);

  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_id);
    return -EINVAL;
  } else {
    SEQUENCE_WAIT_SCOPE_LOCK(hwc_session_->locker_[disp_idx]);
    *connected = hwc_session_->hwc_display_[disp_idx];
  }

  return 0;
}

int HWCSession::SetDisplayStatus(int disp_id, HWCDisplay::DisplayStatus status) {
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

int HWCSession::DisplayConfigImpl::SetDisplayStatus(DispType dpy,
                                                    DisplayConfig::ExternalStatus status) {
  return hwc_session_->SetDisplayStatus(MapDisplayType(dpy), MapExternalStatus(status));
}

int HWCSession::DisplayConfigImpl::ConfigureDynRefreshRate(DisplayConfig::DynRefreshRateOp op,
                                                           uint32_t refresh_rate) {
  SEQUENCE_WAIT_SCOPE_LOCK(hwc_session_->locker_[HWC_DISPLAY_PRIMARY]);
  HWCDisplay *hwc_display = hwc_session_->hwc_display_[HWC_DISPLAY_PRIMARY];

  switch (op) {
    case DisplayConfig::DynRefreshRateOp::kDisableMetadata:
      return hwc_display->Perform(HWCDisplayBuiltIn::SET_METADATA_DYN_REFRESH_RATE, false);

    case DisplayConfig::DynRefreshRateOp::kEnableMetadata:
      return hwc_display->Perform(HWCDisplayBuiltIn::SET_METADATA_DYN_REFRESH_RATE, true);

    case DisplayConfig::DynRefreshRateOp::kSetBinder:
      return hwc_display->Perform(HWCDisplayBuiltIn::SET_BINDER_DYN_REFRESH_RATE, refresh_rate);

    default:
      DLOGW("Invalid operation %d", op);
      return -EINVAL;
  }

  return 0;
}

int HWCSession::GetConfigCount(int disp_id, uint32_t *count) {
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

int HWCSession::DisplayConfigImpl::GetConfigCount(DispType dpy, uint32_t *count) {
  return hwc_session_->GetConfigCount(MapDisplayType(dpy), count);
}

int HWCSession::GetActiveConfigIndex(int disp_id, uint32_t *config) {
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

int HWCSession::DisplayConfigImpl::GetActiveConfig(DispType dpy, uint32_t *config) {
  return hwc_session_->GetActiveConfigIndex(MapDisplayType(dpy), config);
}

int HWCSession::SetActiveConfigIndex(int disp_id, uint32_t config) {
  int disp_idx = GetDisplayIndex(disp_id);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_id);
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
  int error = -EINVAL;
  if (hwc_display_[disp_idx]) {
    error = hwc_display_[disp_idx]->SetActiveDisplayConfig(config);
    if (!error) {
      Refresh(0);
    }
  }

  return error;
}

int HWCSession::DisplayConfigImpl::SetActiveConfig(DispType dpy, uint32_t config) {
  return hwc_session_->SetActiveConfigIndex(MapDisplayType(dpy), config);
}

int HWCSession::DisplayConfigImpl::GetDisplayAttributes(uint32_t config_index, DispType dpy,
                                                        DisplayConfig::Attributes *attributes) {
  int error = -EINVAL;

  int disp_id = MapDisplayType(dpy);
  int disp_idx = hwc_session_->GetDisplayIndex(disp_id);

  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_id);
  } else {
    SEQUENCE_WAIT_SCOPE_LOCK(hwc_session_->locker_[disp_idx]);
    if (hwc_session_->hwc_display_[disp_idx]) {
      DisplayConfigVariableInfo var_info;
      error = hwc_session_->hwc_display_[disp_idx]->GetDisplayAttributesForConfig(INT(config_index),
                                                                                  &var_info);
      if (!error) {
        attributes->vsync_period = var_info.vsync_period_ns;
        attributes->x_res = var_info.x_pixels;
        attributes->y_res = var_info.y_pixels;
        attributes->x_dpi = var_info.x_dpi;
        attributes->y_dpi = var_info.y_dpi;
        attributes->panel_type = DisplayConfig::DisplayPortType::kDefault;
        attributes->is_yuv = var_info.is_yuv;
      }
    }
  }

  return error;
}

int HWCSession::DisplayConfigImpl::SetPanelBrightness(uint32_t level) {
  if (!(0 <= level && level <= 255)) {
    return -EINVAL;
  }

  hwc2_device_t *device = static_cast<hwc2_device_t *>(hwc_session_);
  if (level == 0) {
    return INT32(hwc_session_->SetDisplayBrightness(device, HWC_DISPLAY_PRIMARY, -1.0f));
  } else {
    return INT32(hwc_session_->SetDisplayBrightness(device, HWC_DISPLAY_PRIMARY,
                                                    (level - 1)/254.0f));
  }
}

int HWCSession::DisplayConfigImpl::GetPanelBrightness(uint32_t *level) {
  float brightness = -1.0f;
  int32_t error = -EINVAL;

  error = hwc_session_->getDisplayBrightness(HWC_DISPLAY_PRIMARY, &brightness);
  if (brightness == -1.0f) {
    *level = 0;
  } else {
    *level = static_cast<uint32_t>(254.0f*brightness + 1);
  }

  return error;
}

int HWCSession::MinHdcpEncryptionLevelChanged(int disp_id, uint32_t min_enc_level) {
  DLOGI("Display %d", disp_id);

  int disp_idx = GetDisplayIndex(disp_id);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_id);
    return -EINVAL;
  }

  hwc2_display_t external_display_index =
    (hwc2_display_t)GetDisplayIndex(qdutils::DISPLAY_EXTERNAL);

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
  if (disp_idx != external_display_index) {
    DLOGE("Not supported for display");
  } else if (!hwc_display_[disp_idx]) {
    DLOGW("Display is not connected");
  } else {
    return hwc_display_[disp_idx]->OnMinHdcpEncryptionLevelChange(min_enc_level);
  }

  return -EINVAL;
}

int HWCSession::DisplayConfigImpl::MinHdcpEncryptionLevelChanged(DispType dpy,
                                                                 uint32_t min_enc_level) {
  return hwc_session_->MinHdcpEncryptionLevelChanged(MapDisplayType(dpy), min_enc_level);
}

int HWCSession::DisplayConfigImpl::RefreshScreen() {
  {
    SCOPE_LOCK(hwc_session_->locker_[HWC_DISPLAY_PRIMARY]);
    if (!hwc_session_->hwc_display_[HWC_DISPLAY_PRIMARY]) {
      DLOGE("primary display object is not instantiated");
      return -EINVAL;
    }
  }
  hwc_session_->hwc_display_[HWC_DISPLAY_PRIMARY]->Refresh();
  return 0;
}

int HWCSession::ControlPartialUpdate(int disp_id, bool enable) {
  int disp_idx = GetDisplayIndex(disp_id);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_id);
    return -EINVAL;
  }

  if (disp_idx != HWC_DISPLAY_PRIMARY) {
    DLOGW("CONTROL_PARTIAL_UPDATE is not applicable for display = %d", disp_idx);
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
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
  int error = locker_[disp_idx].WaitFinite(kPartialUpdateControlTimeoutMs);

  return error;
}

int HWCSession::DisplayConfigImpl::ControlPartialUpdate(DispType dpy, bool enable) {
  return hwc_session_->ControlPartialUpdate(MapDisplayType(dpy), enable);
}

int HWCSession::ToggleScreenUpdate(bool on) {
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

int HWCSession::DisplayConfigImpl::ToggleScreenUpdate(bool on) {
  return hwc_session_->ToggleScreenUpdate(on);
}

int HWCSession::SetIdleTimeout(uint32_t value) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
    hwc_display_[HWC_DISPLAY_PRIMARY]->SetIdleTimeoutMs(value);
    return 0;
  }

  DLOGW("Display = %d is not connected.", HWC_DISPLAY_PRIMARY);
  return -ENODEV;
}

int HWCSession::DisplayConfigImpl::SetIdleTimeout(uint32_t value) {
  return hwc_session_->SetIdleTimeout(value);
}

int HWCSession::DisplayConfigImpl::GetHDRCapabilities(DispType dpy,
                                                      DisplayConfig::HDRCapsParams *caps) {
  int error = -EINVAL;

  do {
    int disp_id = MapDisplayType(dpy);
    int disp_idx = hwc_session_->GetDisplayIndex(disp_id);
    if (disp_idx == -1) {
      DLOGE("Invalid display = %d", disp_id);
      break;
    }

    SCOPE_LOCK(hwc_session_->locker_[disp_id]);
    HWCDisplay *hwc_display = hwc_session_->hwc_display_[disp_idx];
    if (!hwc_display) {
      DLOGW("Display = %d is not connected.", disp_idx);
      error = -ENODEV;
      break;
    }

    // query number of hdr types
    uint32_t out_num_types = 0;
    float out_max_luminance = 0.0f;
    float out_max_average_luminance = 0.0f;
    float out_min_luminance = 0.0f;
    if (hwc_display->GetHdrCapabilities(&out_num_types, nullptr, &out_max_luminance,
                                        &out_max_average_luminance, &out_min_luminance)
                                        != HWC2::Error::None) {
      break;
    }
    if (!out_num_types) {
      error = 0;
      break;
    }

    // query hdr caps
    caps->supported_hdr_types.resize(out_num_types);

    if (hwc_display->GetHdrCapabilities(&out_num_types, caps->supported_hdr_types.data(),
                                        &out_max_luminance, &out_max_average_luminance,
                                        &out_min_luminance) == HWC2::Error::None) {
      error = 0;
    }
  } while (false);
    return error;
  }

  int HWCSession::SetCameraLaunchStatus(uint32_t on) {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

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

  int HWCSession::DisplayConfigImpl::SetCameraLaunchStatus(uint32_t on) {
      return hwc_session_->SetCameraLaunchStatus(on);
    }

  int HWCSession::DisplayBWTransactionPending(bool *status) {
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

  int HWCSession::DisplayConfigImpl::DisplayBWTransactionPending(bool *status) {
    return hwc_session_->DisplayBWTransactionPending(status);
  }

int HWCSession::DisplayConfigImpl::SetDisplayAnimating(uint64_t display_id, bool animating ) {
  return hwc_session_->CallDisplayFunction(static_cast<hwc2_device_t *>(hwc_session_), display_id,
                             &HWCDisplay::SetDisplayAnimating, animating);
}

int HWCSession::ControlIdlePowerCollapse(bool enable, bool synchronous) {
  DLOGW("Not implemented.");
  return 0;
}

int HWCSession::DisplayConfigImpl::ControlIdlePowerCollapse(bool enable, bool synchronous) {
  return hwc_session_->ControlIdlePowerCollapse(enable, synchronous);
}

int HWCSession::SetDisplayDppsAdROI(uint32_t display_id, uint32_t h_start,
                                                uint32_t h_end, uint32_t v_start, uint32_t v_end,
                                                uint32_t factor_in, uint32_t factor_out) {
    DLOGW("Not implemented.");
    return 0;
}

int HWCSession::DisplayConfigImpl::SetDisplayDppsAdROI(uint32_t display_id, uint32_t h_start,
                                                       uint32_t h_end, uint32_t v_start,
                                                       uint32_t v_end, uint32_t factor_in,
                                                       uint32_t factor_out) {
  return hwc_session_->SetDisplayDppsAdROI(display_id, h_start, h_end, v_start, v_end,
                                           factor_in, factor_out);
}

int HWCSession::DisplayConfigImpl::UpdateVSyncSourceOnPowerModeOff() {
  return 0;
}

int HWCSession::DisplayConfigImpl::UpdateVSyncSourceOnPowerModeDoze() {
  return 0;
}

int HWCSession::DisplayConfigImpl::SetPowerMode(uint32_t disp_id,
                                                  DisplayConfig::PowerMode power_mode) {
  return 0;
}

int HWCSession::DisplayConfigImpl::IsPowerModeOverrideSupported(uint32_t disp_id,
                                                                bool *supported) {
    *supported = false;
    return 0;
}

int HWCSession::DisplayConfigImpl::IsHDRSupported(uint32_t disp_id, bool *supported) {
  if ((hwc_session_->is_hdr_display_.size()==0) || (disp_id > (hwc_session_->is_hdr_display_.size()-1))) {
    DLOGW("Not valid display. Id = %d",disp_id);
    return false;
  }

  *supported = static_cast<bool>(hwc_session_->is_hdr_display_[disp_id]);
  return 0;
}

int HWCSession::DisplayConfigImpl::IsWCGSupported(uint32_t disp_id, bool *supported) {
  // todo(user): Query wcg from sdm. For now assume them same.
  return IsHDRSupported(disp_id, supported);
}

int HWCSession::DisplayConfigImpl::SetLayerAsMask(uint32_t disp_id, uint64_t layer_id) {
  return 0;
}

int HWCSession::DisplayConfigImpl::GetDebugProperty(const std::string prop_name,
                                                    std::string value) {
  std::string vendor_prop_name = DISP_PROP_PREFIX;
  int error = -EINVAL;
  char val[64] = {};

  vendor_prop_name += prop_name.c_str();
  if (HWCDebugHandler::Get()->GetProperty(vendor_prop_name.c_str(), val) != kErrorNone) {
    value = val;
    error = 0;
  }

  return error;
}


int HWCSession::IsWbUbwcSupported(bool *value) {
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

int HWCSession::DisplayConfigImpl::GetWriteBackCapabilities(bool *isWbUbwcSupported) {
    return hwc_session_->IsWbUbwcSupported(isWbUbwcSupported);
}

int HWCSession::DisplayConfigImpl::GetActiveBuiltinDisplayAttributes(
                                           DisplayConfig::Attributes *attr) {
  int32_t error = -EINVAL;
  hwc2_display_t disp_id = hwc_session_->GetActiveBuiltinDisplay();

  if (disp_id >= HWCCallbacks::kNumDisplays) {
    DLOGE("Invalid display = %d", disp_id);
  } else {
    if (hwc_session_->hwc_display_[disp_id]) {
      uint32_t config_index = 0;
      HWC2::Error ret = hwc_session_->hwc_display_[disp_id]->GetActiveConfig(&config_index);
      if (ret != HWC2::Error::None) {
        goto err;
      }
      DisplayConfigVariableInfo var_info;
      error = hwc_session_->hwc_display_[disp_id]->GetDisplayAttributesForConfig(INT(config_index),
                                                                                 &var_info);
      if (!error) {
        attr->vsync_period = var_info.vsync_period_ns;
        attr->x_res = var_info.x_pixels;
        attr->y_res = var_info.y_pixels;
        attr->x_dpi = var_info.x_dpi;
        attr->y_dpi = var_info.y_dpi;
        attr->panel_type = DisplayConfig::DisplayPortType::kDefault;
        attr->is_yuv = var_info.is_yuv;
      }
    }
  }

err:
  return error;
}

int HWCSession::DisplayConfigImpl::SetPanelLuminanceAttributes(uint32_t disp_id, float pan_min_lum,
                                                               float pan_max_lum) {
  DLOGE("Not supported at present");
  return -1;
}

int HWCSession::DisplayConfigImpl::IsBuiltInDisplay(uint32_t disp_id, bool *is_builtin) {
  if ((hwc_session_->map_info_primary_.client_id == disp_id) &&
      (hwc_session_->map_info_primary_.disp_type == kBuiltIn)) {
    *is_builtin = true;
    return 0;
  }

  for (auto &info : hwc_session_->map_info_builtin_) {
    if (disp_id == info.client_id) {
      *is_builtin = true;
      return 0;
    }
  }

  *is_builtin = false;
  return 0;
}

int32_t HWCSession::getDisplayBrightness(uint32_t display, float *brightness) {
  if (!brightness) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[display]);
  int32_t error = -EINVAL;
  *brightness = -1.0f;

  HWCDisplay *hwc_display = hwc_display_[display];
  if (hwc_display && hwc_display_[display]->GetDisplayClass() == DISPLAY_CLASS_BUILTIN) {
    error = INT32(hwc_display_[display]->GetPanelBrightness(brightness));
    if (error) {
      DLOGE("Failed to get the panel brightness. Error = %d", error);
    }
  }

  return error;
}

int32_t HWCSession::setDisplayBrightness(uint32_t display, float brightness) {
  return SetDisplayBrightness(static_cast<hwc2_device_t *>(this),
                              static_cast<hwc2_display_t>(display), brightness);
}

}  // namespace sdm
