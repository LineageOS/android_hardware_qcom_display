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
  int err = -EINVAL;
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_id);
    return -EINVAL;
  }

  if (disp_idx == qdutils::DISPLAY_PRIMARY) {
    DLOGE("Not supported for this display");
    return err;
  }

  {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
    if (!hwc_display_[disp_idx]) {
      DLOGW("Display is not connected");
      return err;
    }
    DLOGI("Display = %d, Status = %d", disp_idx, status);
    err = hwc_display_[disp_idx]->SetDisplayStatus(status);
    if (err != 0) {
      return err;
    }
  }

  if (status == HWCDisplay::kDisplayStatusResume || status == HWCDisplay::kDisplayStatusPause) {
    hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
    if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
      {
        SEQUENCE_WAIT_SCOPE_LOCK(locker_[active_builtin_disp_id]);
        hwc_display_[active_builtin_disp_id]->ResetValidation();
      }
      callbacks_.Refresh(active_builtin_disp_id);
    }
  }

  return err;
}

int HWCSession::DisplayConfigImpl::SetDisplayStatus(DispType dpy,
                                                    DisplayConfig::ExternalStatus status) {
  return hwc_session_->SetDisplayStatus(MapDisplayType(dpy), MapExternalStatus(status));
}

int HWCSession::DisplayConfigImpl::ConfigureDynRefreshRate(DisplayConfig::DynRefreshRateOp op,
                                                           uint32_t refresh_rate) {
  SEQUENCE_WAIT_SCOPE_LOCK(hwc_session_->locker_[HWC_DISPLAY_PRIMARY]);
  HWCDisplay *hwc_display = hwc_session_->hwc_display_[HWC_DISPLAY_PRIMARY];

  if (!hwc_display) {
    DLOGW("Display = %d is not connected.", HWC_DISPLAY_PRIMARY);
    return -EINVAL;
  }

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

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
  if (disp_idx != HWC_DISPLAY_EXTERNAL) {
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
  SEQUENCE_WAIT_SCOPE_LOCK(hwc_session_->locker_[HWC_DISPLAY_PRIMARY]);
  hwc_session_->Refresh(HWC_DISPLAY_PRIMARY);
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
  int error = locker_[disp_idx].WaitFinite(kCommitDoneTimeoutMs);

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
  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  if (active_builtin_disp_id >= HWCCallbacks::kNumDisplays) {
    DLOGE("No active displays");
    return -EINVAL;
  }
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[active_builtin_disp_id]);

  if (null_display_mode_) {
    return 0;
  }

  if (!core_intf_) {
    DLOGW("core_intf_ not initialized.");
    return -ENOENT;
  }

  if (!hwc_display_[active_builtin_disp_id]) {
    DLOGW("Display = %d is not connected.", active_builtin_disp_id);
    return -ENODEV;
  }

  HWBwModes mode = on > 0 ? kBwCamera : kBwDefault;

  // trigger invalidate to apply new bw caps.
  Refresh(active_builtin_disp_id);

  if (core_intf_->SetMaxBandwidthMode(mode) != kErrorNone) {
    return -EINVAL;
  }

  new_bw_mode_ = true;
  need_invalidate_ = true;
  hwc_display_[active_builtin_disp_id]->ResetValidation();

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

int HWCSession::DisplayConfigImpl::SetDisplayAnimating(uint64_t display_id, bool animating) {
  return hwc_session_->CallDisplayFunction(static_cast<hwc2_device_t *>(hwc_session_), display_id,
                                           &HWCDisplay::SetDisplayAnimating, animating);
}

int HWCSession::ControlIdlePowerCollapse(bool enable, bool synchronous) {
  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  if (active_builtin_disp_id >= HWCCallbacks::kNumDisplays) {
    DLOGE("No active displays");
    return -EINVAL;
  }
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[active_builtin_disp_id]);

  if (hwc_display_[active_builtin_disp_id]) {
    if (!enable) {
      if (!idle_pc_ref_cnt_) {
        auto err = hwc_display_[active_builtin_disp_id]->ControlIdlePowerCollapse(enable,
                                                                                  synchronous);
        if (err != kErrorNone) {
          return (err == kErrorNotSupported) ? 0 : -EINVAL;
        }
        Refresh(active_builtin_disp_id);
        int error = locker_[active_builtin_disp_id].WaitFinite(kCommitDoneTimeoutMs);
        if (error == ETIMEDOUT) {
          DLOGE("Timed out!! Next frame commit done event not received!!");
          return error;
        }
        DLOGI("Idle PC disabled!!");
      }
      idle_pc_ref_cnt_++;
    } else if (idle_pc_ref_cnt_ > 0) {
      if (!(idle_pc_ref_cnt_ - 1)) {
        auto err = hwc_display_[active_builtin_disp_id]->ControlIdlePowerCollapse(enable,
                                                                                  synchronous);
        if (err != kErrorNone) {
          return (err == kErrorNotSupported) ? 0 : -EINVAL;
        }
        DLOGI("Idle PC enabled!!");
      }
      idle_pc_ref_cnt_--;
    }
    return 0;
  }

  DLOGW("Display = %d is not connected.", active_builtin_disp_id);
  return -ENODEV;
}

int HWCSession::DisplayConfigImpl::ControlIdlePowerCollapse(bool enable, bool synchronous) {
  return hwc_session_->ControlIdlePowerCollapse(enable, synchronous);
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

int HWCSession::SetDisplayDppsAdROI(uint32_t display_id, uint32_t h_start,
                                    uint32_t h_end, uint32_t v_start, uint32_t v_end,
                                    uint32_t factor_in, uint32_t factor_out) {
  return CallDisplayFunction(static_cast<hwc2_device_t *>(this), display_id,
                             &HWCDisplay::SetDisplayDppsAdROI, h_start, h_end, v_start, v_end,
                             factor_in, factor_out);
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

int HWCSession::DisplayConfigImpl::IsPowerModeOverrideSupported(uint32_t disp_id,
                                                                bool *supported) {
  if (!hwc_session_->async_powermode_ || (disp_id > HWCCallbacks::kNumRealDisplays)) {
    *supported = false;
  } else {
    *supported = true;
  }

  return 0;
}

int HWCSession::DisplayConfigImpl::SetPowerMode(uint32_t disp_id,
                                                DisplayConfig::PowerMode power_mode) {
  SCOPE_LOCK(hwc_session_->display_config_locker_);

  bool supported = false;
  IsPowerModeOverrideSupported(disp_id, &supported);
  if (!supported) {
    return 0;
  }

  DLOGI("disp_id: %d power_mode: %d", disp_id, power_mode);
  HWCDisplay::HWCLayerStack stack = {};
  hwc2_display_t dummy_disp_id = hwc_session_->map_hwc_display_.at(disp_id);

  {
    // Power state transition start.
    Locker::ScopeLock lock_power(hwc_session_->power_state_[disp_id]);
    Locker::ScopeLock lock_primary(hwc_session_->locker_[disp_id]);
    Locker::ScopeLock lock_dummy(hwc_session_->locker_[dummy_disp_id]);

    hwc_session_->power_state_transition_[disp_id] = true;
    // Pass on the complete stack to dummy display.
    hwc_session_->hwc_display_[disp_id]->GetLayerStack(&stack);
    // Update the same stack onto dummy display.
    hwc_session_->hwc_display_[dummy_disp_id]->SetLayerStack(&stack);
  }

  {
    SCOPE_LOCK(hwc_session_->locker_[disp_id]);
    auto mode = static_cast<HWC2::PowerMode>(power_mode);
    hwc_session_->hwc_display_[disp_id]->SetPowerMode(mode, false /* teardown */);
  }

  {
    // Power state transition end.
    Locker::ScopeLock lock_power(hwc_session_->power_state_[disp_id]);
    Locker::ScopeLock lock_primary(hwc_session_->locker_[disp_id]);
    Locker::ScopeLock lock_dummy(hwc_session_->locker_[dummy_disp_id]);
    // Pass on the layer stack to real display.
    hwc_session_->hwc_display_[dummy_disp_id]->GetLayerStack(&stack);
    // Update the same stack onto real display.
    hwc_session_->hwc_display_[disp_id]->SetLayerStack(&stack);
    // Read display has got layerstack. Update the fences.
    hwc_session_->hwc_display_[disp_id]->PostPowerMode();

    hwc_session_->power_state_transition_[disp_id] = false;
  }

  return 0;
}

int HWCSession::DisplayConfigImpl::IsHDRSupported(uint32_t disp_id, bool *supported) {
  if (disp_id < 0 || disp_id >= HWCCallbacks::kNumDisplays) {
    DLOGE("Not valid display");
    return -EINVAL;
  }
  SCOPE_LOCK(hwc_session_->locker_[disp_id]);

  if (hwc_session_->is_hdr_display_.size() <= disp_id) {
    DLOGW("is_hdr_display_ is not initialized for display %d!! Reporting it as HDR not supported",
          disp_id);
    *supported = false;
    return 0;
  }

  *supported = static_cast<bool>(hwc_session_->is_hdr_display_[disp_id]);
  return 0;
}

int HWCSession::DisplayConfigImpl::IsWCGSupported(uint32_t disp_id, bool *supported) {
  // todo(user): Query wcg from sdm. For now assume them same.
  return IsHDRSupported(disp_id, supported);
}

int HWCSession::DisplayConfigImpl::SetLayerAsMask(uint32_t disp_id, uint64_t layer_id) {
  SCOPE_LOCK(hwc_session_->locker_[disp_id]);
  HWCDisplay *hwc_display = hwc_session_->hwc_display_[disp_id];
  if (!hwc_display) {
    DLOGW("Display = %d is not connected.", disp_id);
    return -EINVAL;
  }

  if (hwc_session_->disable_mask_layer_hint_) {
    DLOGW("Mask layer hint is disabled!");
    return -EINVAL;
  }

  auto hwc_layer = hwc_display->GetHWCLayer(layer_id);
  if (hwc_layer == nullptr) {
    return -EINVAL;
  }

  hwc_layer->SetLayerAsMask();

  return 0;
}

int HWCSession::DisplayConfigImpl::GetDebugProperty(const std::string prop_name,
                                                    std::string *value) {
  std::string vendor_prop_name = DISP_PROP_PREFIX;
  int error = -EINVAL;
  char val[64] = {};

  vendor_prop_name += prop_name.c_str();
  if (HWCDebugHandler::Get()->GetProperty(vendor_prop_name.c_str(), val) == kErrorNone) {
    *value = val;
    error = 0;
  }

  return error;
}

int HWCSession::DisplayConfigImpl::GetActiveBuiltinDisplayAttributes(
                                          DisplayConfig::Attributes *attr) {
  int error = -EINVAL;
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
  // currently doing only for virtual display
  if (disp_id != qdutils::DISPLAY_VIRTUAL) {
    return -EINVAL;
  }

  std::lock_guard<std::mutex> obj(hwc_session_->mutex_lum_);
  hwc_session_->set_min_lum_ = pan_min_lum;
  hwc_session_->set_max_lum_ = pan_max_lum;
  DLOGI("set max_lum %f, min_lum %f", pan_max_lum, pan_min_lum);

  return 0;
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

int HWCSession::DisplayConfigImpl::GetSupportedDSIBitClks(uint32_t disp_id,
                                                          std::vector<uint64_t> *bit_clks) {
  SCOPE_LOCK(hwc_session_->locker_[disp_id]);
  if (!hwc_session_->hwc_display_[disp_id]) {
    return -EINVAL;
  }

  hwc_session_->hwc_display_[disp_id]->GetSupportedDSIClock(bit_clks);
  return 0;
}

int HWCSession::DisplayConfigImpl::GetDSIClk(uint32_t disp_id, uint64_t *bit_clk) {
  SCOPE_LOCK(hwc_session_->locker_[disp_id]);
  if (!hwc_session_->hwc_display_[disp_id]) {
    return -EINVAL;
  }

  hwc_session_->hwc_display_[disp_id]->GetDynamicDSIClock(bit_clk);

  return 0;
}

int HWCSession::DisplayConfigImpl::SetDSIClk(uint32_t disp_id, uint64_t bit_clk) {
  SCOPE_LOCK(hwc_session_->locker_[disp_id]);
  if (!hwc_session_->hwc_display_[disp_id]) {
    return -1;
  }

  return hwc_session_->hwc_display_[disp_id]->SetDynamicDSIClock(bit_clk);
}

int HWCSession::DisplayConfigImpl::SetCWBOutputBuffer(uint32_t disp_id,
                                                      const DisplayConfig::Rect rect,
                                                      bool post_processed,
                                                      const native_handle_t *buffer) {
  return -1;
}

bool HWCSession::isSmartPanelConfig(uint32_t disp_id, uint32_t config_id) {
  if (disp_id != qdutils::DISPLAY_PRIMARY) {
    return false;
  }

  SCOPE_LOCK(locker_[disp_id]);
  if (!hwc_display_[disp_id]) {
    DLOGE("Display %d is not created yet.", disp_id);
    return false;
  }

  return hwc_display_[disp_id]->IsSmartPanelConfig(config_id);
}

int HWCSession::DisplayConfigImpl::SetQsyncMode(uint32_t disp_id, DisplayConfig::QsyncMode mode) {
  SEQUENCE_WAIT_SCOPE_LOCK(hwc_session_->locker_[disp_id]);
  if (!hwc_session_->hwc_display_[disp_id]) {
    return -1;
  }

  QSyncMode qsync_mode = kQSyncModeNone;
  switch (mode) {
    case DisplayConfig::QsyncMode::kNone:
      qsync_mode = kQSyncModeNone;
      break;
    case DisplayConfig::QsyncMode::kWaitForFencesOneFrame:
      qsync_mode = kQsyncModeOneShot;
      break;
    case DisplayConfig::QsyncMode::kWaitForFencesEachFrame:
      qsync_mode = kQsyncModeOneShotContinuous;
      break;
    case DisplayConfig::QsyncMode::kWaitForCommitEachFrame:
      qsync_mode = kQSyncModeContinuous;
      break;
  }

  hwc_session_->hwc_display_[disp_id]->SetQSyncMode(qsync_mode);
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
