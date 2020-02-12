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

#include "hwc_buffer_sync_handler.h"
#include "hwc_session.h"
#include "hwc_debugger.h"

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

bool WaitForResourceNeeded(HWC2::PowerMode prev_mode, HWC2::PowerMode new_mode) {
  return ((prev_mode == HWC2::PowerMode::Off) &&
          (new_mode == HWC2::PowerMode::On || new_mode == HWC2::PowerMode::Doze));
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
    if (active_builtin_disp_id < HWCCallbacks::kNumRealDisplays) {
      {
        SEQUENCE_WAIT_SCOPE_LOCK(locker_[active_builtin_disp_id]);
        hwc_display_[active_builtin_disp_id]->ResetValidation();
      }
      callbacks_.Refresh(active_builtin_disp_id);
    }
  }

  return err;
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
      callbacks_.Refresh(0);
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
  if (!(0 <= level && level <= 255)) {
    return -EINVAL;
  }

  if (level == 0) {
    return INT32(SetDisplayBrightness(HWC_DISPLAY_PRIMARY, -1.0f));
  } else {
    return INT32(SetDisplayBrightness(HWC_DISPLAY_PRIMARY, (level - 1)/254.0f));
  }
}

Return<void> HWCSession::getPanelBrightness(getPanelBrightness_cb _hidl_cb) {
  float brightness = -1.0f;
  int32_t error = -EINVAL;

  error = getDisplayBrightness(HWC_DISPLAY_PRIMARY, &brightness);
  if (brightness == -1.0f) {
    _hidl_cb(error, 0);
  } else {
    _hidl_cb(error, static_cast<uint32_t>(254.0f*brightness + 1));
  }

  return Void();
}

int32_t HWCSession::MinHdcpEncryptionLevelChanged(int disp_id, uint32_t min_enc_level) {
  DLOGI("Display %d", disp_id);

  // SSG team hardcoded disp_id as external because it applies to external only but SSG team sends
  // this level irrespective of external connected or not. So to honor the call, make disp_id to
  // primary & set level.
  disp_id = HWC_DISPLAY_PRIMARY;
  int disp_idx = GetDisplayIndex(disp_id);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_id);
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
  HWCDisplay *hwc_display = hwc_display_[disp_idx];
  if (!hwc_display) {
    DLOGE("Display = %d is not connected.", disp_idx);
    return -EINVAL;
  }

  return hwc_display->OnMinHdcpEncryptionLevelChange(min_enc_level);
}

Return<int32_t> HWCSession::minHdcpEncryptionLevelChanged(IDisplayConfig::DisplayType dpy,
                                                          uint32_t min_enc_level) {
  return MinHdcpEncryptionLevelChanged(MapDisplayType(dpy), min_enc_level);
}

Return<int32_t> HWCSession::refreshScreen() {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);
  callbacks_.Refresh(HWC_DISPLAY_PRIMARY);

  return 0;
}

int32_t HWCSession::ControlPartialUpdate(int disp_id, bool enable) {
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
  callbacks_.Refresh(HWC_DISPLAY_PRIMARY);

  // Wait until partial update control is complete
  int32_t error = locker_[disp_idx].WaitFinite(kCommitDoneTimeoutMs);

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

    SCOPE_LOCK(locker_[disp_id]);
    HWCDisplay *hwc_display = hwc_display_[disp_idx];
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
    hdr_caps.supportedHdrTypes.resize(out_num_types);

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
  if (null_display_mode_) {
    return 0;
  }

  if (!core_intf_) {
    DLOGW("core_intf_ not initialized.");
    return -ENOENT;
  }

  HWBwModes mode = on > 0 ? kBwVFEOn : kBwVFEOff;

  if (core_intf_->SetMaxBandwidthMode(mode) != kErrorNone) {
    return -EINVAL;
  }

  // trigger invalidate to apply new bw caps.
  callbacks_.Refresh(0);

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

Return<int32_t> HWCSession::IdlePowerCollapse(bool enable, bool synchronous) {
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
        callbacks_.Refresh(active_builtin_disp_id);
        int32_t error = locker_[active_builtin_disp_id].WaitFinite(kCommitDoneTimeoutMs);
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

  DLOGW("Display = %d is not connected.", UINT32(active_builtin_disp_id));
  return -ENODEV;
}

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

int32_t HWCSession::getDisplayMaxBrightness(uint32_t display, uint32_t *max_brightness_level) {
  if (!max_brightness_level) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  int32_t error = -EINVAL;
  HWCDisplay *hwc_display = hwc_display_[display];
  if (hwc_display && hwc_display_[display]->GetDisplayClass() == DISPLAY_CLASS_BUILTIN) {
    error = INT32(hwc_display_[display]->GetPanelMaxBrightness(max_brightness_level));
    if (error) {
      DLOGE("Failed to get the panel max brightness, display %u error %d", display, error);
    }
  }

  return error;
}

int32_t HWCSession::setDisplayBrightness(uint32_t display, float brightness) {
  return SetDisplayBrightness(static_cast<hwc2_display_t>(display), brightness);
}

#ifndef DISPLAY_CONFIG_VERSION_OPTIMAL
Return<int32_t> HWCSession::setDisplayAnimating(uint64_t display_id, bool animating ) {
  return CallDisplayFunction(display_id,
                             &HWCDisplay::SetDisplayAnimating, animating);
}

Return<int32_t> HWCSession::setDisplayIndex(IDisplayConfig::DisplayTypeExt disp_type,
                                            uint32_t base, uint32_t count) {
  return -1;
}

Return<int32_t> HWCSession::controlIdlePowerCollapse(bool enable, bool synchronous) {
  return IdlePowerCollapse(enable, synchronous);
}

Return<void> HWCSession::getWriteBackCapabilities(getWriteBackCapabilities_cb _hidl_cb) {
  int value = 0;
  IDisplayConfig::WriteBackCapabilities wb_caps = {};
  int32_t error = IsWbUbwcSupported(&value);
  wb_caps.isWbUbwcSupported = value;
  _hidl_cb(error, wb_caps);

  return Void();
}

Return<int32_t> HWCSession::SetDisplayDppsAdROI(uint32_t display_id, uint32_t h_start,
                                                uint32_t h_end, uint32_t v_start, uint32_t v_end,
                                                uint32_t factor_in, uint32_t factor_out) {
  return CallDisplayFunction(display_id,
                             &HWCDisplay::SetDisplayDppsAdROI, h_start, h_end, v_start, v_end,
                             factor_in, factor_out);
}

Return<int32_t> HWCSession::updateVSyncSourceOnPowerModeOff() {
  update_vsync_on_power_off_ = true;

  return 0;
}

Return<int32_t> HWCSession::updateVSyncSourceOnPowerModeDoze() {
  update_vsync_on_doze_ = true;

  return 0;
}

Return<bool> HWCSession::isPowerModeOverrideSupported(uint32_t disp_id) {
  if (!async_powermode_ || (disp_id > HWCCallbacks::kNumRealDisplays)) {
    return false;
  }

  return true;
}

Return<int32_t> HWCSession::setPowerMode(uint32_t disp_id, PowerMode power_mode) {
  SCOPE_LOCK(display_config_locker_);

  if (!isPowerModeOverrideSupported(disp_id)) {
    return 0;
  }

  // Active builtin display needs revalidation
  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  HWC2::PowerMode previous_mode = hwc_display_[disp_id]->GetCurrentPowerMode();

  DLOGI("disp_id: %d power_mode: %d", disp_id, power_mode);
  HWCDisplay::HWCLayerStack stack = {};
  hwc2_display_t dummy_disp_id = map_hwc_display_.at(disp_id);

  // Power state transition start.
  power_state_[disp_id].Lock();   // Acquire the display's power-state transition var read lock.
  power_state_transition_[disp_id] = true;
  locker_[disp_id].Lock();        // Lock the real display.
  locker_[dummy_disp_id].Lock();  // Lock the corresponding dummy display.

  // Place the real display's layer-stack on the dummy display.
  hwc_display_[disp_id]->GetLayerStack(&stack);
  hwc_display_[dummy_disp_id]->SetLayerStack(&stack);
  hwc_display_[dummy_disp_id]->UpdatePowerMode(hwc_display_[disp_id]->GetCurrentPowerMode());

  locker_[dummy_disp_id].Unlock();  // Release the dummy display.
  power_state_[disp_id].Unlock();   // Release the display's power-state transition var read lock.

  // From now, till power-state transition ends, for operations that need to be non-blocking, do
  // those operations on the dummy display.

  // Perform the actual [synchronous] power-state change.
  hwc_display_[disp_id]->SetPowerMode(static_cast<HWC2::PowerMode>(power_mode),
                                      false /* teardown */);

  // Power state transition end.
  power_state_[disp_id].Lock();   // Acquire the display's power-state transition var read lock.
  power_state_transition_[disp_id] = false;
  locker_[dummy_disp_id].Lock();  // Lock the dummy display.

  // Retrieve the real display's layer-stack from the dummy display.
  hwc_display_[dummy_disp_id]->GetLayerStack(&stack);
  hwc_display_[disp_id]->SetLayerStack(&stack);
  // Read display has got layerstack. Update the fences.
  hwc_display_[disp_id]->PostPowerMode();

  locker_[dummy_disp_id].Unlock();  // Release the dummy display.
  locker_[disp_id].Unlock();        // Release the real display.
  power_state_[disp_id].Unlock();   // Release the display's power-state transition var read lock.

  HWC2::PowerMode new_mode = hwc_display_[disp_id]->GetCurrentPowerMode();
  if (active_builtin_disp_id < HWCCallbacks::kNumRealDisplays &&
      hwc_display_[disp_id]->IsFirstCommitDone() &&
      WaitForResourceNeeded(previous_mode, new_mode)) {
    WaitForResources(true, active_builtin_disp_id, disp_id);
  }

  return 0;
}

Return<bool> HWCSession::isHDRSupported(uint32_t disp_id) {
  if (disp_id < 0 || disp_id >= HWCCallbacks::kNumDisplays) {
    DLOGE("Not valid display");
    return false;
  }
  SCOPE_LOCK(hdr_locker_[disp_id]);

  if (is_hdr_display_.size() <= disp_id) {
    DLOGW("is_hdr_display_ is not initialized for display %d!! Reporting it as HDR not supported",
          disp_id);
    return false;
  }

  return static_cast<bool>(is_hdr_display_[disp_id]);
}

Return<bool> HWCSession::isWCGSupported(uint32_t disp_id) {
  // todo(user): Query wcg from sdm. For now assume them same.
  return isHDRSupported(disp_id);
}

Return<int32_t> HWCSession::setLayerAsMask(uint32_t disp_id, uint64_t layer_id) {
  SCOPE_LOCK(locker_[disp_id]);
  HWCDisplay *hwc_display = hwc_display_[disp_id];
  if (!hwc_display) {
    DLOGW("Display = %d is not connected.", disp_id);
    return -EINVAL;
  }

  if (disable_mask_layer_hint_) {
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

Return<void> HWCSession::getDebugProperty(const hidl_string &prop_name,
                                          getDebugProperty_cb _hidl_cb) {
  std::string vendor_prop_name = DISP_PROP_PREFIX;
  char value[64] = {};
  hidl_string result = "";
  int32_t error = -EINVAL;

  vendor_prop_name += prop_name.c_str();
  if (HWCDebugHandler::Get()->GetProperty(vendor_prop_name.c_str(), value) == kErrorNone) {
    result = value;
    error = 0;
  }

  _hidl_cb(result, error);

  return Void();
}

Return<void> HWCSession::getActiveBuiltinDisplayAttributes(
                                          getDisplayAttributes_cb _hidl_cb) {
  int32_t error = -EINVAL;
  IDisplayConfig::DisplayAttributes display_attributes = {};
  hwc2_display_t disp_id = GetActiveBuiltinDisplay();

  if (disp_id >= HWCCallbacks::kNumDisplays) {
    DLOGE("Invalid display = %d", UINT32(disp_id));
  } else {
    if (hwc_display_[disp_id]) {
      uint32_t config_index = 0;
      HWC2::Error ret = hwc_display_[disp_id]->GetActiveConfig(&config_index);
      if (ret != HWC2::Error::None) {
        goto err;
      }
      DisplayConfigVariableInfo var_info;
      error = hwc_display_[disp_id]->GetDisplayAttributesForConfig(INT(config_index), &var_info);
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

err:
  _hidl_cb(error, display_attributes);

  return Void();
}

Return<int32_t> HWCSession::setPanelLuminanceAttributes(uint32_t disp_id, float pan_min_lum,
                                                        float pan_max_lum) {
  // currently doing only for virtual display
  if (disp_id != qdutils::DISPLAY_VIRTUAL) {
    return -EINVAL;
  }

  // check for out of range luminance values
  if (pan_min_lum <= 0.0f || pan_min_lum >= 1.0f ||
      pan_max_lum <= 100.0f || pan_max_lum >= 1000.0f) {
    return -EINVAL;
  }

  std::lock_guard<std::mutex> obj(mutex_lum_);
  set_min_lum_ = pan_min_lum;
  set_max_lum_ = pan_max_lum;
  DLOGI("set max_lum %f, min_lum %f", set_max_lum_, set_min_lum_);

  return 0;
}

Return<bool> HWCSession::isBuiltInDisplay(uint32_t disp_id) {
  if ((map_info_primary_.client_id == disp_id) && (map_info_primary_.disp_type == kBuiltIn))
    return true;

  for (auto &info : map_info_builtin_) {
    if (disp_id == info.client_id) {
      return true;
    }
  }

  return false;
}

Return<void> HWCSession::getSupportedDSIBitClks(uint32_t disp_id,
                                                getSupportedDSIBitClks_cb _hidl_cb) {
  SCOPE_LOCK(locker_[disp_id]);
  if (!hwc_display_[disp_id]) {
    return Void();
  }

  std::vector<uint64_t> bit_clks;
  hwc_display_[disp_id]->GetSupportedDSIClock(&bit_clks);

  hidl_vec<uint64_t> hidl_bit_clks = bit_clks;
  _hidl_cb(hidl_bit_clks);

  return Void();
}

Return<uint64_t> HWCSession::getDSIClk(uint32_t disp_id) {
  SCOPE_LOCK(locker_[disp_id]);
  if (!hwc_display_[disp_id]) {
    return 0;
  }

  uint64_t bit_clk = 0;
  hwc_display_[disp_id]->GetDynamicDSIClock(&bit_clk);

  return bit_clk;
}

Return<int32_t> HWCSession::setDSIClk(uint32_t disp_id, uint64_t bit_clk) {
  SCOPE_LOCK(locker_[disp_id]);
  if (!hwc_display_[disp_id]) {
    return -1;
  }

  return hwc_display_[disp_id]->SetDynamicDSIClock(bit_clk);
}

Return<int32_t> HWCSession::setCWBOutputBuffer(const sp<IDisplayCWBCallback> &callback,
                                               uint32_t disp_id, const Rect &rect,
                                               bool post_processed, const hidl_handle& buffer) {
  if (!callback || !buffer.getNativeHandle()) {
    DLOGE("Invalid parameters");
    return -1;
  }

  if (disp_id != qdutils::DISPLAY_PRIMARY) {
    DLOGE("Only supported for primary display at present.");
    return -1;
  }

  if (rect.left || rect.top || rect.right || rect.bottom) {
    DLOGE("Cropping rectangle is not supported.");
    return -1;
  }

  // Output buffer dump is not supported, if External or Virtual display is present.
  int external_dpy_index = GetDisplayIndex(qdutils::DISPLAY_EXTERNAL);
  int virtual_dpy_index = GetDisplayIndex(qdutils::DISPLAY_VIRTUAL);

  if (((external_dpy_index != -1) && hwc_display_[external_dpy_index]) ||
      ((virtual_dpy_index != -1) && hwc_display_[virtual_dpy_index])) {
    DLOGW("Output buffer dump is not supported with External or Virtual display!");
    return -1;
  }

  // Mutex scope
  {
    SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);
    if (!hwc_display_[disp_id]) {
      DLOGE("Display is not created yet.");
      return -1;
    }
  }

  return cwb_.PostBuffer(callback, post_processed, buffer);
}

int32_t HWCSession::CWB::PostBuffer(const sp<IDisplayCWBCallback> &callback, bool post_processed,
                                    const hidl_handle& buffer) {
  SCOPE_LOCK(queue_lock_);

  // Ensure that async task runs only until all queued CWB requests have been fulfilled.
  // If cwb queue is empty, async task has not either started or async task has finished
  // processing previously queued cwb requests. Start new async task on such a case as
  // currently running async task will automatically desolve without processing more requests.
  bool post_future = !queue_.size();

  QueueNode *node = new QueueNode(callback, post_processed, buffer);
  queue_.push(node);

  if (post_future) {
    // No need to do future.get() here for previously running async task. Async method will
    // guarantee to exit after cwb for all queued requests is indeed complete i.e. the respective
    // fences have signaled and client is notified through registered callbacks. This will make
    // sure that the new async task does not concurrently work with previous task. Let async running
    // thread dissolve on its own.
    future_ = std::async(HWCSession::CWB::AsyncTask, this);
  }

  return 0;
}

void HWCSession::CWB::ProcessRequests() {
  HWCDisplay *hwc_display = hwc_session_->hwc_display_[HWC_DISPLAY_PRIMARY];
  Locker &locker = hwc_session_->locker_[HWC_DISPLAY_PRIMARY];

  while (true) {
    QueueNode *node = nullptr;
    int status = 0;

    // Mutex scope
    // Just check if there is a next cwb request queued, exit the thread if nothing is pending.
    // Do not keep mutex locked so that client can freely queue more jobs to the current thread.
    {
      SCOPE_LOCK(queue_lock_);
      if (!queue_.size()) {
        break;
      }

      node = queue_.front();
    }

    // Configure cwb parameters, trigger refresh, wait for commit, get the release fence and
    // wait for fence to signal.

    // Mutex scope
    // Wait for previous commit to finish before configuring next buffer.
    {
      SEQUENCE_WAIT_SCOPE_LOCK(locker);
      if (hwc_display->SetReadbackBuffer(node->buffer.getNativeHandle(), -1, node->post_processed,
                                            kCWBClientExternal) != HWC2::Error::None) {
        DLOGE("CWB buffer could not be set.");
        status = -1;
      }
    }

    if (!status) {
      hwc_session_->callbacks_.Refresh(HWC_DISPLAY_PRIMARY);

      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock);

      int release_fence = -1;
      // Mutex scope
      {
        SCOPE_LOCK(locker);
        hwc_display->GetReadbackBufferFence(&release_fence);
      }

      if (release_fence >= 0) {
        status = sync_wait(release_fence, 1000);
        close(release_fence);
      } else {
        DLOGE("CWB release fence could not be retrieved.");
        status = -1;
      }
    }

    // Notify client about buffer status and erase the node from pending request queue.
    if (!status) {
      node->callback->onBufferReady(node->buffer);
    } else {
      node->callback->onBufferError(node->buffer);
    }

    delete node;

    // Mutex scope
    // Make sure to exit here, if queue becomes empty after erasing current node from queue,
    // so that the current async task does not operate concurrently with a new future task.
    {
      SCOPE_LOCK(queue_lock_);
      queue_.pop();

      if (!queue_.size()) {
        break;
      }
    }
  }
}

void HWCSession::CWB::AsyncTask(CWB *cwb) {
  cwb->ProcessRequests();
}

void HWCSession::CWB::PresentDisplayDone(hwc2_display_t disp_id) {
  if (disp_id != HWC_DISPLAY_PRIMARY) {
    return;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  cv_.notify_one();
}

Return<int32_t> HWCSession::setQsyncMode(uint32_t disp_id, IDisplayConfig::QsyncMode mode) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);
  if (!hwc_display_[disp_id]) {
    return -1;
  }

  QSyncMode qsync_mode = kQSyncModeNone;
  switch (mode) {
    case IDisplayConfig::QsyncMode::NONE:
      qsync_mode = kQSyncModeNone;
      break;
    case IDisplayConfig::QsyncMode::WAIT_FOR_FENCES_ONE_FRAME:
      qsync_mode = kQsyncModeOneShot;
      break;
    case IDisplayConfig::QsyncMode::WAIT_FOR_FENCES_EACH_FRAME:
      qsync_mode = kQsyncModeOneShotContinuous;
      break;
    case IDisplayConfig::QsyncMode::WAIT_FOR_COMMIT_EACH_FRAME:
      qsync_mode = kQSyncModeContinuous;
      break;
  }

  hwc_display_[disp_id]->SetQSyncMode(qsync_mode);
  return 0;
}

Return<bool> HWCSession::isSmartPanelConfig(uint32_t disp_id, uint32_t config_id) {
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

Return<bool> HWCSession::isAsyncVDSCreationSupported() {
  if (!async_vds_creation_) {
    return false;
  }

  return true;
}

Return<int32_t> HWCSession::createVirtualDisplay(uint32_t width, uint32_t height, int32_t format) {
  if (!async_vds_creation_) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  if (!width || !height) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  auto status = CreateVirtualDisplayObj(width, height, &format, &virtual_id_);
  if (status == HWC2::Error::None) {
    DLOGI("Created virtual display id:%" PRIu64 ", res: %dx%d", virtual_id_, width, height);
    if (active_builtin_disp_id < HWCCallbacks::kNumRealDisplays) {
      WaitForResources(true, active_builtin_disp_id, virtual_id_);
    }
  } else {
    DLOGE("Failed to create virtual display: %s", to_string(status).c_str());
  }

  return INT32(status);
}

Return<bool> HWCSession::isRotatorSupportedFormat(int hal_format, bool ubwc) {
  if (!core_intf_) {
    DLOGW("core_intf_ not initialized.");
    return false;
  }
  int flag = ubwc ? private_handle_t::PRIV_FLAGS_UBWC_ALIGNED : 0;

  LayerBufferFormat sdm_format = HWCLayer::GetSDMFormat(hal_format, flag);

  return core_intf_->IsRotatorSupportedFormat(sdm_format);
}

Return<int32_t> HWCSession::registerQsyncCallback(const sp<IDisplayQsyncCallback> &callback) {
  if (qsync_callback_ != nullptr) {
    DLOGE("Qsync callback already registered, rejecting new request");
    return -1;
  }
  qsync_callback_ = callback;

  return 0;
}
#endif

}  // namespace sdm
