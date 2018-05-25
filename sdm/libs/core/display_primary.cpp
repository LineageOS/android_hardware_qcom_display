/*
* Copyright (c) 2014 - 2018, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted
* provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright notice, this list of
*      conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright notice, this list of
*      conditions and the following disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation nor the names of its contributors may be used to
*      endorse or promote products derived from this software without specific prior written
*      permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/rect.h>
#include <map>
#include <algorithm>
#include <functional>
#include <vector>

#include "display_primary.h"
#include "hw_interface.h"
#include "hw_info_interface.h"

#define __CLASS__ "DisplayPrimary"

namespace sdm {

DisplayPrimary::DisplayPrimary(DisplayEventHandler *event_handler, HWInfoInterface *hw_info_intf,
                               BufferSyncHandler *buffer_sync_handler,
                               BufferAllocator *buffer_allocator, CompManager *comp_manager)
  : DisplayBase(kPrimary, event_handler, kDevicePrimary, buffer_sync_handler, buffer_allocator,
                comp_manager, hw_info_intf) {
}

DisplayError DisplayPrimary::Init() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  DisplayError error = HWInterface::Create(kPrimary, hw_info_intf_, buffer_sync_handler_,
                                           buffer_allocator_, &hw_intf_);
  if (error != kErrorNone) {
    DLOGE("Failed to create hardware interface on. Error = %d", error);
    return error;
  }

  error = DisplayBase::Init();
  if (error != kErrorNone) {
    HWInterface::Destroy(hw_intf_);
    return error;
  }

  if (hw_panel_info_.mode == kModeCommand && Debug::IsVideoModeEnabled()) {
    error = hw_intf_->SetDisplayMode(kModeVideo);
    if (error != kErrorNone) {
      DLOGW("Retaining current display mode. Current = %d, Requested = %d", hw_panel_info_.mode,
            kModeVideo);
    }
  }

  if (hw_panel_info_.mode == kModeCommand) {
    event_list_ = { HWEvent::VSYNC,
                    HWEvent::EXIT,
                    HWEvent::SHOW_BLANK_EVENT,
                    HWEvent::THERMAL_LEVEL,
                    HWEvent::IDLE_POWER_COLLAPSE,
                    HWEvent::PINGPONG_TIMEOUT,
                    HWEvent::PANEL_DEAD,
                    HWEvent::HW_RECOVERY };
  } else {
    event_list_ = { HWEvent::VSYNC,
                    HWEvent::EXIT,
                    HWEvent::IDLE_NOTIFY,
                    HWEvent::SHOW_BLANK_EVENT,
                    HWEvent::THERMAL_LEVEL,
                    HWEvent::PINGPONG_TIMEOUT,
                    HWEvent::PANEL_DEAD,
                    HWEvent::HW_RECOVERY };
  }

  avr_prop_disabled_ = Debug::IsAVRDisabled();

  error = HWEventsInterface::Create(INT(display_type_), this, event_list_, hw_intf_,
                                    &hw_events_intf_);
  if (error != kErrorNone) {
    DisplayBase::Deinit();
    HWInterface::Destroy(hw_intf_);
    DLOGE("Failed to create hardware events interface on. Error = %d", error);
  }

  current_refresh_rate_ = hw_panel_info_.max_fps;

  return error;
}

DisplayError DisplayPrimary::Deinit() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  dpps_info_.Deinit();
  return DisplayBase::Deinit();
}

DisplayError DisplayPrimary::Prepare(LayerStack *layer_stack) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  uint32_t new_mixer_width = 0;
  uint32_t new_mixer_height = 0;
  uint32_t display_width = display_attributes_.x_pixels;
  uint32_t display_height = display_attributes_.y_pixels;

  if (reset_panel_) {
    DLOGW("panel is in bad state, resetting the panel");
    ResetPanel();
    reset_panel_ = false;
  }

  if (NeedsMixerReconfiguration(layer_stack, &new_mixer_width, &new_mixer_height)) {
    error = ReconfigureMixer(new_mixer_width, new_mixer_height);
    if (error != kErrorNone) {
      ReconfigureMixer(display_width, display_height);
    }
  }

  // Clean hw layers for reuse.
  hw_layers_ = HWLayers();
  hw_layers_.hw_avr_info.enable = NeedsAVREnable();

  return DisplayBase::Prepare(layer_stack);
}

DisplayError DisplayPrimary::Commit(LayerStack *layer_stack) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  uint32_t app_layer_count = hw_layers_.info.app_layer_count;

  // Enabling auto refresh is async and needs to happen before commit ioctl
  if (hw_panel_info_.mode == kModeCommand) {
    bool enable = (app_layer_count == 1) && layer_stack->flags.single_buffered_layer_present;
    bool need_refresh = layer_stack->flags.single_buffered_layer_present && (app_layer_count > 1);

    hw_intf_->SetAutoRefresh(enable);
    if (need_refresh) {
      event_handler_->Refresh();
    }
  }

  error = DisplayBase::Commit(layer_stack);
  if (error != kErrorNone) {
    return error;
  }

  DisplayBase::ReconfigureDisplay();

  int idle_time_ms = hw_layers_.info.set_idle_time_ms;
  if (idle_time_ms >= 0) {
    hw_intf_->SetIdleTimeoutMs(UINT32(idle_time_ms));
  }

  if (switch_to_cmd_) {
    uint32_t pending;
    switch_to_cmd_ = false;
    ControlPartialUpdate(true /* enable */, &pending);
  }

  dpps_info_.Init(this);

  return error;
}

DisplayError DisplayPrimary::SetDisplayState(DisplayState state, int *release_fence) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  error = DisplayBase::SetDisplayState(state, release_fence);
  if (error != kErrorNone) {
    return error;
  }

  // Set vsync enable state to false, as driver disables vsync during display power off.
  if (state == kStateOff) {
    vsync_enable_ = false;
  }

  return kErrorNone;
}

void DisplayPrimary::SetIdleTimeoutMs(uint32_t active_ms) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  comp_manager_->SetIdleTimeoutMs(display_comp_ctx_, active_ms);
}

DisplayError DisplayPrimary::SetDisplayMode(uint32_t mode) {
  DisplayError error = kErrorNone;

  // Limit scope of mutex to this block
  {
    lock_guard<recursive_mutex> obj(recursive_mutex_);
    HWDisplayMode hw_display_mode = static_cast<HWDisplayMode>(mode);
    uint32_t pending = 0;

    if (!active_) {
      DLOGW("Invalid display state = %d. Panel must be on.", state_);
      return kErrorNotSupported;
    }

    if (hw_display_mode != kModeCommand && hw_display_mode != kModeVideo) {
      DLOGW("Invalid panel mode parameters. Requested = %d", hw_display_mode);
      return kErrorParameters;
    }

    if (hw_display_mode == hw_panel_info_.mode) {
      DLOGW("Same display mode requested. Current = %d, Requested = %d", hw_panel_info_.mode,
            hw_display_mode);
      return kErrorNone;
    }

    error = hw_intf_->SetDisplayMode(hw_display_mode);
    if (error != kErrorNone) {
      DLOGW("Retaining current display mode. Current = %d, Requested = %d", hw_panel_info_.mode,
            hw_display_mode);
      return error;
    }

    if (mode == kModeVideo) {
      ControlPartialUpdate(false /* enable */, &pending);
    } else if (mode == kModeCommand) {
      // Flush idle timeout value currently set.
      comp_manager_->SetIdleTimeoutMs(display_comp_ctx_, 0);
      switch_to_cmd_ = true;
    }
  }

  // Request for a new draw cycle. New display mode will get applied on next draw cycle.
  // New idle time will get configured as part of this.
  event_handler_->Refresh();

  return error;
}

DisplayError DisplayPrimary::SetPanelBrightness(int level) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  return hw_intf_->SetPanelBrightness(level);
}

DisplayError DisplayPrimary::CachePanelBrightness(int level) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  return hw_intf_->CachePanelBrightness(level);
}

DisplayError DisplayPrimary::GetRefreshRateRange(uint32_t *min_refresh_rate,
                                                 uint32_t *max_refresh_rate) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;

  if (hw_panel_info_.min_fps && hw_panel_info_.max_fps) {
    *min_refresh_rate = hw_panel_info_.min_fps;
    *max_refresh_rate = hw_panel_info_.max_fps;
  } else {
    error = DisplayBase::GetRefreshRateRange(min_refresh_rate, max_refresh_rate);
  }

  return error;
}

DisplayError DisplayPrimary::SetRefreshRate(uint32_t refresh_rate, bool final_rate) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  if (!active_ || !hw_panel_info_.dynamic_fps) {
    return kErrorNotSupported;
  }

  if (refresh_rate < hw_panel_info_.min_fps || refresh_rate > hw_panel_info_.max_fps) {
    DLOGE("Invalid Fps = %d request", refresh_rate);
    return kErrorParameters;
  }

  if (handle_idle_timeout_ && !final_rate) {
    refresh_rate = hw_panel_info_.min_fps;
  }

  if ((current_refresh_rate_ != refresh_rate) || handle_idle_timeout_) {
    DisplayError error = hw_intf_->SetRefreshRate(refresh_rate);
    if (error != kErrorNone) {
      return error;
    }
  }

  // On success, set current refresh rate to new refresh rate
  current_refresh_rate_ = refresh_rate;
  handle_idle_timeout_ = false;

  return DisplayBase::ReconfigureDisplay();
}

DisplayError DisplayPrimary::VSync(int64_t timestamp) {
  if (vsync_enable_) {
    DisplayEventVSync vsync;
    vsync.timestamp = timestamp;
    event_handler_->VSync(vsync);
  }

  return kErrorNone;
}

void DisplayPrimary::IdleTimeout() {
  if (hw_panel_info_.mode == kModeVideo) {
    event_handler_->HandleEvent(kIdleTimeout);
    handle_idle_timeout_ = true;
    event_handler_->Refresh();
    lock_guard<recursive_mutex> obj(recursive_mutex_);
    comp_manager_->ProcessIdleTimeout(display_comp_ctx_);
  }
}

void DisplayPrimary::PingPongTimeout() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  hw_intf_->DumpDebugData();
}

void DisplayPrimary::ThermalEvent(int64_t thermal_level) {
  event_handler_->HandleEvent(kThermalEvent);
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  comp_manager_->ProcessThermalEvent(display_comp_ctx_, thermal_level);
}

void DisplayPrimary::IdlePowerCollapse() {
  if (hw_panel_info_.mode == kModeCommand) {
    event_handler_->HandleEvent(kIdlePowerCollapse);
    lock_guard<recursive_mutex> obj(recursive_mutex_);
    comp_manager_->ProcessIdlePowerCollapse(display_comp_ctx_);
  }
}

void DisplayPrimary::PanelDead() {
  event_handler_->HandleEvent(kPanelDeadEvent);
  event_handler_->Refresh();
  {
    lock_guard<recursive_mutex> obj(recursive_mutex_);
    reset_panel_ = true;
  }
}

// HWEventHandler overload, not DisplayBase
void DisplayPrimary::HwRecovery(const HWRecoveryEvent sdm_event_code) {
  DisplayBase::HwRecovery(sdm_event_code);
}

DisplayError DisplayPrimary::GetPanelBrightness(int *level) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  return hw_intf_->GetPanelBrightness(level);
}

DisplayError DisplayPrimary::ControlPartialUpdate(bool enable, uint32_t *pending) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!pending) {
    return kErrorParameters;
  }

  if (!hw_panel_info_.partial_update) {
    // Nothing to be done.
    DLOGI("partial update is not applicable for display=%d", display_type_);
    return kErrorNotSupported;
  }

  *pending = 0;
  if (enable == partial_update_control_) {
    DLOGI("Same state transition is requested.");
    return kErrorNone;
  }

  partial_update_control_ = enable;

  if (!enable) {
    // If the request is to turn off feature, new draw call is required to have
    // the new setting into effect.
    *pending = 1;
  }

  return kErrorNone;
}

DisplayError DisplayPrimary::DisablePartialUpdateOneFrame() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  disable_pu_one_frame_ = true;

  return kErrorNone;
}

bool DisplayPrimary::NeedsAVREnable() {
  if (avr_prop_disabled_) {
    return false;
  }

  return (hw_panel_info_.mode == kModeVideo && ((hw_panel_info_.dynamic_fps &&
          hw_panel_info_.dfps_porch_mode) || (!hw_panel_info_.dynamic_fps &&
          hw_panel_info_.min_fps != hw_panel_info_.max_fps)));
}

void DisplayPrimary::ResetPanel() {
  DisplayError status = kErrorNone;
  int release_fence = -1;

  DLOGI("Powering off primary");
  status = SetDisplayState(kStateOff, &release_fence);
  if (status != kErrorNone) {
    DLOGE("power-off on primary failed with error = %d", status);
  }
  if (release_fence >= 0) {
    ::close(release_fence);
  }

  DLOGI("Restoring power mode on primary");
  DisplayState mode = GetLastPowerMode();
  status = SetDisplayState(mode, &release_fence);
  if (status != kErrorNone) {
    DLOGE("Setting power mode = %d on primary failed with error = %d", mode, status);
  }
  if (release_fence >= 0) {
    ::close(release_fence);
  }

  DLOGI("Enabling HWVsync");
  status = SetVSyncState(true);
  if (status != kErrorNone) {
    DLOGE("enabling vsync failed for primary with error = %d", status);
  }
}

DisplayError DisplayPrimary::DppsProcessOps(enum DppsOps op, void *payload, size_t size) {
  DisplayError error = kErrorNone;
  uint32_t pending;
  bool enable = false;

  switch (op) {
  case kDppsSetFeature:
    if (!payload) {
      DLOGE("Invalid payload parameter for op %d", op);
      error = kErrorParameters;
      break;
    }
    error = hw_intf_->SetDppsFeature(payload, size);
    break;
  case kDppsGetFeatureInfo:
    if (!payload) {
      DLOGE("Invalid payload parameter for op %d", op);
      error = kErrorParameters;
      break;
    }
    error = hw_intf_->GetDppsFeatureInfo(payload, size);
    break;
  case kDppsScreenRefresh:
    event_handler_->Refresh();
    break;
  case kDppsPartialUpdate:
    if (!payload) {
      DLOGE("Invalid payload parameter for op %d", op);
      error = kErrorParameters;
      break;
    }
    enable = *(reinterpret_cast<bool *>(payload));
    ControlPartialUpdate(enable, &pending);
    break;
  default:
    DLOGE("Invalid input op %d", op);
    error = kErrorParameters;
    break;
  }
  return error;
}

void DppsInfo::Init(DppsPropIntf* intf) {
  int error = 0;

  if (dpps_initialized_) {
    return;
  }

  if (!dpps_impl_lib.Open(kDppsLib)) {
    DLOGW("Failed to load Dpps lib %s", kDppsLib);
    goto exit;
  }

  if (!dpps_impl_lib.Sym("GetDppsInterface",
         reinterpret_cast<void **>(&GetDppsInterface))) {
    DLOGE("GetDppsInterface not found!, err %s", dlerror());
    goto exit;
  }

  dpps_intf = GetDppsInterface();
  if (!dpps_intf) {
    DLOGE("Failed to get Dpps Interface!");
    goto exit;
  }

  error = dpps_intf->Init(intf);
  if (!error) {
    DLOGI("DPPS Interface init successfully");
    dpps_initialized_ = true;
    return;
  } else {
    DLOGE("DPPS Interface init failure with err %d", error);
  }

exit:
  Deinit();
  dpps_intf = new DppsDummyImpl();
  dpps_initialized_ = true;
}

void DppsInfo::Deinit() {
  if (dpps_intf) {
    dpps_intf->Deinit();
    dpps_intf = NULL;
  }
  dpps_impl_lib.~DynLib();
}

DisplayError DisplayPrimary::HandleSecureEvent(SecureEvent secure_event) {
  return hw_intf_->HandleSecureEvent(secure_event);
}

}  // namespace sdm

