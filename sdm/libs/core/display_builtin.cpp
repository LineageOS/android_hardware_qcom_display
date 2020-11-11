/*
* Copyright (c) 2014 - 2020, The Linux Foundation. All rights reserved.
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
#include <utils/utils.h>

#include <algorithm>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "display_builtin.h"
#include "hw_info_interface.h"
#include "hw_interface.h"

#define __CLASS__ "DisplayBuiltIn"

namespace sdm {

DisplayBuiltIn::DisplayBuiltIn(DisplayEventHandler *event_handler, HWInfoInterface *hw_info_intf,
                               BufferSyncHandler *buffer_sync_handler,
                               BufferAllocator *buffer_allocator, CompManager *comp_manager)
  : DisplayBase(kBuiltIn, event_handler, kDeviceBuiltIn, buffer_sync_handler, buffer_allocator,
                comp_manager, hw_info_intf) {}

DisplayBuiltIn::DisplayBuiltIn(int32_t display_id, DisplayEventHandler *event_handler,
                               HWInfoInterface *hw_info_intf,
                               BufferSyncHandler *buffer_sync_handler,
                               BufferAllocator *buffer_allocator, CompManager *comp_manager)
  : DisplayBase(display_id, kBuiltIn, event_handler, kDeviceBuiltIn, buffer_sync_handler,
                buffer_allocator, comp_manager, hw_info_intf) {}

DisplayBuiltIn::~DisplayBuiltIn() {
  CloseFd(&previous_retire_fence_);
}

DisplayError DisplayBuiltIn::Init() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  DisplayError error = HWInterface::Create(display_id_, kBuiltIn, hw_info_intf_,
                                           buffer_sync_handler_, buffer_allocator_, &hw_intf_);
  if (error != kErrorNone) {
    DLOGE("Failed to create hardware interface on. Error = %d", error);
    return error;
  }

  if (-1 == display_id_) {
    hw_intf_->GetDisplayId(&display_id_);
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
    event_list_ = {HWEvent::VSYNC,
                   HWEvent::EXIT,
                   HWEvent::IDLE_NOTIFY,
                   HWEvent::SHOW_BLANK_EVENT,
                   HWEvent::THERMAL_LEVEL,
                   HWEvent::IDLE_POWER_COLLAPSE,
                   HWEvent::PINGPONG_TIMEOUT,
                   HWEvent::PANEL_DEAD,
                   HWEvent::HW_RECOVERY};
  } else {
    event_list_ = {HWEvent::VSYNC,         HWEvent::EXIT,
                   HWEvent::IDLE_NOTIFY,   HWEvent::SHOW_BLANK_EVENT,
                   HWEvent::THERMAL_LEVEL, HWEvent::PINGPONG_TIMEOUT,
                   HWEvent::PANEL_DEAD,    HWEvent::HW_RECOVERY};
  }

  avr_prop_disabled_ = Debug::IsAVRDisabled();

  error = HWEventsInterface::Create(display_id_, kBuiltIn, this, event_list_, hw_intf_,
                                    &hw_events_intf_);
  if (error != kErrorNone) {
    DisplayBase::Deinit();
    HWInterface::Destroy(hw_intf_);
    DLOGE("Failed to create hardware events interface on. Error = %d", error);
  }

  current_refresh_rate_ = hw_panel_info_.max_fps;

  int value = 0;
  Debug::Get()->GetProperty(DEFER_FPS_FRAME_COUNT, &value);
  deferred_config_.frame_count = (value > 0) ? UINT32(value) : 0;

  return error;
}

DisplayError DisplayBuiltIn::Deinit() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  dpps_info_.Deinit();
  return DisplayBase::Deinit();
}

DisplayError DisplayBuiltIn::Prepare(LayerStack *layer_stack) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  uint32_t new_mixer_width = 0;
  uint32_t new_mixer_height = 0;
  uint32_t display_width = display_attributes_.x_pixels;
  uint32_t display_height = display_attributes_.y_pixels;

  DTRACE_SCOPED();
  if (NeedsMixerReconfiguration(layer_stack, &new_mixer_width, &new_mixer_height)) {
    error = ReconfigureMixer(new_mixer_width, new_mixer_height);
    if (error != kErrorNone) {
      ReconfigureMixer(display_width, display_height);
    }
  } else {
    if (CanSkipDisplayPrepare(layer_stack)) {
      hw_layers_.hw_avr_info.update = needs_avr_update_;
      hw_layers_.hw_avr_info.mode = GetAvrMode(qsync_mode_);
      return kErrorNone;
    }
  }

  // Clean hw layers for reuse.
  DTRACE_BEGIN("PrepareHWLayers");
  hw_layers_ = HWLayers();
  DTRACE_END();

  hw_layers_.hw_avr_info.update = needs_avr_update_;
  hw_layers_.hw_avr_info.mode = GetAvrMode(qsync_mode_);

  left_frame_roi_ = {};
  right_frame_roi_ = {};

  error = DisplayBase::Prepare(layer_stack);

  // Cache the Frame ROI.
  if (error == kErrorNone) {
    if (hw_layers_.info.left_frame_roi.size() && hw_layers_.info.right_frame_roi.size()) {
      left_frame_roi_ = hw_layers_.info.left_frame_roi.at(0);
      right_frame_roi_ = hw_layers_.info.right_frame_roi.at(0);
    }
  }

  return error;
}

HWAVRModes DisplayBuiltIn::GetAvrMode(QSyncMode mode) {
  switch (mode) {
     case kQSyncModeNone:
       return kQsyncNone;
     case kQSyncModeContinuous:
       return kContinuousMode;
     case kQsyncModeOneShot:
     case kQsyncModeOneShotContinuous:
       return kOneShotMode;
     default:
       return kQsyncNone;
  }
}

DisplayError DisplayBuiltIn::Commit(LayerStack *layer_stack) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  uint32_t app_layer_count = hw_layers_.info.app_layer_count;
  HWDisplayMode panel_mode = hw_panel_info_.mode;

  DTRACE_SCOPED();

  // Enabling auto refresh is async and needs to happen before commit ioctl
  if (hw_panel_info_.mode == kModeCommand) {
    bool enable = (app_layer_count == 1) && layer_stack->flags.single_buffered_layer_present;
    bool need_refresh = layer_stack->flags.single_buffered_layer_present && (app_layer_count > 1);

    hw_intf_->SetAutoRefresh(enable);
    if (need_refresh) {
      event_handler_->Refresh();
    }
  }

  if (vsync_enable_) {
    DTRACE_BEGIN("RegisterVsync");
    // wait for previous frame's retire fence to signal.
    buffer_sync_handler_->SyncWait(previous_retire_fence_);

    // Register for vsync and then commit the frame.
    hw_events_intf_->SetEventState(HWEvent::VSYNC, true);
    DTRACE_END();
  }

  error = DisplayBase::Commit(layer_stack);
  if (error != kErrorNone) {
    return error;
  }
  if (pending_brightness_) {
    buffer_sync_handler_->SyncWait(layer_stack->retire_fence_fd);
    SetPanelBrightness(cached_brightness_);
    pending_brightness_ = false;
  }

  if (commit_event_enabled_) {
    dpps_info_.DppsNotifyOps(kDppsCommitEvent, &display_type_, sizeof(display_type_));
  }

  deferred_config_.UpdateDeferCount();

  ReconfigureDisplay();

  if (deferred_config_.CanApplyDeferredState()) {
    event_handler_->HandleEvent(kInvalidateDisplay);
    deferred_config_.Clear();
  }

  int idle_time_ms = hw_layers_.info.set_idle_time_ms;
  if (idle_time_ms >= 0) {
    hw_intf_->SetIdleTimeoutMs(UINT32(idle_time_ms));
  }

  if (switch_to_cmd_) {
    uint32_t pending;
    switch_to_cmd_ = false;
    ControlPartialUpdate(true /* enable */, &pending);
  }

  if (panel_mode != hw_panel_info_.mode) {
    UpdateDisplayModeParams();
  }
  dpps_info_.Init(this, hw_panel_info_.panel_name);

  if (qsync_mode_ == kQsyncModeOneShot) {
    // Reset qsync mode.
    SetQSyncMode(kQSyncModeNone);
  } else if (qsync_mode_ == kQsyncModeOneShotContinuous) {
    // No action needed.
  } else if (qsync_mode_ == kQSyncModeContinuous) {
    needs_avr_update_ = false;
  } else if (qsync_mode_ == kQSyncModeNone) {
    needs_avr_update_ = false;
  }

  first_cycle_ = false;

  CloseFd(&previous_retire_fence_);
  previous_retire_fence_ = Sys::dup_(layer_stack->retire_fence_fd);

  return error;
}

void DisplayBuiltIn::UpdateDisplayModeParams() {
  if (hw_panel_info_.mode == kModeVideo) {
    uint32_t pending = 0;
    ControlPartialUpdate(false /* enable */, &pending);
  } else if (hw_panel_info_.mode == kModeCommand) {
    // Flush idle timeout value currently set.
    comp_manager_->SetIdleTimeoutMs(display_comp_ctx_, 0);
    switch_to_cmd_ = true;
  }
}

DisplayError DisplayBuiltIn::SetDisplayState(DisplayState state, bool teardown,
                                             int *release_fence) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  HWDisplayMode panel_mode = hw_panel_info_.mode;

  if ((state == kStateOn) && deferred_config_.IsDeferredState()) {
    SetDeferredFpsConfig();
  }

  error = DisplayBase::SetDisplayState(state, teardown, release_fence);
  if (error != kErrorNone) {
    return error;
  }

  if (hw_panel_info_.mode != panel_mode) {
    UpdateDisplayModeParams();
  }

  // Set vsync enable state to false, as driver disables vsync during display power off.
  if (state == kStateOff) {
    vsync_enable_ = false;
  }

  if (pending_doze_ || pending_power_on_) {
    event_handler_->Refresh();
  }

  return kErrorNone;
}

void DisplayBuiltIn::SetIdleTimeoutMs(uint32_t active_ms) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  comp_manager_->SetIdleTimeoutMs(display_comp_ctx_, active_ms);
}

DisplayError DisplayBuiltIn::SetDisplayMode(uint32_t mode) {
  DisplayError error = kErrorNone;

  // Limit scope of mutex to this block
  {
    lock_guard<recursive_mutex> obj(recursive_mutex_);
    HWDisplayMode hw_display_mode = static_cast<HWDisplayMode>(mode);
    uint32_t pending = 0;

    if (!active_ && !pending_doze_ && !pending_power_on_) {
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

    DisplayBase::ReconfigureDisplay();

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

DisplayError DisplayBuiltIn::SetPanelBrightness(float brightness) {
  lock_guard<recursive_mutex> obj(brightness_lock_);

  if (brightness != -1.0f && !(0.0f <= brightness && brightness <= 1.0f)) {
    DLOGE("Bad brightness value = %f", brightness);
    return kErrorParameters;
  }

  if (state_ == kStateOff) {
    return kErrorNone;
  }

  // -1.0f = off, 0.0f = min, 1.0f = max
  float level_remainder = 0.0f;
  int level = 0;
  if (brightness == -1.0f) {
    level = 0;
  } else {
    // Node only supports int level, so store the float remainder for accurate GetPanelBrightness
    float max = hw_panel_info_.panel_max_brightness;
    float min = hw_panel_info_.panel_min_brightness;
    if (min >= max) {
      DLOGE("Minimum brightness is greater than or equal to maximum brightness");
      return kErrorDriverData;
    }
    float t = (brightness * (max - min)) + min;
    level = static_cast<int>(t);
    level_remainder = t - level;
  }

  DisplayError err = hw_intf_->SetPanelBrightness(level);
  if (err == kErrorNone) {
    level_remainder_ = level_remainder;
    DLOGI_IF(kTagDisplay, "Setting brightness to level %d (%f percent)", level,
             brightness * 100);
  } else if (err == kErrorDeferred) {
    // TODO(user): I8508d64a55c3b30239c6ed2886df391407d22f25 causes mismatch between perceived
    // power state and actual panel power state. Requires a rework. Below check will set up
    // deferment of brightness operation if DAL reports defer use case.
    cached_brightness_ = brightness;
    pending_brightness_ = true;
    return kErrorNone;
  }

  return err;
}

DisplayError DisplayBuiltIn::GetRefreshRateRange(uint32_t *min_refresh_rate,
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

DisplayError DisplayBuiltIn::TeardownConcurrentWriteback(void) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  return hw_intf_->TeardownConcurrentWriteback();
}

DisplayError DisplayBuiltIn::SetRefreshRate(uint32_t refresh_rate, bool final_rate) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  if (!active_ || !hw_panel_info_.dynamic_fps || qsync_mode_ != kQSyncModeNone) {
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
      // Attempt to update refresh rate can fail if rf interfenence is detected.
      // Just drop min fps settting for now.
      handle_idle_timeout_ = false;
      return error;
    }

    if (handle_idle_timeout_) {
      is_idle_timeout_ = true;
    } else {
      is_idle_timeout_ = false;
    }

    error = comp_manager_->CheckEnforceSplit(display_comp_ctx_, refresh_rate);
    if (error != kErrorNone) {
      return error;
    }
  }

  // On success, set current refresh rate to new refresh rate
  current_refresh_rate_ = refresh_rate;
  handle_idle_timeout_ = false;
  deferred_config_.MarkDirty();

  return ReconfigureDisplay();
}

DisplayError DisplayBuiltIn::VSync(int64_t timestamp) {
  if (vsync_enable_ && !drop_hw_vsync_) {
    DisplayEventVSync vsync;
    vsync.timestamp = timestamp;
    event_handler_->VSync(vsync);
  }

  return kErrorNone;
}

void DisplayBuiltIn::IdleTimeout() {
  if (hw_panel_info_.mode == kModeVideo) {
    if (event_handler_->HandleEvent(kIdleTimeout) != kErrorNone) {
      return;
    }
    handle_idle_timeout_ = true;
    event_handler_->Refresh();
    lock_guard<recursive_mutex> obj(recursive_mutex_);
    comp_manager_->ProcessIdleTimeout(display_comp_ctx_);
  }
}

void DisplayBuiltIn::PingPongTimeout() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  hw_intf_->DumpDebugData();
}

void DisplayBuiltIn::ThermalEvent(int64_t thermal_level) {
  event_handler_->HandleEvent(kThermalEvent);
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  comp_manager_->ProcessThermalEvent(display_comp_ctx_, thermal_level);
}

void DisplayBuiltIn::IdlePowerCollapse() {
  if (hw_panel_info_.mode == kModeCommand) {
    event_handler_->HandleEvent(kIdlePowerCollapse);
    lock_guard<recursive_mutex> obj(recursive_mutex_);
    comp_manager_->ProcessIdlePowerCollapse(display_comp_ctx_);
  }
}

void DisplayBuiltIn::PanelDead() {
  event_handler_->HandleEvent(kPanelDeadEvent);
}

// HWEventHandler overload, not DisplayBase
void DisplayBuiltIn::HwRecovery(const HWRecoveryEvent sdm_event_code) {
  DisplayBase::HwRecovery(sdm_event_code);
}

DisplayError DisplayBuiltIn::GetPanelBrightness(float *brightness) {
  lock_guard<recursive_mutex> obj(brightness_lock_);

  DisplayError err = kErrorNone;
  int level = 0;
  if ((err = hw_intf_->GetPanelBrightness(&level)) != kErrorNone) {
    return err;
  }

  // -1.0f = off, 0.0f = min, 1.0f = max
  float max = hw_panel_info_.panel_max_brightness;
  float min = hw_panel_info_.panel_min_brightness;
  if (level == 0) {
    *brightness = -1.0f;
  } else if ((max > min) && (min <= level && level <= max)) {
    *brightness = (static_cast<float>(level) + level_remainder_ - min) / (max - min);
  } else {
    min >= max ? DLOGE("Minimum brightness is greater than or equal to maximum brightness") :
                 DLOGE("Invalid brightness level %d", level);
    return kErrorDriverData;
  }


  DLOGI_IF(kTagDisplay, "Received level %d (%f percent)", level, *brightness * 100);

  return kErrorNone;
}

DisplayError DisplayBuiltIn::ControlPartialUpdate(bool enable, uint32_t *pending) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!pending) {
    return kErrorParameters;
  }

  if (!hw_panel_info_.partial_update) {
    // Nothing to be done.
    DLOGI("partial update is not applicable for display id = %d", display_id_);
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

DisplayError DisplayBuiltIn::DisablePartialUpdateOneFrame() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  disable_pu_one_frame_ = true;

  return kErrorNone;
}

DisplayError DisplayBuiltIn::DppsProcessOps(enum DppsOps op, void *payload, size_t size) {
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
      {
        lock_guard<recursive_mutex> obj(recursive_mutex_);
        error = hw_intf_->SetDppsFeature(payload, size);
      }
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
    case kDppsRequestCommit:
      if (!payload) {
        DLOGE("Invalid payload parameter for op %d", op);
        error = kErrorParameters;
        break;
      }
      {
        lock_guard<recursive_mutex> obj(recursive_mutex_);
        commit_event_enabled_ = *(reinterpret_cast<bool *>(payload));
      }
      break;
    default:
      DLOGE("Invalid input op %d", op);
      error = kErrorParameters;
      break;
  }
  return error;
}

DisplayError DisplayBuiltIn::SetDisplayDppsAdROI(void *payload) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError err = kErrorNone;

  err = hw_intf_->SetDisplayDppsAdROI(payload);
  if (err != kErrorNone)
    DLOGE("Failed to set ad roi config, err %d", err);

  return err;
}

void DppsInfo::Init(DppsPropIntf *intf, const std::string &panel_name) {
  int error = 0;

  if (dpps_initialized_) {
    return;
  }

  if (!dpps_impl_lib.Open(kDppsLib)) {
    DLOGW("Failed to load Dpps lib %s", kDppsLib);
    goto exit;
  }

  if (!dpps_impl_lib.Sym("GetDppsInterface", reinterpret_cast<void **>(&GetDppsInterface))) {
    DLOGE("GetDppsInterface not found!, err %s", dlerror());
    goto exit;
  }

  dpps_intf = GetDppsInterface();
  if (!dpps_intf) {
    DLOGE("Failed to get Dpps Interface!");
    goto exit;
  }

  error = dpps_intf->Init(intf, panel_name);
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

void DppsInfo::DppsNotifyOps(enum DppsNotifyOps op, void *payload, size_t size) {
  int ret = 0;
  ret = dpps_intf->DppsNotifyOps(op, payload, size);
  if (ret)
    DLOGE("DppsNotifyOps op %d error %d", op, ret);
}

DisplayError DisplayBuiltIn::HandleSecureEvent(SecureEvent secure_event, LayerStack *layer_stack) {
  hw_layers_.info.stack = layer_stack;
  DisplayError err = hw_intf_->HandleSecureEvent(secure_event, &hw_layers_);
  if (err != kErrorNone) {
    return err;
  }
  comp_manager_->HandleSecureEvent(display_comp_ctx_, secure_event);

  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetQSyncMode(QSyncMode qsync_mode) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!hw_panel_info_.qsync_support || qsync_mode_ == qsync_mode || first_cycle_) {
    DLOGE("Failed: qsync_support: %d first_cycle %d mode: %d -> %d", hw_panel_info_.qsync_support,
          first_cycle_, qsync_mode_, qsync_mode);
    return kErrorNotSupported;
  }

  qsync_mode_ = qsync_mode;
  needs_avr_update_ = true;
  event_handler_->Refresh();

  return kErrorNone;
}

DisplayError DisplayBuiltIn::ControlIdlePowerCollapse(bool enable, bool synchronous) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!active_) {
    DLOGW("Invalid display state = %d. Panel must be on.", state_);
    return kErrorPermission;
  }
  if (hw_panel_info_.mode == kModeVideo) {
    DLOGW("Idle power collapse not supported for video mode panel.");
    return kErrorNotSupported;
  }
  return hw_intf_->ControlIdlePowerCollapse(enable, synchronous);
}

DisplayError DisplayBuiltIn::GetSupportedDSIClock(std::vector<uint64_t> *bitclk_rates) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!hw_panel_info_.dyn_bitclk_support) {
    return kErrorNotSupported;
  }

  *bitclk_rates = hw_panel_info_.bitclk_rates;
  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetDynamicDSIClock(uint64_t bit_clk_rate) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!active_) {
    DLOGW("Invalid display state = %d. Panel must be on.", state_);
    return kErrorNotSupported;
  }

  if (!hw_panel_info_.dyn_bitclk_support) {
    return kErrorNotSupported;
  }

  uint64_t current_clk = 0;
  std::vector<uint64_t> &clk_rates = hw_panel_info_.bitclk_rates;
  GetDynamicDSIClock(&current_clk);
  bool valid = std::find(clk_rates.begin(), clk_rates.end(), bit_clk_rate) != clk_rates.end();
  if (current_clk == bit_clk_rate || !valid) {
    DLOGI("Invalid setting %d, Clk. already set %d", !valid, (current_clk == bit_clk_rate));
    return kErrorNone;
  }

  return hw_intf_->SetDynamicDSIClock(bit_clk_rate);
}

DisplayError DisplayBuiltIn::GetDynamicDSIClock(uint64_t *bit_clk_rate) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!hw_panel_info_.dyn_bitclk_support) {
    return kErrorNotSupported;
  }

  return hw_intf_->GetDynamicDSIClock(bit_clk_rate);
}

bool DisplayBuiltIn::CanCompareFrameROI(LayerStack *layer_stack) {
  // Check Display validation and safe-mode states.
  if (needs_validate_ || comp_manager_->IsSafeMode()) {
    return false;
  }

  // Check Panel and Layer Stack attributes.
  if (!hw_panel_info_.partial_update || (hw_panel_info_.left_roi_count != 1) ||
      layer_stack->flags.geometry_changed || layer_stack->flags.config_changed ||
      (layer_stack->layers.size() != (hw_layers_.info.app_layer_count + 1))) {
    return false;
  }

  // Check for Partial Update disable requests/scenarios.
  if (color_mgr_ && color_mgr_->NeedsPartialUpdateDisable()) {
    DisablePartialUpdateOneFrame();
  }

  if (!partial_update_control_ || disable_pu_one_frame_ || disable_pu_on_dest_scaler_) {
    return false;
  }

  bool surface_damage = false;
  uint32_t surface_damage_mask_value = (1 << kSurfaceDamage);
  for (uint32_t i = 0; i < layer_stack->layers.size(); i++) {
    Layer *layer = layer_stack->layers.at(i);
    if (layer->update_mask.none()) {
      continue;
    }
    // Only kSurfaceDamage bit should be set in layer's update-mask.
    if (layer->update_mask.to_ulong() == surface_damage_mask_value) {
      surface_damage = true;
    } else {
      return false;
    }
  }

  return surface_damage;
}

bool DisplayBuiltIn::CanSkipDisplayPrepare(LayerStack *layer_stack) {
  if (!CanCompareFrameROI(layer_stack)) {
    return false;
  }

  DisplayError error = BuildLayerStackStats(layer_stack);
  if (error != kErrorNone) {
    return false;
  }

  hw_layers_.info.left_frame_roi.clear();
  hw_layers_.info.right_frame_roi.clear();
  hw_layers_.info.dest_scale_info_map.clear();
  comp_manager_->GenerateROI(display_comp_ctx_, &hw_layers_);

  if (!hw_layers_.info.left_frame_roi.size() || !hw_layers_.info.right_frame_roi.size()) {
    return false;
  }

  // Compare the cached and calculated Frame ROIs.
  bool same_roi = IsCongruent(left_frame_roi_, hw_layers_.info.left_frame_roi.at(0)) &&
                  IsCongruent(right_frame_roi_, hw_layers_.info.right_frame_roi.at(0));

  if (same_roi) {
    // Update Surface Damage rectangle(s) in HW layers.
    uint32_t hw_layer_count = UINT32(hw_layers_.info.hw_layers.size());
    for (uint32_t j = 0; j < hw_layer_count; j++) {
      Layer &hw_layer = hw_layers_.info.hw_layers.at(j);
      Layer *sdm_layer = layer_stack->layers.at(hw_layers_.info.index.at(j));
      if (hw_layer.dirty_regions.size() != sdm_layer->dirty_regions.size()) {
        return false;
      }
      for (uint32_t k = 0; k < hw_layer.dirty_regions.size(); k++) {
        hw_layer.dirty_regions.at(k) = sdm_layer->dirty_regions.at(k);
      }
    }

    // Set the composition type for SDM layers.
    for (uint32_t i = 0; i < (layer_stack->layers.size() - 1); i++) {
      layer_stack->layers.at(i)->composition = kCompositionSDE;
    }
  }

  return same_roi;
}

DisplayError DisplayBuiltIn::GetRefreshRate(uint32_t *refresh_rate) {
  *refresh_rate = current_refresh_rate_;
  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetActiveConfig(uint32_t index) {
  deferred_config_.MarkDirty();
  return DisplayBase::SetActiveConfig(index);
}

DisplayError DisplayBuiltIn::ReconfigureDisplay() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  HWDisplayAttributes display_attributes;
  HWMixerAttributes mixer_attributes;
  HWPanelInfo hw_panel_info;
  uint32_t active_index = 0;

  DTRACE_SCOPED();

  error = hw_intf_->GetActiveConfig(&active_index);
  if (error != kErrorNone) {
    return error;
  }

  error = hw_intf_->GetDisplayAttributes(active_index, &display_attributes);
  if (error != kErrorNone) {
    return error;
  }

  error = hw_intf_->GetMixerAttributes(&mixer_attributes);
  if (error != kErrorNone) {
    return error;
  }

  error = hw_intf_->GetHWPanelInfo(&hw_panel_info);
  if (error != kErrorNone) {
    return error;
  }

  const bool dirty = deferred_config_.IsDirty();
  if (deferred_config_.IsDeferredState()) {
    if (dirty) {
      SetDeferredFpsConfig();
    } else {
      // In Deferred state, use current config for comparison.
      GetFpsConfig(&display_attributes, &hw_panel_info);
    }
  }

  const bool display_unchanged = (display_attributes == display_attributes_);
  const bool mixer_unchanged = (mixer_attributes == mixer_attributes_);
  const bool panel_unchanged = (hw_panel_info == hw_panel_info_);
  if (!dirty && display_unchanged && mixer_unchanged && panel_unchanged) {
    return kErrorNone;
  }

  if (CanDeferFpsConfig(display_attributes.fps)) {
    deferred_config_.Init(display_attributes.fps, display_attributes.vsync_period_ns,
                          hw_panel_info.transfer_time_us);

    // Apply current config until new Fps is deferred.
    GetFpsConfig(&display_attributes, &hw_panel_info);
  }

  error = comp_manager_->ReconfigureDisplay(display_comp_ctx_, display_attributes, hw_panel_info,
                                            mixer_attributes, fb_config_,
                                            &(default_qos_data_.clock_hz));
  if (error != kErrorNone) {
    return error;
  }

  bool disble_pu = true;
  if (mixer_unchanged && panel_unchanged) {
    // Do not disable Partial Update for one frame, if only FPS has changed.
    // Because if first frame after transition, has a partial Frame-ROI and
    // is followed by Skip Validate frames, then it can benefit those frames.
    disble_pu = !display_attributes_.OnlyFpsChanged(display_attributes);
  }

  if (disble_pu) {
    DisablePartialUpdateOneFrame();
  }

  display_attributes_ = display_attributes;
  mixer_attributes_ = mixer_attributes;
  hw_panel_info_ = hw_panel_info;

  // TODO(user): Temporary changes, to be removed when DRM driver supports
  // Partial update with Destination scaler enabled.
  SetPUonDestScaler();

  return kErrorNone;
}

bool DisplayBuiltIn::CanDeferFpsConfig(uint32_t fps) {
  if (deferred_config_.CanApplyDeferredState()) {
    // Deferred Fps Config needs to be applied.
    return false;
  }

  // In case of higher to lower Fps transition on a Builtin display, defer the Fps
  // (Transfer time) configuration, for the number of frames based on frame_count.
  return ((deferred_config_.frame_count != 0) && (display_attributes_.fps > fps));
}

void DisplayBuiltIn::SetDeferredFpsConfig() {
  // Update with the deferred Fps Config.
  display_attributes_.fps = deferred_config_.fps;
  display_attributes_.vsync_period_ns = deferred_config_.vsync_period_ns;
  hw_panel_info_.transfer_time_us = deferred_config_.transfer_time_us;
  deferred_config_.Clear();
}

void DisplayBuiltIn::GetFpsConfig(HWDisplayAttributes *display_attr, HWPanelInfo *panel_info) {
  display_attr->fps = display_attributes_.fps;
  display_attr->vsync_period_ns = display_attributes_.vsync_period_ns;
  panel_info->transfer_time_us = hw_panel_info_.transfer_time_us;
}

DisplayError DisplayBuiltIn::GetConfig(DisplayConfigFixedInfo *fixed_info) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  fixed_info->is_cmdmode = (hw_panel_info_.mode == kModeCommand);

  HWResourceInfo hw_resource_info = HWResourceInfo();
  hw_info_intf_->GetHWResourceInfo(&hw_resource_info);

  fixed_info->hdr_supported = hw_resource_info.has_hdr;
  // Populate luminance values only if hdr will be supported on that display
  fixed_info->max_luminance = fixed_info->hdr_supported ? hw_panel_info_.peak_luminance: 0;
  fixed_info->average_luminance = fixed_info->hdr_supported ? hw_panel_info_.average_luminance : 0;
  fixed_info->min_luminance = fixed_info->hdr_supported ?  hw_panel_info_.blackness_level: 0;
  fixed_info->hdr_eotf = hw_panel_info_.hdr_eotf;
  fixed_info->hdr_metadata_type_one = hw_panel_info_.hdr_metadata_type_one;
  fixed_info->partial_update = hw_panel_info_.partial_update;

  return kErrorNone;
}

}  // namespace sdm
