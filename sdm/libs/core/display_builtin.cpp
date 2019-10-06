/*
* Copyright (c) 2014 - 2019, The Linux Foundation. All rights reserved.
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

DisplayError DisplayBuiltIn::Init() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  int32_t disable_defer_power_state = 0;

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

  Debug::GetProperty(DISABLE_DEFER_POWER_STATE, &disable_defer_power_state);
  defer_power_state_ = !disable_defer_power_state;

  DLOGI("defer_power_state %d", defer_power_state_);

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
  if (reset_panel_) {
    DLOGW("panel is in bad state, resetting the panel");
    ResetPanel();
    reset_panel_ = false;
  }

  DTRACE_SCOPED();
  if (NeedsMixerReconfiguration(layer_stack, &new_mixer_width, &new_mixer_height)) {
    error = ReconfigureMixer(new_mixer_width, new_mixer_height);
    if (error != kErrorNone) {
      ReconfigureMixer(display_width, display_height);
    }
  } else {
    if (CanSkipDisplayPrepare(layer_stack)) {
      hw_layers_.hw_avr_info.enable = NeedsAVREnable();
      return kErrorNone;
    }
  }

  // Clean hw layers for reuse.
  hw_layers_ = HWLayers();
  hw_layers_.hw_avr_info.enable = NeedsAVREnable();

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

DisplayError DisplayBuiltIn::Commit(LayerStack *layer_stack) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  uint32_t app_layer_count = hw_layers_.info.app_layer_count;

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

  if (trigger_mode_debug_ != kFrameTriggerMax) {
    error = hw_intf_->SetFrameTrigger(trigger_mode_debug_);
    if (error != kErrorNone) {
      DLOGE("Failed to set frame trigger mode %d, err %d", (int)trigger_mode_debug_, error);
    } else {
      DLOGV_IF(kTagDisplay, "Set frame trigger mode %d", trigger_mode_debug_);
      trigger_mode_debug_ = kFrameTriggerMax;
    }
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

  if (dpps_pu_nofiy_pending_) {
    dpps_pu_nofiy_pending_ = false;
    dpps_pu_lock_.Broadcast();
  }
  dpps_info_.Init(this, hw_panel_info_.panel_name);

  return error;
}

DisplayError DisplayBuiltIn::SetDisplayState(DisplayState state, bool teardown,
                                             int *release_fence) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  error = DisplayBase::SetDisplayState(state, teardown, release_fence);
  if (error != kErrorNone) {
    return error;
  }

  // Set vsync enable state to false, as driver disables vsync during display power off.
  if (state == kStateOff) {
    vsync_enable_ = false;
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
      // Attempt to update refresh rate can fail if rf interfenence is detected.
      // Just drop min fps settting for now.
      handle_idle_timeout_ = false;
      return error;
    }

    error = comp_manager_->CheckEnforceSplit(display_comp_ctx_, refresh_rate);
    if (error != kErrorNone) {
      return error;
    }
  }

  // On success, set current refresh rate to new refresh rate
  current_refresh_rate_ = refresh_rate;
  handle_idle_timeout_ = false;

  return DisplayBase::ReconfigureDisplay();
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
  event_handler_->Refresh();
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  {
    reset_panel_ = true;
  }
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

bool DisplayBuiltIn::NeedsAVREnable() {
  if (avr_prop_disabled_ || qsync_mode_ == kQSyncModeNone) {
    return false;
  }

  return hw_panel_info_.qsync_support;
}

DisplayError DisplayBuiltIn::DppsProcessOps(enum DppsOps op, void *payload, size_t size) {
  DisplayError error = kErrorNone;
  uint32_t pending;
  bool enable = false;
  DppsDisplayInfo *info;

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
    case kDppsPartialUpdate: {
      int ret;
      if (!payload) {
        DLOGE("Invalid payload parameter for op %d", op);
        error = kErrorParameters;
        break;
      }
      enable = *(reinterpret_cast<bool *>(payload));
      ControlPartialUpdate(enable, &pending);
      event_handler_->HandleEvent(kInvalidateDisplay);
      event_handler_->Refresh();
      {
         lock_guard<recursive_mutex> obj(recursive_mutex_);
         dpps_pu_nofiy_pending_ = true;
      }
      ret = dpps_pu_lock_.WaitFinite(kPuTimeOutMs);
      if (ret) {
        DLOGE("failed to %s partial update ret %d", ((enable) ? "enable" : "disable"), ret);
        error = kErrorTimeOut;
      }
      break;
    }
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
    case kDppsGetDisplayInfo:
      if (!payload) {
        DLOGE("Invalid payload parameter for op %d", op);
        error = kErrorParameters;
        break;
      }
      info = reinterpret_cast<DppsDisplayInfo *>(payload);
      info->width = display_attributes_.x_pixels;
      info->height = display_attributes_.y_pixels;
      info->is_primary = IsPrimaryDisplay();
      info->display_id = display_id_;
      info->display_type = display_type_;

      error = hw_intf_->GetPanelBrightnessBasePath(&(info->brightness_base_path));
      if (error != kErrorNone) {
        DLOGE("Failed to get brightness base path %d", error);
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

DisplayError DisplayBuiltIn::SetFrameTriggerMode(FrameTriggerMode mode) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  trigger_mode_debug_ = mode;
  return kErrorNone;
}

DppsInterface* DppsInfo::dpps_intf_ = NULL;
std::vector<int32_t> DppsInfo::display_id_ = {};

void DppsInfo::Init(DppsPropIntf *intf, const std::string &panel_name) {
  std::lock_guard<std::mutex> guard(lock_);
  int error = 0;

  if (!intf) {
    DLOGE("Invalid intf is null");
    return;
  }

  DppsDisplayInfo info_payload = {};
  DisplayError ret = intf->DppsProcessOps(kDppsGetDisplayInfo, &info_payload, sizeof(info_payload));
  if (ret != kErrorNone) {
    DLOGE("Get display information failed, ret %d", ret);
    return;
  }

  if (std::find(display_id_.begin(), display_id_.end(), info_payload.display_id)
    != display_id_.end()) {
    return;
  }
  DLOGI("Ready to register display id %d ", info_payload.display_id);

  if (!dpps_intf_) {
    if (!dpps_impl_lib_.Open(kDppsLib_)) {
      DLOGW("Failed to load Dpps lib %s", kDppsLib_);
      goto exit;
    }

    if (!dpps_impl_lib_.Sym("GetDppsInterface", reinterpret_cast<void **>(&GetDppsInterface))) {
      DLOGE("GetDppsInterface not found!, err %s", dlerror());
      goto exit;
    }

    dpps_intf_ = GetDppsInterface();
    if (!dpps_intf_) {
      DLOGE("Failed to get Dpps Interface!");
      goto exit;
    }
  }
  error = dpps_intf_->Init(intf, panel_name);
  if (error) {
    DLOGE("DPPS Interface init failure with err %d", error);
    goto exit;
  }

  display_id_.push_back(info_payload.display_id);
  DLOGI("Register display id %d successfully", info_payload.display_id);
  return;

exit:
  Deinit();
  dpps_intf_ = new DppsDummyImpl();
}

void DppsInfo::Deinit() {
  if (dpps_intf_) {
    dpps_intf_->Deinit();
    dpps_intf_ = NULL;
  }
  dpps_impl_lib_.~DynLib();
}

void DppsInfo::DppsNotifyOps(enum DppsNotifyOps op, void *payload, size_t size) {
  int ret = 0;
  ret = dpps_intf_->DppsNotifyOps(op, payload, size);
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
  if (qsync_mode == kQsyncModeOneShot) {
    return kErrorNotSupported;
  }
  qsync_mode_ = qsync_mode;

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

void DisplayBuiltIn::ResetPanel() {
  DisplayError status = kErrorNone;
  int release_fence = -1;
  DisplayState last_display_state = {};

  GetDisplayState(&last_display_state);
  DLOGI("Power off display id = %d", display_id_);

  status = SetDisplayState(kStateOff, true /* teardown */, &release_fence);
  if (status != kErrorNone) {
    DLOGE("Power off for display id = %d failed with error = %d", display_id_, status);
  }
  CloseFd(&release_fence);

  DLOGI("Set display %d to state = %d", display_id_, last_display_state);
  status = SetDisplayState(last_display_state, false /* teardown */, &release_fence);
  if (status != kErrorNone) {
     DLOGE("%d state for display id = %d failed with error = %d", last_display_state, display_id_,
           status);
  }
  CloseFd(&release_fence);

  // If panel does not support color modes, do not set color mode.
  if (color_mode_map_.size() > 0) {
    status = SetColorMode(current_color_mode_);
    if (status != kErrorNone) {
      DLOGE("SetColorMode failed for display id = %d error = %d", display_id_, status);
    }
  }

  status = SetVSyncState(true);
  if (status != kErrorNone) {
    DLOGE("Enable vsync failed for display id = %d with error = %d", display_id_, status);
  }
}

DisplayError DisplayBuiltIn::GetRefreshRate(uint32_t *refresh_rate) {
  *refresh_rate = current_refresh_rate_;
  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetBLScale(uint32_t level) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  DisplayError err = hw_intf_->SetBLScale(level);
  if (err) {
    DLOGE("Failed to set backlight scale to level %d", level);
  } else {
    DLOGI_IF(kTagDisplay, "Setting backlight scale to level %d", level);
  }
  return err;
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

}  // namespace sdm
