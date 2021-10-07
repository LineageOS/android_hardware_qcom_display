/*
* Copyright (c) 2014-2018, 2020, The Linux Foundation. All rights reserved.
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

DisplayPrimary::DisplayPrimary(int32_t display_id, DisplayEventHandler *event_handler,
                              HWInfoInterface *hw_info_intf,
                              BufferSyncHandler *buffer_sync_handler,
                              BufferAllocator *buffer_allocator, CompManager *comp_manager)
  : DisplayBase(display_id, kPrimary, event_handler, kDevicePrimary, buffer_sync_handler,
                buffer_allocator, comp_manager, hw_info_intf) {
}

DisplayError DisplayPrimary::Init() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  DisplayError error = HWInterface::Create(display_id_, kPrimary, hw_info_intf_,
                                           buffer_sync_handler_, buffer_allocator_, &hw_intf_);
  if (error != kErrorNone) {
    return error;
  }

  hw_intf_->GetDisplayId(&display_id_);

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
                    HWEvent::PANEL_DEAD };
  } else {
    event_list_ = { HWEvent::VSYNC,
                    HWEvent::EXIT,
                    HWEvent::IDLE_NOTIFY,
                    HWEvent::SHOW_BLANK_EVENT,
                    HWEvent::THERMAL_LEVEL,
                    HWEvent::PINGPONG_TIMEOUT,
                    HWEvent::PANEL_DEAD };
  }

  avr_prop_disabled_ = Debug::IsAVRDisabled();

  error = HWEventsInterface::Create(display_id_, kPrimary, this, event_list_, hw_intf_,
                                    &hw_events_intf_);
  if (error != kErrorNone) {
    DLOGE("Failed to create hardware events interface. Error = %d", error);
    DisplayBase::Deinit();
    HWInterface::Destroy(hw_intf_);
  }

  current_refresh_rate_ = hw_panel_info_.max_fps;

  return error;
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

DisplayError DisplayPrimary::SetPanelBrightness(float brightness) {
  lock_guard<recursive_mutex> obj(brightness_lock_);

  if (brightness != -1.0f && !(0.0f <= brightness && brightness <= 1.0f)) {
    DLOGE("Bad brightness value = %f", brightness);
    return kErrorParameters;
  }

  if (state_ == kStateOff) {
    return kErrorNone;
  }

  // -1.0f = off, 0.0f = min, 1.0f = max
  level_remainder_ = 0.0f;
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
    level_remainder_ = t - level;
  }

  DisplayError err = kErrorNone;
  if ((err = hw_intf_->SetPanelBrightness(level)) == kErrorNone) {
    DLOGI_IF(kTagDisplay, "Setting brightness to level %d (%f percent)", level,
             brightness * 100);
  }

  return err;
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
      // Attempt to update refresh rate can fail if rf interfenence is detected.
      // Just drop min fps settting for now.
      handle_idle_timeout_ = false;
      return error;
    }
  }

  // On success, set current refresh rate to new refresh rate
  current_refresh_rate_ = refresh_rate;
  handle_idle_timeout_ = false;

  return DisplayBase::ReconfigureDisplay();
}

DisplayError DisplayPrimary::VSync(int64_t timestamp) {
  if (vsync_enable_ && !drop_hw_vsync_) {
    DisplayEventVSync vsync;
    vsync.timestamp = timestamp;
    event_handler_->VSync(vsync);
  }

  return kErrorNone;
}

void DisplayPrimary::IdleTimeout() {
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

DisplayError DisplayPrimary::GetPanelBrightness(float *brightness) {
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

DisplayError DisplayPrimary::ControlPartialUpdate(bool enable, uint32_t *pending) {
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

  DLOGI("Powering off built-in/primary %d", display_id_);
  status = SetDisplayState(kStateOff, &release_fence);
  if (status != kErrorNone) {
    DLOGE("power-off on built-in/primary %d failed with error = %d", display_id_, status);
  }
  if (release_fence >= 0) {
    ::close(release_fence);
  }

  DLOGI("Restoring power mode on built-in/primary %d", display_id_);
  DisplayState mode = GetLastPowerMode();
  status = SetDisplayState(mode, &release_fence);
  if (status != kErrorNone) {
    DLOGE("Setting power mode = %d on built-in/primary %d failed with error = %d", mode,
          display_id_, status);
  }
  if (release_fence >= 0) {
    ::close(release_fence);
  }

  DLOGI("Enabling HWVsync");
  status = SetVSyncState(true);
  if (status != kErrorNone) {
    DLOGE("enabling vsync failed for built-in/primary %d with error = %d", display_id_, status);
  }
}

DisplayError DisplayPrimary::GetSupportedDSIClock(std::vector<uint64_t> *bitclk_rates) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!hw_panel_info_.dyn_bitclk_support) {
    return kErrorNotSupported;
  }

  *bitclk_rates = hw_panel_info_.bitclk_rates;
  return kErrorNone;
}

DisplayError DisplayPrimary::SetDynamicDSIClock(uint64_t bit_clk_rate){
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

DisplayError DisplayPrimary::GetDynamicDSIClock(uint64_t *bit_clk_rate){
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!hw_panel_info_.dyn_bitclk_support) {
    return kErrorNotSupported;
  }

  return hw_intf_->GetDynamicDSIClock(bit_clk_rate);
}

}  // namespace sdm

