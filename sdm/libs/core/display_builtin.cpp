/*
* Copyright (c) 2014 - 2021, The Linux Foundation. All rights reserved.
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

/*
* Changes from Qualcomm Innovation Center are provided under the following license:
*
* Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the
* disclaimer below) provided that the following conditions are met:
*
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*
*    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/rect.h>
#include <utils/utils.h>
#include <utils/formats.h>
#include <iomanip>
#include <algorithm>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "display_builtin.h"
#include "drm_interface.h"
#include "drm_master.h"
#include "hw_info_interface.h"
#include "hw_interface.h"

#define __CLASS__ "DisplayBuiltIn"

namespace sdm {

DisplayBuiltIn::DisplayBuiltIn(DisplayEventHandler *event_handler, HWInfoInterface *hw_info_intf,
                               BufferAllocator *buffer_allocator, CompManager *comp_manager,
                               std::shared_ptr<IPCIntf> ipc_intf)
  : DisplayBase(kBuiltIn, event_handler, kDeviceBuiltIn, buffer_allocator, comp_manager,
                hw_info_intf), ipc_intf_(ipc_intf) {}

DisplayBuiltIn::DisplayBuiltIn(int32_t display_id, DisplayEventHandler *event_handler,
                               HWInfoInterface *hw_info_intf,
                               BufferAllocator *buffer_allocator, CompManager *comp_manager,
                               std::shared_ptr<IPCIntf> ipc_intf)
  : DisplayBase(display_id, kBuiltIn, event_handler, kDeviceBuiltIn, buffer_allocator, comp_manager,
                hw_info_intf), ipc_intf_(ipc_intf) {}

DisplayBuiltIn::~DisplayBuiltIn() {
}

static uint64_t GetTimeInMs(struct timespec ts) {
  return (ts.tv_sec * 1000 + (ts.tv_nsec + 500000) / 1000000);
}

DisplayError DisplayBuiltIn::Init() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  DisplayError error = HWInterface::Create(display_id_, kBuiltIn, hw_info_intf_,
                                           buffer_allocator_, &hw_intf_);
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

  if (color_mgr_) {
    color_mgr_->ColorMgrGetStcModes(&stc_color_modes_);
  }

  if (hw_panel_info_.mode == kModeCommand && Debug::IsVideoModeEnabled()) {
    error = hw_intf_->SetDisplayMode(kModeVideo);
    if (error != kErrorNone) {
      DLOGW("Retaining current display mode. Current = %d, Requested = %d", hw_panel_info_.mode,
            kModeVideo);
    }
  }
#ifdef TRUSTED_VM
  event_list_ = {HWEvent::VSYNC, HWEvent::EXIT, HWEvent::PINGPONG_TIMEOUT, HWEvent::PANEL_DEAD,
                 HWEvent::HW_RECOVERY};
#else
  if (hw_panel_info_.mode == kModeCommand) {
    event_list_ = {HWEvent::VSYNC, HWEvent::EXIT,
                   /*HWEvent::IDLE_NOTIFY, */
                   HWEvent::SHOW_BLANK_EVENT, HWEvent::THERMAL_LEVEL, HWEvent::IDLE_POWER_COLLAPSE,
                   HWEvent::PINGPONG_TIMEOUT, HWEvent::PANEL_DEAD, HWEvent::HW_RECOVERY,
                   HWEvent::HISTOGRAM, HWEvent::BACKLIGHT_EVENT};
  } else {
    event_list_ = {HWEvent::VSYNC,         HWEvent::EXIT,
                   HWEvent::IDLE_NOTIFY,   HWEvent::SHOW_BLANK_EVENT,
                   HWEvent::THERMAL_LEVEL, HWEvent::PINGPONG_TIMEOUT,
                   HWEvent::PANEL_DEAD,    HWEvent::HW_RECOVERY,
                   HWEvent::HISTOGRAM,     HWEvent::BACKLIGHT_EVENT};
  }
#endif
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
  Debug::Get()->GetProperty(ENABLE_HISTOGRAM_INTR, &value);
  if (value == 1) {
    initColorSamplingState();
  }

  value = 0;
  Debug::Get()->GetProperty(DEFER_FPS_FRAME_COUNT, &value);
  deferred_config_.frame_count = (value > 0) ? UINT32(value) : 0;

  spr_prop_value_ = 0;
  // Enable SPR as default is disabled.
  Debug::GetProperty(ENABLE_SPR, &spr_prop_value_);

  error = CreatePanelfeatures();
  if (error != kErrorNone) {
    DLOGE("Failed to setup panel feature factory, error: %d", error);
  } else {
    // Get status of RC enablement property. Default RC is disabled.
    int rc_prop_value = 0;
    Debug::GetProperty(ENABLE_ROUNDED_CORNER, &rc_prop_value);
    rc_enable_prop_ = rc_prop_value ? true : false;
    DLOGI("RC feature %s.", rc_enable_prop_ ? "enabled" : "disabled");
  }
  value = 0;
  DebugHandler::Get()->GetProperty(DISABLE_DYNAMIC_FPS, &value);
  disable_dyn_fps_ = (value == 1);

  value = 0;
  DebugHandler::Get()->GetProperty(ENABLE_QSYNC_IDLE, &value);
  enable_qsync_idle_ = hw_panel_info_.qsync_support && (value == 1);
  if (enable_qsync_idle_) {
    DLOGI("Enabling qsync on idling");
  }

  value = 0;
  DebugHandler::Get()->GetProperty(ENHANCE_IDLE_TIME, &value);
  enhance_idle_time_ = (value == 1);

  return error;
}

DisplayError DisplayBuiltIn::Deinit() {
  {
    lock_guard<recursive_mutex> obj(recursive_mutex_);

    dpps_info_.Deinit();
  }
  return DisplayBase::Deinit();
}

// Create instance for RC, SPR and demura feature.
DisplayError DisplayBuiltIn::CreatePanelfeatures() {
  if (pf_factory_ && prop_intf_) {
    return kErrorNone;
  }

  if (!GetPanelFeatureFactoryIntfFunc_) {
    DynLib feature_impl_lib;
    if (feature_impl_lib.Open(EXTENSION_LIBRARY_NAME)) {
      if (!feature_impl_lib.Sym("GetPanelFeatureFactoryIntf",
                                reinterpret_cast<void **>(&GetPanelFeatureFactoryIntfFunc_))) {
        DLOGE("Unable to load symbols, error = %s", feature_impl_lib.Error());
        return kErrorUndefined;
      }
    } else {
      DLOGW("Unable to load = %s, error = %s", EXTENSION_LIBRARY_NAME, feature_impl_lib.Error());
      DLOGW("Panel features are not supported");
      return kErrorNotSupported;
    }
  }

  pf_factory_ = GetPanelFeatureFactoryIntfFunc_();
  if (!pf_factory_) {
    DLOGE("Failed to create PanelFeatureFactoryIntf");
    return kErrorResources;
  }

  prop_intf_ = hw_intf_->GetPanelFeaturePropertyIntf();
  if (!prop_intf_) {
    DLOGE("Failed to create PanelFeaturePropertyIntf");
    pf_factory_ = nullptr;
    return kErrorResources;
  }

  return kErrorNone;
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
      UpdateQsyncMode();
      return kErrorNone;
    }
  }

  // Clean hw layers for reuse.
  hw_layers_ = HWLayers();

  error = ConfigureCwb(layer_stack);
  if (error != kErrorNone) {
    return error;
  }

  UpdateQsyncMode();

  left_frame_roi_ = {};
  right_frame_roi_ = {};

  if (spr_) {
    GenericPayload out;
    uint32_t *enable = nullptr;
    int ret = out.CreatePayload<uint32_t>(enable);
    if (ret) {
      DLOGE("Failed to create the payload. Error:%d", ret);
      return kErrorUndefined;
    }
    ret = spr_->GetParameter(kSPRFeatureEnable, &out);
    if (ret) {
      DLOGE("Failed to get the spr status. Error:%d", ret);
      return kErrorUndefined;
    }
    spr_enable_ = *enable;
  }

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

void DisplayBuiltIn::UpdateQsyncMode() {
  if (!hw_panel_info_.qsync_support) {
    return;
  }

  QSyncMode mode = kQSyncModeNone;
  if (handle_idle_timeout_ && enable_qsync_idle_) {
    // Override to continuous mode upon idling.
    mode = kQSyncModeContinuous;
    DLOGV_IF(kTagDisplay, "Qsync entering continuous mode");
  } else {
    // Set Qsync mode requested by client.
    mode = qsync_mode_;
    DLOGV_IF(kTagDisplay, "Restoring client's qsync mode: %d", mode);
  }

  hw_layers_.hw_avr_info.update = (mode != active_qsync_mode_) || needs_avr_update_;
  hw_layers_.hw_avr_info.mode = GetAvrMode(mode);

  DLOGV_IF(kTagDisplay, "update: %d mode: %d", hw_layers_.hw_avr_info.update, mode);

  // Store active mde.
  active_qsync_mode_ = mode;
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

void DisplayBuiltIn::initColorSamplingState() {
  samplingState = SamplingState::Off;
  histogramCtrl.object_type = DRM_MODE_OBJECT_CRTC;
  histogramCtrl.feature_id = sde_drm::DRMDPPSFeatureID::kFeatureAbaHistCtrl;
  histogramCtrl.value = sde_drm::HistModes::kHistDisabled;

  histogramIRQ.object_type = DRM_MODE_OBJECT_CRTC;
  histogramIRQ.feature_id = sde_drm::DRMDPPSFeatureID::kFeatureAbaHistIRQ;
  histogramIRQ.value = sde_drm::HistModes::kHistDisabled;
  histogramSetup = true;
}

DisplayError DisplayBuiltIn::setColorSamplingState(SamplingState state) {
  samplingState = state;
  if (samplingState == SamplingState::On) {
    histogramCtrl.value = sde_drm::HistModes::kHistEnabled;
    histogramIRQ.value = sde_drm::HistModes::kHistEnabled;
    if (hw_panel_info_.mode == kModeCommand) {
      uint32_t pending;
      ControlPartialUpdate(false /* enable */, &pending);
    }
  } else {
    histogramCtrl.value = sde_drm::HistModes::kHistDisabled;
    histogramIRQ.value = sde_drm::HistModes::kHistDisabled;
    if (hw_panel_info_.mode == kModeCommand) {
      uint32_t pending;
      ControlPartialUpdate(true /* enable */, &pending);
    }
  }

  // effectively drmModeAtomicAddProperty for the SDE_DSPP_HIST_CTRL_V1
  return DppsProcessOps(kDppsSetFeature, &histogramCtrl, sizeof(histogramCtrl));
}

DisplayError DisplayBuiltIn::colorSamplingOn() {
  if (!histogramSetup) {
    return kErrorParameters;
  }
  return setColorSamplingState(SamplingState::On);
}

DisplayError DisplayBuiltIn::colorSamplingOff() {
  if (!histogramSetup) {
    return kErrorParameters;
  }
  return setColorSamplingState(SamplingState::Off);
}

DisplayError DisplayBuiltIn::SetupSPR() {
  SPRInputConfig spr_cfg;
  spr_cfg.panel_name = std::string(hw_panel_info_.panel_name);
  spr_ = pf_factory_->CreateSPRIntf(spr_cfg, prop_intf_);
  if (!spr_ || spr_->Init() != 0) {
    DLOGE("Failed to initialize SPR");
    return kErrorResources;
  }

  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetupDemura() {
  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetupPanelfeatures() {
  if (!pf_factory_ || !prop_intf_) {
    DLOGE("Failed to create PanelFeatures");
    return kErrorResources;
  }

  DisplayError ret = kErrorNone;
  if ((ret = SetupSPR()) != kErrorNone) return ret;
  if ((ret = SetupDemura()) != kErrorNone) return ret;

  return ret;
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

  if (trigger_mode_debug_ != kFrameTriggerMax) {
    error = hw_intf_->SetFrameTrigger(trigger_mode_debug_);
    if (error != kErrorNone) {
      DLOGE("Failed to set frame trigger mode %d, err %d", (int)trigger_mode_debug_, error);
    } else {
      DLOGV_IF(kTagDisplay, "Set frame trigger mode %d", trigger_mode_debug_);
      trigger_mode_debug_ = kFrameTriggerMax;
    }
  }

  if (vsync_enable_) {
    DTRACE_BEGIN("RegisterVsync");
    // wait for previous frame's retire fence to signal.
    Fence::Wait(previous_retire_fence_);

    // Register for vsync and then commit the frame.
    hw_events_intf_->SetEventState(HWEvent::VSYNC, true);
    DTRACE_END();
  }
  // effectively drmModeAtomicAddProperty for SDE_DSPP_HIST_IRQ_V1
  if (histogramSetup) {
    DppsProcessOps(kDppsSetFeature, &histogramIRQ, sizeof(histogramIRQ));
  }

  error = DisplayBase::Commit(layer_stack);
  if (error != kErrorNone) {
    return error;
  }

  if (pending_brightness_) {
    Fence::Wait(layer_stack->retire_fence);
    SetPanelBrightness(cached_brightness_);
    pending_brightness_ = false;
  } else {
    if (secure_event_ == kTUITransitionStart) {
      // Send the panel brightness event to secondary VM on TUI session start
      SendBacklight();
    }
  }

  if (secure_event_ == kTUITransitionStart) {
    // Send display config information to secondary VM on TUI session start
    SendDisplayConfigs();
  }

  if (commit_event_enabled_) {
    dpps_info_.DppsNotifyOps(kDppsCommitEvent, &display_type_, sizeof(display_type_));
  }

  deferred_config_.UpdateDeferCount();

  if (needs_validate_on_pu_enable_) {
    // After PU was disabled for one frame, need to revalidate when enabled.
    event_handler_->HandleEvent(kInvalidateDisplay);
    needs_validate_on_pu_enable_ = false;
  }

  ReconfigureDisplay();

  if (deferred_config_.CanApplyDeferredState()) {
    event_handler_->HandleEvent(kInvalidateDisplay);
    deferred_config_.Clear();
  }

  clock_gettime(CLOCK_MONOTONIC, &idle_timer_start_);
  int idle_time_ms = hw_layers_.info.set_idle_time_ms;
  if (idle_time_ms >= 0) {
    hw_intf_->SetIdleTimeoutMs(UINT32(idle_time_ms));
    idle_time_ms_ = idle_time_ms;
  }

  if (switch_to_cmd_) {
    uint32_t pending;
    switch_to_cmd_ = false;
    ControlPartialUpdate(true /* enable */, &pending);
  }

  if (panel_mode != hw_panel_info_.mode) {
    UpdateDisplayModeParams();
  }

  if (dpps_pu_nofiy_pending_) {
    dpps_pu_nofiy_pending_ = false;
    dpps_pu_lock_.Broadcast();
  }
  dpps_info_.Init(this, hw_panel_info_.panel_name);

  HandleQsyncPostCommit(layer_stack);

  first_cycle_ = false;

  previous_retire_fence_ = layer_stack->retire_fence;

  handle_idle_timeout_ = false;

  return error;
}

void DisplayBuiltIn::HandleQsyncPostCommit(LayerStack *layer_stack) {
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

  SetVsyncStatus(true /*Re-enable vsync.*/);

  bool notify_idle = enable_qsync_idle_ && (active_qsync_mode_ != kQSyncModeNone) &&
                     handle_idle_timeout_;
  if (notify_idle) {
    event_handler_->HandleEvent(kPostIdleTimeout);
  }
}

void DisplayBuiltIn::UpdateDisplayModeParams() {
  if (hw_panel_info_.mode == kModeVideo) {
    uint32_t pending = 0;
    ControlPartialUpdate(false /* enable */, &pending);
  } else if (hw_panel_info_.mode == kModeCommand) {
    // Flush idle timeout value currently set.
    comp_manager_->SetIdleTimeoutMs(display_comp_ctx_, 0, 0);
    switch_to_cmd_ = true;
  }
}

DisplayError DisplayBuiltIn::SetDisplayState(DisplayState state, bool teardown,
                                             shared_ptr<Fence> *release_fence) {
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

  if (pending_power_state_ != kPowerStateNone) {
    event_handler_->Refresh();
  }

  if (spr_prop_value_ && !panel_feature_init_ && state != kStateOff && state != kStateStandby) {
    error = SetupPanelfeatures();
    panel_feature_init_ = true;
    if (error != kErrorNone) {
      DLOGE("SetupPanelfeatures failed with error :%d, ignoring!", error);
    }
  }

  return kErrorNone;
}

void DisplayBuiltIn::SetIdleTimeoutMs(uint32_t active_ms, uint32_t inactive_ms) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  comp_manager_->SetIdleTimeoutMs(display_comp_ctx_, active_ms, inactive_ms);
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
      comp_manager_->SetIdleTimeoutMs(display_comp_ctx_, 0, 0);
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


DisplayError DisplayBuiltIn::SetRefreshRate(uint32_t refresh_rate, bool final_rate,
                                            bool idle_screen) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  if (!active_ || !hw_panel_info_.dynamic_fps || qsync_mode_ != kQSyncModeNone ||
      disable_dyn_fps_) {
    return kErrorNotSupported;
  }

  if (refresh_rate < hw_panel_info_.min_fps || refresh_rate > hw_panel_info_.max_fps) {
    DLOGE("Invalid Fps = %d request", refresh_rate);
    return kErrorParameters;
  }

  if (CanLowerFps(idle_screen) && !final_rate && !enable_qsync_idle_) {
    refresh_rate = hw_panel_info_.min_fps;
  }

  if (current_refresh_rate_ != refresh_rate) {
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

  // Set safe mode upon success.
  if (enhance_idle_time_ && handle_idle_timeout_ && (refresh_rate == hw_panel_info_.min_fps)) {
    comp_manager_->ProcessIdleTimeout(display_comp_ctx_);
  }

  // On success, set current refresh rate to new refresh rate
  current_refresh_rate_ = refresh_rate;
  deferred_config_.MarkDirty();

  return ReconfigureDisplay();
}

bool DisplayBuiltIn::CanLowerFps(bool idle_screen) {
  if (!enhance_idle_time_) {
    return handle_idle_timeout_;
  }

  if (!handle_idle_timeout_ || !idle_screen) {
    return false;
  }

  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  uint64_t elapsed_time_ms = GetTimeInMs(now) - GetTimeInMs(idle_timer_start_);
  bool can_lower = elapsed_time_ms >= UINT32(idle_time_ms_);
  DLOGV_IF(kTagDisplay, "lower fps: %d", can_lower);

  return can_lower;
}

DisplayError DisplayBuiltIn::VSync(int64_t timestamp) {
  DTRACE_SCOPED();
  bool qsync_enabled = enable_qsync_idle_ && (active_qsync_mode_ != kQSyncModeNone);
  // Client isn't aware of underlying qsync mode.
  // Disable vsync propagation as long as qsync is enabled.
  bool propagate_vsync = vsync_enable_ && !drop_hw_vsync_ && !qsync_enabled;
  if (!propagate_vsync) {
    // Re enable when display updates.
    SetVsyncStatus(false /*Disable vsync events.*/);
    return kErrorNone;
  }

  DisplayEventVSync vsync;
  vsync.timestamp = timestamp;
  event_handler_->VSync(vsync);

  return kErrorNone;
}

void DisplayBuiltIn::SetVsyncStatus(bool enable) {
  string trace_name = enable ? "enable" : "disable";
  DTRACE_BEGIN(trace_name.c_str());
  if (enable) {
    // Enable if vsync is still enabled.
    hw_events_intf_->SetEventState(HWEvent::VSYNC, vsync_enable_);
    pending_vsync_enable_ = false;
  } else {
    hw_events_intf_->SetEventState(HWEvent::VSYNC, false);
    pending_vsync_enable_ = true;
  }
  DTRACE_END();
}

void DisplayBuiltIn::IdleTimeout() {
  if (hw_panel_info_.mode == kModeVideo) {
    if (event_handler_->HandleEvent(kIdleTimeout) != kErrorNone) {
      return;
    }
    handle_idle_timeout_ = true;
    event_handler_->Refresh();
    hw_intf_->EnableSelfRefresh();
    if (!enhance_idle_time_) {
      lock_guard<recursive_mutex> obj(recursive_mutex_);
      comp_manager_->ProcessIdleTimeout(display_comp_ctx_);
    }
  }
}

void DisplayBuiltIn::PingPongTimeout() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  hw_intf_->DumpDebugData();
}

void DisplayBuiltIn::IdlePowerCollapse() {
  if (hw_panel_info_.mode == kModeCommand) {
    event_handler_->HandleEvent(kIdlePowerCollapse);
    lock_guard<recursive_mutex> obj(recursive_mutex_);
    comp_manager_->ProcessIdlePowerCollapse(display_comp_ctx_);
  }
}

DisplayError DisplayBuiltIn::ClearLUTs() {
  comp_manager_->ProcessIdlePowerCollapse(display_comp_ctx_);
  return kErrorNone;
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

void DisplayBuiltIn::Histogram(int histogram_fd, uint32_t blob_id) {
  event_handler_->HistogramEvent(histogram_fd, blob_id);
}

void DisplayBuiltIn::HandleBacklightEvent(float brightness_level) {
  DLOGI("backlight event occurred %f ipc_intf %p", brightness_level, ipc_intf_.get());
  if (ipc_intf_) {
    GenericPayload in;
    IPCBacklightParams *backlight_params = nullptr;
    int ret = in.CreatePayload<IPCBacklightParams>(backlight_params);
    if (ret) {
      DLOGW("failed to create the payload. Error:%d", ret);
      return;
    }
    float brightness = 0.0f;
    if (GetPanelBrightnessFromLevel(brightness_level, &brightness) != kErrorNone) {
      return;
    }
    backlight_params->brightness = brightness;
    backlight_params->is_primary = IsPrimaryDisplay();
    if ((ret = ipc_intf_->SetParameter(kIpcParamSetBacklight, in))) {
      DLOGW("Failed to set backlight, error = %d", ret);
    }
    lock_guard<recursive_mutex> obj(brightness_lock_);
    cached_brightness_ = brightness;
    pending_brightness_ = true;
  }
}

DisplayError DisplayBuiltIn::GetPanelBrightness(float *brightness) {
  lock_guard<recursive_mutex> obj(brightness_lock_);

  DisplayError err = kErrorNone;
  int level = 0;
  if ((err = hw_intf_->GetPanelBrightness(&level)) != kErrorNone) {
    return err;
  }
  return GetPanelBrightnessFromLevel(level, brightness);
}

DisplayError DisplayBuiltIn::GetPanelBrightnessFromLevel(float level, float *brightness) {
  // -1.0f = off, 0.0f = min, 1.0f = max
  float max = hw_panel_info_.panel_max_brightness;
  float min = hw_panel_info_.panel_min_brightness;
  if (level == 0) {
    *brightness = -1.0f;
  } else if ((max > min) && (min <= level && level <= max)) {
    *brightness = (static_cast<float>(level) + level_remainder_ - min) / (max - min);
  } else {
    min >= max ? DLOGE("Minimum brightness is greater than or equal to maximum brightness") :
                 DLOGE("Invalid brightness level %f", level);
    return kErrorDriverData;
  }

  DLOGI_IF(kTagDisplay, "Received level %f (%f percent)", level, *brightness * 100);

  return kErrorNone;
}

DisplayError DisplayBuiltIn::GetPanelMaxBrightness(uint32_t *max_brightness_level) {
  lock_guard<recursive_mutex> obj(brightness_lock_);

  if (!max_brightness_level) {
    DLOGE("Invalid input pointer is null");
    return kErrorParameters;
  }

  *max_brightness_level = static_cast<uint32_t>(hw_panel_info_.panel_max_brightness);

  DLOGI_IF(kTagDisplay, "Get panel max_brightness_level %u", *max_brightness_level);
  return kErrorNone;
}

DisplayError DisplayBuiltIn::ControlPartialUpdate(bool enable, uint32_t *pending) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!pending) {
    return kErrorParameters;
  }

  if (dpps_info_.disable_pu_ && enable) {
    // Nothing to be done.
    DLOGI("partial update is disabled by DPPS for display id = %d", display_id_);
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
  needs_validate_on_pu_enable_ = true;

  return kErrorNone;
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
      dpps_info_.disable_pu_ = !enable;
      ControlPartialUpdate(enable, &pending);
      event_handler_->HandleEvent(kSyncInvalidateDisplay);
      event_handler_->Refresh();
      {
         lock_guard<recursive_mutex> obj(recursive_mutex_);
         dpps_pu_nofiy_pending_ = true;
      }
      ret = dpps_pu_lock_.WaitFinite(kPuTimeOutMs);
      if (ret) {
        DLOGW("failed to %s partial update ret %d", ((enable) ? "enable" : "disable"), ret);
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

DisplayError DisplayBuiltIn::GetStcColorModes(snapdragoncolor::ColorModeList *mode_list) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!mode_list) {
    return kErrorParameters;
  }

  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  mode_list->list = stc_color_modes_.list;
  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetStcColorMode(const snapdragoncolor::ColorMode &color_mode) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!color_mgr_) {
    return kErrorNotSupported;
  }
  DisplayError ret = kErrorNone;
  PrimariesTransfer blend_space = {};
  blend_space = GetBlendSpaceFromStcColorMode(color_mode);
  ret = comp_manager_->SetBlendSpace(display_comp_ctx_, blend_space);
  if (ret != kErrorNone) {
    DLOGE("SetBlendSpace failed, ret = %d display_type_ = %d", ret, display_type_);
  }

  ret = hw_intf_->SetBlendSpace(blend_space);
  if (ret != kErrorNone) {
    DLOGE("Failed to pass blend space, ret = %d display_type_ = %d", ret, display_type_);
  }

  ret = color_mgr_->ColorMgrSetStcMode(color_mode);
  if (ret != kErrorNone) {
    DLOGE("Failed to set stc color mode, ret = %d display_type_ = %d", ret, display_type_);
    return ret;
  }
  current_color_mode_ = color_mode;

  DynamicRangeType dynamic_range = kSdrType;
  if (std::find(color_mode.hw_assets.begin(), color_mode.hw_assets.end(),
                snapdragoncolor::kPbHdrBlob) != color_mode.hw_assets.end()) {
    dynamic_range = kHdrType;
  }
  if ((color_mode.gamut == ColorPrimaries_BT2020 && color_mode.gamma == Transfer_SMPTE_ST2084) ||
      (color_mode.gamut == ColorPrimaries_BT2020 && color_mode.gamma == Transfer_HLG)) {
    dynamic_range = kHdrType;
  }
  comp_manager_->ControlDpps(dynamic_range != kHdrType);

  return ret;
}

DisplayError DisplayBuiltIn::NotifyDisplayCalibrationMode(bool in_calibration) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!color_mgr_) {
    return kErrorNotSupported;
  }
  DisplayError ret = kErrorNone;
  ret = color_mgr_->NotifyDisplayCalibrationMode(in_calibration);
  if (ret != kErrorNone) {
    DLOGE("Failed to notify QDCM Mode status, ret = %d state = %d", ret, in_calibration);
  }

  return ret;
}

std::string DisplayBuiltIn::Dump() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  HWDisplayAttributes attrib;
  uint32_t active_index = 0;
  uint32_t num_modes = 0;
  std::ostringstream os;

  hw_intf_->GetNumDisplayAttributes(&num_modes);
  hw_intf_->GetActiveConfig(&active_index);
  hw_intf_->GetDisplayAttributes(active_index, &attrib);

  os << "device type:" << display_type_;
  os << "\nstate: " << state_ << " vsync on: " << vsync_enable_
     << " max. mixer stages: " << max_mixer_stages_;
  os << "\nnum configs: " << num_modes << " active config index: " << active_index;
  os << "\nDisplay Attributes:";
  os << "\n Mode:" << (hw_panel_info_.mode == kModeVideo ? "Video" : "Command");
  os << std::boolalpha;
  os << " Primary:" << hw_panel_info_.is_primary_panel;
  os << " DynFPS:" << hw_panel_info_.dynamic_fps;
  os << "\n HDR Panel:" << hw_panel_info_.hdr_enabled;
  os << " QSync:" << hw_panel_info_.qsync_support;
  os << " DynBitclk:" << hw_panel_info_.dyn_bitclk_support;
  os << "\n Left Split:" << hw_panel_info_.split_info.left_split
     << " Right Split:" << hw_panel_info_.split_info.right_split;
  os << "\n PartialUpdate:" << hw_panel_info_.partial_update;
  if (hw_panel_info_.partial_update) {
    os << "\n ROI Min w:" << hw_panel_info_.min_roi_width;
    os << " Min h:" << hw_panel_info_.min_roi_height;
    os << " NeedsMerge: " << hw_panel_info_.needs_roi_merge;
    os << " Alignment: l:" << hw_panel_info_.left_align << " w:" << hw_panel_info_.width_align;
    os << " t:" << hw_panel_info_.top_align << " b:" << hw_panel_info_.height_align;
  }
  os << "\n FPS min:" << hw_panel_info_.min_fps << " max:" << hw_panel_info_.max_fps
     << " cur:" << display_attributes_.fps;
  os << " TransferTime: " << hw_panel_info_.transfer_time_us << "us";
  os << " AllowedModeSwitch: " << hw_panel_info_.allowed_mode_switch;
  os << " MaxBrightness:" << hw_panel_info_.panel_max_brightness;
  os << "\n Display WxH: " << display_attributes_.x_pixels << "x" << display_attributes_.y_pixels;
  os << " MixerWxH: " << mixer_attributes_.width << "x" << mixer_attributes_.height;
  os << " DPI: " << display_attributes_.x_dpi << "x" << display_attributes_.y_dpi;
  os << " LM_Split: " << display_attributes_.is_device_split;
  os << "\n vsync_period " << display_attributes_.vsync_period_ns;
  os << " v_back_porch: " << display_attributes_.v_back_porch;
  os << " v_front_porch: " << display_attributes_.v_front_porch;
  os << " v_pulse_width: " << display_attributes_.v_pulse_width;
  os << "\n v_total: " << display_attributes_.v_total;
  os << " h_total: " << display_attributes_.h_total;
  os << " clk: " << display_attributes_.clock_khz;
  os << " Topology: " << display_attributes_.topology;
  os << " Qsync mode: " << active_qsync_mode_;
  os << std::noboolalpha;

  DynamicRangeType curr_dynamic_range = kSdrType;
  if (std::find(current_color_mode_.hw_assets.begin(), current_color_mode_.hw_assets.end(),
                snapdragoncolor::kPbHdrBlob) != current_color_mode_.hw_assets.end()) {
    curr_dynamic_range = kHdrType;
  }
  os << "\nCurrent Color Mode: gamut " << current_color_mode_.gamut << " gamma "
     << current_color_mode_.gamma << " intent " << current_color_mode_.intent << " Dynamice_range"
     << (curr_dynamic_range == kSdrType ? " SDR" : " HDR");

  uint32_t num_hw_layers = 0;
  if (hw_layers_.info.stack) {
    num_hw_layers = UINT32(hw_layers_.info.hw_layers.size());
  }

  if (num_hw_layers == 0) {
    os << "\nNo hardware layers programmed";
    return os.str();
  }

  LayerBuffer *out_buffer = hw_layers_.info.stack->output_buffer;
  if (out_buffer) {
    os << "\n Output buffer res: " << out_buffer->width << "x" << out_buffer->height
       << " format: " << GetFormatString(out_buffer->format);
  }
  HWLayersInfo &layer_info = hw_layers_.info;
  for (uint32_t i = 0; i < layer_info.left_frame_roi.size(); i++) {
    LayerRect &l_roi = layer_info.left_frame_roi.at(i);
    LayerRect &r_roi = layer_info.right_frame_roi.at(i);

    os << "\nROI(LTRB)#" << i << " LEFT(" << INT(l_roi.left) << " " << INT(l_roi.top) << " " <<
      INT(l_roi.right) << " " << INT(l_roi.bottom) << ")";
    if (IsValid(r_roi)) {
    os << " RIGHT(" << INT(r_roi.left) << " " << INT(r_roi.top) << " " << INT(r_roi.right) << " "
      << INT(r_roi.bottom) << ")";
    }
  }

  LayerRect &fb_roi = layer_info.partial_fb_roi;
  if (IsValid(fb_roi)) {
    os << "\nPartial FB ROI(LTRB):(" << INT(fb_roi.left) << " " << INT(fb_roi.top) << " " <<
      INT(fb_roi.right) << " " << INT(fb_roi.bottom) << ")";
  }

  const char *header  = "\n| Idx |   Comp Type   |   Split   | Pipe |    W x H    |          Format          |  Src Rect (L T R B) |  Dst Rect (L T R B) |  Z | Pipe Flags | Deci(HxV) | CS | Rng | Tr |";  //NOLINT
  const char *newline = "\n|-----|---------------|-----------|------|-------------|--------------------------|---------------------|---------------------|----|------------|-----------|----|-----|----|";  //NOLINT
  const char *format  = "\n| %3s | %13s | %9s | %4d | %4d x %4d | %24s | %4d %4d %4d %4d | %4d %4d %4d %4d | %2s | %10s | %9s | %2s | %3s | %2s |";  //NOLINT

  os << "\n";
  os << newline;
  os << header;
  os << newline;

  for (uint32_t i = 0; i < num_hw_layers; i++) {
    uint32_t layer_index = hw_layers_.info.index.at(i);
    // sdm-layer from client layer stack
    Layer *sdm_layer = hw_layers_.info.stack->layers.at(layer_index);
    // hw-layer from hw layers info
    Layer &hw_layer = hw_layers_.info.hw_layers.at(i);
    LayerBuffer *input_buffer = &hw_layer.input_buffer;
    HWLayerConfig &layer_config = hw_layers_.config[i];
    HWRotatorSession &hw_rotator_session = layer_config.hw_rotator_session;

    const char *comp_type = GetName(sdm_layer->composition);
    const char *buffer_format = GetFormatString(input_buffer->format);
    const char *pipe_split[2] = { "Pipe-1", "Pipe-2" };
    const char *rot_pipe[2] = { "Rot-inl-1", "Rot-inl-2" };
    char idx[8];

    snprintf(idx, sizeof(idx), "%d", layer_index);

    for (uint32_t count = 0; count < hw_rotator_session.hw_block_count; count++) {
      char row[1024];
      HWRotateInfo &rotate = hw_rotator_session.hw_rotate_info[count];
      LayerRect &src_roi = rotate.src_roi;
      LayerRect &dst_roi = rotate.dst_roi;
      char rot[12] = { 0 };

      snprintf(rot, sizeof(rot), "Rot-%s-%d", layer_config.use_inline_rot ?
               "inl" : "off", count + 1);

      snprintf(row, sizeof(row), format, idx, comp_type, rot,
               0, input_buffer->width, input_buffer->height, buffer_format,
               INT(src_roi.left), INT(src_roi.top), INT(src_roi.right), INT(src_roi.bottom),
               INT(dst_roi.left), INT(dst_roi.top), INT(dst_roi.right), INT(dst_roi.bottom),
               "-", "-    ", "-    ", "-", "-", "-");
      os << row;
      // print the below only once per layer block, fill with spaces for rest.
      idx[0] = 0;
      comp_type = "";
    }

    if (hw_rotator_session.hw_block_count > 0) {
      input_buffer = &hw_rotator_session.output_buffer;
      buffer_format = GetFormatString(input_buffer->format);
    }

    if (layer_config.use_solidfill_stage) {
      LayerRect src_roi = layer_config.hw_solidfill_stage.roi;
      const char *decimation = "";
      char flags[16] = { 0 };
      char z_order[8] = { 0 };
      const char *color_primary = "";
      const char *range = "";
      const char *transfer = "";
      char row[1024] = { 0 };

      snprintf(z_order, sizeof(z_order), "%d", layer_config.hw_solidfill_stage.z_order);
      snprintf(flags, sizeof(flags), "0x%08x", hw_layer.flags.flags);
      snprintf(row, sizeof(row), format, idx, comp_type, pipe_split[0],
               0, INT(src_roi.right), INT(src_roi.bottom),
               buffer_format, INT(src_roi.left), INT(src_roi.top),
               INT(src_roi.right), INT(src_roi.bottom), INT(src_roi.left),
               INT(src_roi.top), INT(src_roi.right), INT(src_roi.bottom),
               z_order, flags, decimation, color_primary, range, transfer);
      os << row;
      continue;
    }

    for (uint32_t count = 0; count < 2; count++) {
      char decimation[16] = { 0 };
      char flags[16] = { 0 };
      char z_order[8] = { 0 };
      char color_primary[8] = { 0 };
      char range[8] = { 0 };
      char transfer[8] = { 0 };
      bool rot = layer_config.use_inline_rot;

      HWPipeInfo &pipe = (count == 0) ? layer_config.left_pipe : layer_config.right_pipe;

      if (!pipe.valid) {
        continue;
      }

      LayerRect src_roi = pipe.src_roi;
      LayerRect &dst_roi = pipe.dst_roi;

      snprintf(z_order, sizeof(z_order), "%d", pipe.z_order);
      snprintf(flags, sizeof(flags), "0x%08x", pipe.flags);
      snprintf(decimation, sizeof(decimation), "%3d x %3d", pipe.horizontal_decimation,
               pipe.vertical_decimation);
      ColorMetaData &color_metadata = hw_layer.input_buffer.color_metadata;
      snprintf(color_primary, sizeof(color_primary), "%d", color_metadata.colorPrimaries);
      snprintf(range, sizeof(range), "%d", color_metadata.range);
      snprintf(transfer, sizeof(transfer), "%d", color_metadata.transfer);

      char row[1024];
      snprintf(row, sizeof(row), format, idx, comp_type, rot ? rot_pipe[count] : pipe_split[count],
               pipe.pipe_id, input_buffer->width, input_buffer->height,
               buffer_format, INT(src_roi.left), INT(src_roi.top),
               INT(src_roi.right), INT(src_roi.bottom), INT(dst_roi.left),
               INT(dst_roi.top), INT(dst_roi.right), INT(dst_roi.bottom),
               z_order, flags, decimation, color_primary, range, transfer);

      os << row;
      // print the below only once per layer block, fill with spaces for rest.
      idx[0] = 0;
      comp_type = "";
    }
  }

  os << newline << "\n";

  return os.str();
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

DisplayError DisplayBuiltIn::GetQSyncMode(QSyncMode *qsync_mode) {
  *qsync_mode = active_qsync_mode_;
  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetQSyncMode(QSyncMode qsync_mode) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  if (!hw_panel_info_.qsync_support || first_cycle_) {
    DLOGE("Failed: qsync_support: %d first_cycle %d", hw_panel_info_.qsync_support,
          first_cycle_);
    return kErrorNotSupported;
  }

  if (qsync_mode_ == qsync_mode) {
    DLOGW("Qsync mode already set as requested mode: qsync_mode_=%d", qsync_mode_);
    return kErrorNone;
  }

  qsync_mode_ = qsync_mode;
  needs_avr_update_ = true;
  event_handler_->Refresh();
  DLOGI("Qsync mode set to %d successfully", qsync_mode_);

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
  const bool fps_switch = display_unchanged && (display_attributes.fps != current_refresh_rate_);
  if (!dirty && display_unchanged && mixer_unchanged && panel_unchanged && !fps_switch) {
     return kErrorNone;
  }

  if (CanDeferFpsConfig(display_attributes.fps)) {
    deferred_config_.Init(display_attributes.fps, display_attributes.vsync_period_ns,
                          hw_panel_info.transfer_time_us);

    // Apply current config until new Fps is deferred.
    GetFpsConfig(&display_attributes, &hw_panel_info);
  }

  if (fps_switch) {
    uint32_t config;
    error = hw_intf_->GetConfigIndexForFps(current_refresh_rate_, &config);
    if (error == kErrorNone) {
      hw_intf_->GetDisplayAttributes(config, &display_attributes);
    }
  } else {
    current_refresh_rate_ = display_attributes.fps;
  }

  fb_config_.fps = display_attributes.fps;
  error = comp_manager_->ReconfigureDisplay(display_comp_ctx_, display_attributes, hw_panel_info,
                                            mixer_attributes, fb_config_,
                                            &cached_qos_data_);
  if (error != kErrorNone) {
    return error;
  }
  default_clock_hz_ = cached_qos_data_.clock_hz;

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

PrimariesTransfer DisplayBuiltIn::GetBlendSpaceFromStcColorMode(
    const snapdragoncolor::ColorMode &color_mode) {
  PrimariesTransfer blend_space = {};
  if (!color_mgr_) {
    return blend_space;
  }

  // Set sRGB as default blend space.
  if (stc_color_modes_.list.empty() || color_mode.intent == snapdragoncolor::kNative ||
      (color_mode.gamut == ColorPrimaries_Max && color_mode.gamma == Transfer_Max)) {
    return blend_space;
  }

  blend_space.primaries = color_mode.gamut;
  blend_space.transfer = color_mode.gamma;

  return blend_space;
}

DisplayError DisplayBuiltIn::GetConfig(DisplayConfigFixedInfo *fixed_info) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  fixed_info->is_cmdmode = (hw_panel_info_.mode == kModeCommand);

  HWResourceInfo hw_resource_info = HWResourceInfo();
  hw_info_intf_->GetHWResourceInfo(&hw_resource_info);

  fixed_info->hdr_supported = hw_resource_info.has_hdr;
  // Built-in displays always support HDR10+ when the target supports HDR
  fixed_info->hdr_plus_supported = hw_resource_info.has_hdr;
  // Populate luminance values only if hdr will be supported on that display
  fixed_info->max_luminance = fixed_info->hdr_supported ? hw_panel_info_.peak_luminance: 0;
  fixed_info->average_luminance = fixed_info->hdr_supported ? hw_panel_info_.average_luminance : 0;
  fixed_info->min_luminance = fixed_info->hdr_supported ?  hw_panel_info_.blackness_level: 0;
  fixed_info->hdr_eotf = hw_panel_info_.hdr_eotf;
  fixed_info->hdr_metadata_type_one = hw_panel_info_.hdr_metadata_type_one;
  fixed_info->partial_update = hw_panel_info_.partial_update;
  fixed_info->readback_supported = hw_resource_info.has_concurrent_writeback;

  return kErrorNone;
}

void DisplayBuiltIn::SendBacklight() {
  DisplayError err = kErrorNone;
  int level = 0;
  if ((err = hw_intf_->GetPanelBrightness(&level)) != kErrorNone) {
    return;
  }
  HandleBacklightEvent(level);
}

void DisplayBuiltIn::SendDisplayConfigs() {
  if (ipc_intf_) {
    GenericPayload in;
    uint32_t active_index = 0;
    IPCDisplayConfigParams *disp_configs = nullptr;
    int ret = in.CreatePayload<IPCDisplayConfigParams>(disp_configs);
    if (ret) {
      DLOGW("failed to create the payload. Error:%d", ret);
      return;
    }
    DisplayError error = hw_intf_->GetActiveConfig(&active_index);
    if (error != kErrorNone) {
      return;
    }
    disp_configs->x_pixels = display_attributes_.x_pixels;
    disp_configs->y_pixels = display_attributes_.y_pixels;
    disp_configs->fps = display_attributes_.fps;
    disp_configs->config_idx = active_index;
    disp_configs->smart_panel = display_attributes_.smart_panel;
     disp_configs->is_primary = IsPrimaryDisplay();
    if ((ret = ipc_intf_->SetParameter(kIpcParamSetDisplayConfigs, in))) {
      DLOGW("Failed to send display config, error = %d", ret);
    }
  }
}

DisplayError DisplayBuiltIn::TeardownConcurrentWriteback() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  return hw_intf_->TeardownConcurrentWriteback();
}

}  // namespace sdm
