/*
* Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
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

/*
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <cutils/properties.h>
#include <sync/sync.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/utils.h>
#include <stdarg.h>
#include <sys/mman.h>

#include <map>
#include <string>
#include <vector>

#include "hwc_display_builtin.h"
#include "hwc_color_mode_stc.h"
#include "hwc_debugger.h"
#include "hwc_session.h"

#define __CLASS__ "HWCDisplayBuiltIn"

namespace sdm {

static void SetRect(LayerRect &src_rect, GLRect *target) {
  target->left = src_rect.left;
  target->top = src_rect.top;
  target->right = src_rect.right;
  target->bottom = src_rect.bottom;
}

int HWCDisplayBuiltIn::Create(CoreInterface *core_intf, BufferAllocator *buffer_allocator,
                              HWCCallbacks *callbacks, HWCDisplayEventHandler *event_handler,
                              qService::QService *qservice, hwc2_display_t id, int32_t sdm_id,
                              HWCDisplay **hwc_display) {
  int status = 0;
  uint32_t builtin_width = 0;
  uint32_t builtin_height = 0;

  HWCDisplay *hwc_display_builtin =
      new HWCDisplayBuiltIn(core_intf, static_cast<HWCBufferAllocator *>(buffer_allocator),
                            callbacks, event_handler, qservice, id, sdm_id);
  status = hwc_display_builtin->Init();
  if (status) {
    delete hwc_display_builtin;
    return status;
  }

  hwc_display_builtin->GetMixerResolution(&builtin_width, &builtin_height);
  int width = 0, height = 0;
  HWCDebugHandler::Get()->GetProperty(FB_WIDTH_PROP, &width);
  HWCDebugHandler::Get()->GetProperty(FB_HEIGHT_PROP, &height);
  if (width > 0 && height > 0) {
    builtin_width = UINT32(width);
    builtin_height = UINT32(height);
  }

  status = hwc_display_builtin->SetFrameBufferResolution(builtin_width, builtin_height);
  if (status) {
    Destroy(hwc_display_builtin);
    return status;
  }

  *hwc_display = hwc_display_builtin;

  return status;
}

void HWCDisplayBuiltIn::Destroy(HWCDisplay *hwc_display) {
  hwc_display->Deinit();
  delete hwc_display;
}

HWCDisplayBuiltIn::HWCDisplayBuiltIn(CoreInterface *core_intf, HWCBufferAllocator *buffer_allocator,
                                     HWCCallbacks *callbacks, HWCDisplayEventHandler *event_handler,
                                     qService::QService *qservice, hwc2_display_t id,
                                     int32_t sdm_id)
    : HWCDisplay(core_intf, buffer_allocator, callbacks, event_handler, qservice, kBuiltIn, id,
                 sdm_id, DISPLAY_CLASS_BUILTIN),
      buffer_allocator_(buffer_allocator),
      cpu_hint_(NULL), layer_stitch_task_(*this) {
}

int HWCDisplayBuiltIn::Init() {
  cpu_hint_ = new CPUHint();
  if (cpu_hint_->Init(static_cast<HWCDebugHandler *>(HWCDebugHandler::Get())) != kErrorNone) {
    delete cpu_hint_;
    cpu_hint_ = NULL;
  }

  layer_stack_.flags.use_metadata_refresh_rate = true;
  int disable_metadata_dynfps = 0;
  HWCDebugHandler::Get()->GetProperty(DISABLE_METADATA_DYNAMIC_FPS_PROP, &disable_metadata_dynfps);
  if (disable_metadata_dynfps) {
    layer_stack_.flags.use_metadata_refresh_rate = false;
  }

  int status = HWCDisplay::Init();
  if (status) {
    return status;
  }
  color_mode_ = new HWCColorModeStc(display_intf_);
  color_mode_->Init();

  int value = 0;
  HWCDebugHandler::Get()->GetProperty(ENABLE_OPTIMIZE_REFRESH, &value);
  enable_optimize_refresh_ = (value == 1);
  if (enable_optimize_refresh_) {
    DLOGI("Drop redundant drawcycles %" PRIu64 , id_);
  }

  int vsyncs = 0;
  HWCDebugHandler::Get()->GetProperty(TRANSIENT_FPS_CYCLE_COUNT, &vsyncs);
  if (vsyncs > 0) {
    SetVsyncsApplyRateChange(UINT32(vsyncs));
  }

  is_primary_ = display_intf_->IsPrimaryDisplay();

  windowed_display_ = Debug::GetWindowRect(is_primary_, &window_rect_.left, &window_rect_.top,
                             &window_rect_.right, &window_rect_.bottom) == 0;
  DLOGI("Window rect : [%f %f %f %f] is_primary_=%d", window_rect_.left, window_rect_.top,
         window_rect_.right, window_rect_.bottom, is_primary_);

  if (is_primary_) {
    value = 0;
    HWCDebugHandler::Get()->GetProperty(ENABLE_POMS_DURING_DOZE, &value);
    enable_poms_during_doze_ = (value == 1);
    if (enable_poms_during_doze_) {
      DLOGI("Enable POMS during Doze mode %" PRIu64 , id_);
    }
  }

  HWCDebugHandler::Get()->GetProperty(ENABLE_PERF_HINT_LARGE_COMP_CYCLE,
                                      &perf_hint_large_comp_cycle_);

  value = 0;
  DebugHandler::Get()->GetProperty(DISABLE_DYNAMIC_FPS, &value);
  disable_dyn_fps_ = (value == 1);

  value = 0;
  DebugHandler::Get()->GetProperty(ENABLE_ROUNDED_CORNER, &value);
  enable_round_corner_ = (value == 1);

  uint32_t config_index = 0;
  GetActiveDisplayConfig(&config_index);
  DisplayConfigVariableInfo attr = {};
  GetDisplayAttributesForConfig(INT(config_index), &attr);
  active_refresh_rate_ = attr.fps;

  DLOGI("active_refresh_rate: %d", active_refresh_rate_);

  int enhance_idle_time = 0;
  HWCDebugHandler::Get()->GetProperty(ENHANCE_IDLE_TIME, &enhance_idle_time);
  enhance_idle_time_ = (enhance_idle_time == 1);
  DLOGI("enhance_idle_time: %d", enhance_idle_time);

  LoadMixedModePerfHintThreshold();

  return status;
}

void HWCDisplayBuiltIn::Dump(std::ostringstream *os) {
  HWCDisplay::Dump(os);
  *os << histogram.Dump();
}

void HWCDisplayBuiltIn::ValidateUiScaling() {
  if (is_primary_ || !is_cmd_mode_) {
    force_reset_lut_ = false;
    return;
  }

  for (auto &hwc_layer : layer_set_) {
    Layer *layer = hwc_layer->GetSDMLayer();
    if (hwc_layer->IsScalingPresent() && !layer->input_buffer.flags.video) {
      force_reset_lut_ = true;
      return;
    }
  }
  force_reset_lut_ = false;
}

HWC2::Error HWCDisplayBuiltIn::PreValidateDisplay(bool *exit_validate) {
  DTRACE_SCOPED();

  // Draw method gets set as part of first commit.
  SetDrawMethod();

  auto status = HWC2::Error::None;
  bool res_exhausted = false;
  // If no resources are available for the current display, mark it for GPU by pass and continue to
  // do invalidate until the resources are available
  if (display_paused_ || CheckResourceState(&res_exhausted)) {
    MarkLayersForGPUBypass();
    *exit_validate = true;
    bypass_drawcycle_ = true;
    return status;
  }

  if (color_tranform_failed_) {
    // Must fall back to client composition
    MarkLayersForClientComposition();
  }

  // Fill in the remaining blanks in the layers and add them to the SDM layerstack
  BuildLayerStack();

  // Check for scaling layers during Doze mode
  ValidateUiScaling();

  // Add stitch layer to layer stack.
  AppendStitchLayer();

  // Checks and replaces layer stack for solid fill
  SolidFillPrepare();

  // Apply current Color Mode and Render Intent.
  if (color_mode_->ApplyCurrentColorModeWithRenderIntent(
      static_cast<bool>(layer_stack_.flags.hdr_present)) != HWC2::Error::None) {
    // Fallback to GPU Composition, if Color Mode can't be applied.
    MarkLayersForClientComposition();
  }

  SetCwbState();

  uint32_t refresh_rate = 0;
  display_intf_->GetRefreshRate(&refresh_rate);
  current_refresh_rate_ = refresh_rate;

  if (layer_set_.empty()) {
    // Avoid flush for Command mode panel.
    flush_ = !client_connected_;
    *exit_validate = true;
    return status;
  }

  display_idle_ = false;
  has_client_composition_ = false;

  *exit_validate = false;

  return status;
}

HWC2::Error HWCDisplayBuiltIn::CommitLayerStack() {
  return HWCDisplay::CommitLayerStack();
}

bool HWCDisplayBuiltIn::CanSkipCommit() {
  if (layer_stack_invalid_) {
    return false;
  }

  // Reject repeated drawcycle requests if it satisfies all conditions.
  // 1. None of the layerstack attributes changed.
  // 2. No new buffer latched.
  // 3. No refresh request triggered by HWC.
  // 4. This display is not source of vsync.
  // 5. No CWB client
  bool buffers_latched = false;
  bool needs_validation = false;
  for (auto &hwc_layer : layer_set_) {
    buffers_latched |= hwc_layer->BufferLatched();
    hwc_layer->ResetBufferFlip();
    needs_validation |= hwc_layer->NeedsValidation();
  }

  bool vsync_source = (callbacks_->GetVsyncSource() == id_);

  bool skip_commit = false;
  {
    std::lock_guard<std::mutex> lock(cwb_state_lock_);  // setting cwb state lock
    skip_commit = enable_optimize_refresh_ && !pending_commit_ && !buffers_latched &&
                  !pending_refresh_ && !vsync_source && (cwb_state_.cwb_client == kCWBClientNone)
                  && !needs_validation;
  }  // releasing the cwb state lock
  pending_refresh_ = false;

  return skip_commit;
}

HWC2::Error HWCDisplayBuiltIn::CommitStitchLayers() {
  if (disable_layer_stitch_) {
    return HWC2::Error::None;
  }

  if (!display_intf_->IsValidated() || skip_commit_) {
    return HWC2::Error::None;
  }

  LayerStitchContext ctx = {};
  Layer *stitch_layer = stitch_target_->GetSDMLayer();
  LayerBuffer &output_buffer = stitch_layer->input_buffer;
  for (auto &layer : layer_stack_.layers) {
    LayerComposition &composition = layer->composition;
    if (composition != kCompositionStitch) {
      continue;
    }

    StitchParams params = {};
    // Stitch target doesn't have an input fence.
    // Render all layers at specified destination.
    LayerBuffer &input_buffer = layer->input_buffer;
    params.src_hnd = reinterpret_cast<const native_handle_t *>(input_buffer.buffer_id);
    params.dst_hnd = reinterpret_cast<const native_handle_t *>(output_buffer.buffer_id);
    SetRect(layer->stitch_info.dst_rect, &params.dst_rect);
    SetRect(layer->stitch_info.slice_rect, &params.scissor_rect);
    params.src_acquire_fence = input_buffer.acquire_fence;

    ctx.stitch_params.push_back(params);
  }

  if (!ctx.stitch_params.size()) {
    // No layers marked for stitch.
    return HWC2::Error::None;
  }

  layer_stitch_task_.PerformTask(LayerStitchTaskCode::kCodeStitch, &ctx);
  // Set release fence.
  output_buffer.acquire_fence = ctx.release_fence;

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::SetPowerMode(HWC2::PowerMode mode, bool teardown) {
  auto status = HWCDisplay::SetPowerMode(mode, teardown);
  if (status != HWC2::Error::None) {
    return status;
  }
  DLOGV_IF(kTagClient, "Setting Power State as \'%s\' for %d-%d", (mode == HWC2::PowerMode::On)?
           "ON": (mode == HWC2::PowerMode::Off)? "OFF": (mode == HWC2::PowerMode::Doze)? "DOZE":
           "DOZE_SUSPEND", sdm_id_, type_);
  if (cpu_hint_) {
    switch (mode) {
      case HWC2::PowerMode::Doze:
      case HWC2::PowerMode::DozeSuspend:
        // Perf hal doesn't differentiate b/w doze and doze-suspend, so send doze hint for both.
        cpu_hint_->ReqEvent(kPerfHintDisplayDoze);
        break;
      case HWC2::PowerMode::On:
        cpu_hint_->ReqEvent(kPerfHintDisplayOn);
        break;
      case HWC2::PowerMode::Off:
        cpu_hint_->ReqEvent(kPerfHintDisplayOff);
        break;
      default:
        break;
    }
  }

  DisplayConfigFixedInfo fixed_info = {};
  display_intf_->GetConfig(&fixed_info);
  is_cmd_mode_ = fixed_info.is_cmdmode;

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::Present(shared_ptr<Fence> *out_retire_fence) {
  auto status = HWC2::Error::None;
  bool res_exhausted = false;

  DTRACE_SCOPED();

  if (bypass_drawcycle_) {
    bypass_drawcycle_ = false;
    return status;
  }

  // Proceed only if any resources are available to be allocated for the current display,
  // Otherwise keep doing invalidate
  if (CheckResourceState(&res_exhausted)) {
    Refresh();
    return status;
  }

  if (display_paused_ ) {
    return status;
  } else {
    if (status != HWC2::Error::None) {
      DLOGE("Stitch failed: %d", status);
      return status;
    }

    status = CommitLayerStack();
    if (status == HWC2::Error::None) {
      status = PostCommitLayerStack(out_retire_fence);
    }
  }

  // In case of scaling UI layer for command mode, clear LUTs
  if (force_reset_lut_) {
    display_intf_->ClearLUTs();
  }
  return status;
}

void HWCDisplayBuiltIn::PostCommitStitchLayers() {
  if (disable_layer_stitch_) {
    return;
  }

  // Close Stitch buffer acquire fence.
  Layer *stitch_layer = stitch_target_->GetSDMLayer();
  LayerBuffer &output_buffer = stitch_layer->input_buffer;
  for (auto &layer : layer_stack_.layers) {
    LayerComposition &composition = layer->composition;
    if (composition != kCompositionStitch) {
      continue;
    }
    LayerBuffer &input_buffer = layer->input_buffer;
    input_buffer.release_fence = output_buffer.acquire_fence;
  }
}

HWC2::Error HWCDisplayBuiltIn::GetColorModes(uint32_t *out_num_modes, ColorMode *out_modes) {
  if (out_modes == nullptr) {
    *out_num_modes = color_mode_->GetColorModeCount();
  } else {
    color_mode_->GetColorModes(out_num_modes, out_modes);
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::GetRenderIntents(ColorMode mode, uint32_t *out_num_intents,
                                                RenderIntent *out_intents) {
  if (out_intents == nullptr) {
    *out_num_intents = color_mode_->GetRenderIntentCount(mode);
  } else {
    color_mode_->GetRenderIntents(mode, out_num_intents, out_intents);
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::SetColorMode(ColorMode mode) {
  return SetColorModeWithRenderIntent(mode, RenderIntent::COLORIMETRIC);
}

HWC2::Error HWCDisplayBuiltIn::SetColorModeWithRenderIntent(ColorMode mode, RenderIntent intent) {
  auto status = color_mode_->CacheColorModeWithRenderIntent(mode, intent);
  if (status != HWC2::Error::None) {
    DLOGE("failed for mode = %d intent = %d", mode, intent);
    return status;
  }
  callbacks_->Refresh(id_);
  return status;
}

HWC2::Error HWCDisplayBuiltIn::SetColorModeById(int32_t color_mode_id) {
  auto status = color_mode_->SetColorModeById(color_mode_id);
  if (status != HWC2::Error::None) {
    DLOGE("failed for mode = %d", color_mode_id);
    return status;
  }

  callbacks_->Refresh(id_);

  return status;
}

HWC2::Error HWCDisplayBuiltIn::SetColorModeFromClientApi(int32_t color_mode_id) {
  DisplayError error = kErrorNone;
  std::string mode_string;

  error = display_intf_->GetColorModeName(color_mode_id, &mode_string);
  if (error) {
    DLOGE("Failed to get mode name for mode %d", color_mode_id);
    return HWC2::Error::BadParameter;
  }

  auto status = color_mode_->SetColorModeFromClientApi(mode_string);
  if (status != HWC2::Error::None) {
    DLOGE("Failed to set mode = %d", color_mode_id);
    return status;
  }

  return status;
}

HWC2::Error HWCDisplayBuiltIn::RestoreColorTransform() {
  auto status = color_mode_->RestoreColorTransform();
  if (status != HWC2::Error::None) {
    DLOGE("failed to RestoreColorTransform");
    return status;
  }

  callbacks_->Refresh(id_);

  return status;
}

HWC2::Error HWCDisplayBuiltIn::SetColorTransform(const float *matrix,
                                                 android_color_transform_t hint) {
  if (!matrix) {
    return HWC2::Error::BadParameter;
  }

  auto status = color_mode_->SetColorTransform(matrix, hint);
  if (status != HWC2::Error::None) {
    DLOGE("failed for hint = %d", hint);
    color_tranform_failed_ = true;
    return status;
  }

  callbacks_->Refresh(id_);
  color_tranform_failed_ = false;

  return status;
}

HWC2::Error HWCDisplayBuiltIn::SetDisplayDppsAdROI(uint32_t h_start, uint32_t h_end,
                                                   uint32_t v_start, uint32_t v_end,
                                                   uint32_t factor_in, uint32_t factor_out) {
  DisplayError error = kErrorNone;
  DisplayDppsAd4RoiCfg dpps_ad4_roi_cfg = {};
  uint32_t panel_width = 0, panel_height = 0;
  constexpr uint16_t kMaxFactorVal = 0xffff;

  if (h_start >= h_end || v_start >= v_end || factor_in > kMaxFactorVal ||
      factor_out > kMaxFactorVal) {
    DLOGE("Invalid roi region = [%u, %u, %u, %u, %u, %u]",
           h_start, h_end, v_start, v_end, factor_in, factor_out);
    return HWC2::Error::BadParameter;
  }

  GetPanelResolution(&panel_width, &panel_height);

  if (h_start >= panel_width || h_end > panel_width ||
      v_start >= panel_height || v_end > panel_height) {
    DLOGE("Invalid roi region = [%u, %u, %u, %u], panel resolution = [%u, %u]",
           h_start, h_end, v_start, v_end, panel_width, panel_height);
    return HWC2::Error::BadParameter;
  }

  dpps_ad4_roi_cfg.h_start = h_start;
  dpps_ad4_roi_cfg.h_end = h_end;
  dpps_ad4_roi_cfg.v_start = v_start;
  dpps_ad4_roi_cfg.v_end = v_end;
  dpps_ad4_roi_cfg.factor_in = factor_in;
  dpps_ad4_roi_cfg.factor_out = factor_out;

  error = display_intf_->SetDisplayDppsAdROI(&dpps_ad4_roi_cfg);
  if (error)
    return HWC2::Error::BadConfig;

  callbacks_->Refresh(id_);

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::SetFrameTriggerMode(uint32_t mode) {
  DisplayError error = kErrorNone;
  FrameTriggerMode trigger_mode = kFrameTriggerDefault;

  if (mode >= kFrameTriggerMax) {
    DLOGE("Invalid input mode %d", mode);
    return HWC2::Error::BadParameter;
  }

  trigger_mode = static_cast<FrameTriggerMode>(mode);
  error = display_intf_->SetFrameTriggerMode(trigger_mode);
  if (error)
    return HWC2::Error::BadConfig;

  callbacks_->Refresh(HWC_DISPLAY_PRIMARY);

  return HWC2::Error::None;
}

int HWCDisplayBuiltIn::Perform(uint32_t operation, ...) {
  va_list args;
  va_start(args, operation);
  int val = 0;
  LayerSolidFill *solid_fill_color;
  LayerRect *rect = NULL;

  switch (operation) {
    case SET_METADATA_DYN_REFRESH_RATE:
      val = va_arg(args, int32_t);
      SetMetaDataRefreshRateFlag(val);
      break;
    case SET_BINDER_DYN_REFRESH_RATE:
      val = va_arg(args, int32_t);
      ForceRefreshRate(UINT32(val));
      break;
    case SET_DISPLAY_MODE:
      val = va_arg(args, int32_t);
      SetDisplayMode(UINT32(val));
      break;
    case SET_QDCM_SOLID_FILL_INFO:
      solid_fill_color = va_arg(args, LayerSolidFill*);
      SetQDCMSolidFillInfo(true, *solid_fill_color);
      break;
    case UNSET_QDCM_SOLID_FILL_INFO:
      solid_fill_color = va_arg(args, LayerSolidFill*);
      SetQDCMSolidFillInfo(false, *solid_fill_color);
      break;
    case SET_QDCM_SOLID_FILL_RECT:
      rect = va_arg(args, LayerRect*);
      solid_fill_rect_ = *rect;
      break;
    default:
      DLOGW("Invalid operation %d", operation);
      va_end(args);
      return -EINVAL;
  }
  va_end(args);

  return 0;
}

DisplayError HWCDisplayBuiltIn::SetDisplayMode(uint32_t mode) {
  DisplayError error = kErrorNone;

  if (display_intf_) {
    error = display_intf_->SetDisplayMode(mode);
    if (error == kErrorNone) {
      DisplayConfigFixedInfo fixed_info = {};
      display_intf_->GetConfig(&fixed_info);
      is_cmd_mode_ = fixed_info.is_cmdmode;
    }
  }

  return error;
}

void HWCDisplayBuiltIn::SetMetaDataRefreshRateFlag(bool enable) {
  int disable_metadata_dynfps = 0;

  HWCDebugHandler::Get()->GetProperty(DISABLE_METADATA_DYNAMIC_FPS_PROP, &disable_metadata_dynfps);
  if (disable_metadata_dynfps) {
    return;
  }
  layer_stack_.flags.use_metadata_refresh_rate = enable;
}

void HWCDisplayBuiltIn::SetQDCMSolidFillInfo(bool enable, const LayerSolidFill &color) {
  solid_fill_enable_ = enable;
  solid_fill_color_ = color;
}

int HWCDisplayBuiltIn::GetActiveSecureSession(std::bitset<kSecureMax> *secure_sessions) {
  if (!secure_sessions) {
    return -1;
  }
  secure_sessions->reset();
  for (auto hwc_layer : layer_set_) {
    Layer *layer = hwc_layer->GetSDMLayer();
    if (layer->input_buffer.flags.secure_camera) {
      secure_sessions->set(kSecureCamera);
    }
    if (layer->input_buffer.flags.secure_display) {
      secure_sessions->set(kSecureDisplay);
    }
  }
  if (secure_event_ == kTUITransitionStart || secure_event_ == kTUITransitionPrepare) {
    secure_sessions->set(kSecureTUI);
  }
  return 0;
}

int HWCDisplayBuiltIn::HandleSecureSession(const std::bitset<kSecureMax> &secure_sessions,
                                           bool *power_on_pending, bool is_active_secure_display) {
  if (!power_on_pending) {
    return -EINVAL;
  }

  if (!is_active_secure_display) {
    // Do handling as done on non-primary displays.
    DLOGI("Default handling for display %" PRIu64 " %d-%d", id_, sdm_id_, type_);
    return HWCDisplay::HandleSecureSession(secure_sessions, power_on_pending,
                                           is_active_secure_display);
  }

  if (current_power_mode_ != HWC2::PowerMode::On) {
    return 0;
  }

  if (active_secure_sessions_[kSecureDisplay] != secure_sessions[kSecureDisplay]) {
    SecureEvent secure_event =
        secure_sessions.test(kSecureDisplay) ? kSecureDisplayStart : kSecureDisplayEnd;
    bool needs_refresh = false;
    DisplayError err = display_intf_->HandleSecureEvent(secure_event, &needs_refresh);
    if (err != kErrorNone) {
      DLOGE("Set secure event failed");
      return err;
    }

    DLOGI("SecureDisplay state changed from %d to %d for display %" PRIu64 " %d-%d",
          active_secure_sessions_.test(kSecureDisplay), secure_sessions.test(kSecureDisplay),
          id_, sdm_id_, type_);
  }
  active_secure_sessions_ = secure_sessions;
  *power_on_pending = false;
  return 0;
}

void HWCDisplayBuiltIn::ForceRefreshRate(uint32_t refresh_rate) {
  if ((refresh_rate && (refresh_rate < min_refresh_rate_ || refresh_rate > max_refresh_rate_)) ||
      layer_stack_.force_refresh_rate == refresh_rate) {
    // Cannot honor force refresh rate, as its beyond the range or new request is same
    return;
  }

  layer_stack_.force_refresh_rate = refresh_rate;

  callbacks_->Refresh(id_);

  return;
}

void HWCDisplayBuiltIn::SetIdleTimeoutMs(uint32_t timeout_ms, uint32_t inactive_ms) {
  display_intf_->SetIdleTimeoutMs(timeout_ms, inactive_ms);
}

void HWCDisplayBuiltIn::HandleFrameCapture() {
  if (readback_configured_ && output_buffer_.release_fence) {
    frame_capture_status_ = Fence::Wait(output_buffer_.release_fence);
  }

  std::lock_guard<std::mutex> lock(cwb_state_lock_);
  DLOGV_IF(kTagQDCM, "Frame captured successfully for cwb_client %d on cwb_tap_point %d",
           cwb_state_.cwb_client, cwb_config_.tap_point);

  frame_capture_buffer_queued_ = false;
  readback_buffer_queued_ = false;
  cwb_config_ = {};
  readback_configured_ = false;
  output_buffer_ = {};
  cwb_state_.cwb_client = kCWBClientNone;

  DLOGV_IF(kTagQDCM, "Frame captured: frame_capture_buffer_queued_ %d "
           "readback_buffer_queued_ %d cwb_tap_point %d readback_configured_ %d "
           "cwb_client %d", frame_capture_buffer_queued_, readback_buffer_queued_,
           cwb_config_.tap_point, readback_configured_, cwb_state_.cwb_client);
}

int HWCDisplayBuiltIn::FrameCaptureAsync(const BufferInfo &output_buffer_info,
                                         const CwbConfig &cwb_config) {
  {
    std::lock_guard<std::mutex> lock(cwb_state_lock_);
    if (cwb_state_.cwb_client != kCWBClientNone) {
      DLOGE("CWB is in use with client = %d", cwb_state_.cwb_client);
      return -1;
    }
  }  // releasing the cwb state lock

  // Note: This function is called in context of a binder thread and a lock is already held
  if (output_buffer_info.alloc_buffer_info.fd < 0) {
    DLOGE("Invalid fd %d", output_buffer_info.alloc_buffer_info.fd);
    return -1;
  }

  if (cwb_config.tap_point < CwbTapPoint::kLmTapPoint ||
      cwb_config.tap_point > CwbTapPoint::kDemuraTapPoint) {
    DLOGE("Invalid CWB tappoint passed by client ");
    return -1;
  }

  const native_handle_t *buffer = static_cast<native_handle_t *>(output_buffer_info.private_data);
  HWC2::Error err = SetReadbackBuffer(buffer, nullptr, cwb_config, kCWBClientColor);
  if (err != HWC2::Error::None) {
    return -1;
  }
  frame_capture_buffer_queued_ = true;
  frame_capture_status_ = -EAGAIN;

  return 0;
}

DisplayError HWCDisplayBuiltIn::SetDetailEnhancerConfig
                                   (const DisplayDetailEnhancerData &de_data) {
  DisplayError error = kErrorNotSupported;

  if (display_intf_) {
    error = display_intf_->SetDetailEnhancerData(de_data);
  }
  return error;
}

DisplayError HWCDisplayBuiltIn::SetHWDetailedEnhancerConfig(void *params) {
  DisplayError err = kErrorNone;
  DisplayDetailEnhancerData de_data;

  PPDETuningCfgData *de_tuning_cfg_data = reinterpret_cast<PPDETuningCfgData*>(params);
  if (de_tuning_cfg_data->cfg_pending) {
    if (!de_tuning_cfg_data->cfg_en) {
      de_data.enable = 0;
      DLOGV_IF(kTagQDCM, "Disable DE config");
    } else {
      de_data.override_flags = kOverrideDEEnable;
      de_data.enable = 1;

      DLOGV_IF(kTagQDCM, "Enable DE: flags %u, sharp_factor %d, thr_quiet %d, thr_dieout %d, "
        "thr_low %d, thr_high %d, clip %d, quality %d, content_type %d, de_blend %d",
        de_tuning_cfg_data->params.flags, de_tuning_cfg_data->params.sharp_factor,
        de_tuning_cfg_data->params.thr_quiet, de_tuning_cfg_data->params.thr_dieout,
        de_tuning_cfg_data->params.thr_low, de_tuning_cfg_data->params.thr_high,
        de_tuning_cfg_data->params.clip, de_tuning_cfg_data->params.quality,
        de_tuning_cfg_data->params.content_type, de_tuning_cfg_data->params.de_blend);

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagSharpFactor) {
        de_data.override_flags |= kOverrideDESharpen1;
        de_data.sharp_factor = de_tuning_cfg_data->params.sharp_factor;
      }

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagClip) {
        de_data.override_flags |= kOverrideDEClip;
        de_data.clip = de_tuning_cfg_data->params.clip;
      }

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagThrQuiet) {
        de_data.override_flags |= kOverrideDEThrQuiet;
        de_data.thr_quiet = de_tuning_cfg_data->params.thr_quiet;
      }

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagThrDieout) {
        de_data.override_flags |= kOverrideDEThrDieout;
        de_data.thr_dieout = de_tuning_cfg_data->params.thr_dieout;
      }

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagThrLow) {
        de_data.override_flags |= kOverrideDEThrLow;
        de_data.thr_low = de_tuning_cfg_data->params.thr_low;
      }

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagThrHigh) {
        de_data.override_flags |= kOverrideDEThrHigh;
        de_data.thr_high = de_tuning_cfg_data->params.thr_high;
      }

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagContentQualLevel) {
        switch (de_tuning_cfg_data->params.quality) {
          case kDeContentQualLow:
            de_data.quality_level = kContentQualityLow;
            break;
          case kDeContentQualMedium:
            de_data.quality_level = kContentQualityMedium;
            break;
          case kDeContentQualHigh:
            de_data.quality_level = kContentQualityHigh;
            break;
          case kDeContentQualUnknown:
          default:
            de_data.quality_level = kContentQualityUnknown;
            break;
        }
      }

      switch (de_tuning_cfg_data->params.content_type) {
        case kDeContentTypeVideo:
          de_data.content_type = kContentTypeVideo;
          break;
        case kDeContentTypeGraphics:
          de_data.content_type = kContentTypeGraphics;
          break;
        case kDeContentTypeUnknown:
        default:
          de_data.content_type = kContentTypeUnknown;
          break;
      }

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagDeBlend) {
        de_data.override_flags |= kOverrideDEBlend;
        de_data.de_blend = de_tuning_cfg_data->params.de_blend;
      }
    }
    err = SetDetailEnhancerConfig(de_data);
    if (err) {
      DLOGW("SetDetailEnhancerConfig failed. err = %d", err);
    }
    de_tuning_cfg_data->cfg_pending = false;
  }
  return err;
}

DisplayError HWCDisplayBuiltIn::ControlPartialUpdate(bool enable, uint32_t *pending) {
  DisplayError error = kErrorNone;

  if (display_intf_) {
    error = display_intf_->ControlPartialUpdate(enable, pending);
  }

  return error;
}

DisplayError HWCDisplayBuiltIn::DisablePartialUpdateOneFrame() {
  DisplayError error = kErrorNone;

  if (display_intf_) {
    error = display_intf_->DisablePartialUpdateOneFrame();
  }

  return error;
}

HWC2::Error HWCDisplayBuiltIn::SetDisplayedContentSamplingEnabledVndService(bool enabled) {
  std::unique_lock<decltype(sampling_mutex)> lk(sampling_mutex);
  vndservice_sampling_vote = enabled;
  if (api_sampling_vote || vndservice_sampling_vote) {
    histogram.start();
    display_intf_->colorSamplingOn();
  } else {
    display_intf_->colorSamplingOff();
    histogram.stop();
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::SetDisplayedContentSamplingEnabled(int32_t enabled,
                                                                  uint8_t component_mask,
                                                                  uint64_t max_frames) {
  if ((enabled != HWC2_DISPLAYED_CONTENT_SAMPLING_ENABLE) &&
      (enabled != HWC2_DISPLAYED_CONTENT_SAMPLING_DISABLE))
    return HWC2::Error::BadParameter;

  std::unique_lock<decltype(sampling_mutex)> lk(sampling_mutex);
  if (enabled == HWC2_DISPLAYED_CONTENT_SAMPLING_ENABLE) {
    api_sampling_vote = true;
  } else {
    api_sampling_vote = false;
  }

  auto start = api_sampling_vote || vndservice_sampling_vote;
  if (start && max_frames == 0) {
    histogram.start();
    display_intf_->colorSamplingOn();
  } else if (start) {
    histogram.start(max_frames);
    display_intf_->colorSamplingOn();
  } else {
    display_intf_->colorSamplingOff();
    histogram.stop();
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::GetDisplayedContentSamplingAttributes(
    int32_t *format, int32_t *dataspace, uint8_t *supported_components) {
  return histogram.getAttributes(format, dataspace, supported_components);
}

HWC2::Error HWCDisplayBuiltIn::GetDisplayedContentSample(
    uint64_t max_frames, uint64_t timestamp, uint64_t *numFrames,
    int32_t samples_size[NUM_HISTOGRAM_COLOR_COMPONENTS],
    uint64_t *samples[NUM_HISTOGRAM_COLOR_COMPONENTS]) {
  histogram.collect(max_frames, timestamp, samples_size, samples, numFrames);
  return HWC2::Error::None;
}

DisplayError HWCDisplayBuiltIn::SetMixerResolution(uint32_t width, uint32_t height) {
  DisplayError error = display_intf_->SetMixerResolution(width, height);
  callbacks_->Refresh(id_);
  return error;
}

DisplayError HWCDisplayBuiltIn::GetMixerResolution(uint32_t *width, uint32_t *height) {
  return display_intf_->GetMixerResolution(width, height);
}

HWC2::Error HWCDisplayBuiltIn::SetQSyncMode(QSyncMode qsync_mode) {
  // Client needs to ensure that config change and qsync mode change
  // are not triggered in the same drawcycle.
  if (pending_config_) {
    DLOGE("Failed to set qsync mode. Pending active config transition");
    return HWC2::Error::Unsupported;
  }

  auto err = display_intf_->SetQSyncMode(qsync_mode);
  if (err != kErrorNone) {
    return HWC2::Error::Unsupported;
  }

  return HWC2::Error::None;
}

DisplayError HWCDisplayBuiltIn::ControlIdlePowerCollapse(bool enable, bool synchronous) {
  DisplayError error = kErrorNone;

  if (display_intf_) {
    error = display_intf_->ControlIdlePowerCollapse(enable, synchronous);
  }
  return error;
}

DisplayError HWCDisplayBuiltIn::SetDynamicDSIClock(uint64_t bitclk) {
  DisablePartialUpdateOneFrame();
  DisplayError error = display_intf_->SetDynamicDSIClock(bitclk);
  if (error != kErrorNone) {
    DLOGE(" failed: Clk: %" PRIu64 " Error: %d", bitclk, error);
    return error;
  }

  callbacks_->Refresh(id_);

  return kErrorNone;
}

DisplayError HWCDisplayBuiltIn::GetDynamicDSIClock(uint64_t *bitclk) {
  if (display_intf_) {
    return display_intf_->GetDynamicDSIClock(bitclk);
  }

  return kErrorNotSupported;
}

DisplayError HWCDisplayBuiltIn::GetSupportedDSIClock(std::vector<uint64_t> *bitclk_rates) {
  if (display_intf_) {
    return display_intf_->GetSupportedDSIClock(bitclk_rates);
  }

  return kErrorNotSupported;
}

HWC2::Error HWCDisplayBuiltIn::UpdateDisplayId(hwc2_display_t id) {
  id_ = id;
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::SetPendingRefresh() {
  pending_refresh_ = true;
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::SetPanelBrightness(float brightness) {
  DisplayError ret = display_intf_->SetPanelBrightness(brightness);
  if (ret != kErrorNone) {
    return HWC2::Error::NoResources;
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::GetPanelBrightness(float *brightness) {
  DisplayError ret = display_intf_->GetPanelBrightness(brightness);
  if (ret != kErrorNone) {
    return HWC2::Error::NoResources;
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::GetPanelMaxBrightness(uint32_t *max_brightness_level) {
  DisplayError ret = display_intf_->GetPanelMaxBrightness(max_brightness_level);
  if (ret != kErrorNone) {
    return HWC2::Error::NoResources;
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::SetBLScale(uint32_t level) {
  DisplayError ret = display_intf_->SetBLScale(level);
  if (ret != kErrorNone) {
    return HWC2::Error::NoResources;
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::SetClientTarget(buffer_handle_t target,
                                               shared_ptr<Fence> acquire_fence,
                                               int32_t dataspace, hwc_region_t damage) {
  DTRACE_SCOPED();
  HWC2::Error error = HWCDisplay::SetClientTarget(target, acquire_fence, dataspace, damage);
  if (error != HWC2::Error::None) {
    return error;
  }

  // windowed_display and dynamic scaling are not supported.
  if (windowed_display_) {
    return HWC2::Error::None;
  }

  Layer *sdm_layer = client_target_->GetSDMLayer();
  uint32_t fb_width = 0, fb_height = 0;

  GetFrameBufferResolution(&fb_width, &fb_height);

  if (fb_width != sdm_layer->input_buffer.unaligned_width ||
      fb_height != sdm_layer->input_buffer.unaligned_height) {
    if (SetFrameBufferConfig(sdm_layer->input_buffer.unaligned_width,
                             sdm_layer->input_buffer.unaligned_height)) {
      return HWC2::Error::BadParameter;
    }
  }

  return HWC2::Error::None;
}

bool HWCDisplayBuiltIn::IsSmartPanelConfig(uint32_t config_id) {
  if (config_id < hwc_config_map_.size()) {
    uint32_t index = hwc_config_map_.at(config_id);
    return variable_config_map_.at(index).smart_panel;
  }

  return false;
}

bool HWCDisplayBuiltIn::HasSmartPanelConfig(void) {
  if (!enable_poms_during_doze_) {
    uint32_t config = 0;
    GetActiveDisplayConfig(&config);
    return IsSmartPanelConfig(config);
  }

  for (auto &config : variable_config_map_) {
    if (config.second.smart_panel) {
      return true;
    }
  }

  return false;
}

int HWCDisplayBuiltIn::Deinit() {
  // Destory color convert instance. This destroys thread and underlying GL resources.
  if (gl_layer_stitch_) {
    layer_stitch_task_.PerformTask(LayerStitchTaskCode::kCodeDestroyInstance, nullptr);
  }

  histogram.stop();
  return HWCDisplay::Deinit();
}

void HWCDisplayBuiltIn::OnTask(const LayerStitchTaskCode &task_code,
                               SyncTask<LayerStitchTaskCode>::TaskContext *task_context) {
  switch (task_code) {
    case LayerStitchTaskCode::kCodeGetInstance: {
        gl_layer_stitch_ = GLLayerStitch::GetInstance(false /* Non-secure */);
      }
      break;
    case LayerStitchTaskCode::kCodeStitch: {
        DTRACE_SCOPED();
        LayerStitchContext* ctx = reinterpret_cast<LayerStitchContext*>(task_context);
        gl_layer_stitch_->Blit(ctx->stitch_params, &(ctx->release_fence));
      }
      break;
    case LayerStitchTaskCode::kCodeDestroyInstance: {
        if (gl_layer_stitch_) {
          GLLayerStitch::Destroy(gl_layer_stitch_);
        }
      }
      break;
  }
}

bool HWCDisplayBuiltIn::InitLayerStitch() {
  if (!is_primary_) {
    // Disable on all non-primary builtins.
    DLOGI("Non-primary builtin.");
    disable_layer_stitch_ = true;
    return true;
  }

  // Disable by default.
  int value = 1;
  Debug::Get()->GetProperty(DISABLE_LAYER_STITCH, &value);
  disable_layer_stitch_ = (value == 1);

  if (disable_layer_stitch_) {
    DLOGI("Layer Stitch Disabled !!!");
    return true;
  }

  // Initialize stitch context. This will be non-secure.
  layer_stitch_task_.PerformTask(LayerStitchTaskCode::kCodeGetInstance, nullptr);
  if (gl_layer_stitch_ == nullptr) {
    DLOGE("Failed to get LayerStitch Instance");
    return false;
  }

  if (!AllocateStitchBuffer()) {
    return true;
  }

  stitch_target_ = new HWCLayer(id_, static_cast<HWCBufferAllocator *>(buffer_allocator_));

  // Populate buffer params and pvt handle.
  InitStitchTarget();

  DLOGI("Created LayerStitch instance: %p", gl_layer_stitch_);

  return true;
}

bool HWCDisplayBuiltIn::AllocateStitchBuffer() {
  // Buffer dimensions: FB width * (1.5 * height)
  DTRACE_SCOPED();

  DisplayError error = display_intf_->GetFrameBufferConfig(&fb_config_);
  if (error != kErrorNone) {
    DLOGE("Get frame buffer config failed. Error = %d", error);
    return false;
  }

  BufferConfig &config = buffer_info_.buffer_config;
  config.width = fb_config_.x_pixels;
  config.height = fb_config_.y_pixels * kBufferHeightFactor;

  // By default UBWC is enabled and below property is global enable/disable for all
  // buffers allocated through gralloc , including framebuffer targets.
  int ubwc_disabled = 0;
  HWCDebugHandler::Get()->GetProperty(DISABLE_UBWC_PROP, &ubwc_disabled);
  config.format = ubwc_disabled ? kFormatRGBA8888 : kFormatRGBA8888Ubwc;

  config.gfx_client = true;

  // Populate default params.
  config.secure = false;
  config.cache = false;
  config.secure_camera = false;

  int err = buffer_allocator_->AllocateBuffer(&buffer_info_);

  if (err != 0) {
    DLOGE("Failed to allocate buffer. Error: %d", error);
    return false;
  }

  return true;
}

void HWCDisplayBuiltIn::InitStitchTarget() {
  LayerBuffer buffer = {};
  buffer.planes[0].fd = buffer_info_.alloc_buffer_info.fd;
  buffer.planes[0].offset = 0;
  buffer.planes[0].stride = buffer_info_.alloc_buffer_info.stride;
  buffer.size = buffer_info_.alloc_buffer_info.size;
  buffer.handle_id = buffer_info_.alloc_buffer_info.id;
  buffer.width = buffer_info_.alloc_buffer_info.aligned_width;
  buffer.height = buffer_info_.alloc_buffer_info.aligned_height;
  buffer.unaligned_width = fb_config_.x_pixels;
  buffer.unaligned_height = fb_config_.y_pixels * kBufferHeightFactor;
  buffer.format = buffer_info_.alloc_buffer_info.format;

  Layer *sdm_stitch_target = stitch_target_->GetSDMLayer();
  sdm_stitch_target->composition = kCompositionStitchTarget;
  sdm_stitch_target->input_buffer = buffer;
  sdm_stitch_target->input_buffer.buffer_id = reinterpret_cast<uint64_t>(buffer_info_.private_data);
}

void HWCDisplayBuiltIn::AppendStitchLayer() {
  if (disable_layer_stitch_) {
    return;
  }

  // Append stitch target buffer to layer stack.
  Layer *sdm_stitch_target = stitch_target_->GetSDMLayer();
  sdm_stitch_target->composition = kCompositionStitchTarget;
  sdm_stitch_target->dst_rect = {0, 0, FLOAT(fb_config_.x_pixels), FLOAT(fb_config_.y_pixels)};
  sdm_stitch_target->layer_id = stitch_target_->GetId();
  sdm_stitch_target->geometry_changes = stitch_target_->GetGeometryChanges();
  layer_stack_.layers.push_back(sdm_stitch_target);
}

DisplayError HWCDisplayBuiltIn::HistogramEvent(int fd, uint32_t blob_id) {
  histogram.notify_histogram_event(fd, blob_id);
  return kErrorNone;
}

int HWCDisplayBuiltIn::PostInit() {
  auto status = InitLayerStitch();
  if (!status) {
    DLOGW("Failed to initialize Layer Stitch context");
    // Disable layer stitch.
    disable_layer_stitch_ = true;
  }

  return 0;
}

bool HWCDisplayBuiltIn::NeedsLargeCompPerfHint() {
  if (!cpu_hint_) {
    DLOGV_IF(kTagResources, "CPU hint is not initialized");
    return false;
  }

  if (active_refresh_rate_ < 90) {
    return false;
  }

  // Send hints when the device is in multi-display or when a skip layer is present.
  if (layer_stack_.flags.skip_present || is_multi_display_) {
    DLOGV_IF(kTagResources, "Found skip_layer:%d or is_multidisplay:%d. Set perf hint for large "
             "comp cycle", layer_stack_.flags.skip_present, is_multi_display_);
    return true;
  }

  int gpu_layer_count = 0;
  for (auto hwc_layer : layer_set_) {
    Layer *layer = hwc_layer->GetSDMLayer();
    if (layer->composition == kCompositionGPU) {
      gpu_layer_count++;
    }
  }

  // Return immediately if full MDP comp is in use
  if (!gpu_layer_count) {
    return false;
  }

  auto it = mixed_mode_threshold_.find(active_refresh_rate_);;
  if (it != mixed_mode_threshold_.end()) {
    if (gpu_layer_count < it->second) {
      DLOGV_IF(kTagResources, "Number of GPU layers :%d does not meet mixed mode perf hints "
               "threshold:%d for %d fps", gpu_layer_count, it->second, active_refresh_rate_);
      return false;
    }
  } else {
    DLOGV_IF(kTagResources, "Mixed mode perf hints is not supported for %d fps",
             active_refresh_rate_);
    return false;
  }

  // Send hints when the number of GPU layers reaches the threshold for the active refresh rate.
  DLOGV_IF(kTagResources, "Reached max GPU layers for %dfps. Set perf hint for large comp cycle",
           active_refresh_rate_);
  return true;
}

HWC2::Error HWCDisplayBuiltIn::PostCommitLayerStack(shared_ptr<Fence> *out_retire_fence) {
  DTRACE_SCOPED();
  // Block on output buffer fence if client is internal.
  // External clients will wait on their thread.
  if (layer_stack_.output_buffer != nullptr && (cwb_state_.cwb_client != kCWBClientExternal)) {
    auto &fence = layer_stack_.output_buffer->release_fence;
    display_intf_->GetOutputBufferAcquireFence(&fence);
  }

  HandleFrameOutput();
  PostCommitStitchLayers();

  {
    std::lock_guard<std::mutex> lock(cwb_state_lock_);
    if (flush_ && cwb_state_.cwb_client == kCWBClientNone) {
      ResetCwbState();
      display_intf_->FlushConcurrentWriteback();
    } else if (cwb_state_.cwb_status == CWBStatus::kCWBTeardown) {  // cwb teardown frame.
      cwb_state_.teardown_frame_retire_fence = layer_stack_.retire_fence;
      cwb_state_.cwb_disp_id = -1;
      cwb_state_.cwb_status = CWBStatus::kCWBPostTeardown;
      DLOGV_IF(kTagClient, "CWB display id = %d , cwb status = %d", cwb_state_.cwb_disp_id,
               cwb_state_.cwb_status);
    }
  }  // releasing the cwb state lock

  auto status = HWCDisplay::PostCommitLayerStack(out_retire_fence);
/*  display_intf_->GetConfig(&fixed_info);
  is_cmd_mode_ = fixed_info.is_cmdmode;

  // For video mode panel with dynamic fps, update the active mode index.
  // This is needed to report the correct Vsync period when client queries
  // using GetDisplayVsyncPeriod API.
  if (!is_cmd_mode_ && !disable_dyn_fps_) {
    hwc2_config_t active_config = hwc_config_map_.at(0);
    GetActiveConfig(&active_config);
    SetActiveConfigIndex(active_config);
  }*/

  pending_commit_ = false;

  if (layer_stack_.request_flags.trigger_refresh) {
    callbacks_->Refresh(id_);
  }

  return status;
}

bool HWCDisplayBuiltIn::IsDisplayIdle() {
  // Notify only if this display is source of vsync.
  bool vsync_source = (callbacks_->GetVsyncSource() == id_);
  return vsync_source && display_idle_;
}

bool HWCDisplayBuiltIn::HasReadBackBufferSupport() {
  DisplayConfigFixedInfo fixed_info = {};
  display_intf_->GetConfig(&fixed_info);

  return fixed_info.readback_supported;
}

HWC2::Error HWCDisplayBuiltIn::NotifyDisplayCalibrationMode(bool in_calibration) {
  auto status = color_mode_->NotifyDisplayCalibrationMode(in_calibration);
  if (status != HWC2::Error::None) {
    DLOGE("Failed for notify QDCM mode = %d", in_calibration);
    return status;
  }

  return status;
}

uint32_t HWCDisplayBuiltIn::GetUpdatingAppLayersCount() {
  uint32_t updating_count = 0;

  for (uint i = 0; i < layer_stack_.layers.size(); i++) {
    auto layer = layer_stack_.layers.at(i);
    if (layer->composition == kCompositionGPUTarget) {
      break;
    }
    if (layer->flags.updating) {
      updating_count++;
    }
  }

  return updating_count;
}

HWC2::Error HWCDisplayBuiltIn::CommitOrPrepare(bool validate_only,
                                               shared_ptr<Fence> *out_retire_fence,
                                               uint32_t *out_num_types,
                                               uint32_t *out_num_requests, bool *needs_commit) {
  DTRACE_SCOPED();

  if(!validate_only) {
    skip_commit_ = CanSkipCommit();
    if (skip_commit_) {
      *needs_commit = false;
      DLOGV_IF(kTagClient, "Skipping Refresh on display %" PRIu64 , id_);
      auto status = PostCommitLayerStack(out_retire_fence);
      return status;
    }
  }

  auto status = HWCDisplay::CommitOrPrepare(validate_only, out_retire_fence, out_num_types,
                                            out_num_requests, needs_commit);

  if (perf_hint_large_comp_cycle_) {
    bool needs_hint = NeedsLargeCompPerfHint();
    HandleLargeCompositionHint(!needs_hint);
  }

  return status;
}

void HWCDisplayBuiltIn::LoadMixedModePerfHintThreshold() {
  // For mixed mode composition, if perf hint for large composition cycles is enabled and if the
  // use case meets the threshold, SF and HWC will be running on the gold CPU cores.

  // For 120 fps, 8 layers should fall back to GPU
  mixed_mode_threshold_.insert(std::make_pair<int32_t, int32_t>(120, 8));

  // For 144 fps, 6 layers should fall back to GPU
  mixed_mode_threshold_.insert(std::make_pair<int32_t, int32_t>(144, 6));

  // TODO(user): Profile performance on 180 and 240 Hz without maxing out the CPU cores
  // For 180 fps, 8 layers should fall back to GPU
  mixed_mode_threshold_.insert(std::make_pair<int32_t, int32_t>(180, 8));

  // For 240 fps, 4 layers should fall back to GPU
  mixed_mode_threshold_.insert(std::make_pair<int32_t, int32_t>(240, 4));
}

HWC2::Error HWCDisplayBuiltIn::SetAlternateDisplayConfig(bool set) {
  hwc2_config_t alt_config = 0;
  DisplayError error = kErrorNone;

  // return early if non-DSC mode is already set
  if (set && alternate_config_ != -1) {
    return HWC2::Error::None;
  }

  if (!set && alternate_config_ == -1) {
    return HWC2::Error::None;
  }

  error = display_intf_->SetAlternateDisplayConfig(&alt_config);
  if (error != kErrorNone) {
    return HWC2::Error::Unsupported;
  }

  auto status = SetActiveConfig(alt_config);
  if (set && status == HWC2::Error::None) {
      alternate_config_ = alt_config;
  }

  if (!set) { // set alternate config to -1 on reset call
    alternate_config_ = -1;
  }

  // Trigger refresh. This config gets applied on next commit.
  callbacks_->Refresh(id_);

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::SetDimmingEnable(int int_enabled) {
  DLOGV("Display ID: %" PRId64 " enabled: %d", id_, int_enabled);
  DisplayError error = display_intf_->SetDimmingEnable(int_enabled);

  if (error != kErrorNone) {
    DLOGE("Failed. enabled = %d, error = %d", int_enabled, error);
    return HWC2::Error::BadDisplay;
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::SetDimmingMinBl(int min_bl) {
  DLOGV("Display ID: %" PRId64 " min_bl: %d", id_, min_bl);
  DisplayError error = display_intf_->SetDimmingMinBl(min_bl);

  if (error != kErrorNone) {
    DLOGE("Failed. min_bl = %d, error = %d", min_bl, error);
    return HWC2::Error::BadDisplay;
  }

  return HWC2::Error::None;
}

void HWCDisplayBuiltIn::HandleLargeCompositionHint(bool release) {
  if (!cpu_hint_) {
    return;
  }

  int tid = gettid();

  if (release) {
    if (hwc_tid_ != tid) {
      DLOGV_IF(kTagResources, "HWC's tid:%d is updated to :%d", hwc_tid_, tid);
      int ret = cpu_hint_->ReqTidChangeOffload(kHWC, tid);
      if (!ret) {
        hwc_tid_ = tid;
      }
    }

    // For long term large composition hint, release the acquired handle after a consecutive number
    // of basic frames to avoid resending hints in animation launch use cases and others.
    num_basic_frames_++;

    if (num_basic_frames_ >= active_refresh_rate_) {
      cpu_hint_->ReqHintRelease();
    }
    return;
  }

  if (hwc_tid_ != tid) {
    DLOGV_IF(kTagResources, "HWC's tid:%d is updated to :%d", hwc_tid_, tid);
    cpu_hint_->ReqHintsOffload(kPerfHintLargeCompCycle, tid);
    hwc_tid_ = tid;
  } else {
    // Sending tid as 0 indicates to Perf HAL that HWC's tid is unchanged for the current frame
    cpu_hint_->ReqHintsOffload(kPerfHintLargeCompCycle, 0);
  }

  num_basic_frames_ = 0;
}

void HWCDisplayBuiltIn::ReqPerfHintRelease() {
  if (!cpu_hint_) {
    return;
  }
  cpu_hint_->ReqHintRelease();
}

}  // namespace sdm
