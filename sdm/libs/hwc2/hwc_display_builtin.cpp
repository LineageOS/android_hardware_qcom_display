/*
* Copyright (c) 2014-2019, The Linux Foundation. All rights reserved.
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
#include "hwc_debugger.h"
#include "hwc_session.h"

#define __CLASS__ "HWCDisplayBuiltIn"

namespace sdm {

DisplayError HWCDisplayBuiltIn::PMICInterface::Init() {
  std::string str_lcd_bias("/sys/class/lcd_bias/secure_mode");
  fd_lcd_bias_ = ::open(str_lcd_bias.c_str(), O_WRONLY);
  if (fd_lcd_bias_ < 0) {
    DLOGW("File '%s' could not be opened. errno = %d, desc = %s", str_lcd_bias.c_str(), errno,
          strerror(errno));
    return kErrorHardware;
  }

  std::string str_leds_wled("/sys/class/leds/wled/secure_mode");
  fd_wled_ = ::open(str_leds_wled.c_str(), O_WRONLY);
  if (fd_wled_ < 0) {
    DLOGW("File '%s' could not be opened. errno = %d, desc = %s", str_leds_wled.c_str(), errno,
          strerror(errno));
    return kErrorHardware;
  }

  return kErrorNone;
}

void HWCDisplayBuiltIn::PMICInterface::Deinit() {
  ::close(fd_lcd_bias_);
  ::close(fd_wled_);
}

DisplayError HWCDisplayBuiltIn::PMICInterface::Notify(SecureEvent event) {
  std::string str_event = (event == kSecureDisplayStart) ? std::to_string(1) : std::to_string(0);
  ssize_t err = ::pwrite(fd_lcd_bias_, str_event.c_str(), str_event.length(), 0);
  if (err <= 0) {
    DLOGW("Write failed for lcd_bias, Error = %s", strerror(errno));
    return kErrorHardware;
  }

  err = ::pwrite(fd_wled_, str_event.c_str(), str_event.length(), 0);
  if (err <= 0) {
    DLOGW("Write failed for wled, Error = %s", strerror(errno));
    return kErrorHardware;
  }

  DLOGI("Successfully notifed about secure display %s to PMIC driver",
        (event == kSecureDisplayStart) ? "start": "end");
  return kErrorNone;
}

int HWCDisplayBuiltIn::Create(CoreInterface *core_intf, BufferAllocator *buffer_allocator,
                              HWCCallbacks *callbacks, HWCDisplayEventHandler *event_handler,
                              qService::QService *qservice, hwc2_display_t id, int32_t sdm_id,
                              HWCDisplay **hwc_display) {
  int status = 0;
  uint32_t builtin_width = 0;
  uint32_t builtin_height = 0;

  HWCDisplay *hwc_display_builtin =
      new HWCDisplayBuiltIn(core_intf, buffer_allocator, callbacks, event_handler, qservice, id,
                            sdm_id);
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

HWCDisplayBuiltIn::HWCDisplayBuiltIn(CoreInterface *core_intf, BufferAllocator *buffer_allocator,
                                     HWCCallbacks *callbacks, HWCDisplayEventHandler *event_handler,
                                     qService::QService *qservice, hwc2_display_t id,
                                     int32_t sdm_id)
    : HWCDisplay(core_intf, buffer_allocator, callbacks, event_handler, qservice, kBuiltIn, id,
                 sdm_id, true, DISPLAY_CLASS_BUILTIN),
      buffer_allocator_(buffer_allocator),
      cpu_hint_(NULL) {
}

int HWCDisplayBuiltIn::Init() {
  cpu_hint_ = new CPUHint();
  if (cpu_hint_->Init(static_cast<HWCDebugHandler *>(HWCDebugHandler::Get())) != kErrorNone) {
    delete cpu_hint_;
    cpu_hint_ = NULL;
  }

  use_metadata_refresh_rate_ = true;
  int disable_metadata_dynfps = 0;
  HWCDebugHandler::Get()->GetProperty(DISABLE_METADATA_DYNAMIC_FPS_PROP, &disable_metadata_dynfps);
  if (disable_metadata_dynfps) {
    use_metadata_refresh_rate_ = false;
  }

  int status = HWCDisplay::Init();
  if (status) {
    return status;
  }
  color_mode_ = new HWCColorMode(display_intf_);
  color_mode_->Init();
  HWCDebugHandler::Get()->GetProperty(ENABLE_DEFAULT_COLOR_MODE,
                                      &default_mode_status_);

  int optimize_refresh = 0;
  HWCDebugHandler::Get()->GetProperty(ENABLE_OPTIMIZE_REFRESH, &optimize_refresh);
  enable_optimize_refresh_ = (optimize_refresh == 1);
  if (enable_optimize_refresh_) {
    DLOGI("Drop redundant drawcycles %d", id_);
  }
  pmic_intf_ = new PMICInterface();
  pmic_intf_->Init();

  return status;
}

int HWCDisplayBuiltIn::Deinit() {
  int status = HWCDisplay::Deinit();
  if (status) {
    return status;
  }
  pmic_intf_->Deinit();
  delete pmic_intf_;

  return 0;
}

HWC2::Error HWCDisplayBuiltIn::Validate(uint32_t *out_num_types, uint32_t *out_num_requests) {
  auto status = HWC2::Error::None;
  DisplayError error = kErrorNone;

  DTRACE_SCOPED();
  if (display_paused_) {
    MarkLayersForGPUBypass();
    return status;
  }

  if (color_tranform_failed_) {
    // Must fall back to client composition
    MarkLayersForClientComposition();
  }

  // Fill in the remaining blanks in the layers and add them to the SDM layerstack
  BuildLayerStack();
  // Checks and replaces layer stack for solid fill
  SolidFillPrepare();

  // Apply current Color Mode and Render Intent.
  if (color_mode_->ApplyCurrentColorModeWithRenderIntent(
      static_cast<bool>(layer_stack_.flags.hdr_present)) != HWC2::Error::None) {
    // Fallback to GPU Composition, if Color Mode can't be applied.
    MarkLayersForClientComposition();
  }

  bool pending_output_dump = dump_frame_count_ && dump_output_to_file_;

  if (readback_buffer_queued_ || pending_output_dump) {
    CloseFd(&output_buffer_.release_fence_fd);
    // RHS values were set in FrameCaptureAsync() called from a binder thread. They are picked up
    // here in a subsequent draw round. Readback is not allowed for any secure use case.
    readback_configured_ = !layer_stack_.flags.secure_present;
    if (readback_configured_) {
      DisablePartialUpdateOneFrame();
      layer_stack_.output_buffer = &output_buffer_;
      layer_stack_.flags.post_processed_output = post_processed_output_;
    }
  }
  // Todo: relook this case
  if (layer_stack_.flags.hdr_present != hdr_present_) {
    error = display_intf_->ControlIdlePowerCollapse(!layer_stack_.flags.hdr_present, true);
    hdr_present_ = layer_stack_.flags.hdr_present;
  }

  uint32_t num_updating_layers = GetUpdatingLayersCount();
  bool one_updating_layer = (num_updating_layers == 1);
  if (num_updating_layers != 0) {
    ToggleCPUHint(one_updating_layer);
  }

  uint32_t refresh_rate = GetOptimalRefreshRate(one_updating_layer);
  error = display_intf_->SetRefreshRate(refresh_rate, force_refresh_rate_);

  // Get the refresh rate set.
  display_intf_->GetRefreshRate(&refresh_rate);
  bool vsync_source = (callbacks_->GetVsyncSource() == id_);

  if (error == kErrorNone) {
    if (vsync_source && (current_refresh_rate_ < refresh_rate)) {
      DTRACE_BEGIN("HWC2::Vsync::Enable");
      // Display is ramping up from idle.
      // Client realizes need for resync upon change in config.
      // Since we know config has changed, triggering vsync proactively
      // can help in reducing pipeline delays to enable events.
      SetVsyncEnabled(HWC2::Vsync::Enable);
      DTRACE_END();
    }
    // On success, set current refresh rate to new refresh rate.
    current_refresh_rate_ = refresh_rate;
  }

  if (layer_set_.empty()) {
    // Avoid flush for Command mode panel.
    flush_ = !client_connected_;
    validated_ = true;
    return status;
  }

  status = PrepareLayerStack(out_num_types, out_num_requests);
  pending_commit_ = true;
  return status;
}

HWC2::Error HWCDisplayBuiltIn::CommitLayerStack() {
  skip_commit_ = CanSkipCommit();
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
  bool buffers_latched = false;
  for (auto &hwc_layer : layer_set_) {
    buffers_latched |= hwc_layer->BufferLatched();
    hwc_layer->ResetBufferFlip();
  }

  bool vsync_source = (callbacks_->GetVsyncSource() == id_);
  bool skip_commit = enable_optimize_refresh_ && !pending_commit_ && !buffers_latched &&
                     !pending_refresh_ && !vsync_source;
  pending_refresh_ = false;

  return skip_commit;
}

HWC2::Error HWCDisplayBuiltIn::Present(int32_t *out_retire_fence) {
  auto status = HWC2::Error::None;

  DTRACE_SCOPED();
  ATRACE_INT("PartialUpdate", partial_update_enabled_);
  ATRACE_INT("FastPath", layer_stack_.flags.fast_path);
  ATRACE_INT("GeometryChanged", layer_stack_.flags.geometry_changed);
  ATRACE_INT("NumLayers", static_cast <int32_t> (layer_stack_.layers.size()));
  ATRACE_INT("HasClientComposition", HasClientComposition());

  if (display_paused_) {
    DisplayError error = display_intf_->Flush(&layer_stack_);
    validated_ = false;
    if (error != kErrorNone) {
      DLOGE("Flush failed. Error = %d", error);
    }
  } else {
    status = CommitLayerStack();
    if (status == HWC2::Error::None) {
      HandleFrameOutput();
      SolidFillCommit();
      status = PostCommitLayerStack(out_retire_fence);
    }
  }

  CloseFd(&output_buffer_.acquire_fence_fd);
  pending_commit_ = false;
  return status;
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
  validated_ = false;
  return status;
}

HWC2::Error HWCDisplayBuiltIn::SetColorModeById(int32_t color_mode_id) {
  auto status = color_mode_->SetColorModeById(color_mode_id);
  if (status != HWC2::Error::None) {
    DLOGE("failed for mode = %d", color_mode_id);
    return status;
  }

  callbacks_->Refresh(id_);
  validated_ = false;

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
  validated_ = false;

  return status;
}

HWC2::Error HWCDisplayBuiltIn::SetReadbackBuffer(const native_handle_t *buffer,
                                                 int32_t acquire_fence,
                                                 bool post_processed_output) {
  const private_handle_t *handle = reinterpret_cast<const private_handle_t *>(buffer);
  if (!handle || (handle->fd < 0)) {
    return HWC2::Error::BadParameter;
  }

  // Configure the output buffer as Readback buffer
  output_buffer_.width = UINT32(handle->width);
  output_buffer_.height = UINT32(handle->height);
  output_buffer_.unaligned_width = UINT32(handle->unaligned_width);
  output_buffer_.unaligned_height = UINT32(handle->unaligned_height);
  output_buffer_.format = HWCLayer::GetSDMFormat(handle->format, handle->flags);
  output_buffer_.planes[0].fd = handle->fd;
  output_buffer_.planes[0].stride = UINT32(handle->width);
  output_buffer_.acquire_fence_fd = dup(acquire_fence);
  output_buffer_.release_fence_fd = -1;
  output_buffer_.handle_id = handle->id;

  post_processed_output_ = post_processed_output;
  readback_buffer_queued_ = true;
  readback_configured_ = false;
  validated_ = false;

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::GetReadbackBufferFence(int32_t *release_fence) {
  auto status = HWC2::Error::None;

  if (readback_configured_ && (output_buffer_.release_fence_fd >= 0)) {
    *release_fence = output_buffer_.release_fence_fd;
  } else {
    status = HWC2::Error::Unsupported;
    *release_fence = -1;
  }

  post_processed_output_ = false;
  readback_buffer_queued_ = false;
  readback_configured_ = false;
  output_buffer_ = {};

  return status;
}

DisplayError HWCDisplayBuiltIn::TeardownConcurrentWriteback(void) {
  DisplayError error = kErrorNotSupported;

  if (output_buffer_.release_fence_fd >= 0) {
    int32_t release_fence_fd = dup(output_buffer_.release_fence_fd);
    int ret = sync_wait(output_buffer_.release_fence_fd, 1000);
    if (ret < 0) {
      DLOGE("sync_wait error errno = %d, desc = %s", errno, strerror(errno));
    }

    ::close(release_fence_fd);
    if (ret)
      return kErrorResources;
  }

  if (display_intf_) {
    error = display_intf_->TeardownConcurrentWriteback();
  }

  return error;
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
  validated_ = false;

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
      partial_update_enabled_ = fixed_info.partial_update;
      for (auto hwc_layer : layer_set_) {
        hwc_layer->SetPartialUpdate(partial_update_enabled_);
      }
      client_target_->SetPartialUpdate(partial_update_enabled_);
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
  use_metadata_refresh_rate_ = enable;
}

void HWCDisplayBuiltIn::SetQDCMSolidFillInfo(bool enable, const LayerSolidFill &color) {
  solid_fill_enable_ = enable;
  solid_fill_color_ = color;
}

void HWCDisplayBuiltIn::ToggleCPUHint(bool set) {
  if (!cpu_hint_) {
    return;
  }

  if (set) {
    cpu_hint_->Set();
  } else {
    cpu_hint_->Reset();
  }
}

int HWCDisplayBuiltIn::HandleSecureSession(const std::bitset<kSecureMax> &secure_sessions,
                                           bool *power_on_pending) {
  if (!power_on_pending) {
    return -EINVAL;
  }

  if (current_power_mode_ != HWC2::PowerMode::On) {
    return 0;
  }

  if (active_secure_sessions_[kSecureDisplay] != secure_sessions[kSecureDisplay]) {
    SecureEvent secure_event =
        secure_sessions.test(kSecureDisplay) ? kSecureDisplayStart : kSecureDisplayEnd;
    DisplayError err = display_intf_->HandleSecureEvent(secure_event, &layer_stack_);
    if (err != kErrorNone) {
      DLOGE("Set secure event failed");
      return err;
    }
    if (secure_event == kSecureDisplayStart) {
      pmic_intf_->Notify(kSecureDisplayStart);
    } else {
      pmic_notification_pending_ = true;
    }

    DLOGI("SecureDisplay state changed from %d to %d for display %d",
          active_secure_sessions_.test(kSecureDisplay), secure_sessions.test(kSecureDisplay),
          type_);
  }
  active_secure_sessions_ = secure_sessions;
  *power_on_pending = false;
  return 0;
}

void HWCDisplayBuiltIn::ForceRefreshRate(uint32_t refresh_rate) {
  if ((refresh_rate && (refresh_rate < min_refresh_rate_ || refresh_rate > max_refresh_rate_)) ||
      force_refresh_rate_ == refresh_rate) {
    // Cannot honor force refresh rate, as its beyond the range or new request is same
    return;
  }

  force_refresh_rate_ = refresh_rate;

  callbacks_->Refresh(id_);

  return;
}

uint32_t HWCDisplayBuiltIn::GetOptimalRefreshRate(bool one_updating_layer) {
  if (force_refresh_rate_) {
    return force_refresh_rate_;
  } else if (use_metadata_refresh_rate_ && one_updating_layer && metadata_refresh_rate_) {
    return metadata_refresh_rate_;
  }

  return max_refresh_rate_;
}

DisplayError HWCDisplayBuiltIn::Refresh() {
  DisplayError error = kErrorNone;

  callbacks_->Refresh(id_);

  return error;
}

void HWCDisplayBuiltIn::SetIdleTimeoutMs(uint32_t timeout_ms) {
  display_intf_->SetIdleTimeoutMs(timeout_ms);
  validated_ = false;
}

void HWCDisplayBuiltIn::HandleFrameOutput() {
  if (readback_buffer_queued_) {
    validated_ = false;
  }

  if (frame_capture_buffer_queued_) {
    HandleFrameCapture();
  } else if (dump_output_to_file_) {
    HandleFrameDump();
  }
}

void HWCDisplayBuiltIn::HandleFrameCapture() {
  if (readback_configured_ && (output_buffer_.release_fence_fd >= 0)) {
    frame_capture_status_ = sync_wait(output_buffer_.release_fence_fd, 1000);
    ::close(output_buffer_.release_fence_fd);
    output_buffer_.release_fence_fd = -1;
  }

  frame_capture_buffer_queued_ = false;
  readback_buffer_queued_ = false;
  post_processed_output_ = false;
  readback_configured_ = false;
  output_buffer_ = {};
}

void HWCDisplayBuiltIn::HandleFrameDump() {
  if (!readback_configured_) {
    dump_frame_count_ = 0;
  }

  if (dump_frame_count_ && output_buffer_.release_fence_fd >= 0) {
    int ret = sync_wait(output_buffer_.release_fence_fd, 1000);
    ::close(output_buffer_.release_fence_fd);
    output_buffer_.release_fence_fd = -1;
    if (ret < 0) {
      DLOGE("sync_wait error errno = %d, desc = %s", errno, strerror(errno));
    } else {
      DumpOutputBuffer(output_buffer_info_, output_buffer_base_, layer_stack_.retire_fence_fd);
      readback_buffer_queued_ = false;
      validated_ = false;
    }
  }

  if (0 == dump_frame_count_) {
    dump_output_to_file_ = false;
    // Unmap and Free buffer
    if (munmap(output_buffer_base_, output_buffer_info_.alloc_buffer_info.size) != 0) {
      DLOGE("unmap failed with err %d", errno);
    }
    if (buffer_allocator_->FreeBuffer(&output_buffer_info_) != 0) {
      DLOGE("FreeBuffer failed");
    }

    readback_buffer_queued_ = false;
    post_processed_output_ = false;
    readback_configured_ = false;

    output_buffer_ = {};
    output_buffer_info_ = {};
    output_buffer_base_ = nullptr;
  }
}

HWC2::Error HWCDisplayBuiltIn::SetFrameDumpConfig(uint32_t count, uint32_t bit_mask_layer_type,
                                                  int32_t format, bool post_processed) {
  HWCDisplay::SetFrameDumpConfig(count, bit_mask_layer_type, format, post_processed);
  dump_output_to_file_ = bit_mask_layer_type & (1 << OUTPUT_LAYER_DUMP);
  DLOGI("output_layer_dump_enable %d", dump_output_to_file_);

  if (!count || !dump_output_to_file_ || (output_buffer_info_.alloc_buffer_info.fd >= 0)) {
    return HWC2::Error::None;
  }

  // Allocate and map output buffer
  if (post_processed) {
    // To dump post-processed (DSPP) output, use Panel resolution.
    GetPanelResolution(&output_buffer_info_.buffer_config.width,
                       &output_buffer_info_.buffer_config.height);
  } else {
    // To dump Layer Mixer output, use FrameBuffer resolution.
    GetFrameBufferResolution(&output_buffer_info_.buffer_config.width,
                             &output_buffer_info_.buffer_config.height);
  }

  output_buffer_info_.buffer_config.format = HWCLayer::GetSDMFormat(format, 0);
  output_buffer_info_.buffer_config.buffer_count = 1;
  if (buffer_allocator_->AllocateBuffer(&output_buffer_info_) != 0) {
    DLOGE("Buffer allocation failed");
    output_buffer_info_ = {};
    return HWC2::Error::NoResources;
  }

  void *buffer = mmap(NULL, output_buffer_info_.alloc_buffer_info.size, PROT_READ | PROT_WRITE,
                      MAP_SHARED, output_buffer_info_.alloc_buffer_info.fd, 0);

  if (buffer == MAP_FAILED) {
    DLOGE("mmap failed with err %d", errno);
    buffer_allocator_->FreeBuffer(&output_buffer_info_);
    output_buffer_info_ = {};
    return HWC2::Error::NoResources;
  }

  output_buffer_base_ = buffer;
  const native_handle_t *handle = static_cast<native_handle_t *>(output_buffer_info_.private_data);
  SetReadbackBuffer(handle, -1, post_processed);

  return HWC2::Error::None;
}

int HWCDisplayBuiltIn::FrameCaptureAsync(const BufferInfo &output_buffer_info,
                                         bool post_processed_output) {
  // Note: This function is called in context of a binder thread and a lock is already held
  if (output_buffer_info.alloc_buffer_info.fd < 0) {
    DLOGE("Invalid fd %d", output_buffer_info.alloc_buffer_info.fd);
    return -1;
  }

  auto panel_width = 0u;
  auto panel_height = 0u;
  auto fb_width = 0u;
  auto fb_height = 0u;

  GetPanelResolution(&panel_width, &panel_height);
  GetFrameBufferResolution(&fb_width, &fb_height);

  if (post_processed_output && (output_buffer_info.buffer_config.width < panel_width ||
                                output_buffer_info.buffer_config.height < panel_height)) {
    DLOGE("Buffer dimensions should not be less than panel resolution");
    return -1;
  } else if (!post_processed_output && (output_buffer_info.buffer_config.width < fb_width ||
                                        output_buffer_info.buffer_config.height < fb_height)) {
    DLOGE("Buffer dimensions should not be less than FB resolution");
    return -1;
  }

  const native_handle_t *buffer = static_cast<native_handle_t *>(output_buffer_info.private_data);
  SetReadbackBuffer(buffer, -1, post_processed_output);
  frame_capture_buffer_queued_ = true;
  frame_capture_status_ = -EAGAIN;

  return 0;
}

DisplayError HWCDisplayBuiltIn::SetDetailEnhancerConfig
                                   (const DisplayDetailEnhancerData &de_data) {
  DisplayError error = kErrorNotSupported;

  if (display_intf_) {
    error = display_intf_->SetDetailEnhancerData(de_data);
    validated_ = false;
  }
  return error;
}

DisplayError HWCDisplayBuiltIn::ControlPartialUpdate(bool enable, uint32_t *pending) {
  DisplayError error = kErrorNone;

  if (display_intf_) {
    error = display_intf_->ControlPartialUpdate(enable, pending);
    validated_ = false;
  }

  return error;
}

DisplayError HWCDisplayBuiltIn::DisablePartialUpdateOneFrame() {
  DisplayError error = kErrorNone;

  if (display_intf_) {
    error = display_intf_->DisablePartialUpdateOneFrame();
    validated_ = false;
  }

  return error;
}


DisplayError HWCDisplayBuiltIn::SetMixerResolution(uint32_t width, uint32_t height) {
  DisplayError error = display_intf_->SetMixerResolution(width, height);
  validated_ = false;
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

  validated_ = false;
  return HWC2::Error::None;
}

DisplayError HWCDisplayBuiltIn::ControlIdlePowerCollapse(bool enable, bool synchronous) {
  DisplayError error = kErrorNone;

  if (display_intf_) {
    error = display_intf_->ControlIdlePowerCollapse(enable, synchronous);
    validated_ = false;
  }
  return error;
}

DisplayError HWCDisplayBuiltIn::SetDynamicDSIClock(uint64_t bitclk) {
  DisplayError error = display_intf_->SetDynamicDSIClock(bitclk);
  if (error != kErrorNone) {
    DLOGE(" failed: Clk: %llu Error: %d", bitclk, error);
    return error;
  }

  callbacks_->Refresh(id_);
  validated_ = false;

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

HWC2::Error HWCDisplayBuiltIn::UpdatePowerMode(HWC2::PowerMode mode) {
  current_power_mode_ = mode;
  validated_ = false;
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::PostCommitLayerStack(int32_t *out_retire_fence) {
  if (pmic_notification_pending_) {
    // Wait for current commit to complete
    if (*out_retire_fence >= 0) {
      int ret = sync_wait(*out_retire_fence, 1000);
      if (ret < 0) {
        DLOGE("sync_wait error errno = %d, desc = %s", errno, strerror(errno));
      }
    }
    pmic_intf_->Notify(kSecureDisplayEnd);
    pmic_notification_pending_ = false;
  }
  return HWCDisplay::PostCommitLayerStack(out_retire_fence);
}

}  // namespace sdm
