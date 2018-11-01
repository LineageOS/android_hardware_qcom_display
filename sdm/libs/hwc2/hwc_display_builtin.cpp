/*
* Copyright (c) 2014-2020, The Linux Foundation. All rights reserved.
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

int HWCDisplayBuiltIn::Create(CoreInterface *core_intf, BufferAllocator *buffer_allocator,
                              HWCCallbacks *callbacks, qService::QService *qservice,
                              hwc2_display_t id, int32_t sdm_id, bool is_primary,
                              HWCDisplay **hwc_display) {
  int status = 0;
  uint32_t builtin_width = 0;
  uint32_t builtin_height = 0;

  HWCDisplay *hwc_display_builtin = new HWCDisplayBuiltIn(core_intf, buffer_allocator, callbacks,
                                                          qservice, id, sdm_id, is_primary);
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
                                     HWCCallbacks *callbacks, qService::QService *qservice,
                                     hwc2_display_t id, int32_t sdm_id, bool is_primary)
    : HWCDisplay(core_intf, buffer_allocator, callbacks, qservice, kBuiltIn, id, sdm_id, true,
                 DISPLAY_CLASS_BUILTIN),
      buffer_allocator_(buffer_allocator),
      cpu_hint_(NULL),
      is_primary_(is_primary) {
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
  HWCDebugHandler::Get()->GetProperty(ENABLE_DEFAULT_COLOR_MODE, &default_mode_status_);

  if (is_primary_) {
    Debug::GetWindowRect(&window_rect_.left, &window_rect_.top,
                         &window_rect_.right, &window_rect_.bottom);
    DLOGI("Window rect :[%f %f %f %f]", window_rect_.left, window_rect_.top,
           window_rect_.right, window_rect_.bottom);
  }

  int drop_refresh = 0;
  HWCDebugHandler::Get()->GetProperty(ENABLE_DROP_REFRESH, &drop_refresh);
  enable_drop_refresh_ = (drop_refresh == 1);
  if (enable_drop_refresh_) {
    DLOGI("Drop redundant drawcycles %d", id_);
  }

  return status;
}

void HWCDisplayBuiltIn::ProcessBootAnimCompleted() {
  bool bootanim_exit = false;

  /* All other checks namely "init.svc.bootanim" or
  * HWC_GEOMETRY_CHANGED fail in correctly identifying the
  * exact bootup transition to homescreen
  */
  char property[PROPERTY_VALUE_MAX];
  bool isEncrypted = false;
  bool main_class_services_started = false;
  property_get("ro.crypto.state", property, "unencrypted");
  if (!strcmp(property, "encrypted")) {
    property_get("ro.crypto.type", property, "block");
    if (!strcmp(property, "block")) {
      isEncrypted = true;
      property_get("vold.decrypt", property, "");
      if (!strcmp(property, "trigger_restart_framework")) {
        main_class_services_started = true;
      }
    }
  }

  property_get("service.bootanim.exit", property, "0");
  if (!strcmp(property, "1")) {
    bootanim_exit = true;
  }

  if ((!isEncrypted || (isEncrypted && main_class_services_started)) &&
      bootanim_exit) {
    DLOGI("Applying default mode");
    boot_animation_completed_ = true;
    // Applying default mode after bootanimation is finished And
    // If Data is Encrypted, it is ready for access.
    if (display_intf_) {
      display_intf_->ApplyDefaultDisplayMode();
      RestoreColorTransform();
    }
  }
}

HWC2::Error HWCDisplayBuiltIn::Validate(uint32_t *out_num_types, uint32_t *out_num_requests) {
  auto status = HWC2::Error::None;
  DisplayError error = kErrorNone;

  if (default_mode_status_ && !boot_animation_completed_) {
    ProcessBootAnimCompleted();
  }

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

  bool pending_output_dump = dump_frame_count_ && dump_output_to_file_;

  if (frame_capture_buffer_queued_ || pending_output_dump) {
    // RHS values were set in FrameCaptureAsync() called from a binder thread. They are picked up
    // here in a subsequent draw round.
    layer_stack_.output_buffer = &output_buffer_;
    layer_stack_.flags.post_processed_output = post_processed_output_;
  }

  uint32_t num_updating_layers = GetUpdatingLayersCount();
  bool one_updating_layer = (num_updating_layers == 1);
  if (num_updating_layers != 0) {
    ToggleCPUHint(one_updating_layer);
  }

  uint32_t refresh_rate = GetOptimalRefreshRate(one_updating_layer);
  bool final_rate = force_refresh_rate_ ? true : false;
  error = display_intf_->SetRefreshRate(refresh_rate, final_rate);
  if (error == kErrorNone) {
    // On success, set current refresh rate to new refresh rate
    current_refresh_rate_ = refresh_rate;
  }

  if (layer_set_.empty()) {
    // Avoid flush for Command mode panel.
    DisplayConfigFixedInfo display_config;
    display_intf_->GetConfig(&display_config);
    flush_ = !(display_config.is_cmdmode && secure_display_active_);
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
  bool skip_commit = enable_drop_refresh_ && !pending_commit_ && !buffers_latched &&
                     !pending_refresh_ && !vsync_source;
  pending_refresh_ = false;

  return skip_commit;
}

HWC2::Error HWCDisplayBuiltIn::Present(int32_t *out_retire_fence) {
  auto status = HWC2::Error::None;
  if (display_paused_) {
    // TODO(user): From old HWC implementation
    // If we do not handle the frame set retireFenceFd to outbufAcquireFenceFd
    // Revisit this when validating display_paused
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
      status = HWCDisplay::PostCommitLayerStack(out_retire_fence);
    }
  }
  pending_commit_ = false;
  return status;
}

HWC2::Error HWCDisplayBuiltIn::GetColorModes(uint32_t *out_num_modes,
                                             android_color_mode_t *out_modes) {
  if (out_modes == nullptr) {
    *out_num_modes = color_mode_->GetColorModeCount();
  } else {
    color_mode_->GetColorModes(out_num_modes, out_modes);
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayBuiltIn::SetColorMode(android_color_mode_t mode) {
  auto status = color_mode_->SetColorMode(mode);
  if (status != HWC2::Error::None) {
    DLOGE("failed for mode = %d", mode);
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

void HWCDisplayBuiltIn::SetSecureDisplay(bool secure_display_active) {
  if (secure_display_active_ != secure_display_active) {
    // Skip Prepare and call Flush for null commit
    DLOGI("SecureDisplay state changed from %d to %d Needs Flush!!", secure_display_active_,
          secure_display_active);
    secure_display_active_ = secure_display_active;

    // Avoid flush for Command mode panel.
    DisplayConfigFixedInfo display_config;
    display_intf_->GetConfig(&display_config);
    skip_prepare_ = !display_config.is_cmdmode;
  }
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
  if (pending_commit_) {
    return kErrorNone;
  }
  callbacks_->Refresh(id_);
  return kErrorNone;
}

void HWCDisplayBuiltIn::SetIdleTimeoutMs(uint32_t timeout_ms) {
  display_intf_->SetIdleTimeoutMs(timeout_ms);
  validated_ = false;
}

static void SetLayerBuffer(const BufferInfo &output_buffer_info, LayerBuffer *output_buffer) {
  const BufferConfig& buffer_config = output_buffer_info.buffer_config;
  const AllocatedBufferInfo &alloc_buffer_info = output_buffer_info.alloc_buffer_info;

  output_buffer->width = alloc_buffer_info.aligned_width;
  output_buffer->height = alloc_buffer_info.aligned_height;
  output_buffer->unaligned_width = buffer_config.width;
  output_buffer->unaligned_height = buffer_config.height;
  output_buffer->format = buffer_config.format;
  output_buffer->planes[0].fd = alloc_buffer_info.fd;
  output_buffer->planes[0].stride = alloc_buffer_info.stride;
}

void HWCDisplayBuiltIn::HandleFrameOutput() {
  if (frame_capture_buffer_queued_) {
    HandleFrameCapture();
  } else if (dump_output_to_file_) {
    HandleFrameDump();
  }
}

void HWCDisplayBuiltIn::HandleFrameCapture() {
  if (output_buffer_.release_fence_fd >= 0) {
    frame_capture_status_ = sync_wait(output_buffer_.release_fence_fd, 1000);
    ::close(output_buffer_.release_fence_fd);
    output_buffer_.release_fence_fd = -1;
  }

  frame_capture_buffer_queued_ = false;
  post_processed_output_ = false;
  output_buffer_ = {};
}

void HWCDisplayBuiltIn::HandleFrameDump() {
  if (dump_frame_count_ && output_buffer_.release_fence_fd >= 0) {
    int ret = sync_wait(output_buffer_.release_fence_fd, 1000);
    ::close(output_buffer_.release_fence_fd);
    output_buffer_.release_fence_fd = -1;
    if (ret < 0) {
      DLOGE("sync_wait error errno = %d, desc = %s", errno, strerror(errno));
    } else {
      DumpOutputBuffer(output_buffer_info_, output_buffer_base_, layer_stack_.retire_fence_fd);
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

    post_processed_output_ = false;
    output_buffer_ = {};
    output_buffer_info_ = {};
    output_buffer_base_ = nullptr;
  }
}

HWC2::Error HWCDisplayBuiltIn::SetFrameDumpConfig(uint32_t count, uint32_t bit_mask_layer_type) {
  HWCDisplay::SetFrameDumpConfig(count, bit_mask_layer_type);
  dump_output_to_file_ = bit_mask_layer_type & (1 << OUTPUT_LAYER_DUMP);
  DLOGI("output_layer_dump_enable %d", dump_output_to_file_);

  if (!count || !dump_output_to_file_) {
    return HWC2::Error::None;
  }

  // Allocate and map output buffer
  output_buffer_info_ = {};
  // Since we dump DSPP output use Panel resolution.
  GetPanelResolution(&output_buffer_info_.buffer_config.width,
                     &output_buffer_info_.buffer_config.height);
  output_buffer_info_.buffer_config.format = kFormatRGB888;
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
  post_processed_output_ = true;
  DisablePartialUpdateOneFrame();
  validated_ = false;
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

  SetLayerBuffer(output_buffer_info, &output_buffer_);
  post_processed_output_ = post_processed_output;
  frame_capture_buffer_queued_ = true;
  // Status is only cleared on a new call to dump and remains valid otherwise
  frame_capture_status_ = -EAGAIN;
  DisablePartialUpdateOneFrame();

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

DisplayError HWCDisplayBuiltIn::SetDynamicDSIClock(uint64_t bitclk) {
  {
    SEQUENCE_WAIT_SCOPE_LOCK(HWCSession::locker_[type_]);
      DisplayError error = display_intf_->SetDynamicDSIClock(bitclk);
      if (error != kErrorNone) {
        DLOGE(" failed: Clk: %llu Error: %d", bitclk, error);
        return error;
      }
  }

  callbacks_->Refresh(id_);
  return kErrorNone;
}

DisplayError HWCDisplayBuiltIn::GetDynamicDSIClock(uint64_t *bitclk) {
  SEQUENCE_WAIT_SCOPE_LOCK(HWCSession::locker_[type_]);
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

}  // namespace sdm
