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
#include <utils/constants.h>
#include <utils/debug.h>
#include <algorithm>

#include "hwc_display_pluggable.h"
#include "hwc_debugger.h"

#define __CLASS__ "HWCDisplayPluggable"

namespace sdm {

int HWCDisplayPluggable::Create(CoreInterface *core_intf, HWCBufferAllocator *buffer_allocator,
                               HWCCallbacks *callbacks, HWCDisplayEventHandler *event_handler,
                               qService::QService *qservice, hwc2_display_t id, int32_t sdm_id,
                               uint32_t primary_width, uint32_t primary_height,
                               bool use_primary_res, HWCDisplay **hwc_display) {
  uint32_t pluggable_width = 0;
  uint32_t pluggable_height = 0;
  DisplayError error = kErrorNone;

  HWCDisplay *hwc_display_pluggable = new HWCDisplayPluggable(core_intf, buffer_allocator,
                                                  callbacks, event_handler, qservice, id, sdm_id);
  int status = hwc_display_pluggable->Init();
  if (status) {
    delete hwc_display_pluggable;
    return status;
  }

  error = hwc_display_pluggable->GetMixerResolution(&pluggable_width, &pluggable_height);
  if (error != kErrorNone) {
    Destroy(hwc_display_pluggable);
    return -EINVAL;
  }

  if (primary_width && primary_height) {
    // use_primary_res means HWCDisplayPluggable should directly set framebuffer resolution to the
    // provided primary_width and primary_height
    if (use_primary_res) {
      pluggable_width = primary_width;
      pluggable_height = primary_height;
    } else {
      int downscale_enabled = 0;
      HWCDebugHandler::Get()->GetProperty(ENABLE_EXTERNAL_DOWNSCALE_PROP, &downscale_enabled);
      if (downscale_enabled) {
        GetDownscaleResolution(primary_width, primary_height, &pluggable_width, &pluggable_height);
      }
    }
  }

  status = hwc_display_pluggable->SetFrameBufferResolution(pluggable_width, pluggable_height);
  if (status) {
    Destroy(hwc_display_pluggable);
    return status;
  }

  *hwc_display = hwc_display_pluggable;

  return status;
}

int HWCDisplayPluggable::Init() {
  int status = HWCDisplay::Init();
  if (status) {
    return status;
  }
  color_mode_ = new HWCColorMode(display_intf_);
  color_mode_->Init();

  return status;
}

void HWCDisplayPluggable::Destroy(HWCDisplay *hwc_display) {
  // Flush the display to have outstanding fences signaled.
  hwc_display->Flush();
  hwc_display->Deinit();
  delete hwc_display;
}

HWCDisplayPluggable::HWCDisplayPluggable(CoreInterface *core_intf,
                                         HWCBufferAllocator *buffer_allocator,
                                         HWCCallbacks *callbacks,
                                         HWCDisplayEventHandler *event_handler,
                                         qService::QService *qservice,
                                         hwc2_display_t id,
                                         int32_t sdm_id)
    : HWCDisplay(core_intf, buffer_allocator, callbacks, event_handler, qservice, kPluggable, id,
                 sdm_id, DISPLAY_CLASS_PLUGGABLE) {
}

HWC2::Error HWCDisplayPluggable::PreValidateDisplay(bool *exit_validate) {
  DTRACE_SCOPED();

  // Draw method gets set as part of first commit.
  SetDrawMethod();

  auto status = HWC2::Error::None;
  bool res_exhausted = false;
  // If no resources are available for the current display, mark it for GPU by pass and continue to
  // do invalidate until the resources are available
  if (active_secure_sessions_[kSecureDisplay] || display_paused_ ||
     (mmrm_restricted_ && (current_power_mode_ == HWC2::PowerMode::Off ||
     current_power_mode_ == HWC2::PowerMode::DozeSuspend)) || CheckResourceState(&res_exhausted)) {
    MarkLayersForGPUBypass();
    *exit_validate = true;
    bypass_drawcycle_ = true;
    return status;
  }

  BuildLayerStack();

  if (layer_set_.empty()) {
    flush_ = !client_connected_;
    *exit_validate = true;
    return status;
  }

  // Apply current Color Mode and Render Intent.
  status = color_mode_->ApplyCurrentColorModeWithRenderIntent(
                                                 static_cast<bool>(layer_stack_.flags.hdr_present));
  if (status != HWC2::Error::None || has_color_tranform_) {
    // Fallback to GPU Composition if Color Mode can't be applied or if a color tranform needs to be
    // applied.
    MarkLayersForClientComposition();
  }

  SetCwbState();
  *exit_validate = false;

  return status;
}

HWC2::Error HWCDisplayPluggable::Validate(uint32_t *out_num_types, uint32_t *out_num_requests) {
  bool exit_validate = false;
  auto status = PreValidateDisplay(&exit_validate);
  if (exit_validate) {
    return status;
  }

  // TODO(user): SetRefreshRate need to follow new interface when added.

  return PrepareLayerStack(out_num_types, out_num_requests);
}

HWC2::Error HWCDisplayPluggable::PostCommitLayerStack(shared_ptr<Fence> *out_retire_fence) {
  DTRACE_SCOPED();
  auto status = HWC2::Error::None;

  // Block on output buffer fence if client is internal.
  // External clients will wait on their thread.
  if (layer_stack_.output_buffer != nullptr && (cwb_state_.cwb_client != kCWBClientExternal)) {
    auto &fence = layer_stack_.output_buffer->release_fence;
    display_intf_->GetOutputBufferAcquireFence(&fence);
  }

  HandleFrameOutput();
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
  status = HWCDisplay::PostCommitLayerStack(out_retire_fence);

  return status;
}

HWC2::Error HWCDisplayPluggable::Present(shared_ptr<Fence> *out_retire_fence) {
  auto status = HWC2::Error::None;
  bool res_exhausted = false;

  if (bypass_drawcycle_) {
    bypass_drawcycle_ = false;
    return status;
  }

  if (!active_secure_sessions_[kSecureDisplay] && !display_paused_ &&
     !(mmrm_restricted_ && (current_power_mode_ == HWC2::PowerMode::Off ||
     current_power_mode_ == HWC2::PowerMode::DozeSuspend))) {
    // Proceed only if any resources are available to be allocated for the current display,
    // Otherwise keep doing invalidate
    if (CheckResourceState(&res_exhausted)) {
      Refresh();
      return status;
    }

    status = HWCDisplay::CommitLayerStack();
    if (status == HWC2::Error::None) {
      status = PostCommitLayerStack(out_retire_fence);
    }
  }
  return status;
}

void HWCDisplayPluggable::ApplyScanAdjustment(hwc_rect_t *display_frame) {
  if ((underscan_width_ <= 0) || (underscan_height_ <= 0)) {
    return;
  }

  float width_ratio = FLOAT(underscan_width_) / 100.0f;
  float height_ratio = FLOAT(underscan_height_) / 100.0f;

  uint32_t mixer_width = 0;
  uint32_t mixer_height = 0;
  GetMixerResolution(&mixer_width, &mixer_height);

  if (mixer_width == 0 || mixer_height == 0) {
    DLOGV("Invalid mixer dimensions (%d, %d)", mixer_width, mixer_height);
    return;
  }

  uint32_t new_mixer_width = UINT32(mixer_width * FLOAT(1.0f - width_ratio));
  uint32_t new_mixer_height = UINT32(mixer_height * FLOAT(1.0f - height_ratio));

  int x_offset = INT((FLOAT(mixer_width) * width_ratio) / 2.0f);
  int y_offset = INT((FLOAT(mixer_height) * height_ratio) / 2.0f);

  display_frame->left = (display_frame->left * INT32(new_mixer_width) / INT32(mixer_width))
                        + x_offset;
  display_frame->top = (display_frame->top * INT32(new_mixer_height) / INT32(mixer_height)) +
                       y_offset;
  display_frame->right = ((display_frame->right * INT32(new_mixer_width)) / INT32(mixer_width)) +
                         x_offset;
  display_frame->bottom = ((display_frame->bottom * INT32(new_mixer_height)) / INT32(mixer_height))
                          + y_offset;
}

static void AdjustSourceResolution(uint32_t dst_width, uint32_t dst_height, uint32_t *src_width,
                                   uint32_t *src_height) {
  *src_height = (dst_width * (*src_height)) / (*src_width);
  *src_width = dst_width;
}

void HWCDisplayPluggable::GetDownscaleResolution(uint32_t primary_width, uint32_t primary_height,
                                                uint32_t *non_primary_width,
                                                uint32_t *non_primary_height) {
  uint32_t primary_area = primary_width * primary_height;
  uint32_t non_primary_area = (*non_primary_width) * (*non_primary_height);

  if (primary_area > non_primary_area) {
    if (primary_height > primary_width) {
      std::swap(primary_height, primary_width);
    }
    AdjustSourceResolution(primary_width, primary_height, non_primary_width, non_primary_height);
  }
}

void HWCDisplayPluggable::GetUnderScanConfig() {
  if (!display_intf_->IsUnderscanSupported()) {
    // Read user defined underscan width and height
    HWCDebugHandler::Get()->GetProperty(EXTERNAL_ACTION_SAFE_WIDTH_PROP, &underscan_width_);
    HWCDebugHandler::Get()->GetProperty(EXTERNAL_ACTION_SAFE_HEIGHT_PROP, &underscan_height_);
  }
}

DisplayError HWCDisplayPluggable::Flush() {
  return display_intf_->Flush(&layer_stack_);
}

HWC2::Error HWCDisplayPluggable::GetColorModes(uint32_t *out_num_modes, ColorMode *out_modes) {
  if (out_modes == nullptr) {
    *out_num_modes = color_mode_->GetColorModeCount();
  } else {
    color_mode_->GetColorModes(out_num_modes, out_modes);
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayPluggable::GetRenderIntents(ColorMode mode, uint32_t *out_num_intents,
                                                 RenderIntent *out_intents) {
  if (out_intents == nullptr) {
    *out_num_intents = color_mode_->GetRenderIntentCount(mode);
  } else {
    color_mode_->GetRenderIntents(mode, out_num_intents, out_intents);
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayPluggable::SetColorMode(ColorMode mode) {
  return SetColorModeWithRenderIntent(mode, RenderIntent::COLORIMETRIC);
}

HWC2::Error HWCDisplayPluggable::SetColorModeWithRenderIntent(ColorMode mode, RenderIntent intent) {
  auto status = color_mode_->CacheColorModeWithRenderIntent(mode, intent);
  if (status != HWC2::Error::None) {
    DLOGE("failed for mode = %d intent = %d", mode, intent);
    return status;
  }

  callbacks_->Refresh(id_);

  return status;
}

HWC2::Error HWCDisplayPluggable::SetColorTransform(const float *matrix,
                                                   android_color_transform_t hint) {
  if (HAL_COLOR_TRANSFORM_IDENTITY == hint) {
    has_color_tranform_ = false;
    // From 2.1 IComposerClient.hal:
    // If the device is not capable of either using the hint or the matrix to apply the desired
    // color transform, it must force all layers to client composition during VALIDATE_DISPLAY.
  } else {
    // Also, interpret HAL_COLOR_TRANSFORM_ARBITRARY_MATRIX hint as non-identity matrix.
    has_color_tranform_ = true;
  }

  geometry_changes_ |= GeometryChanges::kColorTransform;
  callbacks_->Refresh(id_);

  return HWC2::Error::None;
}

}  // namespace sdm
