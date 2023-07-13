/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

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

#include "hwc_display_virtual_gpu.h"
#include "hwc_session.h"
#include "QtiGralloc.h"

#define __CLASS__ "HWCDisplayVirtualGPU"

namespace sdm {

int HWCDisplayVirtualGPU::Init() {
  // Create client target.
  client_target_ = new HWCLayer(id_, buffer_allocator_);

  // Create Null Display interface.
  DisplayError error = core_intf_->CreateNullDisplay(&display_intf_);
  if (error != kErrorNone) {
    DLOGE("Null Display create failed. Error = %d display_id = %d disp_intf = %p",
          error, sdm_id_, display_intf_);
    return -EINVAL;
  }

  disable_animation_ = Debug::IsExtAnimDisabled();

  return HWCDisplayVirtual::Init();
}

int HWCDisplayVirtualGPU::Deinit() {
  // Destory color convert instance. This destroys thread and underlying GL resources.
  if (gl_color_convert_) {
    color_convert_task_.PerformTask(ColorConvertTaskCode::kCodeDestroyInstance, nullptr);
  }

  DisplayError error = core_intf_->DestroyNullDisplay(display_intf_);
  if (error != kErrorNone) {
    DLOGE("Null Display destroy failed. Error = %d", error);
    return -EINVAL;
  }

  delete client_target_;

  for (auto hwc_layer : layer_set_) {
    delete hwc_layer;
  }

  return 0;
}

HWCDisplayVirtualGPU::HWCDisplayVirtualGPU(CoreInterface *core_intf, HWCBufferAllocator
                                           *buffer_allocator, HWCCallbacks *callbacks,
                                           HWCDisplayEventHandler *event_handler,
                                           hwc2_display_t id, int32_t sdm_id, uint32_t width,
                                           uint32_t height, float min_lum, float max_lum) :
  HWCDisplayVirtual(core_intf, buffer_allocator, callbacks, event_handler, id, sdm_id,
                    width, height),
  color_convert_task_(*this) {
}

HWC2::Error HWCDisplayVirtualGPU::Validate(uint32_t *out_num_types, uint32_t *out_num_requests) {
  DTRACE_SCOPED();

  // Reset previous changes.
  layer_changes_.clear();
  layer_requests_.clear();

  // Mark all layers to GPU if there is no need to bypass.
  bool needs_gpu_bypass = NeedsGPUBypass() || FreezeScreen();
  for (auto hwc_layer : layer_set_) {
    auto layer = hwc_layer->GetSDMLayer();
    layer->composition = needs_gpu_bypass ? kCompositionSDE : kCompositionGPU;

    if (needs_gpu_bypass) {
      if (hwc_layer->GetClientRequestedCompositionType() == HWC2::Composition::Client) {
       layer_changes_[hwc_layer->GetId()] = HWC2::Composition::Device;
       layer_requests_[hwc_layer->GetId()] = HWC2::LayerRequest::ClearClientTarget;
      }
    } else {
      if (hwc_layer->GetClientRequestedCompositionType() != HWC2::Composition::Client) {
       layer_changes_[hwc_layer->GetId()] = HWC2::Composition::Client;
      }
    }
  }

  // Derive client target dataspace based on the color mode - bug/115482728
  int32_t client_target_dataspace = GetDataspaceFromColorMode(GetCurrentColorMode());
  SetClientTargetDataSpace(client_target_dataspace);

  *out_num_types = UINT32(layer_changes_.size());
  *out_num_requests = UINT32(layer_requests_.size());;
  has_client_composition_ = !needs_gpu_bypass;
  validate_done_ = true;

  return ((*out_num_types > 0) ? HWC2::Error::HasChanges : HWC2::Error::None);
}

HWC2::Error HWCDisplayVirtualGPU::CommitOrPrepare(bool validate_only,
                                                  shared_ptr<Fence> *out_retire_fence,
                                                  uint32_t *out_num_types,
                                                  uint32_t *out_num_requests, bool *needs_commit) {
  // Perform validate and commit.
  auto status = Validate(out_num_types, out_num_requests);

  *needs_commit = true;
  return status;
}

HWC2::Error HWCDisplayVirtualGPU::SetOutputBuffer(buffer_handle_t buf,
                                                  shared_ptr<Fence> release_fence) {
  HWC2::Error error = HWCDisplayVirtual::SetOutputBuffer(buf, release_fence);
  if (error != HWC2::Error::None) {
    return error;
  }

  native_handle_t *hnd = const_cast<native_handle_t *>(buf);
  buffer_allocator_->GetWidth(hnd, output_buffer_.width);
  buffer_allocator_->GetHeight(hnd, output_buffer_.height);
  buffer_allocator_->GetUnalignedWidth(hnd, output_buffer_.unaligned_width);
  buffer_allocator_->GetUnalignedHeight(hnd, output_buffer_.unaligned_height);

  // Update active dimensions.
  if (qtigralloc::getMetadataState(hnd, android::gralloc4::MetadataType_Crop.value)) {
    int32_t slice_width = 0, slice_height = 0;
    if (!buffer_allocator_->GetBufferGeometry(hnd, slice_width, slice_height)) {
      output_buffer_.unaligned_width = slice_width;
      output_buffer_.unaligned_height = slice_height;
      // Update buffer width and height.
      int new_aligned_w = 0;
      int new_aligned_h = 0;
      int output_handle_format = 0;;
      buffer_allocator_->GetFormat(hnd, output_handle_format);
      buffer_allocator_->GetAlignedWidthAndHeight(INT(slice_width), INT(slice_height),
                                                  output_handle_format, 0, &new_aligned_w,
                                                  &new_aligned_h);
      output_buffer_.width = UINT32(new_aligned_w);
      output_buffer_.height = UINT32(new_aligned_h);
      color_convert_task_.PerformTask(ColorConvertTaskCode::kCodeReset, nullptr);
    }
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtualGPU::Present(shared_ptr<Fence> *out_retire_fence) {
  DTRACE_SCOPED();

  auto status = HWC2::Error::None;

  if (!output_buffer_.buffer_id) {
    return HWC2::Error::NoResources;
  }

  if (NeedsGPUBypass()) {
    return status;
  }

  layer_stack_.output_buffer = &output_buffer_;

  // Ensure that blit is initialized.
  // GPU context gets in secure or non-secure mode depending on output buffer provided.
  if (!gl_color_convert_) {
    // Get instance.
    color_convert_task_.PerformTask(ColorConvertTaskCode::kCodeGetInstance, nullptr);
    if (gl_color_convert_ == nullptr) {
      DLOGE("Failed to get Color Convert Instance");
      return HWC2::Error::NoResources;
    } else {
      DLOGI("Created ColorConvert instance: %p", gl_color_convert_);
    }
  }

  ColorConvertBlitContext ctx = {};

  Layer *sdm_layer = client_target_->GetSDMLayer();
  LayerBuffer &input_buffer = sdm_layer->input_buffer;
  ctx.src_hnd = reinterpret_cast<const native_handle_t *>(input_buffer.buffer_id);
  ctx.dst_hnd = reinterpret_cast<const native_handle_t *>(output_handle_);
  ctx.dst_rect = {0, 0, FLOAT(output_buffer_.unaligned_width),
                  FLOAT(output_buffer_.unaligned_height)};
  ctx.src_acquire_fence = input_buffer.acquire_fence;
  ctx.dst_acquire_fence = output_buffer_.acquire_fence;

  color_convert_task_.PerformTask(ColorConvertTaskCode::kCodeBlit, &ctx);

  // todo blit
  DumpVDSBuffer();

  *out_retire_fence = ctx.release_fence;

  return status;
}

void HWCDisplayVirtualGPU::OnTask(const ColorConvertTaskCode &task_code,
                                  SyncTask<ColorConvertTaskCode>::TaskContext *task_context) {
  switch (task_code) {
    case ColorConvertTaskCode::kCodeGetInstance: {
        gl_color_convert_ = GLColorConvert::GetInstance(kTargetYUV, output_buffer_.flags.secure);
      }
      break;
    case ColorConvertTaskCode::kCodeBlit: {
        DTRACE_SCOPED();
        ColorConvertBlitContext* ctx = reinterpret_cast<ColorConvertBlitContext*>(task_context);
        gl_color_convert_->Blit(ctx->src_hnd, ctx->dst_hnd, ctx->src_rect, ctx->dst_rect,
                                ctx->src_acquire_fence, ctx->dst_acquire_fence,
                                &(ctx->release_fence));
      }
      break;
    case ColorConvertTaskCode::kCodeReset: {
        DTRACE_SCOPED();
        if (gl_color_convert_) {
          gl_color_convert_->Reset();
        }
      }
      break;
    case ColorConvertTaskCode::kCodeDestroyInstance: {
        if (gl_color_convert_) {
          GLColorConvert::Destroy(gl_color_convert_);
        }
      }
      break;
  }
}

bool HWCDisplayVirtualGPU::FreezeScreen() {
  if (!disable_animation_) {
    return false;
  }

  bool freeze_screen = false;
  if (animating_ && !animation_in_progress_) {
    // Start of animation. GPU comp is needed.
    animation_in_progress_ = true;
  } else if (!animating_ && animation_in_progress_) {
    // End of animation. Start composing.
    animation_in_progress_ = false;
  } else if (animating_ && animation_in_progress_) {
    // Animation in progress...
    freeze_screen = true;
  }

  return freeze_screen;
}

}  // namespace sdm
