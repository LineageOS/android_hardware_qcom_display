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

/* Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <utils/constants.h>
#include <utils/debug.h>
#include <sync/sync.h>
#include <stdarg.h>
#include <QtiGralloc.h>

#include "hwc_display_virtual.h"
#include "hwc_debugger.h"

#define __CLASS__ "HWCDisplayVirtual"

namespace sdm {

void HWCDisplayVirtual::Destroy(HWCDisplay *hwc_display) {
  hwc_display->Deinit();
  delete hwc_display;
}

HWCDisplayVirtual::HWCDisplayVirtual(CoreInterface *core_intf, HWCBufferAllocator *buffer_allocator,
                                     HWCCallbacks *callbacks, hwc2_display_t id, int32_t sdm_id,
                                     uint32_t width, uint32_t height) :
      HWCDisplay(core_intf, buffer_allocator, callbacks, nullptr, nullptr, kVirtual, id, sdm_id,
                 DISPLAY_CLASS_VIRTUAL), width_(width), height_(height)  {
}

int HWCDisplayVirtual::Init() {
  flush_on_error_ = true;
  return 0;
}

int HWCDisplayVirtual::Deinit() {
  return HWCDisplay::Deinit();
}

bool HWCDisplayVirtual::NeedsGPUBypass() {
  return display_paused_ || active_secure_sessions_.any() || layer_set_.empty();
}

HWC2::Error HWCDisplayVirtual::Present(shared_ptr<Fence> *out_retire_fence) {
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtual::PreValidateDisplay(bool *exit_validate) {
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtual::CommitOrPrepare(bool validate_only,
                                               shared_ptr<Fence> *out_retire_fence,
                                               uint32_t *out_num_types,
                                               uint32_t *out_num_requests, bool *needs_commit) {
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtual::DumpVDSBuffer() {
  if (dump_frame_count_ && !flush_ && dump_output_layer_) {
    if (output_handle_) {
      BufferInfo buffer_info;
      const native_handle_t *output_handle =
          reinterpret_cast<const native_handle_t *>(output_buffer_->buffer_id);
      void *base_ptr = NULL;
      int error = buffer_allocator_->MapBuffer(output_handle, nullptr, &base_ptr);
      if (error != 0) {
        DLOGE("Failed to map output buffer, error = %d", error);
        return HWC2::Error::BadParameter;
      }
      uint32_t width, height, alloc_size = 0;
      int32_t format, flags = 0;
      buffer_allocator_->GetWidth((void *)output_handle, width);
      buffer_allocator_->GetHeight((void *)output_handle, height);
      buffer_allocator_->GetFormat((void *)output_handle, format);
      buffer_allocator_->GetPrivateFlags((void *)output_handle, flags);
      buffer_allocator_->GetAllocationSize((void *)output_handle, alloc_size);

      buffer_info.buffer_config.width = width;
      buffer_info.buffer_config.height = height;
      buffer_info.buffer_config.format = HWCLayer::GetSDMFormat(format, flags);
      buffer_info.alloc_buffer_info.size = alloc_size;
      DumpOutputBuffer(buffer_info, base_ptr, layer_stack_.retire_fence);

      int release_fence = -1;
      error = buffer_allocator_->UnmapBuffer(output_handle, &release_fence);
      if (error != 0) {
        DLOGE("Failed to unmap buffer, error = %d", error);
        return HWC2::Error::BadParameter;
      }
    }
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtual::SetOutputBuffer(buffer_handle_t buf,
                                               shared_ptr<Fence> release_fence) {
  if (buf == nullptr) {
    return HWC2::Error::BadParameter;
  }
  const native_handle_t *output_handle = static_cast<const native_handle_t *>(buf);

  if (output_handle) {
    int output_handle_format, output_handle_flags = 0;
    buffer_allocator_->GetPrivateFlags((void *)output_handle, output_handle_flags);
    buffer_allocator_->GetFormat((void *)output_handle, output_handle_format);
    ColorMetaData color_metadata = {};

    if (output_handle_format == static_cast<int>(PixelFormat::RGBA_8888)) {
      output_handle_format = static_cast<int>(PixelFormat::RGBX_8888);
    }

    LayerBufferFormat new_sdm_format =
        HWCLayer::GetSDMFormat(output_handle_format, output_handle_flags);
    if (new_sdm_format == kFormatInvalid) {
      return HWC2::Error::BadParameter;
    }

    if (sdm::SetCSC(output_handle, &color_metadata) != kErrorNone) {
      return HWC2::Error::BadParameter;
    }

    output_buffer_->flags.secure = 0;
    output_buffer_->flags.video = 0;
    output_buffer_->buffer_id = reinterpret_cast<uint64_t>(output_handle);
    output_buffer_->format = new_sdm_format;
    output_buffer_->color_metadata = color_metadata;
    output_handle_ = output_handle;

    // TZ Protected Buffer - L1
    if (output_handle_flags & qtigralloc::PRIV_FLAGS_SECURE_BUFFER) {
      output_buffer_->flags.secure = 1;
    }

    // ToDo: Need to extend for non-RGB formats
    int fd = 0;
    uint32_t width = 0;
    buffer_allocator_->GetFd((void *)output_handle, fd);
    buffer_allocator_->GetWidth((void *)output_handle, width);
    output_buffer_->planes[0].fd = fd;
    output_buffer_->planes[0].offset = 0;
    output_buffer_->planes[0].stride = width;
  }

  output_buffer_->acquire_fence = release_fence;

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtual::SetFrameDumpConfig(uint32_t count, uint32_t bit_mask_layer_type,
                                                  int32_t format, CwbConfig &cwb_config) {
  HWCDisplay::SetFrameDumpConfig(count, bit_mask_layer_type, format);
  dump_output_layer_ = ((bit_mask_layer_type & (1 << OUTPUT_LAYER_DUMP)) != 0);

  DLOGI("output_layer_dump_enable %d", dump_output_layer_);
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtual::GetDisplayType(int32_t *out_type) {
  if (out_type == nullptr) {
    return HWC2::Error::BadParameter;
  }

  *out_type = HWC2_DISPLAY_TYPE_VIRTUAL;

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtual::SetColorMode(ColorMode mode) {
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtual::SetColorModeWithRenderIntent(ColorMode mode, RenderIntent intent) {
  return HWC2::Error::None;
}

}  // namespace sdm
