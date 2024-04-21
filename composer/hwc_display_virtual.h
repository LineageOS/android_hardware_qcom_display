/*
 * Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
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

/* Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef __HWC_DISPLAY_VIRTUAL_H__
#define __HWC_DISPLAY_VIRTUAL_H__

#include "hwc_display.h"
#include "hwc_display_event_handler.h"

namespace sdm {

class HWCDisplayVirtual : public HWCDisplay {
 public:
  static void Destroy(HWCDisplay *hwc_display);
  virtual int Init();
  virtual int Deinit();
  virtual HWC2::Error Present(shared_ptr<Fence> *out_retire_fence);
  virtual HWC2::Error SetFrameDumpConfig(uint32_t count, uint32_t bit_mask_layer_type,
                                         int32_t format, CwbConfig &cwb_config);
  virtual HWC2::Error GetDisplayType(int32_t *out_type);
  virtual HWC2::Error SetColorMode(ColorMode mode);
  virtual HWC2::Error SetColorModeWithRenderIntent(ColorMode mode, RenderIntent intent);
  virtual HWC2::Error SetOutputBuffer(buffer_handle_t buf, shared_ptr<Fence> release_fence);
  virtual HWC2::Error DumpVDSBuffer();
  bool NeedsGPUBypass();
  virtual HWC2::Error PreValidateDisplay(bool *exit_validate);
  virtual HWC2::Error CommitOrPrepare(bool validate_only, shared_ptr<Fence> *out_retire_fence,
                                      uint32_t *out_num_types, uint32_t *out_num_requests,
                                      bool *needs_commit);
  HWCDisplayVirtual(CoreInterface *core_intf, HWCBufferAllocator *buffer_allocator,
                    HWCCallbacks *callbacks, hwc2_display_t id, int32_t sdm_id,
                    uint32_t width, uint32_t height);

 protected:
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  std::shared_ptr<LayerBuffer> output_buffer_ = std::make_shared<LayerBuffer>();
  const native_handle_t *output_handle_ = nullptr;

 private:
  bool dump_output_layer_ = false;
};

}  // namespace sdm

#endif  // __HWC_DISPLAY_VIRTUAL_H__
