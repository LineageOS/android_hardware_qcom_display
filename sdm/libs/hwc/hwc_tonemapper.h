/*
* Copyright (c) 2016, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
*  * Neither the name of The Linux Foundation nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
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

#ifndef __HWC_TONEMAPPER_H__
#define __HWC_TONEMAPPER_H__

#include <fcntl.h>
#include <sys/mman.h>

#include <hardware/hwcomposer.h>

#include <core/layer_stack.h>
#include <utils/sys.h>

#include "hwc_buffer_sync_handler.h"

class Tonemapper;

namespace sdm {

class HWCToneMapper {
 public:
  HWCToneMapper() {}
  ~HWCToneMapper() {}

  int Init();
  void DeInit();
  int HandleToneMap(hwc_display_contents_1_t *content_list, LayerStack *layer_stack);
  void Terminate();
  void PostCommit(LayerStack *layer_stack);
  bool IsActive() { return active_; }
  void SetFrameDumpConfig(uint32_t count);

 private:
  int AllocateIntermediateBuffers(uint32_t width, uint32_t height, uint32_t format, uint32_t usage);
  void FreeIntermediateBuffers();
  void SetReleaseFence(int fence_fd);
  void CloseFd(int *fd);
  void DumpToneMapOutput(int *acquire_fence);

  static const uint32_t kNumIntermediateBuffers = 2;
  bool active_ = false;

  private_handle_t *intermediate_buffer_[kNumIntermediateBuffers] = {NULL, NULL};
  uint32_t current_intermediate_buffer_index_ = 0;
  int release_fence_fd_[kNumIntermediateBuffers];

  HWCBufferSyncHandler buffer_sync_handler_ = {};
  Tonemapper *gpu_tone_mapper_ = NULL;
  uint32_t dump_frame_count_ = 0;
  uint32_t dump_frame_index_ = 0;
};

}  // namespace sdm
#endif  // __HWC_TONEMAPPER_H__
