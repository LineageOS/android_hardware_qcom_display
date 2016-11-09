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

#include <alloc_controller.h>
#include <gr.h>
#include <gralloc_priv.h>
#include <memalloc.h>
#include <sync/sync.h>

#include <TonemapFactory.h>

#include <core/buffer_allocator.h>

#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/formats.h>
#include <utils/rect.h>

#include <vector>

#include "hwc_debugger.h"
#include "hwc_tonemapper.h"

#define __CLASS__ "HWCToneMapper"

namespace sdm {

int HWCToneMapper::Init() {
  for (uint32_t i = 0; i < kNumIntermediateBuffers; i++) {
    intermediate_buffer_[i] = NULL;
    release_fence_fd_[i] = -1;
  }
  return 0;
}

void HWCToneMapper::DeInit() {
  return;
}

int HWCToneMapper::HandleToneMap(hwc_display_contents_1_t *content_list, LayerStack *layer_stack) {
  uint32_t layer_count = UINT32(layer_stack->layers.size());
  std::vector<uint32_t> tonemap_layer_index = {};
  Layer *layer = NULL;
  uint32_t i = 0;
  uint32_t gpu_count = 0;
  int fence_fd = -1;
  int acquire_fd = -1;
  int merged_fd = -1;
  hwc_layer_1_t *hwc_layer = NULL;
  const private_handle_t *dst_hnd = NULL;
  const private_handle_t *src_hnd = NULL;

  for (; i < layer_count; i++) {
    layer = layer_stack->layers.at(i);
    if (layer->request.flags.tone_map) {
      tonemap_layer_index.push_back(i);
      break;
    }
    if (layer->composition == kCompositionGPU) {
      gpu_count++;
    }
  }

  if (tonemap_layer_index.empty()) {
    return 0;
  }
  // gpu count can be 0 when a layer is on FB and in next cycle it doesn't update and SDM marks
  // it as SDE comp
  if (gpu_count == 0) {
    // TODO(akumarkr): Remove goto when added multiple instance support
    // intermediate buffer can be null
    goto update_fd;
  }

  if (intermediate_buffer_[0] == NULL) {
    DLOGI("format = %d width = %d height = %d", layer->request.format, layer->request.width,
         layer->request.height);
     // TODO(akumarkr): use flags from LayerRequestFlags for format change etc.,
     uint32_t usage = GRALLOC_USAGE_PRIVATE_IOMMU_HEAP | GRALLOC_USAGE_HW_TEXTURE |
                      GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
     AllocateIntermediateBuffers(layer->input_buffer.width, layer->input_buffer.height,
                                 HAL_PIXEL_FORMAT_RGBA_8888, usage);
  }
  current_intermediate_buffer_index_ =
    (current_intermediate_buffer_index_ + 1) % kNumIntermediateBuffers;

  if (!gpu_tone_mapper_) {
    Color10Bit *grid_entries = NULL;
    int grid_size = 0;
    if (layer->lut_3d.validGridEntries) {
      grid_entries = layer->lut_3d.gridEntries;
      grid_size = INT(layer->lut_3d.gridSize);
    }
    gpu_tone_mapper_ = TonemapperFactory_GetInstance(TONEMAP_INVERSE, layer->lut_3d.lutEntries,
                                                     layer->lut_3d.dim, grid_entries, grid_size);
    if (gpu_tone_mapper_ == NULL) {
      DLOGE("Get Tonemapper failed");
      return -1;
    }
  }

  hwc_layer = &content_list->hwLayers[i];
  dst_hnd = intermediate_buffer_[current_intermediate_buffer_index_];
  src_hnd = static_cast<const private_handle_t *>(hwc_layer->handle);
  acquire_fd = dup(layer->input_buffer.acquire_fence_fd);
  buffer_sync_handler_.SyncMerge(acquire_fd, release_fence_fd_[current_intermediate_buffer_index_],
                                 &merged_fd);
  if (acquire_fd >= 0) {
    CloseFd(&acquire_fd);
  }

  if (release_fence_fd_[current_intermediate_buffer_index_] >= 0) {
    CloseFd(&release_fence_fd_[current_intermediate_buffer_index_]);
  }
  DTRACE_BEGIN("GPU_TM_BLIT");
  fence_fd = gpu_tone_mapper_->blit(reinterpret_cast<const void *>(dst_hnd),
                                    reinterpret_cast<const void *>(src_hnd), merged_fd);
  DTRACE_END();


  DumpToneMapOutput(&fence_fd);

update_fd:
  // Acquire fence will be closed by HWC Display
  // Fence returned by GPU will be closed in PostCommit
  layer->input_buffer.acquire_fence_fd = fence_fd;
  layer->input_buffer.planes[0].fd = intermediate_buffer_[current_intermediate_buffer_index_]->fd;

  active_ = true;

  tonemap_layer_index.clear();

  return 0;
}

int HWCToneMapper::AllocateIntermediateBuffers(uint32_t width, uint32_t height, uint32_t format,
                                               uint32_t usage) {
  int status = 0;
  if (width <= 0 || height <= 0) {
    return false;
  }

  for (uint32_t i = 0; i < kNumIntermediateBuffers; i++) {
    status = alloc_buffer(&intermediate_buffer_[i], INT(width), INT(height), INT(format),
                          INT(usage));
    if (status < 0) {
      DLOGE("Allocation of Intermediate Buffer failed");
      FreeIntermediateBuffers();
      break;
    }
  }

  return status;
}
void HWCToneMapper::FreeIntermediateBuffers() {
  if (!intermediate_buffer_[0]) {
    return;
  }
  for (uint32_t i = 0; i < kNumIntermediateBuffers; i++) {
    private_handle_t *buffer = intermediate_buffer_[i];
    if (buffer) {
      // Free the valid fence
      if (release_fence_fd_[i] >= 0) {
        CloseFd(&release_fence_fd_[i]);
      }
      free_buffer(buffer);
      intermediate_buffer_[i] = NULL;
    }
  }
}

void HWCToneMapper::PostCommit(LayerStack *layer_stack) {
  uint32_t layer_count = UINT32(layer_stack->layers.size());
  std::vector<uint32_t> tonemap_layer_index = {};
  Layer *layer = NULL;
  int rel_fence_fd = -1;
  bool has_tonemap = false;
  uint32_t i;

  for (i = 0; i < layer_count; i++) {
    layer = layer_stack->layers.at(i);
    if (layer->request.flags.tone_map) {
      tonemap_layer_index.push_back(i);
      has_tonemap = true;
      break;
    }
  }

  if (has_tonemap) {
    LayerBuffer &layer_buffer = layer->input_buffer;

    rel_fence_fd = layer_buffer.release_fence_fd;
    // close the fd returned by GPU Tonemapper
    CloseFd(&layer_buffer.acquire_fence_fd);

    SetReleaseFence(rel_fence_fd);
  }

  active_ = false;
}

void HWCToneMapper::SetReleaseFence(int fd) {
  CloseFd(&release_fence_fd_[current_intermediate_buffer_index_]);
  // used to give to GPU tonemapper along with input layer fd
  release_fence_fd_[current_intermediate_buffer_index_] = dup(fd);
}

void HWCToneMapper::CloseFd(int *fd) {
  if (*fd >= 0) {
    close(*fd);
    *fd = -1;
  }
}

void HWCToneMapper::Terminate() {
  if (!gpu_tone_mapper_) {
    return;
  }
  // fix this on multiple instance: only delete obj and call ToneMapperDestroy on deInit.
  delete gpu_tone_mapper_;
  gpu_tone_mapper_ = NULL;

  TonemapperFactory_Destroy();
  FreeIntermediateBuffers();
  active_ = false;
}

void HWCToneMapper::SetFrameDumpConfig(uint32_t count) {
  DLOGI("Dump FrameConfig count = %d", count);
  dump_frame_count_ = count;
  dump_frame_index_ = 0;
}

void HWCToneMapper::DumpToneMapOutput(int *acquire_fd) {
  if (!dump_frame_count_) {
    return;
  }

  private_handle_t *target_buffer = intermediate_buffer_[current_intermediate_buffer_index_];

  if (*acquire_fd >= 0) {
    int error = sync_wait(*acquire_fd, 1000);
    if (error < 0) {
      DLOGW("sync_wait error errno = %d, desc = %s", errno, strerror(errno));
      return;
    }
  }

  char dump_file_name[PATH_MAX];
  size_t result = 0;
  snprintf(dump_file_name, sizeof(dump_file_name), "/data/misc/display/frame_dump_primary"
           "/tonemap_%d.raw", (dump_frame_index_));
  FILE* fp = fopen(dump_file_name, "w+");
  if (fp) {
    DLOGI("base addr = %x", target_buffer->base);
    result = fwrite(reinterpret_cast<void *>(target_buffer->base), target_buffer->size, 1, fp);
    fclose(fp);
  }
  dump_frame_count_--;
  dump_frame_index_++;
  CloseFd(acquire_fd);
}

}  // namespace sdm
