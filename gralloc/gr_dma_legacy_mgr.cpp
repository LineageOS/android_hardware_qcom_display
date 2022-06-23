/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of The Linux Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#define ATRACE_TAG (ATRACE_TAG_GRAPHICS | ATRACE_TAG_HAL)
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/dma-buf.h>
#include <ion/ion.h>
#include <stdlib.h>
#include <fcntl.h>
#include <log/log.h>
#include <cutils/trace.h>
#include <cutils/properties.h>
#include <errno.h>
#include <utils/Trace.h>
#ifndef QMAA
#include <linux/msm_ion.h>
#endif
#include <string>
#include <vector>

#include "gr_utils.h"
#include "gr_dma_legacy_mgr.h"

#ifndef ION_FLAG_CP_PIXEL
#define ION_FLAG_CP_PIXEL 0
#endif

#ifndef ION_FLAG_ALLOW_NON_CONTIG
#define ION_FLAG_ALLOW_NON_CONTIG 0
#endif

#ifndef ION_FLAG_CP_CAMERA_PREVIEW
#define ION_FLAG_CP_CAMERA_PREVIEW 0
#endif

#ifndef ION_SECURE
#define ION_SECURE ION_FLAG_SECURE
#endif

#ifndef ION_FLAG_CP_CDSP
#define ION_FLAG_CP_CDSP 0
#endif

#ifndef ION_SYSTEM_HEAP_ID
#define SYS_HEAP_ID ION_HEAP_SYSTEM
#else
#define SYS_HEAP_ID ION_SYSTEM_HEAP_ID
#endif

#ifdef SLAVE_SIDE_CP
#define CP_HEAP_ID ION_CP_MM_HEAP_ID
#define SD_HEAP_ID CP_HEAP_ID
#define ION_CP_FLAGS (ION_SECURE | ION_FLAG_ALLOW_NON_CONTIG)
#define ION_SD_FLAGS ION_SECURE
#define ION_SC_FLAGS ION_SECURE
#define ION_SC_PREVIEW_FLAGS ION_SECURE
#else  // MASTER_SIDE_CP
#define CP_HEAP_ID ION_SECURE_HEAP_ID
#define SD_HEAP_ID ION_SECURE_DISPLAY_HEAP_ID
#define ION_CP_FLAGS (ION_SECURE | ION_FLAG_CP_PIXEL)
#define ION_SD_FLAGS (ION_SECURE | ION_FLAG_CP_SEC_DISPLAY)
#define ION_SC_FLAGS (ION_SECURE | ION_FLAG_CP_CAMERA)
#define ION_SC_PREVIEW_FLAGS (ION_SECURE | ION_FLAG_CP_CAMERA_PREVIEW)
#endif

#define SIZE_2MB 0x200000

namespace gralloc {

DmaLegacyManager *DmaLegacyManager::dma_legacy_manager_ = NULL;

DmaLegacyManager *DmaLegacyManager::GetInstance() {
  if (!dma_legacy_manager_) {
    dma_legacy_manager_ = new DmaLegacyManager();
    dma_legacy_manager_->enable_logs_ = property_get_bool(ENABLE_LOGS_PROP, 0);
  }
  return dma_legacy_manager_;
}

void DmaLegacyManager::Deinit() {
  if (dma_legacy_dev_fd_ > FD_INIT) {
    close(dma_legacy_dev_fd_);
  }

  dma_legacy_dev_fd_ = FD_INIT;
}

int DmaLegacyManager::AllocBuffer(AllocData *data) {
  ATRACE_CALL();
  unsigned int flags = data->flags;

  flags |= data->uncached ? 0 : ION_FLAG_CACHED;

  std::string tag_name{};
  if (ATRACE_ENABLED()) {
    tag_name = "libdmalegacy alloc size: " + std::to_string(data->size);
  }

  ATRACE_BEGIN("GrallocAllocation");
  dma_legacy_dev_fd_ = buffer_allocator_.Alloc(data->heap_name, data->size, flags, data->align);
  ATRACE_END();
  if (dma_legacy_dev_fd_ < 0) {
    ALOGE("libdmalegacy alloc failed ion_fd %d size %d align %d heap_name %s flags %x",
          dma_legacy_dev_fd_, data->size, data->align, data->heap_name.c_str(), flags);
    return dma_legacy_dev_fd_;
  }

  data->fd = dma_legacy_dev_fd_;
  data->ion_handle = dma_legacy_dev_fd_;
  ALOGD_IF(enable_logs_, "libdmalegacy: Allocated buffer size:%u fd:%d", data->size, data->fd);

  return 0;
}

int DmaLegacyManager::FreeBuffer(void *base, unsigned int size, unsigned int offset, int fd,
                                 int /*ion_handle*/) {
  ATRACE_CALL();
  int err = 0;
  ALOGD_IF(enable_logs_, "libdmalegacy: Freeing buffer base:%p size:%u fd:%d", base, size, fd);

  if (base) {
    err = UnmapBuffer(base, size, offset);
  }

  close(fd);
  return err;
}

int DmaLegacyManager::ImportBuffer(int fd) {
  return fd;
}

int DmaLegacyManager::CleanBuffer(void * /*base*/, unsigned int /*size*/, unsigned int /*offset*/,
                                  int /*handle*/, int op, int dma_buf_fd) {
  ATRACE_CALL();
  ATRACE_INT("operation id", op);

  struct dma_buf_sync sync;
  int err = 0;

  switch (op) {
    case CACHE_CLEAN:
      sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
      break;
    case CACHE_INVALIDATE:
      sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
      break;
    case CACHE_READ_DONE:
      sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
      break;
    default:
      ALOGE("%s: Invalid operation %d", __FUNCTION__, op);
      return -1;
  }

  if (ioctl(dma_buf_fd, INT(DMA_BUF_IOCTL_SYNC), &sync)) {
    err = -errno;
    ALOGE("%s: DMA_BUF_IOCTL_SYNC failed with error - %s", __FUNCTION__, strerror(errno));
    return err;
  }

  return 0;
}

int DmaLegacyManager::MapBuffer(void **base, unsigned int size, unsigned int offset, int fd) {
  ATRACE_CALL();
  int err = 0;
  void *addr = 0;

  addr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  *base = addr;
  if (addr == MAP_FAILED) {
    err = -errno;
    ALOGE("ion: Failed to map memory in the client: %s", strerror(errno));
  } else {
    ALOGD_IF(enable_logs_, "ion: Mapped buffer base:%p size:%u offset:%u fd:%d", addr, size, offset,
             fd);
  }

  return err;
}

int DmaLegacyManager::UnmapBuffer(void *base, unsigned int size, unsigned int /*offset*/) {
  ATRACE_CALL();
  ALOGD_IF(enable_logs_, "ion: Unmapping buffer  base:%p size:%u", base, size);

  int err = 0;
  if (munmap(base, size)) {
    err = -errno;
    ALOGE("ion: Failed to unmap memory at %p : %s", base, strerror(errno));
  }

  return err;
}

int DmaLegacyManager::SecureMemPerms(AllocData *data) {
  // DmaLegacyManager will use ION flags handle secure permissions
  int ret = 0;
  return ret;
}

void DmaLegacyManager::GetHeapInfo(uint64_t usage, bool sensor_flag, std::string *ion_heap_name,
                                   std::vector<std::string> *vm_names, unsigned int *alloc_type,
                                   unsigned int *ion_flags, unsigned int *alloc_size) {
  std::string heap_name = "qcom,system";
  unsigned int type = 0;
  uint32_t flags = 0;
  bool is_default = true;
#ifndef QMAA
  if (usage & GRALLOC_USAGE_PROTECTED) {
    if (usage & GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY) {
      is_default = false;
      heap_name = "qcom,display";
      /*
       * There is currently no flag in ION for Secure Display
       * VM. Please add it to the define once available.
       */
      flags |= UINT(ION_SD_FLAGS);
      buffer_allocator_.MapNameToIonHeap(heap_name, "secure_display", flags,
                                         ION_HEAP(ION_SECURE_DISPLAY_HEAP_ID), flags);
    } else if (usage & BufferUsage::CAMERA_OUTPUT) {
      int secure_preview_only = 0;
      char property[PROPERTY_VALUE_MAX];
      if (property_get(SECURE_PREVIEW_ONLY_PROP, property, NULL) > 0) {
        secure_preview_only = atoi(property);
      }
      is_default = false;
      heap_name = "qcom,display";
      if (usage & GRALLOC_USAGE_PRIVATE_CDSP) {
        flags |= UINT(ION_SECURE | ION_FLAG_CP_CDSP);
      }
      if (usage & BufferUsage::COMPOSER_OVERLAY) {
        if (secure_preview_only) {  // holi target
          flags |= UINT(ION_SC_PREVIEW_FLAGS);
        } else {  // Default
          flags |= UINT(ION_SC_PREVIEW_FLAGS | ION_SC_FLAGS);
        }
      } else {
        flags |= UINT(ION_SC_FLAGS);
      }
      buffer_allocator_.MapNameToIonHeap(heap_name, "secure_display", flags,
                                         ION_HEAP(ION_SECURE_DISPLAY_HEAP_ID), flags);
    } else if (usage & GRALLOC_USAGE_PRIVATE_CDSP) {
      is_default = false;
      heap_name = "qcom,secure-cdsp";
      flags |= UINT(ION_SECURE | ION_FLAG_CP_CDSP);
      buffer_allocator_.MapNameToIonHeap(heap_name, "secure_carveout", flags,
                                         ION_HEAP(ION_SECURE_CARVEOUT_HEAP_ID), flags);
    } else {
      is_default = false;
      heap_name = "qcom,secure-pixel";
      flags |= UINT(ION_CP_FLAGS);
      buffer_allocator_.MapNameToIonHeap(heap_name, "secure_heap", flags,
                                         ION_HEAP(ION_SECURE_HEAP_ID), flags);
    }
  } else if (usage & GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY) {
    // Reuse GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY with no GRALLOC_USAGE_PROTECTED
    // for tursted UI use case and align the size to 2MB
    is_default = false;
    heap_name = "qcom,display";
    buffer_allocator_.MapNameToIonHeap(heap_name, "display", flags,
                                       ION_HEAP(ION_DISPLAY_HEAP_ID), flags);
    *alloc_size = ALIGN(*alloc_size, SIZE_2MB);
  }

  if (usage & BufferUsage::SENSOR_DIRECT_DATA) {
    if (sensor_flag) {
      ALOGI("gralloc::sns_direct_data with system_heap");
      heap_name = "qcom,system";
      is_default = false;
      buffer_allocator_.MapNameToIonHeap(heap_name, "system", flags, ION_HEAP(ION_SYSTEM_HEAP_ID),
                                         flags);
    } else {
      ALOGI("gralloc::sns_direct_data with adsp_heap");
      is_default = false;
      heap_name = "qcom,adsp";
      buffer_allocator_.MapNameToIonHeap(heap_name, "adsp", flags, ION_HEAP(ION_ADSP_HEAP_ID),
                                         flags);
    }
  }

  if (flags & UINT(ION_SECURE)) {
    type |= qtigralloc::PRIV_FLAGS_SECURE_BUFFER;
  }

  if (is_default) {
    buffer_allocator_.MapNameToIonHeap(heap_name, "system", flags, ION_HEAP(ION_SYSTEM_HEAP_ID),
                                       flags);
  }
#endif

  *alloc_type = type;
  *ion_flags = flags;
  *ion_heap_name = heap_name;

  return;
}

int DmaLegacyManager::SetBufferPermission(int fd, BufferPermission *buf_perm, int64_t *mem_hdl) {
  return 0;
}
}  // namespace gralloc
