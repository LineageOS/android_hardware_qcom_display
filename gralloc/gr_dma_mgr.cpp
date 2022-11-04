/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.

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

#define DEBUG 0
#define ATRACE_TAG (ATRACE_TAG_GRAPHICS | ATRACE_TAG_HAL)
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/dma-buf.h>
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
#include <utility>
#include <vector>
#include <dlfcn.h>

#include "gr_utils.h"
#include "gralloc_priv.h"
#include "gr_dma_mgr.h"

#define SIZE_2MB 0x200000

namespace gralloc {

DmaManager *DmaManager::dma_manager_ = NULL;

DmaManager::DmaManager() {
    libvmmemPointer = dlopen("libvmmem.so", RTLD_LAZY);

    if(!libvmmemPointer) {
        ALOGE("Could not load libvmmem: %s", dlerror());
    }

    createVmMem = reinterpret_cast<std::unique_ptr<VmMem> (*)()>(dlsym(libvmmemPointer, "CreateVmMem"));
    const char* dlsym_error = dlerror();
    if (dlsym_error) {
         ALOGE("Cannot load symbol CreateVmMem: %s", dlsym_error);
    }

}

DmaManager *DmaManager::GetInstance() {
  if (!dma_manager_) {
    dma_manager_ = new DmaManager();
  }
  return dma_manager_;
}

void DmaManager::Deinit() {
  if (dma_dev_fd_ > FD_INIT) {
    close(dma_dev_fd_);
  }

  dma_dev_fd_ = FD_INIT;
}

int DmaManager::AllocBuffer(AllocData *data) {
  ATRACE_CALL();
  unsigned int flags = data->flags;

  std::string tag_name{};
  if (ATRACE_ENABLED()) {
    tag_name = "libdma alloc size: " + std::to_string(data->size);
  }

  ATRACE_BEGIN("GrallocAllocation");
  dma_dev_fd_ = buffer_allocator_.Alloc(data->heap_name, data->size, flags, data->align);
  ATRACE_END();
  if (dma_dev_fd_ < 0) {
    ALOGE("libdma alloc failed ion_fd %d size %d align %d heap_name %s flags %x", dma_dev_fd_,
          data->size, data->align, data->heap_name.c_str(), flags);
    return dma_dev_fd_;
  }

  data->fd = dma_dev_fd_;
  data->ion_handle = dma_dev_fd_;
  ALOGD_IF(DEBUG, "libdma: Allocated buffer size:%u fd:%d", data->size, data->fd);

  return 0;
}

int DmaManager::FreeBuffer(void *base, unsigned int size, unsigned int offset, int fd,
                           int /*ion_handle*/) {
  ATRACE_CALL();
  int err = 0;
  ALOGD_IF(DEBUG, "libdma: Freeing buffer base:%p size:%u fd:%d", base, size, fd);

  if (base) {
    err = UnmapBuffer(base, size, offset);
  }

  close(fd);
  return err;
}

int DmaManager::ImportBuffer(int fd) {
  return fd;
}

int DmaManager::CleanBuffer(void * /*base*/, unsigned int /*size*/, unsigned int /*offset*/,
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

int DmaManager::MapBuffer(void **base, unsigned int size, unsigned int offset, int fd) {
  ATRACE_CALL();
  int err = 0;
  void *addr = 0;

  addr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  *base = addr;
  if (addr == MAP_FAILED) {
    err = -errno;
    ALOGE("dma: Failed to map memory in the client: %s", strerror(errno));
  } else {
    ALOGD_IF(DEBUG, "ion: Mapped buffer base:%p size:%u offset:%u fd:%d", addr, size, offset, fd);
  }

  return err;
}

int DmaManager::UnmapBuffer(void *base, unsigned int size, unsigned int /*offset*/) {
  ATRACE_CALL();
  ALOGD_IF(DEBUG, "dma: Unmapping buffer  base:%p size:%u", base, size);

  int err = 0;
  if (munmap(base, size)) {
    err = -errno;
    ALOGE("dma: Failed to unmap memory at %p : %s", base, strerror(errno));
  }

  return err;
}

int DmaManager::SecureMemPerms(AllocData *data) {
  int ret = 0;
  std::unique_ptr<VmMem> vmmem = createVmMem();
  if (!vmmem) {
    return -ENOMEM;
  }
  VmPerm vm_perms;

  for (auto vm_name : data->vm_names) {
    VmHandle handle = vmmem->FindVmByName(vm_name);
    if (vm_name == "qcom,cp_sec_display" || vm_name == "qcom,cp_camera_preview") {
      vm_perms.push_back(std::make_pair(handle, VMMEM_READ));
    } else if (vm_name == "qcom,cp_cdsp") {
      vm_perms.push_back(std::make_pair(handle, VMMEM_READ | VMMEM_WRITE | VMMEM_EXEC));
    } else {
      vm_perms.push_back(std::make_pair(handle, VMMEM_READ | VMMEM_WRITE));
    }
  }

  ret = vmmem->LendDmabuf(data->fd, vm_perms);
  return ret;
  }

void DmaManager::GetHeapInfo(uint64_t usage, bool sensor_flag, std::string *dma_heap_name,
                             std::vector<std::string> *dma_vm_names, unsigned int *alloc_type,
                             unsigned int * /* dmaflags */, unsigned int *alloc_size) {
  std::string heap_name = "qcom,system";
  unsigned int type = 0;
#ifndef QMAA
  if (usage & GRALLOC_USAGE_PROTECTED) {
    if (usage & GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY) {
      heap_name = "qcom,display";
      dma_vm_names->push_back("qcom,cp_sec_display");
    } else if (usage & BufferUsage::CAMERA_OUTPUT) {
      int secure_preview_only = 0;
      char property[PROPERTY_VALUE_MAX];
      if (property_get("vendor.gralloc.secure_preview_only", property, NULL) > 0) {
        secure_preview_only = atoi(property);
      }
      heap_name = "qcom,display";
      if (usage & GRALLOC_USAGE_PRIVATE_CDSP) {
        dma_vm_names->push_back("qcom,cp_cdsp");
      }
      if (usage & BufferUsage::COMPOSER_OVERLAY) {
        if (secure_preview_only) {
          dma_vm_names->push_back("qcom,cp_camera_preview");
        } else {
          dma_vm_names->push_back("qcom,cp_camera");
          dma_vm_names->push_back("qcom,cp_camera_preview");
        }
      } else {
        dma_vm_names->push_back("qcom,cp_camera");
      }
    } else if (usage & GRALLOC_USAGE_PRIVATE_CDSP) {
      heap_name = "qcom,secure-cdsp";
    } else {
      heap_name = "qcom,secure-pixel";
    }
    type |= private_handle_t::PRIV_FLAGS_SECURE_BUFFER;
  } else if (usage & GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY) {
    // Reuse GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY with no GRALLOC_USAGE_PROTECTED
    // for tursted UI use case and align the size to 2MB
    heap_name = "qcom,demura";
    *alloc_size = ALIGN(*alloc_size, SIZE_2MB);
  }

  if (usage & BufferUsage::SENSOR_DIRECT_DATA) {
    if (sensor_flag) {
      ALOGI("gralloc::sns_direct_data with system_heap");
      heap_name = "qcom,system";
    } else {
      ALOGI("gralloc::sns_direct_data with adsp_heap");
      heap_name = "qcom,adsp";
    }
  }


#endif

  *alloc_type = type;
  *dma_heap_name = heap_name;

  return;
  }
}  // namespace gralloc
