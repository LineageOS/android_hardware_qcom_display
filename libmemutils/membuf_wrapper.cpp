/*
* Copyright (c) 2020 The Linux Foundation. All rights reserved.
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

#include <linux/mem-buf.h>
#include <linux/msm_ion_ids.h>
#include <errno.h>
#include <unistd.h>
#include <utils/Log.h>
#include <linux/mem-buf.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <algorithm>

#include "membuf_wrapper.h"

#ifndef TRUSTED_VM

/** adb log */
#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "TUI_TEST: "
#ifdef LOG_NDDEBUG
#undef LOG_NDDEBUG
#endif
#define LOG_NDDEBUG 0  // Define to enable LOGD
#ifdef LOG_NDEBUG
#undef LOG_NDEBUG
#endif
#define LOG_NDEBUG  0  // Define to enable LOGV

#define DLOGI(...) do { ALOGD(__VA_ARGS__); printf(__VA_ARGS__); printf("\n"); } while (0)
#define DLOGE(...) do { ALOGE(__VA_ARGS__); printf(__VA_ARGS__); printf("\n"); } while (0)
#define DLOGW(...) do { ALOGW(__VA_ARGS__); printf(__VA_ARGS__); printf("\n"); } while (0)
#else
#define DLOGI(...) printf("%s  I: %s: ", LOG_TAG, __FUNCTION__);printf(__VA_ARGS__); printf("\n");
#define DLOGE(...) printf("%s  E: %s: ", LOG_TAG, __FUNCTION__);printf(__VA_ARGS__); printf("\n");
#define DLOGW(...) printf("%s  W: %s: ", LOG_TAG, __FUNCTION__);printf(__VA_ARGS__); printf("\n");
#endif


MemBuf *MemBuf::mem_buf_ = nullptr;
mutex MemBuf::lock_;
uint32_t MemBuf::ref_count_ = 0;

int MemBuf::GetInstance(MemBuf **mem_buf_hnd) {
  std::lock_guard<mutex> obj(lock_);
  if (!ref_count_) {
    mem_buf_ = new MemBuf();
    int ret = mem_buf_->Init();
    if (ret != 0) {
      DLOGE("Membuf init failed!! %d", ret);
      delete mem_buf_;
      mem_buf_ = nullptr;
      return ret;
    }
  }

  ref_count_++;
  *mem_buf_hnd = mem_buf_;
  return 0;
}

void MemBuf::PutInstance() {
  std::lock_guard<mutex> obj(lock_);
  ref_count_--;
  if (!ref_count_) {
    mem_buf_->Deinit();
    delete mem_buf_;
    mem_buf_ = nullptr;
  }
}

int MemBuf::Init() {
  dev_fd_ = open("/dev/membuf", O_RDONLY | O_CLOEXEC);
  if (dev_fd_ < 0) {
    DLOGE("Mem buf device  open fails!!\n");
    return -EINVAL;
  }
  return 0;
}

void MemBuf::Deinit() {
  if (dev_fd_ != -1) {
    close(dev_fd_);
  }
}

int MemBuf::TransferHeap(const UsageHints &hints, uint32_t size, int *heap_fd) {
  if (!heap_fd) {
    return -EINVAL;
  }

  int ret = 0;
  struct acl_entry acl_entries = { 0 };
  struct mem_buf_ion_data src_data = { 0 };
  struct mem_buf_ion_data dst_data = { 0 };
  struct mem_buf_alloc_ioctl_arg alloc_arg = { 0 };

  acl_entries.vmid = MEM_BUF_VMID_TRUSTED_VM;
  acl_entries.perms = MEM_BUF_PERM_FLAG_READ | MEM_BUF_PERM_FLAG_WRITE;

  if (hints.trusted_ui) {
    src_data.heap_id = ION_SECURE_DISPLAY_HEAP_ID;
    dst_data.heap_id = ION_TUI_CARVEOUT_HEAP_ID;
  } else {
    DLOGE("Invalid usage hints %x", (int)hints.hints);
    return -EINVAL;
  }

  alloc_arg.size = size;
  alloc_arg.acl_list = (uint64_t)&acl_entries;
  alloc_arg.nr_acl_entries = 1;
  alloc_arg.src_mem_type = MEM_BUF_ION_MEM_TYPE;
  alloc_arg.dst_mem_type = MEM_BUF_ION_MEM_TYPE;
  alloc_arg.src_data = (uint64_t)&src_data;
  alloc_arg.dst_data = (uint64_t)&dst_data;
  alloc_arg.mem_buf_fd = 0;

  ret = ioctl(dev_fd_, MEM_BUF_IOC_ALLOC, &alloc_arg);
  if (ret < 0) {
    DLOGE("membuf allocation failed rc: %d\n", ret);
    return ret;
  }
  *heap_fd = (int)alloc_arg.mem_buf_fd;

  return ret;
}

void MemBuf::ReturnHeap(int heap_fd) {
  if (heap_fd > 0) {
    close(heap_fd);
  }
}

int MemBuf::Export(int buf_fd, int *export_fd, int *memparcel_hdl) {
  if (!export_fd || !memparcel_hdl || buf_fd < 0) {
    return -EINVAL;
  }

  DLOGI("Input buffer fd to be exported %d", buf_fd);

  struct mem_buf_export_ioctl_arg export_args = { 0 };
  struct acl_entry acl_entries = { 0 };

  acl_entries.vmid = MEM_BUF_VMID_TRUSTED_VM;
  acl_entries.perms = MEM_BUF_PERM_FLAG_READ | MEM_BUF_PERM_FLAG_WRITE;

  export_args.dma_buf_fd = (uint32_t)buf_fd;
  export_args.nr_acl_entries = 1;
  export_args.acl_list = (uint64_t)&acl_entries;

  int ret = ioctl(dev_fd_, MEM_BUF_IOC_EXPORT, &export_args);
  if (ret < 0) {
    DLOGE("Export failed rc: %d", ret);
    return ret;
  }

  DLOGI("export fd: %d handle: 0x%x", export_args.export_fd, export_args.memparcel_hdl);
  *export_fd = (int)export_args.export_fd;
  *memparcel_hdl = (int)export_args.memparcel_hdl;

  return 0;
}

int MemBuf::Import(int memparcel_hdl, int *import_fd) {
  if (!import_fd || memparcel_hdl < 0) {
    return -EINVAL;
  }

  DLOGI("Input memparcel_hdl to be imported %d", memparcel_hdl);

  struct mem_buf_import_ioctl_arg import_args = { 0 };
  struct acl_entry acl_entries = { 0 };
  acl_entries.vmid = MEM_BUF_VMID_TRUSTED_VM;
  acl_entries.perms = MEM_BUF_PERM_FLAG_READ | MEM_BUF_PERM_FLAG_WRITE;

  import_args.memparcel_hdl = (uint32_t)memparcel_hdl;
  import_args.nr_acl_entries = 1;
  import_args.acl_list = (uint64_t)&acl_entries;

  int ret = ioctl(dev_fd_, MEM_BUF_IOC_IMPORT, &import_args);
  if (ret < 0) {
    DLOGE("Import failed ret: %d", ret);
    return ret;
  }

  DLOGI("import fd: %d\n", import_args.dma_buf_import_fd);
  *import_fd = (int)import_args.dma_buf_import_fd;

  return 0;
}
