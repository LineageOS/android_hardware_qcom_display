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

#ifndef __GR_DMA_MGR_H__
#define __GR_DMA_MGR_H__

#include <BufferAllocator/BufferAllocator.h>
#include <vmmem.h>
#include <string>
#include <vector>
#include <bitset>

#include "gr_alloc_interface.h"
#include "membuf_wrapper.h"

#define FD_INIT -1
#define MEMBUF_CLIENT_LIB_NAME "libmemutils.so"

#define CREATE_MEMBUF_INTERFACE_NAME "CreateMemBufInterface"
#define DESTROY_MEMBUF_INTERFACE_NAME "DestroyMemBufInterface"

typedef int (*CreateMemBufInterface)(MemBuf **mem_buf_hnd);
typedef int (*DestroyMemBufInterface)();

namespace gralloc {

class DmaManager : public AllocInterface {
 public:
  ~DmaManager() { Deinit(); }
  virtual int AllocBuffer(AllocData *data);
  virtual int FreeBuffer(void *base, unsigned int size, unsigned int offset, int fd,
                         int ion_handle);
  virtual int MapBuffer(void **base, unsigned int size, unsigned int offset, int fd);
  virtual int ImportBuffer(int fd);
  virtual int CleanBuffer(void *base, unsigned int size, unsigned int offset, int handle, int op,
                          int fd);
  virtual int SecureMemPerms(AllocData *data);
  virtual void GetHeapInfo(uint64_t usage, bool sensor_flag, std::string *heap_name,
                           std::vector<std::string> *vm_names, unsigned int *alloc_type,
                           unsigned int *flags, unsigned int *alloc_size);
  virtual int SetBufferPermission(int fd, BufferPermission *buf_perm, int64_t *mem_hdl);

  static DmaManager *GetInstance();

 private:
  DmaManager() {}
  int UnmapBuffer(void *base, unsigned int size, unsigned int offset);
  void GetVMPermission(BufferPermission perm, std::bitset<kVmPermissionMax> *vm_perm);
  void InitMemUtils();
  void DeinitMemUtils();
  void Deinit();

  int dma_dev_fd_ = FD_INIT;
  BufferAllocator buffer_allocator_;
  static DmaManager *dma_manager_;
  bool enable_logs_;
  MemBuf *mem_buf_ = nullptr;
  void *mem_utils_lib_ = {};
  CreateMemBufInterface CreateMemBuf_ = nullptr;
  DestroyMemBufInterface DestroyMemBuf_ = nullptr;
};

}  // namespace gralloc

#endif  // __GR_DMA_MGR_H__
