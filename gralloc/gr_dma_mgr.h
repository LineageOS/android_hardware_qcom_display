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

#ifndef __GR_DMA_MGR_H__
#define __GR_DMA_MGR_H__

#include <BufferAllocator/BufferAllocator.h>
#include <string>
#include <vector>

#include "gr_alloc_interface.h"
#include "vmmem.h"

#define FD_INIT -1

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

  static DmaManager *GetInstance();

 private:
  DmaManager();
  int UnmapBuffer(void *base, unsigned int size, unsigned int offset);
  void Deinit();

  int dma_dev_fd_ = FD_INIT;
  BufferAllocator buffer_allocator_;
  static DmaManager *dma_manager_;

  void* libvmmemPointer;
  std::unique_ptr<VmMem> (*createVmMem)();
};

}  // namespace gralloc

#endif  // __GR_DMA_MGR_H__
