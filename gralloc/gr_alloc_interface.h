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

#ifndef __GR_ALLOC_INTERFACE_H__
#define __GR_ALLOC_INTERFACE_H__

#include <string>
#include <vector>

namespace gralloc {

enum { CACHE_CLEAN = 0x1, CACHE_INVALIDATE, CACHE_CLEAN_AND_INVALIDATE, CACHE_READ_DONE };

struct AllocData {
  void *base = NULL;
  int fd = -1;
  int ion_handle = -1;
  unsigned int offset = 0;
  unsigned int size = 0;
  unsigned int align = 1;
  uintptr_t handle = 0;
  bool uncached = false;
  unsigned int flags = 0x0;
  std::string heap_name = "";
  std::vector<std::string> vm_names;
  unsigned int alloc_type = 0x0;
};

class AllocInterface {
 public:
  /*! @brief Method to get the instance of allocator interface

    @details This function opens the ion device and provides the ion allocator interface

    @return Returns AllocInterface pointer on sucess otherwise NULL
  */
  static AllocInterface *GetInstance();

  /*! @brief Method to to allocate buffer for a given buffer attributes

    @param[out]  data - \link AllocData \endlink

    @return Returns 0 on sucess otherwise errno
  */
  virtual int AllocBuffer(AllocData *data) = 0;

  /*! @brief Method to deallocate buffer

    @param[in] buffer_handle - \link BufferHandle \endlink
    @param[out] base - virtual base address to be deallocated
    @param[in] size - size of the buffer to be deallocated
    @param[in] offset - offset of the buffer to be deallocated
    @param[in] fd - fd of the buffer to be deallocated
    @param[in] handle - handle of the buffer to be deallocated

    @return Returns 0 on sucess otherwise errno
  */
  virtual int FreeBuffer(void *base, unsigned int size, unsigned int offset, int fd,
                         int handle) = 0;

  /*! @brief This creates a new mapping in the virtual address space of the calling process and
       provides the pointer.

    @param[out] base - virtual base address after mapping
    @param[in] size - size of the buffer to be mapped
    @param[in] offset - offset of the buffer to be mapped
    @param[in] fd - fd of the buffer to be mapped

    @return Returns 0 on sucess otherwise errno
  */
  virtual int MapBuffer(void **base, unsigned int size, unsigned int offset, int fd) = 0;

  /*! @brief This function helps to invalidate and flush the cache for the read write operation.

    @param[out] base - virtual base address
    @param[in] size - size of the buffer
    @param[in] offset - offset of the buffer
    @param[in] handle - handle of the buffer
    @param[in] op - cache operation to be done on buffer
    @param[in] fd - fd of the buffer

    @return Returns 0 on sucess otherwise errno
  */
  virtual int CleanBuffer(void *base, unsigned int size, unsigned int offset, int handle, int op,
                          int fd) = 0;

  /*! @brief Method to import buffer

    @param[in] fd - fd of the buffer to be imported

    @return Returns 0 on success otherwise errno
  */
  virtual int ImportBuffer(int fd) = 0;

  /*! @brief Method to allocate secure buffer

    @param[out]  data - \link AllocData \endlink

    @return Returns 0 on success otherwise errno
  */
  virtual int SecureMemPerms(AllocData *data);

  /*! @brief Method to get heap info

    @param[in] usage - Buffer usage mask
    @param[in]  sensor_flag - flag to set system heap for sensor use
    @param[out] heap_name - Corresponding name of heap
    @param[out] vm_names = Corresponding vector of names of vm to hyp assign in secure use case
    @param[out] alloc_type
    @param[out] flags - flags to specify desired heap allocation configuration
    @param[out] alloc_size - align the size to be allocated based on alignment requirement of heap

  */
  virtual void GetHeapInfo(uint64_t usage, bool sensor_flag, std::string *heap_name,
                           std::vector<std::string> *vm_names, unsigned int *alloc_type,
                           unsigned int *flags, unsigned int *alloc_size);

 protected:
  virtual ~AllocInterface() {}
};

}  // namespace gralloc

#endif  // __GR_ALLOC_INTERFACE_H__
