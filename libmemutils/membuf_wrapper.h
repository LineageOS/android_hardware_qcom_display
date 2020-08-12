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

#ifndef __MEMBUF_WRAPPER_H__
#define __MEMBUF_WRAPPER_H__

#include <stdio.h>

#include <mutex>

using std::mutex;

struct UsageHints {
  union {
    struct {
      uint32_t trusted_ui : 1;  //!< Transfer memory from secure display heap to TUI carveout heap
                                //!<.when this bit is set
    };
    uint64_t hints;
  };
};

class MemBuf {
 public:
  /*! @brief Allocate and transfer a chunk of memory from src VM to dst VM

    @param[in] hints - \link UsageHints \endlink
    @param[in] size - Size of the memory to be trasferred to destination VM

    @param[out] heap_fd - A file descriptor representing the memory that was allocated
    from the source VM and added to the current VM. Calling close() on this file
    descriptor will deallocate the memory from the current VM, and return it to the source VM

    @return 0 on success otherwise errno
  */
  int TransferHeap(const UsageHints &hints, uint32_t size, int *heap_fd);

  /*! @brief Return the memory to the src VM

    @param[out] heap_fd - A file descriptor representing the memory to be closed

    @return 0 on success otherwise errno
  */
  void ReturnHeap(int heap_fd);

  /*! @brief Export the buffer fd from one VM to other VM. The buffer must not be mmap'ed by any
      process prior to invoking this function. The buffer must also be a cached buffer from a
      non-secure ION heap

    @param[in] buf_fd - Allocated ion buffer fd to be exported
    @param[out] export_fd - An fd that corresponds to the buffer that was exported. This fd
    must be kept open until it is no longer required to export the memory to another VM.

    @param[out] memparcel_hdl -The handle associated with the memparcel that was created by
    granting access to the buffer fd for the dst VM

    @return 0 on success otherwise errno
  */
  int Export(int buf_fd, int *export_fd, int *memparcel_hdl);

  /*! @brief Allocate and transfer a chunk of memory from src VM to dst VM

    @param[in] memparcel_hdl - The handle that corresponds to the memparcel we are importing.
    @param[out] import_fd - file descriptor of the buffer that the dst VM can use to
    access the buffer. This fd must be closed to release the memory.

    @return 0 on success otherwise errno
  */
  int Import(int memparcel_hdl, int *import_fd);

  ~MemBuf() { }

  static int GetInstance(MemBuf **mem_buf_hnd);
  static void PutInstance();

 private:
  int Init();
  void Deinit();

  static MemBuf* mem_buf_;
  static mutex lock_;
  static uint32_t ref_count_;

  int dev_fd_ = -1;
};

#endif  // __MEMBUF_WRAPPER_H__
