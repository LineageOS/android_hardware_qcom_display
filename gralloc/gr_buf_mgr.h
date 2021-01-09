/*
 * Copyright (c) 2011-2018, The Linux Foundation. All rights reserved.
 * Not a Contribution
 *
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __GR_BUF_MGR_H__
#define __GR_BUF_MGR_H__

#include <pthread.h>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "gr_allocator.h"
#include "gr_utils.h"
#include "gr_buf_descriptor.h"
#include "gralloc_priv.h"

namespace gralloc {

using android::hardware::graphics::mapper::V2_0::Error;

class BufferManager {
 public:
  ~BufferManager();

  Error AllocateBuffer(const BufferDescriptor &descriptor, buffer_handle_t *handle,
                       unsigned int bufferSize = 0);
  Error RetainBuffer(private_handle_t const *hnd);
  Error ReleaseBuffer(private_handle_t const *hnd);
  Error LockBuffer(const private_handle_t *hnd, uint64_t usage);
  Error UnlockBuffer(const private_handle_t *hnd);
  Error Dump(std::ostringstream *os);
  Error ValidateBufferSize(private_handle_t const *hnd, BufferInfo info);
  Error IsBufferImported(const private_handle_t *hnd);
  static BufferManager *GetInstance();

 private:
  BufferManager();
  Error MapBuffer(private_handle_t const *hnd);

  // Imports the ion fds into the current process. Returns an error for invalid handles
  Error ImportHandleLocked(private_handle_t *hnd);

  // Creates a Buffer from the valid private handle and adds it to the map
  void RegisterHandleLocked(const private_handle_t *hnd, int ion_handle, int ion_handle_meta);

  // Wrapper structure over private handle
  // Values associated with the private handle
  // that do not need to go over IPC can be placed here
  // This structure is also not expected to be ABI stable
  // unlike private_handle_t
  struct Buffer {
    const private_handle_t *handle = nullptr;
    int ref_count = 1;
    // Hold the main and metadata ion handles
    // Freed from the allocator process
    // and unused in the mapping process
    int ion_handle_main = -1;
    int ion_handle_meta = -1;

    Buffer() = delete;
    explicit Buffer(const private_handle_t *h, int ih_main = -1, int ih_meta = -1)
        : handle(h), ion_handle_main(ih_main), ion_handle_meta(ih_meta) {}
    void IncRef() { ++ref_count; }
    bool DecRef() { return --ref_count == 0; }
  };

  Error FreeBuffer(std::shared_ptr<Buffer> buf);

  // Get the wrapper Buffer object from the handle, returns nullptr if handle is not found
  std::shared_ptr<Buffer> GetBufferFromHandleLocked(const private_handle_t *hnd);
  Allocator *allocator_ = NULL;
  std::mutex buffer_lock_;
  std::unordered_map<const private_handle_t *, std::shared_ptr<Buffer>> handles_map_ = {};
  std::atomic<uint64_t> next_id_;
};

}  // namespace gralloc

#endif  // __GR_BUF_MGR_H__
