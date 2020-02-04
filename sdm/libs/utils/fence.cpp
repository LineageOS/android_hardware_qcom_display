/*
* Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
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

#include <utils/fence.h>
#include <debug_handler.h>
#include <assert.h>
#include <string>

#define __CLASS__ "Fence"

namespace sdm {

#define ASSERT_IF_NO_BUFFER_SYNC(x) if (!x) { assert(false); }

BufferSyncHandler* Fence::g_buffer_sync_handler_ = nullptr;

Fence::Fence(int fd) : fd_(fd) {
}

Fence::~Fence() {
  close(fd_);
}

void Fence::Set(BufferSyncHandler *buffer_sync_handler) {
  g_buffer_sync_handler_ = buffer_sync_handler;
}

shared_ptr<Fence> Fence::Create(int fd) {
  // Do not create Fence object for invalid fd, so that nullptr can be used for invalid fences.
  if (fd < 0) {
    return nullptr;
  }

  shared_ptr<Fence> fence(new Fence(fd));
  if (!fence) {
    close(fd);
  }

  return fence;
}

int Fence::Dup(const shared_ptr<Fence> &fence) {
  return (fence ? dup(fence->fd_) : -1);
}

int Fence::Get(const shared_ptr<Fence> &fence) {
  return (fence ? fence->fd_ : -1);
}

shared_ptr<Fence> Fence::Merge(const shared_ptr<Fence> &fence1, const shared_ptr<Fence> &fence2) {
  ASSERT_IF_NO_BUFFER_SYNC(g_buffer_sync_handler_);

  // Sync merge will return a new unique fd if source fds are same.
  int fd1 = fence1 ? fence1->fd_ : -1;
  int fd2 = fence2 ? fence2->fd_ : -1;
  int merged = -1;

  g_buffer_sync_handler_->SyncMerge(fd1, fd2, &merged);

  return Create(merged);
}

DisplayError Fence::Wait(const shared_ptr<Fence> &fence) {
  ASSERT_IF_NO_BUFFER_SYNC(g_buffer_sync_handler_);

  return g_buffer_sync_handler_->SyncWait(Fence::Get(fence), 1000);
}

DisplayError Fence::Wait(const shared_ptr<Fence> &fence, int timeout) {
  ASSERT_IF_NO_BUFFER_SYNC(g_buffer_sync_handler_);

  return g_buffer_sync_handler_->SyncWait(Fence::Get(fence), timeout);
}

Fence::Status Fence::GetStatus(const shared_ptr<Fence> &fence) {
  ASSERT_IF_NO_BUFFER_SYNC(g_buffer_sync_handler_);

  if (!fence) {
    return Fence::Status::kSignaled;
  }

  // Treat only timeout error as pending, assume other errors as signaled.
  return (g_buffer_sync_handler_->SyncWait(Fence::Get(fence), 0) == kErrorTimeOut ?
                                    Fence::Status::kPending : Fence::Status::kSignaled);
}

string Fence::GetStr(const shared_ptr<Fence> &fence) {
  return std::to_string(Fence::Get(fence));
}

Fence::ScopedRef::~ScopedRef() {
  for (int dup_fd : dup_fds_) {
    close(dup_fd);
  }
}

int Fence::ScopedRef::Get(const shared_ptr<Fence> &fence) {
  int dup_fd = Fence::Dup(fence);
  if (dup_fd >= 0) {
    dup_fds_.push_back(dup_fd);
  }

  return dup_fd;
}

}  // namespace sdm
