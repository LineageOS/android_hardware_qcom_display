/*
* Copyright (c) 2019, The Linux Foundation. All rights reserved.
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
#include <string>

#define __CLASS__ "Fence"

namespace sdm {

BufferSyncHandler* Fence::buffer_sync_handler_ = nullptr;

Fence::Fence(int fd) : fd_(fd) {
}

Fence::~Fence() {
  close(fd_);
}

void Fence::Set(BufferSyncHandler *buffer_sync_handler) {
  buffer_sync_handler_ = buffer_sync_handler;
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
  if (!fence) {
    return -1;
  }

  return dup(fence->fd_);
}

shared_ptr<Fence> Fence::Merge(const shared_ptr<Fence> &fence1, const shared_ptr<Fence> &fence2) {
  if (!buffer_sync_handler_) {
    return nullptr;
  }

  int fd1 = fence1 ? fence1->fd_ : -1;
  int fd2 = fence2 ? fence2->fd_ : -1;
  int merged = -1;

  buffer_sync_handler_->SyncMerge(fd1, fd2, &merged);

  return Create(merged);
}

DisplayError Fence::Wait(const shared_ptr<Fence> &fence) {
  if (!buffer_sync_handler_) {
    return kErrorUndefined;
  }

  return buffer_sync_handler_->SyncWait(Fence::Get(fence));
}

DisplayError Fence::Wait(const shared_ptr<Fence> &fence, int timeout) {
  if (!buffer_sync_handler_) {
    return kErrorUndefined;
  }

  return buffer_sync_handler_->SyncWait(Fence::Get(fence), timeout);
}

string Fence::GetStr(const shared_ptr<Fence> &fence) {
  return std::to_string(Fence::Get(fence));
}

int Fence::Get(const shared_ptr<Fence> &fence) {
  return (fence ? fence->fd_ : -1);
}

}  // namespace sdm
