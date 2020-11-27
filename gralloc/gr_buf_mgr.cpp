/*
 * Copyright (c) 2011-2018 The Linux Foundation. All rights reserved.
 * Not a Contribution
 *
 * Copyright (C) 2010 The Android Open Source Project
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

#define DEBUG 0

#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

#include "gr_buf_descriptor.h"
#include "gr_buf_mgr.h"
#include "gr_priv_handle.h"
#include "qdMetaData.h"
#include "qd_utils.h"

namespace gralloc {

static BufferInfo GetBufferInfo(const BufferDescriptor &descriptor) {
  return BufferInfo(descriptor.GetWidth(), descriptor.GetHeight(), descriptor.GetFormat(),
                    descriptor.GetUsage());
}

BufferManager::BufferManager() : next_id_(0) {
  handles_map_.clear();
  allocator_ = new Allocator();
  allocator_->Init();
}

BufferManager *BufferManager::GetInstance() {
  static BufferManager *instance = new BufferManager();
  return instance;
}

BufferManager::~BufferManager() {
  if (allocator_) {
    delete allocator_;
  }
}

Error BufferManager::FreeBuffer(std::shared_ptr<Buffer> buf) {
  auto hnd = buf->handle;
  ALOGD_IF(DEBUG, "FreeBuffer handle:%p", hnd);

  if (private_handle_t::validate(hnd) != 0) {
    ALOGE("FreeBuffer: Invalid handle: %p", hnd);
    return Error::BAD_BUFFER;
  }

  if (allocator_->FreeBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset, hnd->fd,
                             buf->ion_handle_main) != 0) {
    return Error::BAD_BUFFER;
  }

  unsigned int meta_size = ALIGN((unsigned int)sizeof(MetaData_t), PAGE_SIZE);
  if (allocator_->FreeBuffer(reinterpret_cast<void *>(hnd->base_metadata), meta_size,
                             hnd->offset_metadata, hnd->fd_metadata, buf->ion_handle_meta) != 0) {
    return Error::BAD_BUFFER;
  }

  private_handle_t *handle = const_cast<private_handle_t *>(hnd);
  handle->fd = -1;
  handle->fd_metadata = -1;
  if (!(handle->flags & private_handle_t::PRIV_FLAGS_CLIENT_ALLOCATED)) {
    delete handle;
  }
  return Error::NONE;
}

Error BufferManager::ValidateBufferSize(private_handle_t const *hnd, BufferInfo info) {
  unsigned int size, alignedw, alignedh;
  info.format = GetImplDefinedFormat(info.usage, info.format);
  int ret = GetBufferSizeAndDimensions(info, &size, &alignedw, &alignedh);
  if (ret < 0) {
    return Error::BAD_BUFFER;
  }
  auto ion_fd_size = static_cast<unsigned int>(lseek(hnd->fd, 0, SEEK_END));
  if (size != ion_fd_size) {
    return Error::BAD_VALUE;
  }
  return Error::NONE;
}

void BufferManager::RegisterHandleLocked(const private_handle_t *hnd, int ion_handle,
                                         int ion_handle_meta) {
  auto buffer = std::make_shared<Buffer>(hnd, ion_handle, ion_handle_meta);
  handles_map_.emplace(std::make_pair(hnd, buffer));
}

Error BufferManager::ImportHandleLocked(private_handle_t *hnd) {
  if (private_handle_t::validate(hnd) != 0) {
    ALOGE("ImportHandleLocked: Invalid handle: %p", hnd);
    return Error::BAD_BUFFER;
  }
  ALOGD_IF(DEBUG, "Importing handle:%p id: %" PRIu64, hnd, hnd->id);
  int ion_handle = allocator_->ImportBuffer(hnd->fd);
  if (ion_handle < 0) {
    ALOGE("Failed to import ion buffer: hnd: %p, fd:%d, id:%" PRIu64, hnd, hnd->fd, hnd->id);
    return Error::BAD_BUFFER;
  }
  int ion_handle_meta = allocator_->ImportBuffer(hnd->fd_metadata);
  if (ion_handle_meta < 0) {
    ALOGE("Failed to import ion metadata buffer: hnd: %p, fd:%d, id:%" PRIu64, hnd, hnd->fd,
          hnd->id);
    return Error::BAD_BUFFER;
  }
  // Initialize members that aren't transported
  hnd->size = static_cast<unsigned int>(lseek(hnd->fd, 0, SEEK_END));
  hnd->offset = 0;
  hnd->offset_metadata = 0;
  hnd->base = 0;
  hnd->base_metadata = 0;
  hnd->gpuaddr = 0;
  RegisterHandleLocked(hnd, ion_handle, ion_handle_meta);
  return Error::NONE;
}

std::shared_ptr<BufferManager::Buffer> BufferManager::GetBufferFromHandleLocked(
    const private_handle_t *hnd) {
  auto it = handles_map_.find(hnd);
  if (it != handles_map_.end()) {
    return it->second;
  } else {
    return nullptr;
  }
}

Error BufferManager::MapBuffer(private_handle_t const *handle) {
  private_handle_t *hnd = const_cast<private_handle_t *>(handle);
  ALOGD_IF(DEBUG, "Map buffer handle:%p id: %" PRIu64, hnd, hnd->id);

  hnd->base = 0;
  if (allocator_->MapBuffer(reinterpret_cast<void **>(&hnd->base), hnd->size, hnd->offset,
                            hnd->fd) != 0) {
    return Error::BAD_BUFFER;
  }
  return Error::NONE;
}

Error BufferManager::IsBufferImported(const private_handle_t *hnd) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  auto buf = GetBufferFromHandleLocked(hnd);
  if (buf != nullptr) {
    return Error::NONE;
  }
  return Error::BAD_BUFFER;
}

Error BufferManager::RetainBuffer(private_handle_t const *hnd) {
  ALOGD_IF(DEBUG, "Retain buffer handle:%p id: %" PRIu64, hnd, hnd->id);
  auto err = Error::NONE;
  std::lock_guard<std::mutex> lock(buffer_lock_);
  auto buf = GetBufferFromHandleLocked(hnd);
  if (buf != nullptr) {
    buf->IncRef();
  } else {
    private_handle_t *handle = const_cast<private_handle_t *>(hnd);
    err = ImportHandleLocked(handle);
  }
  return err;
}

Error BufferManager::ReleaseBuffer(private_handle_t const *hnd) {
  ALOGD_IF(DEBUG, "Release buffer handle:%p", hnd);
  std::lock_guard<std::mutex> lock(buffer_lock_);
  auto buf = GetBufferFromHandleLocked(hnd);
  if (buf == nullptr) {
    ALOGE("Could not find handle: %p id: %" PRIu64, hnd, hnd->id);
    return Error::BAD_BUFFER;
  } else {
    if (buf->DecRef()) {
      handles_map_.erase(hnd);
      // Unmap, close ion handle and close fd
      FreeBuffer(buf);
    }
  }
  return Error::NONE;
}

Error BufferManager::LockBuffer(const private_handle_t *hnd, uint64_t usage) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  auto err = Error::NONE;
  ALOGD_IF(DEBUG, "LockBuffer buffer handle:%p id: %" PRIu64, hnd, hnd->id);

  // If buffer is not meant for CPU return err
  if (!CpuCanAccess(usage)) {
    return Error::BAD_VALUE;
  }

  auto buf = GetBufferFromHandleLocked(hnd);
  if (buf == nullptr) {
    return Error::BAD_BUFFER;
  }

  if (hnd->base == 0) {
    // we need to map for real
    err = MapBuffer(hnd);
  }

  // Invalidate if CPU reads in software and there are non-CPU
  // writers. No need to do this for the metadata buffer as it is
  // only read/written in software.

  // todo use handle here
  if (err == Error::NONE && (hnd->flags & private_handle_t::PRIV_FLAGS_USES_ION) &&
      (hnd->flags & private_handle_t::PRIV_FLAGS_CACHED)) {
    if (allocator_->CleanBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset,
                                buf->ion_handle_main, CACHE_INVALIDATE, hnd->fd)) {
      return Error::BAD_BUFFER;
    }
  }

  // Mark the buffer to be flushed after CPU write.
  if (err == Error::NONE && CpuCanWrite(usage)) {
    private_handle_t *handle = const_cast<private_handle_t *>(hnd);
    handle->flags |= private_handle_t::PRIV_FLAGS_NEEDS_FLUSH;
  }

  return err;
}

Error BufferManager::UnlockBuffer(const private_handle_t *handle) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  auto status = Error::NONE;

  private_handle_t *hnd = const_cast<private_handle_t *>(handle);
  auto buf = GetBufferFromHandleLocked(hnd);
  if (buf == nullptr) {
    return Error::BAD_BUFFER;
  }

  if (hnd->flags & private_handle_t::PRIV_FLAGS_NEEDS_FLUSH) {
    if (allocator_->CleanBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset,
                                buf->ion_handle_main, CACHE_CLEAN, hnd->fd) != 0) {
      status = Error::BAD_BUFFER;
    }
    hnd->flags &= ~private_handle_t::PRIV_FLAGS_NEEDS_FLUSH;
  } else {
    if (allocator_->CleanBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset,
                                buf->ion_handle_main, CACHE_READ_DONE, hnd->fd) != 0) {
      status = Error::BAD_BUFFER;
    }
  }

  return status;
}

Error BufferManager::AllocateBuffer(const BufferDescriptor &descriptor, buffer_handle_t *handle,
                                    unsigned int bufferSize) {
  if (!handle)
    return Error::BAD_BUFFER;
  std::lock_guard<std::mutex> buffer_lock(buffer_lock_);

  uint64_t usage = descriptor.GetUsage();
  int format = GetImplDefinedFormat(usage, descriptor.GetFormat());
  uint32_t layer_count = descriptor.GetLayerCount();

  unsigned int size;
  unsigned int alignedw, alignedh;
  int err = 0;

  int buffer_type = GetBufferType(format);
  BufferInfo info = GetBufferInfo(descriptor);
  info.format = format;
  info.layer_count = layer_count;

  GraphicsMetadata graphics_metadata = {};
  err = GetBufferSizeAndDimensions(info, &size, &alignedw, &alignedh, &graphics_metadata);
  if (err < 0) {
    return Error::BAD_DESCRIPTOR;
  }

  size = (bufferSize >= size) ? bufferSize : size;
  uint64_t flags = 0;
  auto page_size = UINT(getpagesize());
  AllocData data;
  data.align = GetDataAlignment(format, usage);
  data.size = size;
  data.handle = (uintptr_t)handle;
  data.uncached = UseUncached(format, usage);

  // Allocate buffer memory
  err = allocator_->AllocateMem(&data, usage, format);
  if (err) {
    ALOGE("gralloc failed to allocate err=%s format %d size %d WxH %dx%d usage %" PRIu64,
          strerror(-err), format, size, alignedw, alignedh, usage);
    return Error::NO_RESOURCES;
  }

  // Allocate memory for MetaData
  AllocData e_data;
  e_data.size = ALIGN(UINT(sizeof(MetaData_t)), page_size);
  e_data.handle = data.handle;
  e_data.align = page_size;

  err = allocator_->AllocateMem(&e_data, 0, 0);
  if (err) {
    ALOGE("gralloc failed to allocate metadata error=%s", strerror(-err));
    return Error::NO_RESOURCES;
  }

  flags = GetHandleFlags(format, usage);
  flags |= data.alloc_type;

  // Create handle
  private_handle_t *hnd = new private_handle_t(
      data.fd, e_data.fd, INT(flags), INT(alignedw), INT(alignedh), descriptor.GetWidth(),
      descriptor.GetHeight(), format, buffer_type, data.size, usage);

  hnd->id = ++next_id_;
  hnd->base = 0;
  hnd->base_metadata = 0;
  hnd->layer_count = layer_count;
  // set default csc as 709, but for video(yuv) its 601L
  ColorSpace_t colorSpace = (buffer_type == BUFFER_TYPE_VIDEO) ? ITU_R_601 : ITU_R_709;
  setMetaDataAndUnmap(hnd, UPDATE_COLOR_SPACE, reinterpret_cast<void *>(&colorSpace));

  bool use_adreno_for_size = CanUseAdrenoForSize(buffer_type, usage);
  if (use_adreno_for_size) {
    setMetaDataAndUnmap(hnd, SET_GRAPHICS_METADATA, reinterpret_cast<void *>(&graphics_metadata));
  }

  *handle = hnd;
  RegisterHandleLocked(hnd, data.ion_handle, e_data.ion_handle);
  ALOGD_IF(DEBUG, "Allocated buffer handle: %p id: %" PRIu64, hnd, hnd->id);
  if (DEBUG) {
    private_handle_t::Dump(hnd);
  }
  return Error::NONE;
}

Error BufferManager::Dump(std::ostringstream *os) {
  std::lock_guard<std::mutex> buffer_lock(buffer_lock_);
  for (auto it : handles_map_) {
    auto buf = it.second;
    auto hnd = buf->handle;
    *os << "handle id: " << std::setw(4) << hnd->id;
    *os << " fd: " << std::setw(3) << hnd->fd;
    *os << " fd_meta: " << std::setw(3) << hnd->fd_metadata;
    *os << " wxh: " << std::setw(4) << hnd->width << " x " << std::setw(4) << hnd->height;
    *os << " uwxuh: " << std::setw(4) << hnd->unaligned_width << " x ";
    *os << std::setw(4) << hnd->unaligned_height;
    *os << " size: " << std::setw(9) << hnd->size;
    *os << std::hex << std::setfill('0');
    *os << " priv_flags: "
        << "0x" << std::setw(8) << hnd->flags;
    *os << " usage: "
        << "0x" << std::setw(8) << hnd->usage;
    // TODO(user): get format string from qdutils
    *os << " format: "
        << "0x" << std::setw(8) << hnd->format;
    *os << std::dec << std::setfill(' ') << std::endl;
  }
  return Error::NONE;
}
}  //  namespace gralloc
