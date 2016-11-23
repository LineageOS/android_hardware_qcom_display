/*
 * Copyright (c) 2011-2017 The Linux Foundation. All rights reserved.
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

#include <utility>
#include <vector>

#include "qd_utils.h"
#include "gr_priv_handle.h"
#include "gr_buf_descriptor.h"
#include "gr_utils.h"
#include "gr_buf_mgr.h"
#include "qdMetaData.h"

namespace gralloc1 {
std::atomic<gralloc1_buffer_descriptor_t> BufferDescriptor::next_id_(1);

BufferManager::BufferManager() : next_id_(0) {
  char property[PROPERTY_VALUE_MAX];

  // Map framebuffer memory
  if ((property_get("debug.gralloc.map_fb_memory", property, NULL) > 0) &&
      (!strncmp(property, "1", PROPERTY_VALUE_MAX) ||
       (!strncasecmp(property, "true", PROPERTY_VALUE_MAX)))) {
    map_fb_mem_ = true;
  }

  // Enable UBWC for framebuffer
  if ((property_get("debug.gralloc.enable_fb_ubwc", property, NULL) > 0) &&
      (!strncmp(property, "1", PROPERTY_VALUE_MAX) ||
       (!strncasecmp(property, "true", PROPERTY_VALUE_MAX)))) {
    ubwc_for_fb_ = true;
  }

  handles_map_.clear();
  allocator_ = new Allocator();
  allocator_->Init();
}


gralloc1_error_t BufferManager::CreateBufferDescriptor(
    gralloc1_buffer_descriptor_t *descriptor_id) {
  std::lock_guard<std::mutex> lock(locker_);
  auto descriptor = std::make_shared<BufferDescriptor>();
  descriptors_map_.emplace(descriptor->GetId(), descriptor);
  *descriptor_id = descriptor->GetId();
  return GRALLOC1_ERROR_NONE;
}

gralloc1_error_t BufferManager::DestroyBufferDescriptor(
    gralloc1_buffer_descriptor_t descriptor_id) {
  std::lock_guard<std::mutex> lock(locker_);
  const auto descriptor = descriptors_map_.find(descriptor_id);
  if (descriptor == descriptors_map_.end()) {
    return GRALLOC1_ERROR_BAD_DESCRIPTOR;
  }
  descriptors_map_.erase(descriptor);
  return GRALLOC1_ERROR_NONE;
}

BufferManager::~BufferManager() {
  if (allocator_) {
    delete allocator_;
  }
}

gralloc1_error_t BufferManager::AllocateBuffers(uint32_t num_descriptors,
                                                const gralloc1_buffer_descriptor_t *descriptor_ids,
                                                buffer_handle_t *out_buffers) {
  bool shared = true;
  gralloc1_error_t status = GRALLOC1_ERROR_NONE;

  // since GRALLOC1_CAPABILITY_TEST_ALLOCATE capability is supported
  // client can ask to test the allocation by passing NULL out_buffers
  bool test_allocate = !out_buffers;

  // Validate descriptors
  std::vector<std::shared_ptr<BufferDescriptor>> descriptors;
  for (uint32_t i = 0; i < num_descriptors; i++) {
    const auto map_descriptor = descriptors_map_.find(descriptor_ids[i]);
    if (map_descriptor == descriptors_map_.end()) {
      return GRALLOC1_ERROR_BAD_DESCRIPTOR;
    } else {
      descriptors.push_back(map_descriptor->second);
    }
  }

  //  Resolve implementation defined formats
  for (auto &descriptor : descriptors) {
    descriptor->SetColorFormat(allocator_->GetImplDefinedFormat(descriptor->GetProducerUsage(),
                                                                descriptor->GetConsumerUsage(),
                                                                descriptor->GetFormat()));
  }

  // Check if input descriptors can be supported AND
  // Find out if a single buffer can be shared for all the given input descriptors
  uint32_t i = 0;
  ssize_t max_buf_index = -1;
  shared = allocator_->CheckForBufferSharing(num_descriptors, descriptors, &max_buf_index);

  if (test_allocate) {
    status = shared ? GRALLOC1_ERROR_NOT_SHARED : status;
    return status;
  }

  if (shared && (max_buf_index >= 0)) {
    // Allocate one and duplicate/copy the handles for each descriptor
    if (AllocateBuffer(*descriptors[UINT(max_buf_index)], &out_buffers[max_buf_index])) {
      return GRALLOC1_ERROR_NO_RESOURCES;
    }

    for (i = 0; i < num_descriptors; i++) {
      // Create new handle for a given descriptor.
      // Current assumption is even MetaData memory would be same
      // Need to revisit if there is a need for own metadata memory
      if (i != UINT(max_buf_index)) {
        CreateSharedHandle(out_buffers[max_buf_index], *descriptors[i], &out_buffers[i]);
      }
    }
  } else {
    // Buffer sharing is not feasible.
    // Allocate separate buffer for each descriptor
    for (i = 0; i < num_descriptors; i++) {
      if (AllocateBuffer(*descriptors[i], &out_buffers[i])) {
        return GRALLOC1_ERROR_NO_RESOURCES;
      }
    }
  }

  // Allocation is successful. If backstore is not shared inform the client.
  if (!shared) {
    return GRALLOC1_ERROR_NOT_SHARED;
  }

  return status;
}

void BufferManager::CreateSharedHandle(buffer_handle_t inbuffer, const BufferDescriptor &descriptor,
                                       buffer_handle_t *outbuffer) {
  private_handle_t const *input = reinterpret_cast<private_handle_t const *>(inbuffer);

  // Get Buffer attributes or dimension
  unsigned int alignedw = 0, alignedh = 0;
  allocator_->GetAlignedWidthAndHeight(descriptor, &alignedw, &alignedh);

  // create new handle from input reference handle and given descriptor
  int flags = GetHandleFlags(descriptor.GetFormat(), descriptor.GetProducerUsage(),
                             descriptor.GetConsumerUsage());
  int buffer_type = GetBufferType(descriptor.GetFormat());

  // Duplicate the fds
  // TODO(user): Not sure what to do for fb_id. Use duped fd and new dimensions?
  private_handle_t *out_hnd = new private_handle_t(dup(input->fd),
                                                   dup(input->fd_metadata),
                                                   flags,
                                                   INT(alignedw),
                                                   INT(alignedh),
                                                   descriptor.GetWidth(),
                                                   descriptor.GetHeight(),
                                                   descriptor.GetFormat(),
                                                   buffer_type,
                                                   input->size,
                                                   descriptor.GetProducerUsage(),
                                                   descriptor.GetConsumerUsage());
  out_hnd->id = ++next_id_;
  // TODO(user): Base address of shared handle and ion handles
  auto buffer = std::make_shared<Buffer>(out_hnd);
  handles_map_.emplace(std::make_pair(out_hnd->id, buffer));
  *outbuffer = out_hnd;
}

gralloc1_error_t BufferManager::FreeBuffer(std::shared_ptr<Buffer> buf) {
  auto hnd = buf->handle;
  if (allocator_->FreeBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset,
                             hnd->fd, buf->ion_handle_main) != 0) {
    return GRALLOC1_ERROR_BAD_HANDLE;
  }

  unsigned int meta_size = ALIGN((unsigned int)sizeof(MetaData_t), PAGE_SIZE);
  if (allocator_->FreeBuffer(reinterpret_cast<void *>(hnd->base_metadata), meta_size,
                             hnd->offset_metadata, hnd->fd_metadata, buf->ion_handle_meta) != 0) {
    return GRALLOC1_ERROR_BAD_HANDLE;
  }

  // TODO(user): delete handle once framework bug around this is confirmed
  // to be resolved
  return GRALLOC1_ERROR_NONE;
}

gralloc1_error_t BufferManager::MapBuffer(private_handle_t const *handle) {
  private_handle_t *hnd = const_cast<private_handle_t *>(handle);

  hnd->base = 0;
  hnd->base_metadata = 0;
  if (allocator_->MapBuffer(reinterpret_cast<void **>(&hnd->base), hnd->size, hnd->offset,
                            hnd->fd) != 0) {
    return GRALLOC1_ERROR_BAD_HANDLE;
  }

  unsigned int size = ALIGN((unsigned int)sizeof(MetaData_t), PAGE_SIZE);
  if (allocator_->MapBuffer(reinterpret_cast<void **>(&hnd->base_metadata), size,
                            hnd->offset_metadata, hnd->fd_metadata) != 0) {
    return GRALLOC1_ERROR_BAD_HANDLE;
  }

  return GRALLOC1_ERROR_NONE;
}

gralloc1_error_t BufferManager::RetainBuffer(private_handle_t const *hnd) {
  std::lock_guard<std::mutex> lock(locker_);

  // find if this handle is already in map
  auto it = handles_map_.find(hnd->id);
  if (it != handles_map_.end()) {
    // It's already in map, Just increment refcnt
    // No need to mmap the memory.
    auto buf = it->second;
    buf->ref_count++;
  } else {
    // not present in the map. mmap and then add entry to map
    if (MapBuffer(hnd) == GRALLOC1_ERROR_NONE) {
      auto buffer = std::make_shared<Buffer>(hnd);
      handles_map_.emplace(std::make_pair(hnd->id, buffer));
    }
  }

  return GRALLOC1_ERROR_NONE;
}

gralloc1_error_t BufferManager::ReleaseBuffer(private_handle_t const *hnd) {
  std::lock_guard<std::mutex> lock(locker_);
  // find if this handle is already in map
  auto it = handles_map_.find(hnd->id);
  if (it == handles_map_.end()) {
    // Corrupt handle or map.
    ALOGE("Could not find handle");
    return GRALLOC1_ERROR_BAD_HANDLE;
  } else {
    auto buf = it->second;
    buf->ref_count--;
    if (buf->ref_count == 0) {
      handles_map_.erase(it);
      FreeBuffer(buf);
    }
  }
  return GRALLOC1_ERROR_NONE;
}

gralloc1_error_t BufferManager::LockBuffer(const private_handle_t *hnd,
                                           gralloc1_producer_usage_t prod_usage,
                                           gralloc1_consumer_usage_t cons_usage) {
  gralloc1_error_t err = GRALLOC1_ERROR_NONE;

  // If buffer is not meant for CPU return err
  if (!CpuCanAccess(prod_usage, cons_usage)) {
    return GRALLOC1_ERROR_BAD_VALUE;
  }

  if (hnd->base == 0) {
    // we need to map for real
    locker_.lock();
    err = MapBuffer(hnd);
    locker_.unlock();
  }

  // Invalidate if CPU reads in software and there are non-CPU
  // writers. No need to do this for the metadata buffer as it is
  // only read/written in software.
  if (!err && (hnd->flags & private_handle_t::PRIV_FLAGS_USES_ION) &&
      (hnd->flags & private_handle_t::PRIV_FLAGS_CACHED)) {
    if (allocator_->CleanBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset,
                                hnd->fd, CACHE_INVALIDATE)) {
      return GRALLOC1_ERROR_BAD_HANDLE;
    }
  }

  // Mark the buffer to be flushed after CPU write.
  if (!err && CpuCanWrite(prod_usage)) {
    private_handle_t *handle = const_cast<private_handle_t *>(hnd);
    handle->flags |= private_handle_t::PRIV_FLAGS_NEEDS_FLUSH;
  }

  return err;
}

gralloc1_error_t BufferManager::UnlockBuffer(const private_handle_t *handle) {
  gralloc1_error_t status = GRALLOC1_ERROR_NONE;

  locker_.lock();
  private_handle_t *hnd = const_cast<private_handle_t *>(handle);

  if (hnd->flags & private_handle_t::PRIV_FLAGS_NEEDS_FLUSH) {
    if (allocator_->CleanBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset,
                                hnd->fd, CACHE_CLEAN) != 0) {
      status = GRALLOC1_ERROR_BAD_HANDLE;
    }
    hnd->flags &= ~private_handle_t::PRIV_FLAGS_NEEDS_FLUSH;
  }

  locker_.unlock();
  return status;
}

uint32_t BufferManager::GetDataAlignment(int format, gralloc1_producer_usage_t prod_usage,
                                    gralloc1_consumer_usage_t cons_usage) {
  uint32_t align = UINT(getpagesize());
  if (format == HAL_PIXEL_FORMAT_YCbCr_420_SP_TILED) {
    align = 8192;
  }

  if (prod_usage & GRALLOC1_PRODUCER_USAGE_PROTECTED) {
    if ((prod_usage & GRALLOC1_PRODUCER_USAGE_CAMERA) ||
        (cons_usage & GRALLOC1_CONSUMER_USAGE_PRIVATE_SECURE_DISPLAY)) {
      // The alignment here reflects qsee mmu V7L/V8L requirement
      align = SZ_2M;
    } else {
      align = SECURE_ALIGN;
    }
  }

  return align;
}

int BufferManager::GetHandleFlags(int format, gralloc1_producer_usage_t prod_usage,
                                  gralloc1_consumer_usage_t cons_usage) {
  int flags = 0;
  if (cons_usage & GRALLOC1_CONSUMER_USAGE_PRIVATE_EXTERNAL_ONLY) {
    flags |= private_handle_t::PRIV_FLAGS_EXTERNAL_ONLY;
  }

  if (cons_usage & GRALLOC1_CONSUMER_USAGE_PRIVATE_INTERNAL_ONLY) {
    flags |= private_handle_t::PRIV_FLAGS_INTERNAL_ONLY;
  }

  if (cons_usage & GRALLOC1_CONSUMER_USAGE_VIDEO_ENCODER) {
    flags |= private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  }

  if (prod_usage & GRALLOC1_PRODUCER_USAGE_CAMERA) {
    flags |= private_handle_t::PRIV_FLAGS_CAMERA_WRITE;
  }

  if (prod_usage & GRALLOC1_CONSUMER_USAGE_CAMERA) {
    flags |= private_handle_t::PRIV_FLAGS_CAMERA_READ;
  }

  if (cons_usage & GRALLOC1_CONSUMER_USAGE_HWCOMPOSER) {
    flags |= private_handle_t::PRIV_FLAGS_HW_COMPOSER;
  }

  if (prod_usage & GRALLOC1_CONSUMER_USAGE_GPU_TEXTURE) {
    flags |= private_handle_t::PRIV_FLAGS_HW_TEXTURE;
  }

  if (prod_usage & GRALLOC1_CONSUMER_USAGE_PRIVATE_SECURE_DISPLAY) {
    flags |= private_handle_t::PRIV_FLAGS_SECURE_DISPLAY;
  }

  if (allocator_->IsUBwcEnabled(format, prod_usage, cons_usage)) {
    flags |= private_handle_t::PRIV_FLAGS_UBWC_ALIGNED;
  }

  if (prod_usage & (GRALLOC1_PRODUCER_USAGE_CPU_READ | GRALLOC1_PRODUCER_USAGE_CPU_WRITE)) {
    flags |= private_handle_t::PRIV_FLAGS_CPU_RENDERED;
  }

  // TODO(user): is this correct???
  if ((cons_usage &
       (GRALLOC1_CONSUMER_USAGE_VIDEO_ENCODER | GRALLOC1_CONSUMER_USAGE_CLIENT_TARGET)) ||
      (prod_usage & (GRALLOC1_PRODUCER_USAGE_CAMERA | GRALLOC1_PRODUCER_USAGE_GPU_RENDER_TARGET))) {
    flags |= private_handle_t::PRIV_FLAGS_NON_CPU_WRITER;
  }

  if (cons_usage & GRALLOC1_CONSUMER_USAGE_HWCOMPOSER) {
    flags |= private_handle_t::PRIV_FLAGS_DISP_CONSUMER;
  }

  if (!allocator_->UseUncached(prod_usage)) {
    flags |= private_handle_t::PRIV_FLAGS_CACHED;
  }

  return flags;
}

int BufferManager::AllocateBuffer(unsigned int size, int aligned_w, int aligned_h, int unaligned_w,
                                  int unaligned_h, int format, int bufferType,
                                  gralloc1_producer_usage_t prod_usage,
                                  gralloc1_consumer_usage_t cons_usage, buffer_handle_t *handle) {
  auto page_size = UINT(getpagesize());
  int err = 0;
  int flags = 0;
  AllocData data;
  data.align = GetDataAlignment(format, prod_usage, cons_usage);
  data.size = ALIGN(size, data.align);
  data.handle = (uintptr_t)handle;
  data.uncached = allocator_->UseUncached(prod_usage);

  // Allocate buffer memory
  err = allocator_->AllocateMem(&data, prod_usage, cons_usage);
  if (err) {
    ALOGE("gralloc failed to allocate err=%s", strerror(-err));
    return err;
  }

  // Allocate memory for MetaData
  AllocData e_data;
  e_data.size = ALIGN(UINT(sizeof(MetaData_t)), page_size);
  e_data.handle = data.handle;
  e_data.align = page_size;

  err =
      allocator_->AllocateMem(&e_data, GRALLOC1_PRODUCER_USAGE_NONE, GRALLOC1_CONSUMER_USAGE_NONE);
  if (err) {
    ALOGE("gralloc failed to allocate metadata error=%s", strerror(-err));
    return err;
  }

  flags = GetHandleFlags(format, prod_usage, cons_usage);
  flags |= data.alloc_type;

  // Create handle
  private_handle_t *hnd = new private_handle_t(data.fd,
                                               e_data.fd,
                                               flags,
                                               aligned_w,
                                               aligned_h,
                                               unaligned_w,
                                               unaligned_h,
                                               format,
                                               bufferType,
                                               size,
                                               prod_usage,
                                               cons_usage);

  hnd->id = ++next_id_;
  hnd->base = reinterpret_cast<uint64_t >(data.base);
  hnd->base_metadata = reinterpret_cast<uint64_t >(e_data.base);

  ColorSpace_t colorSpace = ITU_R_601;
  setMetaData(hnd, UPDATE_COLOR_SPACE, reinterpret_cast<void *>(&colorSpace));
  *handle = hnd;
  auto buffer = std::make_shared<Buffer>(hnd, data.ion_handle, e_data.ion_handle);
  handles_map_.emplace(std::make_pair(hnd->id, buffer));
  return err;
}

int BufferManager::GetBufferType(int inputFormat) {
  int buffer_type = BUFFER_TYPE_VIDEO;
  if (IsUncompressedRGBFormat(inputFormat)) {
    // RGB formats
    buffer_type = BUFFER_TYPE_UI;
  }

  return buffer_type;
}

int BufferManager::AllocateBuffer(const BufferDescriptor &descriptor, buffer_handle_t *handle,
                                  unsigned int bufferSize) {
  if (!handle)
    return -EINVAL;

  int format = descriptor.GetFormat();
  gralloc1_producer_usage_t prod_usage = descriptor.GetProducerUsage();
  gralloc1_consumer_usage_t cons_usage = descriptor.GetConsumerUsage();

  // Get implementation defined format
  int gralloc_format = allocator_->GetImplDefinedFormat(prod_usage, cons_usage, format);

  bool use_fb_mem = false;
  if ((cons_usage & GRALLOC1_CONSUMER_USAGE_CLIENT_TARGET) && map_fb_mem_) {
    use_fb_mem = true;
  }

  if ((cons_usage & GRALLOC1_CONSUMER_USAGE_CLIENT_TARGET) && ubwc_for_fb_) {
    prod_usage =
        (gralloc1_producer_usage_t)(prod_usage | GRALLOC1_PRODUCER_USAGE_PRIVATE_ALLOC_UBWC);
  }

  unsigned int size;
  unsigned int alignedw, alignedh;
  int buffer_type = GetBufferType(gralloc_format);
  allocator_->GetBufferSizeAndDimensions(descriptor, &size, &alignedw, &alignedh);

  size = (bufferSize >= size) ? bufferSize : size;

  int err = 0;
  if (use_fb_mem) {
    // TODO(user): TBD Framebuffer specific implementation in a seperate file/class
  } else {
    err = AllocateBuffer(size, INT(alignedw), INT(alignedh), descriptor.GetWidth(),
                         descriptor.GetHeight(), format, buffer_type, descriptor.GetProducerUsage(),
                         descriptor.GetConsumerUsage(), handle);
  }

  if (err < 0) {
    return err;
  }

  return 0;
}

gralloc1_error_t BufferManager::Perform(int operation, va_list args) {
  switch (operation) {
    case GRALLOC_MODULE_PERFORM_CREATE_HANDLE_FROM_BUFFER: {
      int fd = va_arg(args, int);
      unsigned int size = va_arg(args, unsigned int);
      unsigned int offset = va_arg(args, unsigned int);
      void *base = va_arg(args, void *);
      int width = va_arg(args, int);
      int height = va_arg(args, int);
      int format = va_arg(args, int);

      native_handle_t **handle = va_arg(args, native_handle_t **);
      private_handle_t *hnd = reinterpret_cast<private_handle_t *>(
          native_handle_create(private_handle_t::kNumFds, private_handle_t::NumInts()));
      if (hnd) {
        unsigned int alignedw = 0, alignedh = 0;
        hnd->magic = private_handle_t::kMagic;
        hnd->fd = fd;
        hnd->flags = private_handle_t::PRIV_FLAGS_USES_ION;
        hnd->size = size;
        hnd->offset = offset;
        hnd->base = uint64_t(base) + offset;
        hnd->gpuaddr = 0;
        BufferDescriptor descriptor(width, height, format);
        allocator_->GetAlignedWidthAndHeight(descriptor, &alignedw, &alignedh);
        hnd->unaligned_width = width;
        hnd->unaligned_height = height;
        hnd->width = INT(alignedw);
        hnd->height = INT(alignedh);
        hnd->format = format;
        *handle = reinterpret_cast<native_handle_t *>(hnd);
      }
    } break;

    case GRALLOC_MODULE_PERFORM_GET_STRIDE: {
      int width = va_arg(args, int);
      int format = va_arg(args, int);
      int *stride = va_arg(args, int *);
      unsigned int alignedw = 0, alignedh = 0;
      BufferDescriptor descriptor(width, width, format);
      allocator_->GetAlignedWidthAndHeight(descriptor, &alignedw, &alignedh);
      *stride = INT(alignedw);
    } break;

    case GRALLOC_MODULE_PERFORM_GET_CUSTOM_STRIDE_FROM_HANDLE: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      int *stride = va_arg(args, int *);
      if (private_handle_t::validate(hnd) != 0) {
        return GRALLOC1_ERROR_BAD_HANDLE;
      }

      MetaData_t *metadata = reinterpret_cast<MetaData_t *>(hnd->base_metadata);
      if (metadata && metadata->operation & UPDATE_BUFFER_GEOMETRY) {
        *stride = metadata->bufferDim.sliceWidth;
      } else {
        *stride = hnd->width;
      }
    } break;

    // TODO(user) : this alone should be sufficient, ask gfx to get rid of above
    case GRALLOC_MODULE_PERFORM_GET_CUSTOM_STRIDE_AND_HEIGHT_FROM_HANDLE: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      int *stride = va_arg(args, int *);
      int *height = va_arg(args, int *);
      if (private_handle_t::validate(hnd) != 0) {
        return GRALLOC1_ERROR_BAD_HANDLE;
      }

      MetaData_t *metadata = reinterpret_cast<MetaData_t *>(hnd->base_metadata);
      if (metadata && metadata->operation & UPDATE_BUFFER_GEOMETRY) {
        *stride = metadata->bufferDim.sliceWidth;
        *height = metadata->bufferDim.sliceHeight;
      } else {
        *stride = hnd->width;
        *height = hnd->height;
      }
    } break;

    case GRALLOC_MODULE_PERFORM_GET_ATTRIBUTES: {
      // TODO(user): Usage is split now. take care of it from Gfx client.
      // see if we can directly expect descriptor from gfx client.
      int width = va_arg(args, int);
      int height = va_arg(args, int);
      int format = va_arg(args, int);
      uint64_t producer_usage = va_arg(args, uint64_t);
      uint64_t consumer_usage = va_arg(args, uint64_t);
      gralloc1_producer_usage_t prod_usage = static_cast<gralloc1_producer_usage_t>(producer_usage);
      gralloc1_consumer_usage_t cons_usage = static_cast<gralloc1_consumer_usage_t>(consumer_usage);

      int *aligned_width = va_arg(args, int *);
      int *aligned_height = va_arg(args, int *);
      int *tile_enabled = va_arg(args, int *);
      unsigned int alignedw, alignedh;
      BufferDescriptor descriptor(width, height, format, prod_usage, cons_usage);
      *tile_enabled = allocator_->IsUBwcEnabled(format, prod_usage, cons_usage);

      allocator_->GetAlignedWidthAndHeight(descriptor, &alignedw, &alignedh);
      *aligned_width = INT(alignedw);
      *aligned_height = INT(alignedh);
    } break;

    case GRALLOC_MODULE_PERFORM_GET_COLOR_SPACE_FROM_HANDLE: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      int *color_space = va_arg(args, int *);
      if (private_handle_t::validate(hnd) != 0) {
        return GRALLOC1_ERROR_BAD_HANDLE;
      }
      MetaData_t *metadata = reinterpret_cast<MetaData_t *>(hnd->base_metadata);
      if (!metadata) {
        return GRALLOC1_ERROR_BAD_HANDLE;
#ifdef USE_COLOR_METADATA
      } else if (metadata->operation & COLOR_METADATA) {
        ColorMetaData *colorMetadata = &metadata->color;
        switch (colorMetadata->colorPrimaries) {
        case ColorPrimaries_BT709_5:
          *color_space = HAL_CSC_ITU_R_709;
          break;
        case ColorPrimaries_BT601_6_525:
          *color_space = ((colorMetadata->range) ? HAL_CSC_ITU_R_601_FR : HAL_CSC_ITU_R_601);
           break;
        case ColorPrimaries_BT2020:
          *color_space = (colorMetadata->range) ? HAL_CSC_ITU_R_2020_FR : HAL_CSC_ITU_R_2020;
          break;
        default:
          ALOGE("Unknown Color Space = %d", colorMetadata->colorPrimaries);
          break;
        }
#endif
      } else if (metadata->operation & UPDATE_COLOR_SPACE) {
        *color_space = metadata->colorSpace;
      }
    } break;
    case GRALLOC_MODULE_PERFORM_GET_YUV_PLANE_INFO: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      android_ycbcr *ycbcr = va_arg(args, struct android_ycbcr *);
      if (private_handle_t::validate(hnd) != 0) {
        return GRALLOC1_ERROR_BAD_HANDLE;
      }
      if (allocator_->GetYUVPlaneInfo(hnd, ycbcr)) {
        return GRALLOC1_ERROR_UNDEFINED;
      }
    } break;

    case GRALLOC_MODULE_PERFORM_GET_MAP_SECURE_BUFFER_INFO: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      int *map_secure_buffer = va_arg(args, int *);
      if (private_handle_t::validate(hnd) != 0) {
        return GRALLOC1_ERROR_BAD_HANDLE;
      }
      MetaData_t *metadata = reinterpret_cast<MetaData_t *>(hnd->base_metadata);
      if (metadata && metadata->operation & MAP_SECURE_BUFFER) {
        *map_secure_buffer = metadata->mapSecureBuffer;
      } else {
        *map_secure_buffer = 0;
      }
    } break;

    case GRALLOC_MODULE_PERFORM_GET_UBWC_FLAG: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      int *flag = va_arg(args, int *);
      if (private_handle_t::validate(hnd) != 0) {
        return GRALLOC1_ERROR_BAD_HANDLE;
      }
      *flag = hnd->flags &private_handle_t::PRIV_FLAGS_UBWC_ALIGNED;
    } break;

    case GRALLOC_MODULE_PERFORM_GET_RGB_DATA_ADDRESS: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      void **rgb_data = va_arg(args, void **);
      if (private_handle_t::validate(hnd) != 0) {
        return GRALLOC1_ERROR_BAD_HANDLE;
      }
      if (allocator_->GetRgbDataAddress(hnd, rgb_data)) {
        return GRALLOC1_ERROR_UNDEFINED;
      }
    } break;

    case GRALLOC1_MODULE_PERFORM_GET_BUFFER_SIZE_AND_DIMENSIONS: {
      int width = va_arg(args, int);
      int height = va_arg(args, int);
      int format = va_arg(args, int);
      uint64_t p_usage = va_arg(args, uint64_t);
      uint64_t c_usage = va_arg(args, uint64_t);
      gralloc1_producer_usage_t producer_usage = static_cast<gralloc1_producer_usage_t>(p_usage);
      gralloc1_consumer_usage_t consumer_usage = static_cast<gralloc1_consumer_usage_t>(c_usage);
      uint32_t *aligned_width = va_arg(args, uint32_t *);
      uint32_t *aligned_height = va_arg(args, uint32_t *);
      uint32_t *size = va_arg(args, uint32_t *);
      auto descriptor = BufferDescriptor(width, height, format, producer_usage, consumer_usage);
      allocator_->GetBufferSizeAndDimensions(descriptor, size, aligned_width, aligned_height);
      // Align size
      auto align = GetDataAlignment(format, producer_usage, consumer_usage);
      *size = ALIGN(*size, align);
    } break;

      // TODO(user): Break out similar functionality, preferably moving to a common lib.

    case GRALLOC1_MODULE_PERFORM_ALLOCATE_BUFFER: {
      int width = va_arg(args, int);
      int height = va_arg(args, int);
      int format = va_arg(args, int);
      uint64_t p_usage = va_arg(args, uint64_t);
      uint64_t c_usage = va_arg(args, uint64_t);
      buffer_handle_t *hnd = va_arg(args, buffer_handle_t*);
      gralloc1_producer_usage_t producer_usage = static_cast<gralloc1_producer_usage_t>(p_usage);
      gralloc1_consumer_usage_t consumer_usage = static_cast<gralloc1_consumer_usage_t>(c_usage);
      BufferDescriptor descriptor(width, height, format, producer_usage, consumer_usage);
      unsigned int size;
      unsigned int alignedw, alignedh;
      allocator_->GetBufferSizeAndDimensions(descriptor, &size, &alignedw, &alignedh);
      AllocateBuffer(descriptor, hnd, size);
    } break;

    default:
      break;
  }
  return GRALLOC1_ERROR_NONE;
}

static bool IsYuvFormat(const private_handle_t *hnd) {
  switch (hnd->format) {
    case HAL_PIXEL_FORMAT_YCbCr_420_SP:
    case HAL_PIXEL_FORMAT_YCbCr_422_SP:
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:   // Same as YCbCr_420_SP_VENUS
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
    case HAL_PIXEL_FORMAT_YCrCb_420_SP:
    case HAL_PIXEL_FORMAT_YCrCb_422_SP:
    case HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO:
    case HAL_PIXEL_FORMAT_NV21_ZSL:
    case HAL_PIXEL_FORMAT_RAW16:
    case HAL_PIXEL_FORMAT_RAW10:
    case HAL_PIXEL_FORMAT_YV12:
      return true;
    default:
      return false;
  }
}

gralloc1_error_t BufferManager::GetNumFlexPlanes(const private_handle_t *hnd,
                                                 uint32_t *out_num_planes) {
  if (!IsYuvFormat(hnd)) {
    return GRALLOC1_ERROR_UNSUPPORTED;
  } else {
    *out_num_planes = 3;
  }
  return GRALLOC1_ERROR_NONE;
}

gralloc1_error_t BufferManager::GetFlexLayout(const private_handle_t *hnd,
                                              struct android_flex_layout *layout) {
  if (!IsYuvFormat(hnd)) {
    return GRALLOC1_ERROR_UNSUPPORTED;
  }

  android_ycbcr ycbcr;
  int err = allocator_->GetYUVPlaneInfo(hnd, &ycbcr);

  if (err != 0) {
    return GRALLOC1_ERROR_BAD_HANDLE;
  }

  layout->format = FLEX_FORMAT_YCbCr;
  layout->num_planes = 3;

  for (uint32_t i = 0; i < layout->num_planes; i++) {
    layout->planes[i].bits_per_component = 8;
    layout->planes[i].bits_used = 8;
    layout->planes[i].h_increment = 1;
    layout->planes[i].v_increment = 1;
    layout->planes[i].h_subsampling = 2;
    layout->planes[i].v_subsampling = 2;
  }

  layout->planes[0].top_left = static_cast<uint8_t *>(ycbcr.y);
  layout->planes[0].component = FLEX_COMPONENT_Y;
  layout->planes[0].v_increment = static_cast<int32_t>(ycbcr.ystride);

  layout->planes[1].top_left = static_cast<uint8_t *>(ycbcr.cb);
  layout->planes[1].component = FLEX_COMPONENT_Cb;
  layout->planes[1].h_increment = static_cast<int32_t>(ycbcr.chroma_step);
  layout->planes[1].v_increment = static_cast<int32_t>(ycbcr.cstride);

  layout->planes[2].top_left = static_cast<uint8_t *>(ycbcr.cr);
  layout->planes[2].component = FLEX_COMPONENT_Cr;
  layout->planes[2].h_increment = static_cast<int32_t>(ycbcr.chroma_step);
  layout->planes[2].v_increment = static_cast<int32_t>(ycbcr.cstride);
  return GRALLOC1_ERROR_NONE;
}
}  //  namespace gralloc1
