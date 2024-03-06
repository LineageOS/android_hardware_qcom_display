/*
 * Copyright (c) 2011-2018, 2020-2021 The Linux Foundation. All rights reserved.
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

/*
* Changes from Qualcomm Innovation Center are provided under the following license:
*
* Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the
* disclaimer below) provided that the following conditions are met:
*
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*
*    * Neither the name of Qualcomm Innovation Center, Inc. nor the
*      names of its contributors may be used to endorse or promote
*      products derived from this software without specific prior
*      written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define DEBUG 0

#include "gr_buf_mgr.h"

#include <QtiGralloc.h>
#include <QtiGrallocPriv.h>
#include <gralloctypes/Gralloc4.h>
#include <sys/mman.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <fstream>
#include "gr_adreno_info.h"
#include "gr_buf_descriptor.h"
#include "gr_priv_handle.h"
#include "gr_utils.h"
#include "qdMetaData.h"
#include "qd_utils.h"

namespace gralloc {

using aidl::android::hardware::graphics::common::BlendMode;
using aidl::android::hardware::graphics::common::Cta861_3;
using aidl::android::hardware::graphics::common::Dataspace;
using aidl::android::hardware::graphics::common::PlaneLayout;
using aidl::android::hardware::graphics::common::Rect;
using aidl::android::hardware::graphics::common::Smpte2086;
using aidl::android::hardware::graphics::common::StandardMetadataType;
using aidl::android::hardware::graphics::common::XyColor;
using ::android::hardware::graphics::common::V1_2::PixelFormat;
using IMapper_4_0_Error =  ::android::hardware::graphics::mapper::V4_0::Error;

static BufferInfo GetBufferInfo(const BufferDescriptor &descriptor) {
  return BufferInfo(descriptor.GetWidth(), descriptor.GetHeight(), descriptor.GetFormat(),
                    descriptor.GetUsage());
}

static Error dataspaceToColorMetadata(Dataspace dataspace, ColorMetaData *color_metadata) {
  ColorMetaData out;
  uint32_t primaries = (uint32_t)dataspace & (uint32_t)Dataspace::STANDARD_MASK;
  uint32_t transfer = (uint32_t)dataspace & (uint32_t)Dataspace::TRANSFER_MASK;
  uint32_t range = (uint32_t)dataspace & (uint32_t)Dataspace::RANGE_MASK;

  switch (primaries) {
    case (uint32_t)Dataspace::STANDARD_BT709:
      out.colorPrimaries = ColorPrimaries_BT709_5;
      break;
    // TODO(user): verify this is equivalent
    case (uint32_t)Dataspace::STANDARD_BT470M:
      out.colorPrimaries = ColorPrimaries_BT470_6M;
      break;
    case (uint32_t)Dataspace::STANDARD_BT601_625:
    case (uint32_t)Dataspace::STANDARD_BT601_625_UNADJUSTED:
      out.colorPrimaries = ColorPrimaries_BT601_6_625;
      break;
    case (uint32_t)Dataspace::STANDARD_BT601_525:
    case (uint32_t)Dataspace::STANDARD_BT601_525_UNADJUSTED:
      out.colorPrimaries = ColorPrimaries_BT601_6_525;
      break;
    case (uint32_t)Dataspace::STANDARD_FILM:
      out.colorPrimaries = ColorPrimaries_GenericFilm;
      break;
    case (uint32_t)Dataspace::STANDARD_BT2020:
      out.colorPrimaries = ColorPrimaries_BT2020;
      break;
    case (uint32_t)Dataspace::STANDARD_ADOBE_RGB:
      out.colorPrimaries = ColorPrimaries_AdobeRGB;
      break;
    case (uint32_t)Dataspace::STANDARD_DCI_P3:
      out.colorPrimaries = ColorPrimaries_DCIP3;
      break;
    default:
      return Error::UNSUPPORTED;
      /*
       ColorPrimaries_SMPTE_240M;
       ColorPrimaries_SMPTE_ST428;
       ColorPrimaries_EBU3213;
      */
  }

  switch (transfer) {
    case (uint32_t)Dataspace::TRANSFER_SRGB:
      out.transfer = Transfer_sRGB;
      break;
    case (uint32_t)Dataspace::TRANSFER_GAMMA2_2:
      out.transfer = Transfer_Gamma2_2;
      break;
    case (uint32_t)Dataspace::TRANSFER_GAMMA2_8:
      out.transfer = Transfer_Gamma2_8;
      break;
    case (uint32_t)Dataspace::TRANSFER_SMPTE_170M:
      out.transfer = Transfer_SMPTE_170M;
      break;
    case (uint32_t)Dataspace::TRANSFER_LINEAR:
      out.transfer = Transfer_Linear;
      break;
    case (uint32_t)Dataspace::TRANSFER_HLG:
      out.transfer = Transfer_HLG;
      break;
    default:
      return Error::UNSUPPORTED;
      /*
      Transfer_SMPTE_240M
      Transfer_Log
      Transfer_Log_Sqrt
      Transfer_XvYCC
      Transfer_BT1361
      Transfer_sYCC
      Transfer_BT2020_2_1
      Transfer_BT2020_2_2
      Transfer_SMPTE_ST2084
      Transfer_ST_428
      */
  }

  switch (range) {
    case (uint32_t)Dataspace::RANGE_FULL:
      out.range = Range_Full;
      break;
    case (uint32_t)Dataspace::RANGE_LIMITED:
      out.range = Range_Limited;
      break;
    case (uint32_t)Dataspace::RANGE_EXTENDED:
      out.range = Range_Extended;
      break;
    default:
      return Error::UNSUPPORTED;
  }

  color_metadata->colorPrimaries = out.colorPrimaries;
  color_metadata->transfer = out.transfer;
  color_metadata->range = out.range;
  return Error::NONE;
}

#ifdef QTI_YUV_PLANE_INFO
static Error getYUVPlaneInfo(const private_handle_t *hnd, struct android_ycbcr ycbcr[2]) {
  Error err = Error::NONE;
  int ret = 0;
  uint32_t width = UINT(hnd->width);
  uint32_t height = UINT(hnd->height);
  int format = hnd->format;
  uint64_t usage = hnd->usage;
  int32_t interlaced = 0;
  int plane_count = 0;
  int unaligned_width = INT(hnd->unaligned_width);
  int unaligned_height = INT(hnd->unaligned_height);
  BufferInfo info(unaligned_width, unaligned_height, format, usage);

  auto metadata = reinterpret_cast<MetaData_t *>(hnd->base_metadata);

  memset(ycbcr->reserved, 0, sizeof(ycbcr->reserved));

  // Check metadata  if UBWC buffer has been rendered in linear format
  if (metadata->isVendorMetadataSet[GET_VENDOR_METADATA_STATUS_INDEX(QTI_LINEAR_FORMAT)])
    format = metadata->linearFormat;

  // Check metadata if geometry has been updated
  if (metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(
          (int64_t)StandardMetadataType::CROP)])
    BufferInfo info(metadata->crop.right, metadata->crop.bottom, format, usage);

  if (GetAlignedWidthAndHeight(info, &width, &height)) {
    err = Error::BAD_VALUE;
    return err;
  }

  // Check metadata for interlaced content
  if (metadata->isVendorMetadataSet[GET_VENDOR_METADATA_STATUS_INDEX(QTI_PP_PARAM_INTERLACED)])
    interlaced = 1 << 0;

  PlaneLayoutInfo plane_info[8] = {};
  // Get the chroma offsets from the handle width/height. We take advantage
  // of the fact the width _is_ the stride
  ret = GetYUVPlaneInfo(info, format, width, height, interlaced, &plane_count, plane_info);
  if (ret == 0) {
    if (interlaced && format == HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC) {
      CopyPlaneLayoutInfotoAndroidYcbcr(hnd->base, plane_count, &plane_info[0], &ycbcr[0]);
      unsigned int uv_stride = 0, uv_height = 0, uv_size = 0;
      unsigned int alignment = 4096;
      uint64_t field_base;
      height = (height + 1) >> 1;
#ifndef QMAA
      uv_stride = MMM_COLOR_FMT_UV_STRIDE(MMM_COLOR_FMT_NV12_UBWC, INT(width));
      uv_height = MMM_COLOR_FMT_UV_SCANLINES(MMM_COLOR_FMT_NV12_UBWC, INT(height));
#endif
      uv_size = ALIGN((uv_stride * uv_height), alignment);
      field_base = hnd->base + plane_info[1].offset + uv_size;
      memset(ycbcr[1].reserved, 0, sizeof(ycbcr[1].reserved));
      CopyPlaneLayoutInfotoAndroidYcbcr(field_base, plane_count, &plane_info[4], &ycbcr[1]);
    } else {
      CopyPlaneLayoutInfotoAndroidYcbcr(hnd->base, plane_count, plane_info, ycbcr);
      switch (format) {
        case HAL_PIXEL_FORMAT_YCrCb_420_SP:
        case HAL_PIXEL_FORMAT_YCrCb_422_SP:
        case HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO:
        case HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS:
        case HAL_PIXEL_FORMAT_NV21_ZSL:
          std::swap(ycbcr->cb, ycbcr->cr);
      }
    }
  } else {
    err = Error::BAD_VALUE;
  }
  return err;
}
#endif

int BufferManager::GetCustomDimensions(private_handle_t *hnd, int *stride, int *height) {
  auto metadata = reinterpret_cast<MetaData_t *>(hnd->base_metadata);
  hidl_vec<uint8_t> crop;
  int32_t interlaced = 0;
  *stride = hnd->width;
  *height = hnd->height;
  if (metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(
          (int64_t)StandardMetadataType::CROP)]) {
    *stride = metadata->crop.right;
    *height = metadata->crop.bottom;
  } else if (metadata
                 ->isVendorMetadataSet[GET_VENDOR_METADATA_STATUS_INDEX(QTI_PP_PARAM_INTERLACED)]) {
    interlaced = metadata->interlaced;
    if (interlaced && IsUBwcFormat(hnd->format)) {
      uint32_t alignedw = 0, alignedh = 0;
      BufferInfo info(hnd->width, ((hnd->height + 1) >> 1), hnd->format);
      int err = GetAlignedWidthAndHeight(info, &alignedw, &alignedh);
      if (err) {
        *stride = 0;
        *height = 0;
        return err;
      }
      *stride = static_cast<int>(alignedw);
      *height = static_cast<int>(alignedh * 2);
    }
  }
  return 0;
}

BufferManager::BufferManager() : next_id_(0) {
  handles_map_.clear();
  allocator_ = new Allocator();
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

void BufferManager::SetGrallocDebugProperties(gralloc::GrallocProperties props) {
  allocator_->SetProperties(props);
  AdrenoMemInfo::GetInstance()->AdrenoSetProperties(props);
}

Error BufferManager::FreeBuffer(std::shared_ptr<Buffer> buf) {
  auto hnd = buf->handle;
  ALOGD_IF(DEBUG, "FreeBuffer handle:%p", hnd);

  if (private_handle_t::validate(hnd) != 0) {
    ALOGE("FreeBuffer: Invalid handle: %p", hnd);
    return Error::BAD_BUFFER;
  }

  auto meta_size = GetMetaDataSize(hnd->reserved_size);

  if (allocator_->FreeBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset, hnd->fd,
                             buf->ion_handle_main) != 0) {
    return Error::BAD_BUFFER;
  }

  if (allocator_->FreeBuffer(reinterpret_cast<void *>(hnd->base_metadata), meta_size,
                             hnd->offset_metadata, hnd->fd_metadata, buf->ion_handle_meta) != 0) {
    return Error::BAD_BUFFER;
  }

  private_handle_t *handle = const_cast<private_handle_t *>(hnd);
  handle->fd = -1;
  handle->fd_metadata = -1;
  if (!(handle->flags & private_handle_t::PRIV_FLAGS_CLIENT_ALLOCATED)) {
    free(handle);
  }
  return Error::NONE;
}

Error BufferManager::ValidateBufferSize(private_handle_t const *hnd, BufferInfo info) {
  unsigned int size, alignedw, alignedh;
  unsigned int max_bpp = gralloc::GetBppForUncompressedRGB(HAL_PIXEL_FORMAT_RGBA_FP16);
  info.format = GetImplDefinedFormat(info.usage, info.format);
  int ret = GetAlignedWidthAndHeight(info, &alignedw, &alignedh);
  if ((ret != 0) || (OVERFLOW((alignedw * max_bpp), alignedh))) {
    return Error::BAD_BUFFER;
  }

  ret = GetBufferSizeAndDimensions(info, &size, &alignedw, &alignedh);
  if (ret != 0) {
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

  if (hnd->base_metadata) {
#ifdef METADATA_V2
    buffer->reserved_size = hnd->reserved_size;
    if (buffer->reserved_size > 0) {
      buffer->reserved_region_ptr =
          reinterpret_cast<void *>(hnd->base_metadata + sizeof(MetaData_t));
    } else {
      buffer->reserved_region_ptr = nullptr;
    }
#else
    buffer->reserved_region_ptr = reinterpret_cast<void *>(&(metadata->reservedRegion.data));
    buffer->reserved_size = metadata->reservedRegion.size;
#endif
  }

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

  if (ValidateAndMap(hnd)) {
    ALOGE("Failed to map metadata: hnd: %p, fd:%d, id:%" PRIu64, hnd, hnd->fd, hnd->id);
    return Error::BAD_BUFFER;
  }

  RegisterHandleLocked(hnd, ion_handle, ion_handle_meta);
  allocated_ += hnd->size;
  if (allocated_ >=  kAllocThreshold) {
    kAllocThreshold += kMemoryOffset;
    BuffersDump();
  }
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
    ALOGE("Could not find handle: %p", hnd);
    return Error::BAD_BUFFER;
  } else {
    if (buf->DecRef()) {
      handles_map_.erase(hnd);
      // Unmap, close ion handle and close fd
      if (allocated_ >= hnd->size) {
        allocated_ -= hnd->size;
      }
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

Error BufferManager::FlushBuffer(const private_handle_t *handle) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  auto status = Error::NONE;

  private_handle_t *hnd = const_cast<private_handle_t *>(handle);
  auto buf = GetBufferFromHandleLocked(hnd);
  if (buf == nullptr) {
    return Error::BAD_BUFFER;
  }

  if (allocator_->CleanBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset,
                              buf->ion_handle_main, CACHE_CLEAN, hnd->fd) != 0) {
    status = Error::BAD_BUFFER;
  }

  return status;
}

Error BufferManager::RereadBuffer(const private_handle_t *handle) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  auto status = Error::NONE;

  private_handle_t *hnd = const_cast<private_handle_t *>(handle);
  auto buf = GetBufferFromHandleLocked(hnd);
  if (buf == nullptr) {
    return Error::BAD_BUFFER;
  }

  if (allocator_->CleanBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset,
                              buf->ion_handle_main, CACHE_INVALIDATE, hnd->fd) != 0) {
    status = Error::BAD_BUFFER;
  }

  return status;
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

// TODO(user): Move this once private_handle_t definition is out of QSSI
static void InitializePrivateHandle(private_handle_t *hnd, int fd, int meta_fd, int flags,
                                    int width, int height, int uw, int uh, int format, int buf_type,
                                    unsigned int size, uint64_t usage = 0) {
  hnd->fd = fd;
  hnd->fd_metadata = meta_fd;
  hnd->magic = qtigralloc::private_handle_t::kMagic;
  hnd->flags = flags;
  hnd->width = width;
  hnd->height = height;
  hnd->unaligned_width = uw;
  hnd->unaligned_height = uh;
  hnd->format = format;
  hnd->buffer_type = buf_type;
  hnd->layer_count = 1;
  hnd->id = 0;
  hnd->usage = usage;
  hnd->size = size;
  hnd->offset = 0;
  hnd->offset_metadata = 0;
  hnd->base = 0;
  hnd->base_metadata = 0;
  hnd->gpuaddr = 0;
  hnd->version = static_cast<int>(sizeof(native_handle));
  hnd->numInts = qtigralloc::private_handle_t::NumInts();
  hnd->numFds = qtigralloc::private_handle_t::kNumFds;
}

Error BufferManager::AllocateBuffer(const BufferDescriptor &descriptor, buffer_handle_t *handle,
                                    unsigned int bufferSize, bool testAlloc) {
  if (!handle)
    return Error::BAD_BUFFER;
  std::lock_guard<std::mutex> buffer_lock(buffer_lock_);

  uint64_t reserved_size = descriptor.GetReservedSize();
  if (reserved_size + sizeof(MetaData_t) + getpagesize() >= UINT32_MAX) {
    return Error::UNSUPPORTED;
  }

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

  if (testAlloc) {
    if (size == 0) {
       return Error::NO_RESOURCES;
    }
    return Error::NONE;
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
  e_data.size = static_cast<unsigned int>(GetMetaDataSize(descriptor.GetReservedSize()));
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
  // In FreeBuffer(), there's no way to tell if malloc or new was used at allocation time
  // On the importBuffer path, native_handle_clone() uses malloc
  // To avoid mismatch between malloc/free and new/delete in FreeBuffer(),
  // this was changed to malloc
  private_handle_t *hnd = static_cast<private_handle_t *>(malloc(sizeof(private_handle_t)));
  if (hnd == nullptr) {
    ALOGE("gralloc failed to allocate private_handle_t");
    return Error::NO_RESOURCES;
  }

  InitializePrivateHandle(hnd, data.fd, e_data.fd, INT(flags), INT(alignedw), INT(alignedh),
                          descriptor.GetWidth(), descriptor.GetHeight(), format, buffer_type,
                          data.size, usage);

  hnd->reserved_size = static_cast<unsigned int>(descriptor.GetReservedSize());
  hnd->id = ++next_id_;
  hnd->base = 0;
  hnd->base_metadata = 0;
  hnd->layer_count = layer_count;

  bool use_adreno_for_size = CanUseAdrenoForSize(buffer_type, usage);
  if (use_adreno_for_size) {
    setMetaDataAndUnmap(hnd, SET_GRAPHICS_METADATA, reinterpret_cast<void *>(&graphics_metadata));
  }

#ifdef METADATA_V2
  auto error = ValidateAndMap(hnd, descriptor.GetReservedSize());
#else
  auto error = ValidateAndMap(hnd);
#endif

  if (error != 0) {
    ALOGE("ValidateAndMap failed");
    return Error::BAD_BUFFER;
  }
  auto metadata = reinterpret_cast<MetaData_t *>(hnd->base_metadata);
  auto nameLength = std::min(descriptor.GetName().size(), size_t(MAX_NAME_LEN - 1));
  nameLength = descriptor.GetName().copy(metadata->name, nameLength);
  metadata->name[nameLength] = '\0';

#ifdef METADATA_V2
  metadata->reservedSize = descriptor.GetReservedSize();
#else
  metadata->reservedRegion.size =
      std::min(descriptor.GetReservedSize(), (uint64_t)RESERVED_REGION_SIZE);
#endif
  metadata->crop.top = 0;
  metadata->crop.left = 0;
  metadata->crop.right = hnd->width;
  metadata->crop.bottom = hnd->height;

  UnmapAndReset(hnd, descriptor.GetReservedSize());
  *handle = hnd;

  RegisterHandleLocked(hnd, data.ion_handle, e_data.ion_handle);
  ALOGD_IF(DEBUG, "Allocated buffer handle: %p id: %" PRIu64, hnd, hnd->id);
  if (DEBUG) {
    private_handle_t::Dump(hnd);
  }
  return Error::NONE;
}

void BufferManager:: BuffersDump() {
  char timeStamp[32];
  char hms[32];
  uint64_t millis;
  struct timeval tv;
  struct tm ptm;

  gettimeofday(&tv, NULL);
  localtime_r(&tv.tv_sec, &ptm);
  strftime (hms, sizeof (hms), "%H:%M:%S", &ptm);
  millis = tv.tv_usec / 1000;
  snprintf(timeStamp, sizeof(timeStamp), "Timestamp: %s.%03" PRIu64, hms, millis);

  std::fstream fs;
  fs.open(file_dump_.kDumpFile, std::ios::app);
  if (!fs) {
    return;
  }
  fs << "============================" << std::endl;
  fs << timeStamp << std::endl;
  fs << "Total layers = " << handles_map_.size() << std::endl;
  uint64_t totalAllocationSize = 0;
  for (auto it : handles_map_) {
    auto buf = it.second;
    auto hnd = buf->handle;
    auto metadata = reinterpret_cast<MetaData_t *>(hnd->base_metadata);
    fs  << std::setw(80) << "Client:" << (metadata ? metadata->name: "No name");
    fs  << std::setw(20) << "WxH:" << std::setw(4) << hnd->width << " x "
        << std::setw(4) << hnd->height;
    fs  << std::setw(20) << "Size: " << std::setw(9) << hnd->size <<  std::endl;
    totalAllocationSize += hnd->size;
  }
  fs << "Total allocation  = " << totalAllocationSize/1024 << "KiB" << std::endl;
  file_dump_.position = fs.tellp();
  if (file_dump_.position > (20 * 1024 * 1024)) {
    file_dump_.position = 0;
  }
  fs.close();
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

// Get list of private handles in handles_map_
Error BufferManager::GetAllHandles(std::vector<const private_handle_t *> *out_handle_list) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  if (handles_map_.empty()) {
    return Error::NO_RESOURCES;
  }
  out_handle_list->reserve(handles_map_.size());
  for (auto handle : handles_map_) {
    out_handle_list->push_back(handle.first);
  }
  return Error::NONE;
}

Error BufferManager::GetReservedRegion(private_handle_t *handle, void **reserved_region,
                                       uint64_t *reserved_region_size) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  if (!handle)
    return Error::BAD_BUFFER;

  auto buf = GetBufferFromHandleLocked(handle);
  if (buf == nullptr)
    return Error::BAD_BUFFER;
  if (!handle->base_metadata) {
    return Error::BAD_BUFFER;
  }

  *reserved_region = buf->reserved_region_ptr;
  *reserved_region_size = buf->reserved_size;

  return Error::NONE;
}

Error BufferManager::GetMetadataValue(private_handle_t *handle, int64_t metadatatype_value,
                                      void *param) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  if (!handle)
    return Error::BAD_BUFFER;
  auto buf = GetBufferFromHandleLocked(handle);
  if (buf == nullptr)
    return Error::BAD_BUFFER;

  if (!handle->base_metadata) {
    return Error::BAD_BUFFER;
  }

  auto metadata = reinterpret_cast<MetaData_t *>(handle->base_metadata);
  return GetMetaDataValue(handle, metadatatype_value, param);
}

Error BufferManager::GetMetadata(private_handle_t *handle, int64_t metadatatype_value,
                                 hidl_vec<uint8_t> *out) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  if (!handle)
    return Error::BAD_BUFFER;
  auto buf = GetBufferFromHandleLocked(handle);
  if (buf == nullptr)
    return Error::BAD_BUFFER;

  if (!handle->base_metadata) {
    return Error::BAD_BUFFER;
  }

  auto metadata = reinterpret_cast<MetaData_t *>(handle->base_metadata);

  void *metadata_ptr = nullptr;
  auto result = GetMetaDataByReference(handle, metadatatype_value, &metadata_ptr);
  Error error = Error::NONE;
  switch (metadatatype_value) {
    case (int64_t)StandardMetadataType::BUFFER_ID:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeBufferId(*reinterpret_cast<uint64_t *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case (int64_t)StandardMetadataType::NAME: {
      if (metadata_ptr != nullptr) {
        std::string name(reinterpret_cast<char *>(metadata_ptr));
        android::gralloc4::encodeName(name, out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    }
    case (int64_t)StandardMetadataType::WIDTH:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeWidth(
            static_cast<uint64_t>(*reinterpret_cast<int32_t *>(metadata_ptr)), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case (int64_t)StandardMetadataType::HEIGHT:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeHeight(
            static_cast<uint64_t>(*reinterpret_cast<int32_t *>(metadata_ptr)), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case (int64_t)StandardMetadataType::LAYER_COUNT:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeLayerCount(
            static_cast<uint64_t>(*reinterpret_cast<uint32_t *>(metadata_ptr)), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_REQUESTED:
      // TODO(user): need to return IMPLEMENTATION_DEFINED,
      // which wouldn't be known from private_handle_t
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodePixelFormatRequested(
            static_cast<PixelFormat>(*reinterpret_cast<int32_t *>(metadata_ptr)), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_FOURCC: {
      uint32_t drm_format = 0;
      uint64_t drm_format_modifier = 0;
      GetDRMFormat(handle->format, handle->flags, &drm_format, &drm_format_modifier);
      android::gralloc4::encodePixelFormatFourCC(drm_format, out);
      break;
    }
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_MODIFIER: {
      uint32_t drm_format = 0;
      uint64_t drm_format_modifier = 0;
      GetDRMFormat(handle->format, handle->flags, &drm_format, &drm_format_modifier);
      android::gralloc4::encodePixelFormatModifier(drm_format_modifier, out);
      break;
    }
    case (int64_t)StandardMetadataType::USAGE:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeUsage(*reinterpret_cast<uint64_t *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case (int64_t)StandardMetadataType::ALLOCATION_SIZE:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeAllocationSize(*reinterpret_cast<uint64_t *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case (int64_t)StandardMetadataType::PROTECTED_CONTENT: {
      // update to use metadata ptr when implemented
      uint64_t protected_content = (handle->flags & qtigralloc::PRIV_FLAGS_SECURE_BUFFER) ? 1 : 0;
      android::gralloc4::encodeProtectedContent(protected_content, out);
      break;
    }
    case (int64_t)StandardMetadataType::CHROMA_SITING:
      android::gralloc4::encodeChromaSiting(android::gralloc4::ChromaSiting_None, out);
      break;
    case (int64_t)StandardMetadataType::DATASPACE:
#ifdef METADATA_V2
      if (metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value)]) {
#endif
        Dataspace dataspace;
        ColorMetadataToDataspace(metadata->color, &dataspace);
        android::gralloc4::encodeDataspace(dataspace, out);
#ifdef METADATA_V2
      } else {
        android::gralloc4::encodeDataspace(Dataspace::UNKNOWN, out);
      }
#endif
      break;
    case (int64_t)StandardMetadataType::INTERLACED:
      if (metadata->interlaced > 0) {
        android::gralloc4::encodeInterlaced(qtigralloc::Interlaced_Qti, out);
      } else {
        android::gralloc4::encodeInterlaced(android::gralloc4::Interlaced_None, out);
      }
      break;
    case (int64_t)StandardMetadataType::COMPRESSION:
      if (handle->flags & qtigralloc::PRIV_FLAGS_UBWC_ALIGNED ||
          handle->flags & qtigralloc::PRIV_FLAGS_UBWC_ALIGNED_PI) {
        android::gralloc4::encodeCompression(qtigralloc::Compression_QtiUBWC, out);
      } else {
        android::gralloc4::encodeCompression(android::gralloc4::Compression_None, out);
      }
      break;
    case (int64_t)StandardMetadataType::PLANE_LAYOUTS: {
      std::vector<PlaneLayout> plane_layouts;
      GetPlaneLayout(handle, &plane_layouts);
      android::gralloc4::encodePlaneLayouts(plane_layouts, out);
      break;
    }
    case (int64_t)StandardMetadataType::BLEND_MODE:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeBlendMode(*reinterpret_cast<BlendMode *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case (int64_t)StandardMetadataType::SMPTE2086: {
      if (metadata->color.masteringDisplayInfo.colorVolumeSEIEnabled) {
        Smpte2086 mastering_display_values;
        mastering_display_values.primaryRed = {
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[0][0]) /
                50000.0f,
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[0][1]) /
                50000.0f};
        mastering_display_values.primaryGreen = {
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[1][0]) /
                50000.0f,
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[1][1]) /
                50000.0f};
        mastering_display_values.primaryBlue = {
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[2][0]) /
                50000.0f,
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[2][1]) /
                50000.0f};
        mastering_display_values.whitePoint = {
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.whitePoint[0]) /
                50000.0f,
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.whitePoint[1]) /
                50000.0f};
        mastering_display_values.maxLuminance =
            static_cast<float>(metadata->color.masteringDisplayInfo.maxDisplayLuminance);
        mastering_display_values.minLuminance =
            static_cast<float>(metadata->color.masteringDisplayInfo.minDisplayLuminance) / 10000.0f;
        android::gralloc4::encodeSmpte2086(mastering_display_values, out);
      } else {
        android::gralloc4::encodeSmpte2086(std::nullopt, out);
      }
      break;
    }
    case (int64_t)StandardMetadataType::CTA861_3: {
      if (metadata->color.contentLightLevel.lightLevelSEIEnabled) {
        Cta861_3 content_light_level;
        content_light_level.maxContentLightLevel =
            static_cast<float>(metadata->color.contentLightLevel.maxContentLightLevel);
        content_light_level.maxFrameAverageLightLevel =
            static_cast<float>(metadata->color.contentLightLevel.minPicAverageLightLevel) /
            10000.0f;
        android::gralloc4::encodeCta861_3(content_light_level, out);
      } else {
        android::gralloc4::encodeCta861_3(std::nullopt, out);
      }
      break;
    }
    case (int64_t)StandardMetadataType::SMPTE2094_40: {
      if (metadata->color.dynamicMetaDataValid &&
          metadata->color.dynamicMetaDataLen <= HDR_DYNAMIC_META_DATA_SZ) {
        std::vector<uint8_t> dynamic_metadata_payload;
        dynamic_metadata_payload.resize(metadata->color.dynamicMetaDataLen);
        dynamic_metadata_payload.assign(
            metadata->color.dynamicMetaDataPayload,
            metadata->color.dynamicMetaDataPayload + metadata->color.dynamicMetaDataLen);
        android::gralloc4::encodeSmpte2094_40(dynamic_metadata_payload, out);
      } else {
        android::gralloc4::encodeSmpte2094_40(std::nullopt, out);
      }
      break;
    }
    case (int64_t)StandardMetadataType::CROP: {
      // Crop is the same for all planes
      std::vector<Rect> out_crop = {{metadata->crop.left, metadata->crop.top, metadata->crop.right,
                                     metadata->crop.bottom}};
      android::gralloc4::encodeCrop(out_crop, out);
      break;
    }
    case QTI_VT_TIMESTAMP:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeUint64(qtigralloc::MetadataType_VTTimestamp,
                                        *reinterpret_cast<uint64_t *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_COLOR_METADATA:
      if (metadata_ptr != nullptr) {
        qtigralloc::encodeColorMetadata(*reinterpret_cast<ColorMetaData *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_PP_PARAM_INTERLACED:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeInt32(qtigralloc::MetadataType_PPParamInterlaced,
                                       *reinterpret_cast<int32_t *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_VIDEO_PERF_MODE:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeUint32(qtigralloc::MetadataType_VideoPerfMode,
                                        *reinterpret_cast<uint32_t *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_GRAPHICS_METADATA:
      if (metadata_ptr != nullptr) {
        qtigralloc::encodeGraphicsMetadata(*reinterpret_cast<GraphicsMetadata *>(metadata_ptr),
                                           out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_UBWC_CR_STATS_INFO:
      if (metadata_ptr != nullptr) {
        qtigralloc::encodeUBWCStats(reinterpret_cast<UBWCStats *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_REFRESH_RATE:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeFloat(qtigralloc::MetadataType_RefreshRate,
                                       *reinterpret_cast<float *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_MAP_SECURE_BUFFER:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeInt32(qtigralloc::MetadataType_MapSecureBuffer,
                                       *reinterpret_cast<int32_t *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_LINEAR_FORMAT:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeUint32(qtigralloc::MetadataType_LinearFormat,
                                        *reinterpret_cast<uint32_t *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_SINGLE_BUFFER_MODE:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeUint32(qtigralloc::MetadataType_SingleBufferMode,
                                        *reinterpret_cast<uint32_t *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_CVP_METADATA:
      if (metadata_ptr != nullptr) {
        qtigralloc::encodeCVPMetadata(*reinterpret_cast<CVPMetadata *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_VIDEO_HISTOGRAM_STATS:
      if (metadata_ptr != nullptr) {
        qtigralloc::encodeVideoHistogramMetadata(
            *reinterpret_cast<VideoHistogramMetadata *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_FD:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeInt32(qtigralloc::MetadataType_FD,
                                       *reinterpret_cast<int32_t *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_PRIVATE_FLAGS:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeInt32(qtigralloc::MetadataType_PrivateFlags,
                                       *reinterpret_cast<int32_t *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_ALIGNED_WIDTH_IN_PIXELS:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeUint32(qtigralloc::MetadataType_AlignedWidthInPixels,
                                        *reinterpret_cast<uint32_t *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_ALIGNED_HEIGHT_IN_PIXELS:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeUint32(qtigralloc::MetadataType_AlignedHeightInPixels,
                                        *reinterpret_cast<uint32_t *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
#ifdef METADATA_V2
    case QTI_STANDARD_METADATA_STATUS:
      if (metadata_ptr != nullptr) {
        qtigralloc::encodeMetadataState(reinterpret_cast<bool *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case QTI_VENDOR_METADATA_STATUS:
      if (metadata_ptr != nullptr) {
        qtigralloc::encodeMetadataState(reinterpret_cast<bool *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
#endif
#ifdef QTI_BUFFER_TYPE
    case QTI_BUFFER_TYPE:
      if (metadata_ptr != nullptr) {
        android::gralloc4::encodeUint32(qtigralloc::MetadataType_BufferType,
                                        *reinterpret_cast<uint32_t *>(metadata_ptr), out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
#endif
#ifdef QTI_VIDEO_TS_INFO
    case QTI_VIDEO_TS_INFO:
      if (metadata_ptr != nullptr) {
        qtigralloc::encodeVideoTimestampInfo(*reinterpret_cast<VideoTimestampInfo *>(metadata_ptr),
                                             out);
      } else {
        return Error::BAD_VALUE;
      }
      break;
#endif
#ifdef QTI_CUSTOM_DIMENSIONS_STRIDE
    case QTI_CUSTOM_DIMENSIONS_STRIDE: {
      int32_t stride;
      int32_t height;
      if (GetCustomDimensions(handle, &stride, &height) == 0) {
        android::gralloc4::encodeUint32(qtigralloc::MetadataType_CustomDimensionsStride, stride,
                                        out);
        break;
      } else {
        error = Error::BAD_VALUE;
        break;
      }
    }
#endif
#ifdef QTI_CUSTOM_DIMENSIONS_HEIGHT
    case QTI_CUSTOM_DIMENSIONS_HEIGHT: {
      int32_t stride = handle->width;
      int32_t height = handle->height;
      if (GetCustomDimensions(handle, &stride, &height) == 0) {
        android::gralloc4::encodeUint32(qtigralloc::MetadataType_CustomDimensionsHeight, height,
                                        out);
        break;
      } else {
        error = Error::BAD_VALUE;
        break;
      }
    }
#endif
#ifdef QTI_RGB_DATA_ADDRESS
    case QTI_RGB_DATA_ADDRESS: {
      void *rgb_data = nullptr;
      if (GetRgbDataAddress(handle, &rgb_data) == 0) {
        uint64_t addr_ptr = reinterpret_cast<std::uintptr_t>(rgb_data);
        android::gralloc4::encodeUint64(qtigralloc::MetadataType_RgbDataAddress, addr_ptr, out);
        break;
      } else {
        error = Error::BAD_BUFFER;
        break;
      }
    }
#endif
#ifdef QTI_COLORSPACE
    case QTI_COLORSPACE: {
      uint32_t colorspace;
      error = GetColorSpaceFromColorMetaData(metadata->color, &colorspace);
      if (error == Error::NONE) {
        android::gralloc4::encodeUint32(qtigralloc::MetadataType_ColorSpace, colorspace, out);
        break;
      } else {
        error = Error::BAD_BUFFER;
        break;
      }
    }
#endif
#ifdef QTI_YUV_PLANE_INFO
    case QTI_YUV_PLANE_INFO: {
      qti_ycbcr layout[2];
      android_ycbcr yuv_plane_info[2];
      error = getYUVPlaneInfo(handle, yuv_plane_info);
      if (error == Error::NONE) {
        for (int i = 0; i < 2; i++) {
          layout[i].y = yuv_plane_info[i].y;
          layout[i].cr = yuv_plane_info[i].cr;
          layout[i].cb = yuv_plane_info[i].cb;
          layout[i].yStride = static_cast<uint32_t>(yuv_plane_info[i].ystride);
          layout[i].cStride = static_cast<uint32_t>(yuv_plane_info[i].cstride);
          layout[i].chromaStep = static_cast<uint32_t>(yuv_plane_info[i].chroma_step);
        }

        uint64_t yOffset = (reinterpret_cast<uint64_t>(layout[0].y) - handle->base);
        uint64_t crOffset = (reinterpret_cast<uint64_t>(layout[0].cr) - handle->base);
        uint64_t cbOffset = (reinterpret_cast<uint64_t>(layout[0].cb) - handle->base);
        ALOGD_IF(DEBUG, " layout: y: %" PRIu64 " , cr: %" PRIu64 " , cb: %" PRIu64
              " , yStride: %d, cStride: %d, chromaStep: %d ",
              yOffset, crOffset, cbOffset, layout[0].yStride, layout[0].cStride,
              layout[0].chromaStep);

        qtigralloc::encodeYUVPlaneInfoMetadata(layout, out);
        break;
      } else {
        error = Error::BAD_BUFFER;
        break;
      }
    }
#endif

    default:
      error = Error::UNSUPPORTED;
  }

  return error;
}

Error BufferManager::SetMetadata(private_handle_t *handle, int64_t metadatatype_value,
                                 hidl_vec<uint8_t> in) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  if (!handle)
    return Error::BAD_BUFFER;

  auto buf = GetBufferFromHandleLocked(handle);
  if (buf == nullptr)
    return Error::BAD_BUFFER;

  if (!handle->base_metadata) {
    return Error::BAD_BUFFER;
  }
  if (in.size() == 0) {
    return Error::UNSUPPORTED;
  }

  auto metadata = reinterpret_cast<MetaData_t *>(handle->base_metadata);

#ifdef METADATA_V2
  // By default, set these to true
  // Reset to false for special cases below
  if (IS_VENDOR_METADATA_TYPE(metadatatype_value)) {
    if (GET_VENDOR_METADATA_STATUS_INDEX(metadatatype_value) < METADATA_SET_SIZE) {
      metadata->isVendorMetadataSet[GET_VENDOR_METADATA_STATUS_INDEX(metadatatype_value)] = true;
    }
  } else {
    if (GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value) < METADATA_SET_SIZE) {
      metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value)] =
          true;
    }
  }
#endif

  switch (metadatatype_value) {
    // These are constant (unchanged after allocation)
    case (int64_t)StandardMetadataType::BUFFER_ID:
    case (int64_t)StandardMetadataType::NAME:
    case (int64_t)StandardMetadataType::WIDTH:
    case (int64_t)StandardMetadataType::HEIGHT:
    case (int64_t)StandardMetadataType::LAYER_COUNT:
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_REQUESTED:
    case (int64_t)StandardMetadataType::USAGE:
      return Error::BAD_VALUE;
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_FOURCC:
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_MODIFIER:
    case (int64_t)StandardMetadataType::PROTECTED_CONTENT:
    case (int64_t)StandardMetadataType::ALLOCATION_SIZE:
    case (int64_t)StandardMetadataType::PLANE_LAYOUTS:
    case (int64_t)StandardMetadataType::CHROMA_SITING:
    case (int64_t)StandardMetadataType::INTERLACED:
    case (int64_t)StandardMetadataType::COMPRESSION:
    case QTI_FD:
    case QTI_PRIVATE_FLAGS:
    case QTI_ALIGNED_WIDTH_IN_PIXELS:
    case QTI_ALIGNED_HEIGHT_IN_PIXELS:
#ifdef QTI_CUSTOM_DIMENSIONS_STRIDE
    case QTI_CUSTOM_DIMENSIONS_STRIDE:
#endif
#ifdef QTI_CUSTOM_DIMENSIONS_HEIGHT
    case QTI_CUSTOM_DIMENSIONS_HEIGHT:
#endif
#ifdef QTI_RGB_DATA_ADDRESS
    case QTI_RGB_DATA_ADDRESS:
#endif
#ifdef QTI_COLORSPACE
    case QTI_COLORSPACE:
      return Error::UNSUPPORTED;
#endif
    case (int64_t)StandardMetadataType::DATASPACE:
      Dataspace dataspace;
      if (android::gralloc4::decodeDataspace(in, &dataspace)) {
        return Error::UNSUPPORTED;
      }
      dataspaceToColorMetadata(dataspace, &metadata->color);
      break;
    case (int64_t)StandardMetadataType::BLEND_MODE:
      BlendMode mode;
      android::gralloc4::decodeBlendMode(in, &mode);
      metadata->blendMode = (int32_t)mode;
      break;
    case (int64_t)StandardMetadataType::SMPTE2086: {
      std::optional<Smpte2086> mastering_display_values;
      if (android::gralloc4::decodeSmpte2086(in, &mastering_display_values)) {
        return Error::UNSUPPORTED;
      }
      if (mastering_display_values != std::nullopt) {
        metadata->color.masteringDisplayInfo.colorVolumeSEIEnabled = true;

        metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[0][0] =
            static_cast<uint32_t>(mastering_display_values->primaryRed.x * 50000.0f);
        metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[0][1] =
            static_cast<uint32_t>(mastering_display_values->primaryRed.y * 50000.0f);

        metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[1][0] =
            static_cast<uint32_t>(mastering_display_values->primaryGreen.x * 50000.0f);
        metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[1][1] =
            static_cast<uint32_t>(mastering_display_values->primaryGreen.y * 50000.0f);

        metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[2][0] =
            static_cast<uint32_t>(mastering_display_values->primaryBlue.x * 50000.0f);
        metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[2][1] =
            static_cast<uint32_t>(mastering_display_values->primaryBlue.y * 50000.0f);

        metadata->color.masteringDisplayInfo.primaries.whitePoint[0] =
            static_cast<uint32_t>(mastering_display_values->whitePoint.x * 50000.0f);
        metadata->color.masteringDisplayInfo.primaries.whitePoint[1] =
            static_cast<uint32_t>(mastering_display_values->whitePoint.y * 50000.0f);

        metadata->color.masteringDisplayInfo.maxDisplayLuminance =
            static_cast<uint32_t>(mastering_display_values->maxLuminance);
        metadata->color.masteringDisplayInfo.minDisplayLuminance =
            static_cast<uint32_t>(mastering_display_values->minLuminance * 10000.0f);
      } else {
#ifdef METADATA_V2
        metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value)] =
            false;
#endif
        metadata->color.masteringDisplayInfo.colorVolumeSEIEnabled = false;
      }
      break;
    }
    case (int64_t)StandardMetadataType::CTA861_3: {
      std::optional<Cta861_3> content_light_level;
      if (android::gralloc4::decodeCta861_3(in, &content_light_level)) {
        return Error::UNSUPPORTED;
      }
      if (content_light_level != std::nullopt) {
        metadata->color.contentLightLevel.lightLevelSEIEnabled = true;
        metadata->color.contentLightLevel.maxContentLightLevel =
            static_cast<uint32_t>(content_light_level->maxContentLightLevel);
        metadata->color.contentLightLevel.minPicAverageLightLevel =
            static_cast<uint32_t>(content_light_level->maxFrameAverageLightLevel * 10000.0f);
      } else {
#ifdef METADATA_V2
        metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value)] =
            false;
#endif
        metadata->color.contentLightLevel.lightLevelSEIEnabled = false;
      }
      break;
    }
    case (int64_t)StandardMetadataType::SMPTE2094_40: {
      std::optional<std::vector<uint8_t>> dynamic_metadata_payload;
      if (android::gralloc4::decodeSmpte2094_40(in, &dynamic_metadata_payload)) {
        return Error::UNSUPPORTED;
      }
      if (dynamic_metadata_payload != std::nullopt) {
        if (dynamic_metadata_payload->size() > HDR_DYNAMIC_META_DATA_SZ)
          return Error::BAD_VALUE;

        metadata->color.dynamicMetaDataLen = dynamic_metadata_payload->size();
        std::copy(dynamic_metadata_payload->begin(), dynamic_metadata_payload->end(),
                  metadata->color.dynamicMetaDataPayload);
        metadata->color.dynamicMetaDataValid = true;
      } else {
        // Reset metadata by passing in std::nullopt
#ifdef METADATA_V2
        metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value)] =
            false;
#endif
        metadata->color.dynamicMetaDataValid = false;
      }
      break;
    }
    case (int64_t)StandardMetadataType::CROP: {
      std::vector<Rect> in_crop;
      if (android::gralloc4::decodeCrop(in, &in_crop)) {
        return Error::UNSUPPORTED;
      }

      if (in_crop.size() != 1) {
        return Error::UNSUPPORTED;
      }

      metadata->crop.left = in_crop[0].left;
      metadata->crop.top = in_crop[0].top;
      metadata->crop.right = in_crop[0].right;
      metadata->crop.bottom = in_crop[0].bottom;
      break;
    }
    case QTI_VT_TIMESTAMP:
      if (android::gralloc4::decodeUint64(qtigralloc::MetadataType_VTTimestamp, in,
                                      &metadata->vtTimeStamp)) {
        return Error::UNSUPPORTED;
      }
      break;
    case QTI_COLOR_METADATA:
      ColorMetaData color;
      if (qtigralloc::decodeColorMetadata(in, &color) != IMapper_4_0_Error::NONE) {
        return Error::UNSUPPORTED;
      }
      metadata->color = color;
      break;
    case QTI_PP_PARAM_INTERLACED:
      if (android::gralloc4::decodeInt32(qtigralloc::MetadataType_PPParamInterlaced, in,
                                     &metadata->interlaced)) {
        return Error::UNSUPPORTED;
      }
      break;
    case QTI_VIDEO_PERF_MODE:
      if (android::gralloc4::decodeUint32(qtigralloc::MetadataType_VideoPerfMode, in,
                                      &metadata->isVideoPerfMode)) {
        return Error::UNSUPPORTED;
      }
      break;
    case QTI_GRAPHICS_METADATA:
      if (qtigralloc::decodeGraphicsMetadata(in, &metadata->graphics_metadata) !=
                                       IMapper_4_0_Error::NONE) {
        return Error::UNSUPPORTED;
      }
      break;
    case QTI_UBWC_CR_STATS_INFO:
      if (qtigralloc::decodeUBWCStats(in, &metadata->ubwcCRStats[0]) != IMapper_4_0_Error::NONE) {
        return Error::UNSUPPORTED;
      }
      break;
    case QTI_REFRESH_RATE:
      if (android::gralloc4::decodeFloat(qtigralloc::MetadataType_RefreshRate, in,
                                     &metadata->refreshrate)) {
        return Error::UNSUPPORTED;
      }
      break;
    case QTI_MAP_SECURE_BUFFER:
      if (android::gralloc4::decodeInt32(qtigralloc::MetadataType_MapSecureBuffer, in,
                                     &metadata->mapSecureBuffer)) {
        return Error::UNSUPPORTED;
      }
      break;
    case QTI_LINEAR_FORMAT:
      if (android::gralloc4::decodeUint32(qtigralloc::MetadataType_LinearFormat, in,
                                      &metadata->linearFormat)) {
        return Error::UNSUPPORTED;
      }
      break;
    case QTI_SINGLE_BUFFER_MODE:
      if (android::gralloc4::decodeUint32(qtigralloc::MetadataType_SingleBufferMode, in,
                                      &metadata->isSingleBufferMode)) {
        return Error::UNSUPPORTED;
      }
      break;
    case QTI_CVP_METADATA:
      if (qtigralloc::decodeCVPMetadata(in, &metadata->cvpMetadata) != IMapper_4_0_Error::NONE) {
        return Error::UNSUPPORTED;
      }
      break;
    case QTI_VIDEO_HISTOGRAM_STATS:
      if (qtigralloc::decodeVideoHistogramMetadata(in, &metadata->video_histogram_stats) !=
                                      IMapper_4_0_Error::NONE) {
        return Error::UNSUPPORTED;
      }
      break;
#ifdef QTI_VIDEO_TS_INFO
    case QTI_VIDEO_TS_INFO:
      if (qtigralloc::decodeVideoTimestampInfo(in, &metadata->videoTsInfo) !=
                                      IMapper_4_0_Error::NONE) {
        return Error::UNSUPPORTED;
      }
      break;
#endif
    default:
#ifdef METADATA_V2
      if (IS_VENDOR_METADATA_TYPE(metadatatype_value)) {
        if (GET_VENDOR_METADATA_STATUS_INDEX(metadatatype_value) < METADATA_SET_SIZE) {
          metadata->isVendorMetadataSet[GET_VENDOR_METADATA_STATUS_INDEX(metadatatype_value)] =
              false;
        }
      } else {
        if (GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value) < METADATA_SET_SIZE) {
          metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value)] =
              false;
        }
      }
#endif
      return Error::BAD_VALUE;
  }

  return Error::NONE;
}

}  //  namespace gralloc
