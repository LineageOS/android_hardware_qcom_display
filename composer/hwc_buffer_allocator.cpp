/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

/*
* Changes from Qualcomm Innovation Center are provided under the following license:
*
* Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
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
*    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
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

#include <QtiGralloc.h>

#include <core/buffer_allocator.h>
#include <utils/constants.h>
#include <utils/debug.h>

#include "gr_utils.h"
#include "hwc_buffer_allocator.h"
#include "hwc_debugger.h"
#include "hwc_layers.h"

#define __CLASS__ "HWCBufferAllocator"

using android::hardware::hidl_handle;
using android::hardware::hidl_vec;
using android::hardware::graphics::common::V1_2::PixelFormat;
using android::hardware::graphics::mapper::V4_0::BufferDescriptor;
using android::hardware::graphics::mapper::V4_0::Error;
using MapperExtError = vendor::qti::hardware::display::mapperextensions::V1_0::Error;
using vendor::qti::hardware::display::mapper::V4_0::IQtiMapper;

namespace sdm {

int HWCBufferAllocator::GetGrallocInstance() {
  // Lazy initialization of gralloc HALs
  if (mapper_ != nullptr || allocator_ != nullptr) {
    return kErrorNone;
  }

  allocator_ = IAllocator::getService();
  if (allocator_ == nullptr) {
    DLOGE("Unable to get allocator");
    return kErrorCriticalResource;
  }

  mapper_ = IMapper::getService();
  if (mapper_ == nullptr) {
    DLOGE("Unable to get mapper");
    return kErrorCriticalResource;
  }

  android::sp<IQtiMapper> qti_mapper = IQtiMapper::castFrom(mapper_);
  qti_mapper->getMapperExtensions([&](auto _error, auto _extensions) {
    if (_error == Error::NONE)
      mapper_ext_ = _extensions;
  });

  if (mapper_ext_ == nullptr) {
    DLOGE("Unable to get mapper extensions");
    return kErrorCriticalResource;
  }

  return 0;
}

int HWCBufferAllocator::AllocateBuffer(BufferInfo *buffer_info) {
  auto err = GetGrallocInstance();
  if (err != 0) {
    return err;
  }
  const BufferConfig &buffer_config = buffer_info->buffer_config;
  AllocatedBufferInfo *alloc_buffer_info = &buffer_info->alloc_buffer_info;
  int format;
  uint64_t alloc_flags = 0;
  int error = SetBufferInfo(buffer_config.format, &format, &alloc_flags);
  if (error != 0) {
    return -EINVAL;
  }

  if (buffer_config.secure) {
    alloc_flags |= BufferUsage::PROTECTED;
  }

  if (buffer_config.secure_camera) {
    alloc_flags |= BufferUsage::CAMERA_OUTPUT;
  }

  if (!buffer_config.cache) {
    // Allocate uncached buffers
    alloc_flags |= GRALLOC_USAGE_PRIVATE_UNCACHED;
  }

  if (buffer_config.trusted_ui) {
    // Allocate cached buffers for trusted UI
    alloc_flags |= GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY;
    alloc_flags &= ~GRALLOC_USAGE_PRIVATE_UNCACHED;
  }

  if (buffer_config.gfx_client) {
    alloc_flags |= BufferUsage::GPU_TEXTURE;
  }

  alloc_flags |= BufferUsage::COMPOSER_OVERLAY;

  const native_handle_t *buf = nullptr;

  IMapper::BufferDescriptorInfo descriptor_info;
  descriptor_info.width = buffer_config.width;
  descriptor_info.height = buffer_config.height;
  descriptor_info.layerCount = 1;
  descriptor_info.format =
      static_cast<android::hardware::graphics::common::V1_2::PixelFormat>(format);
  descriptor_info.usage = alloc_flags;

  auto hidl_err = Error::NONE;

  auto descriptor = BufferDescriptor();
  mapper_->createDescriptor(descriptor_info, [&](const auto &_error, const auto &_descriptor) {
    hidl_err = _error;
    descriptor = _descriptor;
  });

  if (hidl_err != Error::NONE) {
    DLOGE("Failed to create descriptor");
    return kErrorMemory;
  }

  hidl_handle raw_handle = nullptr;

  allocator_->allocate(descriptor, 1,
                       [&](const auto &_error, const auto &_stride, const auto &_buffers) {
                         if (_error != Error::NONE) {
                           return;
                         }
                         hidl_err = _error;
                         raw_handle = _buffers[0];
                       });

  if (hidl_err != Error::NONE) {
    DLOGE("Failed to allocate buffer");
    return kErrorMemory;
  }

  mapper_->importBuffer(raw_handle, [&](const auto &_error, const auto &_buffer) {
    hidl_err = _error;
    buf = static_cast<const native_handle_t *>(_buffer);
  });

  if (hidl_err != Error::NONE) {
    DLOGE("Failed to import buffer into HWC");
    return kErrorMemory;
  }

  native_handle_t *hnd = nullptr;
  hnd = (native_handle_t *)buf;  // NOLINT

  err = GetFd(hnd, alloc_buffer_info->fd);
  if (err != kErrorNone)
    return kErrorUndefined;

  uint32_t tmp_width;
  err = GetWidth(hnd, tmp_width);
  if (err != kErrorNone)
    return kErrorUndefined;
  alloc_buffer_info->stride = tmp_width;
  alloc_buffer_info->aligned_width = tmp_width;

  err = GetHeight(hnd, alloc_buffer_info->aligned_height);
  if (err != kErrorNone)
    return kErrorUndefined;

  err = GetAllocationSize(hnd, alloc_buffer_info->size);
  if (err != kErrorNone)
    return kErrorUndefined;

  err = GetBufferId(hnd, alloc_buffer_info->id);
  if (err != kErrorNone)
    return kErrorUndefined;

  err = GetSDMFormat(hnd, alloc_buffer_info->format);
  if (err != kErrorNone)
    return kErrorUndefined;

  buffer_info->private_data = reinterpret_cast<void *>(hnd);
  return 0;
}

int HWCBufferAllocator::FreeBuffer(BufferInfo *buffer_info) {
  int err = 0;
  auto hnd = reinterpret_cast<void *>(buffer_info->private_data);
  mapper_->freeBuffer(hnd);

  AllocatedBufferInfo &alloc_buffer_info = buffer_info->alloc_buffer_info;

  alloc_buffer_info.fd = -1;
  alloc_buffer_info.stride = 0;
  alloc_buffer_info.size = 0;
  buffer_info->private_data = NULL;
  return err;
}

int HWCBufferAllocator::GetHeight(void *buf, uint32_t &height) {
  uint32_t tmp_height;
  auto err = qtigralloc::get(buf, QTI_ALIGNED_HEIGHT_IN_PIXELS, &tmp_height);
  if (err == Error::NONE) {
    height = tmp_height;
    return kErrorNone;
  }
  return kErrorParameters;
}

int HWCBufferAllocator::GetWidth(void *buf, uint32_t &width) {
  uint32_t tmp_width;
  auto err = qtigralloc::get(buf, QTI_ALIGNED_WIDTH_IN_PIXELS, &tmp_width);
  if (err == Error::NONE) {
    width = tmp_width;
    return kErrorNone;
  }
  return kErrorParameters;
}

int HWCBufferAllocator::GetUnalignedHeight(void *buf, uint32_t &height) {
  uint64_t tmp_height;
  auto err = Error::UNSUPPORTED;
  mapper_->get(
      buf, android::gralloc4::MetadataType_Height, [&](const auto _error, const auto _bytestream) {
        if (_error == Error::NONE)
          err = static_cast<Error>(android::gralloc4::decodeHeight(_bytestream, &tmp_height));
      });
  if (err == Error::NONE) {
    height = static_cast<uint32_t>(tmp_height);
    return kErrorNone;
  }
  return kErrorParameters;
}

int HWCBufferAllocator::GetUnalignedWidth(void *buf, uint32_t &width) {
  uint64_t tmp_width;
  auto err = Error::UNSUPPORTED;
  mapper_->get(
      buf, android::gralloc4::MetadataType_Width, [&](const auto _error, const auto _bytestream) {
        if (_error == Error::NONE)
          err = static_cast<Error>(android::gralloc4::decodeWidth(_bytestream, &tmp_width));
      });
  if (err == Error::NONE) {
    width = static_cast<uint32_t>(tmp_width);
    return kErrorNone;
  }
  return kErrorParameters;
}

int HWCBufferAllocator::GetFd(void *buf, int &fd) {
  int tmp_fd;
  auto err = qtigralloc::get(buf, QTI_FD, &tmp_fd);
  if (err == Error::NONE) {
    fd = tmp_fd;
    return kErrorNone;
  }
  return kErrorParameters;
}

int HWCBufferAllocator::GetAllocationSize(void *buf, uint32_t &alloc_size) {
  uint64_t tmp_alloc_size;
  auto err = Error::UNSUPPORTED;
  mapper_->get(buf, android::gralloc4::MetadataType_AllocationSize,
               [&](const auto _error, const auto _bytestream) {
                 if (_error == Error::NONE)
                   err = static_cast<Error>(
                       android::gralloc4::decodeAllocationSize(_bytestream, &tmp_alloc_size));
               });
  if (err == Error::NONE) {
    alloc_size = static_cast<uint32_t>(tmp_alloc_size);
    return kErrorNone;
  }
  return kErrorParameters;
}

int HWCBufferAllocator::GetBufferId(void *buf, uint64_t &id) {
  uint64_t tmp_id;
  auto err = Error::UNSUPPORTED;
  mapper_->get(buf, android::gralloc4::MetadataType_BufferId,
               [&](const auto _error, const auto _bytestream) {
                 if (_error == Error::NONE)
                   err =
                       static_cast<Error>(android::gralloc4::decodeBufferId(_bytestream, &tmp_id));
               });
  if (err == Error::NONE) {
    id = tmp_id;
    return kErrorNone;
  }
  return kErrorParameters;
}

int HWCBufferAllocator::GetFormat(void *buf, int32_t &format) {
  PixelFormat pixel_format;
  auto err = Error::UNSUPPORTED;
  mapper_->get(buf, android::gralloc4::MetadataType_PixelFormatRequested,
               [&](const auto _error, const auto _bytestream) {
                 if (_error == Error::NONE)
                   err = static_cast<Error>(
                       android::gralloc4::decodePixelFormatRequested(_bytestream, &pixel_format));
               });
  if (err == Error::NONE) {
    format = static_cast<int32_t>(pixel_format);
    return kErrorNone;
  }
  return kErrorParameters;
}

int HWCBufferAllocator::GetPrivateFlags(void *buf, int32_t &flags) {
  int32_t tmp_flags;
  auto err = qtigralloc::get(buf, QTI_PRIVATE_FLAGS, &tmp_flags);
  if (err == Error::NONE) {
    flags = tmp_flags;
    return kErrorNone;
  }
  return kErrorParameters;
}

int HWCBufferAllocator::GetSDMFormat(void *buf, LayerBufferFormat &sdm_format) {
  int32_t tmp_format, tmp_flags, err;
  err = GetFormat(buf, tmp_format);
  if (err != kErrorNone)
    return kErrorUndefined;

  err = GetPrivateFlags(buf, tmp_flags);
  if (err != kErrorNone)
    return kErrorUndefined;

  sdm_format = HWCLayer::GetSDMFormat(tmp_format, tmp_flags);
  return kErrorNone;
}

int HWCBufferAllocator::GetBufferType(void *buf, uint32_t &buffer_type) {
  int32_t tmp_buffer_type;
  auto err = qtigralloc::get(buf, QTI_BUFFER_TYPE, &tmp_buffer_type);
  if (err == Error::NONE) {
    buffer_type = tmp_buffer_type;
    return kErrorNone;
  }
  return kErrorParameters;
}

int HWCBufferAllocator::GetBufferGeometry(void *buf, int32_t &slice_width, int32_t &slice_height) {
  auto err = Error::UNSUPPORTED;
  std::vector<aidl::android::hardware::graphics::common::Rect> tmp_crop;
  mapper_->get(buf, android::gralloc4::MetadataType_Crop,
               [&](const auto _error, const auto _bytestream) {
                 if (_error == Error::NONE)
                   err = static_cast<Error>(android::gralloc4::decodeCrop(_bytestream, &tmp_crop));
               });
  if (err == Error::NONE) {
    slice_width = tmp_crop[0].right;
    slice_height = tmp_crop[0].bottom;
    return kErrorNone;
  }
  return kErrorParameters;
}

void HWCBufferAllocator::GetCustomWidthAndHeight(const native_handle_t *handle, int *width,
                                                 int *height) {
  void *hnd = const_cast<native_handle_t *>(handle);

  auto err = GetGrallocInstance();
  if (err != 0) {
    DLOGE("Failed to retrieve gralloc instance");
  }

  mapper_ext_->getCustomDimensions(hnd, [&](MapperExtError _error, auto _width, auto _height) {
    if (_error == MapperExtError::NONE) {
      *width = _width;
      *height = _height;
    }
  });
}

void HWCBufferAllocator::GetAdjustedWidthAndHeight(const private_handle_t *handle, int *width,
                                                 int *height) {
  *width = handle->width;
  *height = handle->height;
  gralloc::GetCustomDimensions(const_cast<private_handle_t *>(handle), width, height);
}

void HWCBufferAllocator::GetAlignedWidthAndHeight(int width, int height, int format,
                                                  uint32_t alloc_type, int *aligned_width,
                                                  int *aligned_height) {
  uint64_t usage = 0;
  if (alloc_type & GRALLOC_USAGE_HW_FB) {
    usage |= BufferUsage::COMPOSER_CLIENT_TARGET;
  }
  if (alloc_type & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC) {
    usage |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
  }
  *aligned_width = UINT(width);
  *aligned_height = UINT(height);

  auto err = GetGrallocInstance();
  if (err != 0) {
    DLOGE("Failed to retrieve gralloc instance");
  }

  mapper_ext_->calculateBufferAttributes(
      width, height, format, usage,
      [&](MapperExtError _error, auto _aligned_w, auto _aligned_h, auto _ubwc_enabled) {
        if (_error == MapperExtError::NONE) {
          *aligned_width = _aligned_w;
          *aligned_height = _aligned_h;
        }
      });
}

uint32_t HWCBufferAllocator::GetBufferSize(BufferInfo *buffer_info) {
  const BufferConfig &buffer_config = buffer_info->buffer_config;
  uint64_t alloc_flags = GRALLOC_USAGE_PRIVATE_IOMMU_HEAP;

  int width = INT(buffer_config.width);
  int height = INT(buffer_config.height);
  int format;

  if (buffer_config.secure) {
    alloc_flags |= INT(GRALLOC_USAGE_PROTECTED);
  }

  if (!buffer_config.cache) {
    // Allocate uncached buffers
    alloc_flags |= GRALLOC_USAGE_PRIVATE_UNCACHED;
  }

  if (SetBufferInfo(buffer_config.format, &format, &alloc_flags) < 0) {
    return 0;
  }

  uint32_t aligned_width = 0, aligned_height = 0, buffer_size = 0;
  // TODO(user): Replace with getFromBufferDescriptorInfo
  gralloc::BufferInfo info(width, height, format, alloc_flags);
  int ret = GetBufferSizeAndDimensions(info, &buffer_size, &aligned_width, &aligned_height);
  if (ret < 0) {
    return 0;
  }
  return buffer_size;
}

int HWCBufferAllocator::SetBufferInfo(LayerBufferFormat format, int *target, uint64_t *flags) {
  switch (format) {
    case kFormatRGBA8888:
      *target = HAL_PIXEL_FORMAT_RGBA_8888;
      break;
    case kFormatRGBX8888:
      *target = HAL_PIXEL_FORMAT_RGBX_8888;
      break;
    case kFormatRGB888:
      *target = HAL_PIXEL_FORMAT_RGB_888;
      break;
    case kFormatRGB565:
      *target = HAL_PIXEL_FORMAT_RGB_565;
      break;
    case kFormatBGR565:
      *target = HAL_PIXEL_FORMAT_BGR_565;
      break;
    case kFormatBGR888:
      *target = HAL_PIXEL_FORMAT_BGR_888;
      break;
    case kFormatBGRA8888:
    case kFormatARGB8888:
      *target = HAL_PIXEL_FORMAT_BGRA_8888;
      break;
    case kFormatYCrCb420PlanarStride16:
      *target = HAL_PIXEL_FORMAT_YV12;
      break;
    case kFormatYCrCb420SemiPlanar:
      *target = HAL_PIXEL_FORMAT_YCrCb_420_SP;
      break;
    case kFormatYCbCr420SemiPlanar:
      *target = HAL_PIXEL_FORMAT_YCbCr_420_SP;
      break;
    case kFormatYCbCr422H2V1Packed:
      *target = HAL_PIXEL_FORMAT_YCbCr_422_I;
      break;
    case kFormatCbYCrY422H2V1Packed:
      *target = HAL_PIXEL_FORMAT_CbYCrY_422_I;
      break;
    case kFormatYCbCr422H2V1SemiPlanar:
      *target = HAL_PIXEL_FORMAT_YCbCr_422_SP;
      break;
    case kFormatYCbCr420SemiPlanarVenus:
      *target = HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS;
      break;
    case kFormatYCrCb420SemiPlanarVenus:
      *target = HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS;
      break;
    case kFormatYCbCr420SPVenusUbwc:
    case kFormatYCbCr420SPVenusTile:
      *target = HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatRGBA5551:
      *target = HAL_PIXEL_FORMAT_RGBA_5551;
      break;
    case kFormatRGBA4444:
      *target = HAL_PIXEL_FORMAT_RGBA_4444;
      break;
    case kFormatRGBA1010102:
      *target = HAL_PIXEL_FORMAT_RGBA_1010102;
      break;
    case kFormatARGB2101010:
      *target = HAL_PIXEL_FORMAT_ARGB_2101010;
      break;
    case kFormatRGBX1010102:
      *target = HAL_PIXEL_FORMAT_RGBX_1010102;
      break;
    case kFormatXRGB2101010:
      *target = HAL_PIXEL_FORMAT_XRGB_2101010;
      break;
    case kFormatBGRA1010102:
      *target = HAL_PIXEL_FORMAT_BGRA_1010102;
      break;
    case kFormatABGR2101010:
      *target = HAL_PIXEL_FORMAT_ABGR_2101010;
      break;
    case kFormatBGRX1010102:
      *target = HAL_PIXEL_FORMAT_BGRX_1010102;
      break;
    case kFormatXBGR2101010:
      *target = HAL_PIXEL_FORMAT_XBGR_2101010;
      break;
    case kFormatYCbCr420P010:
      *target = HAL_PIXEL_FORMAT_YCbCr_420_P010;
      break;
    case kFormatYCbCr420TP10Ubwc:
    case kFormatYCbCr420TP10Tile:
      *target = HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatYCbCr420P010Ubwc:
    case kFormatYCbCr420P010Tile:
      *target = HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatYCbCr420P010Venus:
      *target = HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS;
      break;
    case kFormatRGBA8888Ubwc:
      *target = HAL_PIXEL_FORMAT_RGBA_8888;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatRGBX8888Ubwc:
      *target = HAL_PIXEL_FORMAT_RGBX_8888;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatBGR565Ubwc:
      *target = HAL_PIXEL_FORMAT_BGR_565;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatRGBA1010102Ubwc:
      *target = HAL_PIXEL_FORMAT_RGBA_1010102;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatRGBX1010102Ubwc:
      *target = HAL_PIXEL_FORMAT_RGBX_1010102;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatBlob:
      *target = HAL_PIXEL_FORMAT_BLOB;
      break;
    default:
      DLOGW("Unsupported format = 0x%x", format);
      return -EINVAL;
  }
  return 0;
}

int HWCBufferAllocator::GetAllocatedBufferInfo(
    const BufferConfig &buffer_config, AllocatedBufferInfo *allocated_buffer_info) {
  // TODO(user): This API should pass the buffer_info of the already allocated buffer
  // The private_data can then be typecast to the private_handle and used directly.
  uint64_t alloc_flags = GRALLOC_USAGE_PRIVATE_IOMMU_HEAP;

  int width = INT(buffer_config.width);
  int height = INT(buffer_config.height);
  int format;

  if (buffer_config.secure) {
    alloc_flags |= INT(GRALLOC_USAGE_PROTECTED);
  }

  if (!buffer_config.cache) {
    // Allocate uncached buffers
    alloc_flags |= GRALLOC_USAGE_PRIVATE_UNCACHED;
  }

  if (SetBufferInfo(buffer_config.format, &format, &alloc_flags) < 0) {
    return -EINVAL;
  }

  uint32_t aligned_width = 0, aligned_height = 0, buffer_size = 0;
  // TODO(user): Replace with getFromBufferDescriptorInfo
  gralloc::BufferInfo info(width, height, format, alloc_flags);
  int ret = GetBufferSizeAndDimensions(info, &buffer_size, &aligned_width, &aligned_height);
  if (ret < 0) {
    return -EINVAL;
  }
  allocated_buffer_info->stride = UINT32(aligned_width);
  allocated_buffer_info->aligned_width = UINT32(aligned_width);
  allocated_buffer_info->aligned_height = UINT32(aligned_height);
  allocated_buffer_info->size = UINT32(buffer_size);

  return 0;
}

int HWCBufferAllocator::GetBufferLayout(const AllocatedBufferInfo &buf_info,
                                                 uint32_t stride[4], uint32_t offset[4],
                                                 uint32_t *num_planes) {

  // TODO(user): Transition APIs to not need a private handle
  private_handle_t hnd(-1, 0, 0, 0, 0, 0, 0);
  int format = HAL_PIXEL_FORMAT_RGBA_8888;
  uint64_t flags = 0;
  SetBufferInfo(buf_info.format, &format, &flags);
  // Setup only the required stuff, skip rest
  hnd.format = format;
  hnd.width = INT32(buf_info.aligned_width);
  hnd.height = INT32(buf_info.aligned_height);
  if (flags & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC) {
    hnd.flags = private_handle_t::PRIV_FLAGS_UBWC_ALIGNED;
  }
  hnd.usage = buf_info.usage;
  int ret = gralloc::GetBufferLayout(&hnd, stride, offset, num_planes);
  if (ret < 0) {
    DLOGE("GetBufferLayout failed");
  }

  return kErrorNone;
}

int HWCBufferAllocator::MapBuffer(const native_handle_t *handle, shared_ptr<Fence> acquire_fence,
                                  void **base_ptr) {
  auto err = GetGrallocInstance();
  if (err != 0) {
    DLOGW("Could not get gralloc instance");
    return err;
  }

  Fence::ScopedRef scoped_ref;
  NATIVE_HANDLE_DECLARE_STORAGE(acquire_fence_storage, 1, 0);
  hidl_handle acquire_fence_handle;
  if (acquire_fence) {
    auto h = native_handle_init(acquire_fence_storage, 1, 0);
    h->data[0] = scoped_ref.Get(acquire_fence);
    acquire_fence_handle = h;
  }

  auto hnd = const_cast<native_handle_t *>(handle);
  *base_ptr = NULL;
  const IMapper::Rect access_region = {.left = 0, .top = 0, .width = 0, .height = 0};
  mapper_->lock(reinterpret_cast<void *>(hnd), (uint64_t)BufferUsage::CPU_READ_OFTEN, access_region,
                acquire_fence_handle, [&](const auto &_error, const auto &_buffer) {
                  if (_error == Error::NONE) {
                    *base_ptr = _buffer;
                  }
                });

  if (!*base_ptr) {
    DLOGW("*base_ptr is NULL!");
    return kErrorUndefined;
  }

  return kErrorNone;
}

int HWCBufferAllocator::UnmapBuffer(const native_handle_t *handle, int *release_fence) {
  int err = kErrorNone;
  *release_fence = -1;
  auto hnd = const_cast<native_handle_t *>(handle);
  mapper_->unlock(reinterpret_cast<void *>(hnd),
                  [&](const auto &_error, const auto &_release_fence) {
                    if (_error != Error::NONE) {
                      err = -EINVAL;
                    }
                  });
  return err;
}

}  // namespace sdm
