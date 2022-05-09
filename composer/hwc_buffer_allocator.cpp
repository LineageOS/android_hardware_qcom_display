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
 *
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <QtiGralloc.h>

#include <gralloctypes/Gralloc4.h>
#include <core/buffer_allocator.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <gr_utils.h>

#include "hwc_buffer_allocator.h"
#include "hwc_debugger.h"
#include "hwc_layers.h"

#define __CLASS__ "HWCBufferAllocator"

using android::hardware::hidl_handle;
using android::hardware::hidl_vec;
using vendor::qti::hardware::display::mapperextensions::V1_0::PlaneLayout;
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
      mapper_ext_ = IQtiMapperExtensions_v1_3::castFrom(_extensions);
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
  BufferPermission buf_perm[BUFFER_CLIENT_MAX];
  int format;
  uint64_t alloc_flags = 0;
  int error = SetBufferInfo(buffer_config.format, &format, &alloc_flags);
  if (error != 0) {
    return -EINVAL;
  }

  if (buffer_config.access_control.empty()) {
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
      alloc_flags |= GRALLOC_USAGE_PRIVATE_TRUSTED_VM;
      alloc_flags &= ~GRALLOC_USAGE_PRIVATE_UNCACHED;
    }

    if (buffer_config.gfx_client) {
      alloc_flags |= BufferUsage::GPU_TEXTURE;
    }

    alloc_flags |= BufferUsage::COMPOSER_OVERLAY;
  } else {
    for (uint32_t i = 0; i < BUFFER_CLIENT_MAX; i++) {
      buf_perm[i].permission = 0;
    }
    auto it = buffer_config.access_control.find(kBufferClientUnTrustedVM);
    if (it != buffer_config.access_control.end()) {
      if (!buffer_config.cache) {
        // Allocate uncached buffers
        alloc_flags |= GRALLOC_USAGE_PRIVATE_UNCACHED;
      }
      SetBufferAccessControlInfo(it->second, &buf_perm[kBufferClientUnTrustedVM]);
    } else {
       alloc_flags |= BufferUsage::PROTECTED;
    }

    it = buffer_config.access_control.find(kBufferClientTrustedVM);
    if (it != buffer_config.access_control.end()) {
      alloc_flags |= GRALLOC_USAGE_PRIVATE_TRUSTED_VM;
      SetBufferAccessControlInfo(it->second, &buf_perm[kBufferClientTrustedVM]);
    }

    it = buffer_config.access_control.find(kBufferClientDPU);
    if (it != buffer_config.access_control.end()) {
      alloc_flags |= BufferUsage::COMPOSER_OVERLAY;
      SetBufferAccessControlInfo(it->second, &buf_perm[kBufferClientDPU]);
    }
  }

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

  uint32_t tmp_width;
  native_handle_t *hnd = nullptr;
  hnd = (native_handle_t *)buf;  // NOLINT

  if (!buffer_config.access_control.empty()) {
    auto ret = qtigralloc::set(hnd, QTI_BUFFER_PERMISSION, buf_perm);
    if (ret != Error::NONE) {
      DLOGE("qtigralloc::set failed for QTI_BUFFER_PERMISSION %d", ret);
      err = -EINVAL;
      goto cleanup;
    }
    auto error = gralloc::GetMetaDataValue(hnd, QTI_MEM_HANDLE, &alloc_buffer_info->mem_handle);
    if (error != gralloc::Error::NONE) {
      err = -EINVAL;;
      goto cleanup;
    }
  }

  err = GetFd(hnd, alloc_buffer_info->fd);
  if (err != kErrorNone)
    goto cleanup;

  err = GetWidth(hnd, tmp_width);
  if (err != kErrorNone)
    goto cleanup;
  alloc_buffer_info->stride = tmp_width;
  alloc_buffer_info->aligned_width = tmp_width;

  err = GetHeight(hnd, alloc_buffer_info->aligned_height);
  if (err != kErrorNone)
    goto cleanup;

  err = GetAllocationSize(hnd, alloc_buffer_info->size);
  if (err != kErrorNone)
    goto cleanup;

  err = GetBufferId(hnd, alloc_buffer_info->id);
  if (err != kErrorNone)
    goto cleanup;

  err = GetSDMFormat(hnd, alloc_buffer_info->format);
  if (err != kErrorNone)
    goto cleanup;

  buffer_info->private_data = reinterpret_cast<void *>(hnd);
  return 0;
cleanup:
  if (hnd) {
    mapper_->freeBuffer(hnd);
  }
  return err;
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

int HWCBufferAllocator::GetCustomWidthAndHeight(const native_handle_t *handle, int *width,
                                                int *height) {
  void *hnd = const_cast<native_handle_t *>(handle);

  gralloc::GetMetaDataValue(hnd, QTI_ALIGNED_WIDTH_IN_PIXELS, width);
  gralloc::GetMetaDataValue(hnd, QTI_ALIGNED_HEIGHT_IN_PIXELS, height);

  auto err = GetGrallocInstance();
  if (err != 0) {
    DLOGE("Failed to retrieve gralloc instance");
  }

  mapper_ext_->getCustomDimensions(hnd, [&](MapperExtError _error, auto _width, auto _height) {
    if (_error == MapperExtError::NONE) {
      *width = _width;
      *height = _height;
    } else {
      err = -EINVAL;
    }
  });

  return err;
}

int HWCBufferAllocator::GetAlignedWidthAndHeight(int width, int height, int format,
                                                 uint32_t alloc_type, int *aligned_width,
                                                 int *aligned_height) {
  uint64_t usage = 0;
  if (alloc_type & GRALLOC_USAGE_HW_FB) {
    usage |= BufferUsage::COMPOSER_CLIENT_TARGET;
  }
  if (alloc_type & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC) {
    usage |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
  }
  *aligned_width = static_cast<unsigned int>(width);
  *aligned_height = static_cast<unsigned int>(height);

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
        } else {
          err = -EINVAL;
        }
      });

  return err;
}

uint32_t HWCBufferAllocator::GetBufferSize(BufferInfo *buffer_info) {
  const BufferConfig &buffer_config = buffer_info->buffer_config;
  uint64_t alloc_flags = GRALLOC_USAGE_PRIVATE_IOMMU_HEAP;
  auto err = GetGrallocInstance();
  if (err != 0) {
    DLOGW("Could not get gralloc instance");
    return err;
  }

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

  uint64_t buffer_size = 0;
  IMapper::BufferDescriptorInfo info;
  info.width = width;
  info.height = height;
  info.format = static_cast<PixelFormat>(format);
  info.usage = alloc_flags;

  auto error = Error::UNSUPPORTED;
  mapper_->getFromBufferDescriptorInfo(
      info, android::gralloc4::MetadataType_AllocationSize,
      [&](const auto _error, const auto _bytestream) {
        if (_error == Error::NONE)
          error = static_cast<Error>(
              android::gralloc4::decodeAllocationSize(_bytestream, &buffer_size));
      });
  if (error == Error::NONE) {
    return static_cast<uint32_t>(buffer_size);
  }
  return 0;
}

int HWCBufferAllocator::SetBufferInfo(LayerBufferFormat format, int *target, uint64_t *flags) {
  switch (format) {
    case kFormatRGBA8888:
      *target = static_cast<int>(PixelFormat::RGBA_8888);
      break;
    case kFormatRGBX8888:
      *target = static_cast<int>(PixelFormat::RGBX_8888);
      break;
    case kFormatRGB888:
      *target = static_cast<int>(PixelFormat::RGB_888);
      break;
    case kFormatRGB565:
      *target = static_cast<int>(PixelFormat::RGB_565);
      break;
    case kFormatBGR565:
      *target = HAL_PIXEL_FORMAT_BGR_565;
      break;
    case kFormatBGR888:
      *target = HAL_PIXEL_FORMAT_BGR_888;
      break;
    case kFormatBGRA8888:
    case kFormatARGB8888:
      *target = static_cast<int>(PixelFormat::BGRA_8888);
      break;
    case kFormatYCrCb420PlanarStride16:
      *target = static_cast<int>(PixelFormat::YV12);
      break;
    case kFormatYCrCb420SemiPlanar:
      *target = static_cast<int>(PixelFormat::YCRCB_420_SP);
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
      *target = static_cast<int>(PixelFormat::YCBCR_422_SP);
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
      *target = static_cast<int>(PixelFormat::RGBA_1010102);
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
      *target = static_cast<int>(PixelFormat::RGBA_8888);
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatRGBX8888Ubwc:
      *target = static_cast<int>(PixelFormat::RGBX_8888);
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatBGR565Ubwc:
      *target = HAL_PIXEL_FORMAT_BGR_565;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatRGBA1010102Ubwc:
      *target = static_cast<int>(PixelFormat::RGBA_1010102);
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatRGBX1010102Ubwc:
      *target = HAL_PIXEL_FORMAT_RGBX_1010102;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatBlob:
      *target = static_cast<int>(PixelFormat::BLOB);
      break;
    case kFormatRGBA16161616F:
      *target = HAL_PIXEL_FORMAT_RGBA_FP16;
      break;
    case kFormatRGBA16161616FUbwc:
      *target = HAL_PIXEL_FORMAT_RGBA_FP16;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    default:
      DLOGW("Unsupported format = 0x%x", format);
      return -EINVAL;
  }
  return 0;
}

int HWCBufferAllocator::GetAllocatedBufferInfo(
    const BufferConfig &buffer_config, AllocatedBufferInfo *allocated_buffer_info) {
  BufferInfo buffer_info = {buffer_config, *allocated_buffer_info};
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

  uint32_t buffer_size = 0;
  int aligned_width = 0, aligned_height = 0;
  int ret =
      GetAlignedWidthAndHeight(width, height, format, alloc_flags, &aligned_width, &aligned_height);
  buffer_size = GetBufferSize(&buffer_info);
  if (ret < 0 || buffer_size == 0) {
    return -EINVAL;
  }
  allocated_buffer_info->stride = UINT32(aligned_width);
  allocated_buffer_info->aligned_width = UINT32(aligned_width);
  allocated_buffer_info->aligned_height = UINT32(aligned_height);
  allocated_buffer_info->size = UINT32(buffer_size);

  return 0;
}

int HWCBufferAllocator::GetBufferLayout(const AllocatedBufferInfo &buf_info, uint32_t stride[4],
                                        uint32_t offset[4], uint32_t *num_planes) {
  // TODO(user): Transition APIs to not need a private handle
  int format = static_cast<int>(PixelFormat::RGBA_8888);
  hidl_vec<PlaneLayout> plane_layouts;
  auto err = MapperExtError::NONE;
  uint64_t flags = 0;
  SetBufferInfo(buf_info.format, &format, &flags);
  // Setup only the required stuff, skip rest
  if (flags & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC) {
    flags = qtigralloc::PRIV_FLAGS_UBWC_ALIGNED;
  }
  mapper_ext_->getFormatLayout(
      INT32(format), buf_info.usage, INT32(flags), INT32(buf_info.aligned_width),
      INT32(buf_info.aligned_height),
      [&](MapperExtError _error, const auto &_size, const auto &_plane_layouts) {
        err = _error;
        plane_layouts = _plane_layouts;
      });

  if (err != MapperExtError::NONE) {
    DLOGE("GetBufferLayout failed");
    return -EINVAL;
  }

  // We are only returning buffer layout for progressive or single field formats.
  *num_planes = (plane_layouts.size() > 3) ? 2 : plane_layouts.size();

  for (int i = 0; i < *num_planes; i++)
  {
    offset[i] = static_cast<uint32_t>(plane_layouts[i].offset);
    stride[i] = static_cast<uint32_t>(plane_layouts[i].stride_bytes);
  }

  if (buf_info.format == kFormatYCrCb420PlanarStride16) {
    std::swap(offset[1], offset[2]);
  }

  if (flags & qtigralloc::PRIV_FLAGS_UBWC_ALIGNED) {
    std::fill(offset, offset + 4, 0);
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

void HWCBufferAllocator::SetBufferAccessControlInfo(std::bitset<kBufferPermMax> permission,
                                                    BufferPermission *buf_perm) {
  buf_perm->read = permission.test(kBufferPermRead);
  buf_perm->write = permission.test(kBufferPermWrite);
  buf_perm->execute = permission.test(kBufferPermExecute);
}

int HWCBufferAllocator::GetCustomContentMetadata(void *buf, CustomContentMetadata *dest) {
  int err = 0;

  if (!buf || !dest || !mapper_ext_) {
    err = -EINVAL;
  } else {
    auto map_err = mapper_ext_->getMetaDataValue(buf,
                                                 qtigralloc::MetadataType_CustomContentMetadata,
                                                 dest);
    if (map_err != MapperExtError::NONE) {
      err = -ENOTSUP;
    }
  }

  return err;
}

}  // namespace sdm
