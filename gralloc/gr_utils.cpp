/*
 * Copyright (c) 2011-2021, The Linux Foundation. All rights reserved.
 *
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
 *
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <display/media/mmm_color_fmt.h>
#include <display/drm/sde_drm.h>
#include <drm/drm_fourcc.h>

#include <sys/mman.h>
#include <cutils/properties.h>
#include <algorithm>
#include <string>
#include <vector>

#include "gr_adreno_info.h"
#include "gr_camera_info.h"
#include "gr_utils.h"
#include "QtiGralloc.h"
#include "color_extensions.h"

#ifndef GRALLOC_USAGE_PRIVATE_VIDEO_HW
#define GRALLOC_USAGE_PRIVATE_VIDEO_HW 1ULL << 52
#endif

#define ASTC_BLOCK_SIZE 16

#define ONLY_GPU_USAGE_MASK                                                                       \
    (BufferUsage::GPU_TEXTURE | BufferUsage::GPU_RENDER_TARGET | BufferUsage::GPU_CUBE_MAP |      \
     BufferUsage::GPU_MIPMAP_COMPLETE | BufferUsage::GPU_DATA_BUFFER | BufferUsage::RENDERSCRIPT)

#define NON_GPU_USAGE_MASK                                                                        \
    (BufferUsage::COMPOSER_CLIENT_TARGET | BufferUsage::COMPOSER_OVERLAY |                        \
     BufferUsage::COMPOSER_CURSOR | BufferUsage::VIDEO_ENCODER | BufferUsage::CAMERA_OUTPUT |     \
     BufferUsage::CAMERA_INPUT | BufferUsage::VIDEO_DECODER | GRALLOC_USAGE_PRIVATE_CDSP |        \
     GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY | GRALLOC_USAGE_PRIVATE_VIDEO_HW)

#define DEBUG 0

using aidl::android::hardware::graphics::common::Dataspace;
using aidl::android::hardware::graphics::common::PlaneLayout;
using aidl::android::hardware::graphics::common::PlaneLayoutComponent;
using aidl::android::hardware::graphics::common::StandardMetadataType;
using ::android::hardware::graphics::common::V1_2::PixelFormat;

namespace gralloc {

static inline unsigned int MMM_COLOR_FMT_RGB_STRIDE_IN_PIXELS(unsigned int color_fmt,
                                                              unsigned int width) {
  unsigned int stride = 0, bpp = 4;
  if (!(color_fmt & (MMM_COLOR_FMT_RGBA8888 | MMM_COLOR_FMT_RGBA8888_UBWC)) || !width) {
    return stride;
  }
  stride = MMM_COLOR_FMT_RGB_STRIDE(color_fmt, width);
  return (stride / bpp);
}

bool IsYuvFormat(int format) {
  switch (format) {
    case HAL_PIXEL_FORMAT_YCbCr_420_SP:
    case static_cast<int>(PixelFormat::YCBCR_422_SP):
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:  // Same as YCbCr_420_SP_VENUS
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
    case static_cast<int>(PixelFormat::YCRCB_420_SP):
    case HAL_PIXEL_FORMAT_YCrCb_422_SP:
    case HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO:
    case HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV21_ZSL:
    case static_cast<int>(PixelFormat::RAW16):
    case static_cast<int>(PixelFormat::Y16):
    case static_cast<int>(PixelFormat::RAW12):
    case static_cast<int>(PixelFormat::RAW10):
    case HAL_PIXEL_FORMAT_RAW8:
    case static_cast<int>(PixelFormat::YV12):
    case static_cast<int>(PixelFormat::Y8):
    case HAL_PIXEL_FORMAT_YCbCr_420_P010:
    case HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC:
    case HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC:
    case HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS:
    // Below formats used by camera and VR
    case static_cast<int>(PixelFormat::BLOB):
    case static_cast<int>(PixelFormat::RAW_OPAQUE):
    case HAL_PIXEL_FORMAT_NV12_HEIF:
    case HAL_PIXEL_FORMAT_CbYCrY_422_I:
    case HAL_PIXEL_FORMAT_NV12_LINEAR_FLEX:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH:
    case HAL_PIXEL_FORMAT_MULTIPLANAR_FLEX:
    case HAL_PIXEL_FORMAT_NV12_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_NV12_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_NV12_FLEX_8_BATCH:
      return true;
    default:
      return false;
  }
}

bool IsUncompressedRGBFormat(int format) {
  switch (format) {
    case static_cast<int>(PixelFormat::RGBA_8888):
    case static_cast<int>(PixelFormat::RGBX_8888):
    case static_cast<int>(PixelFormat::RGB_888):
    case static_cast<int>(PixelFormat::RGB_565):
    case HAL_PIXEL_FORMAT_BGR_565:
    case static_cast<int>(PixelFormat::BGRA_8888):
    case HAL_PIXEL_FORMAT_RGBA_5551:
    case HAL_PIXEL_FORMAT_RGBA_4444:
    case HAL_PIXEL_FORMAT_R_8:
    case static_cast<int>(aidl::android::hardware::graphics::common::PixelFormat::R_8):
    case HAL_PIXEL_FORMAT_RG_88:
    case HAL_PIXEL_FORMAT_BGRX_8888:
    case static_cast<int>(PixelFormat::RGBA_1010102):
    case HAL_PIXEL_FORMAT_ARGB_2101010:
    case HAL_PIXEL_FORMAT_RGBX_1010102:
    case HAL_PIXEL_FORMAT_XRGB_2101010:
    case HAL_PIXEL_FORMAT_BGRA_1010102:
    case HAL_PIXEL_FORMAT_ABGR_2101010:
    case HAL_PIXEL_FORMAT_BGRX_1010102:
    case HAL_PIXEL_FORMAT_XBGR_2101010:
    case static_cast<int>(PixelFormat::RGBA_FP16):
    case HAL_PIXEL_FORMAT_BGR_888:
      return true;
    default:
      break;
  }

  return false;
}

bool IsCompressedRGBFormat(int format) {
  switch (format) {
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_4x4_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_5x4_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_5x5_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_6x5_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_6x6_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x5_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x6_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x8_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x5_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x6_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x8_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x10_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_12x10_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_12x12_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:
      return true;
    default:
      break;
  }

  return false;
}

bool IsGpuDepthStencilFormat(int format) {
  switch (format) {
    case static_cast<int>(PixelFormat::DEPTH_16):
    case static_cast<int>(PixelFormat::DEPTH_24):
    case static_cast<int>(PixelFormat::DEPTH_24_STENCIL_8):
    case static_cast<int>(PixelFormat::DEPTH_32F):
    case static_cast<int>(PixelFormat::STENCIL_8):
      return true;
    default:
      break;
  }
  return false;
}

bool IsCameraCustomFormat(int format, uint64_t usage) {
  switch (format) {
    case HAL_PIXEL_FORMAT_NV21_ZSL:
    case HAL_PIXEL_FORMAT_NV12_LINEAR_FLEX:
    case HAL_PIXEL_FORMAT_MULTIPLANAR_FLEX:
    case static_cast<int>(PixelFormat::RAW_OPAQUE):
    case static_cast<int>(PixelFormat::RAW10):
    case static_cast<int>(PixelFormat::RAW12):
    case HAL_PIXEL_FORMAT_NV12_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_NV12_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_NV12_FLEX_8_BATCH:
      if (usage & GRALLOC_USAGE_HW_COMPOSER) {
        ALOGW("%s: HW_Composer flag is set for camera custom format: 0x%x, Usage: 0x%" PRIx64,
              __FUNCTION__, format, usage);
        return false;
      }
      return true;
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH:
      ALOGD("%s: UBWC_FLEX* format is moved to Gralloc: 0x%x, Usage: 0x%" PRIx64, __FUNCTION__,
            format, usage);
      return false;
    default:
      break;
  }

  return false;
}

uint32_t GetBatchSize(int format) {
  uint32_t batchsize = 1;
  switch (format) {
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH:
      batchsize = 2;
      break;
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH:
      batchsize = 4;
      break;
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH:
      batchsize = 8;
      break;
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX:
      batchsize = 16;
      break;
    default:
      break;
  }
  return batchsize;
}

bool IsUbwcFlexFormat(int format) {
  switch (format) {
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH:
      return true;
    default:
      break;
  }

  return false;
}

uint32_t GetBppForUncompressedRGB(int format) {
  uint32_t bpp = 0;
  switch (format) {
    case static_cast<int>(PixelFormat::RGBA_FP16):
      bpp = 8;
      break;
    case static_cast<int>(PixelFormat::RGBA_8888):
    case static_cast<int>(PixelFormat::RGBX_8888):
    case static_cast<int>(PixelFormat::BGRA_8888):
    case HAL_PIXEL_FORMAT_BGRX_8888:
    case static_cast<int>(PixelFormat::RGBA_1010102):
    case HAL_PIXEL_FORMAT_ARGB_2101010:
    case HAL_PIXEL_FORMAT_RGBX_1010102:
    case HAL_PIXEL_FORMAT_XRGB_2101010:
    case HAL_PIXEL_FORMAT_BGRA_1010102:
    case HAL_PIXEL_FORMAT_ABGR_2101010:
    case HAL_PIXEL_FORMAT_BGRX_1010102:
    case HAL_PIXEL_FORMAT_XBGR_2101010:
      bpp = 4;
      break;
    case static_cast<int>(PixelFormat::RGB_888):
    case HAL_PIXEL_FORMAT_BGR_888:
      bpp = 3;
      break;
    case static_cast<int>(PixelFormat::RGB_565):
    case HAL_PIXEL_FORMAT_BGR_565:
    case HAL_PIXEL_FORMAT_RGBA_5551:
    case HAL_PIXEL_FORMAT_RGBA_4444:
    case HAL_PIXEL_FORMAT_RG_88:
      bpp = 2;
      break;
    case HAL_PIXEL_FORMAT_R_8:
    case static_cast<int>(aidl::android::hardware::graphics::common::PixelFormat::R_8):
      bpp = 1;
      break;
    default:
      ALOGE("Error : %s New format request = 0x%x", __FUNCTION__, format);
      break;
  }

  return bpp;
}

bool CpuCanAccess(uint64_t usage) {
  return CpuCanRead(usage) || CpuCanWrite(usage);
}

bool CpuCanRead(uint64_t usage) {
  if (usage & BufferUsage::CPU_READ_MASK) {
    return true;
  }

  return false;
}

bool AdrenoAlignmentRequired(uint64_t usage) {
  if ((usage & BufferUsage::GPU_TEXTURE) || (usage & BufferUsage::GPU_RENDER_TARGET)) {
    // Certain formats may need to bypass adreno alignment requirements to
    // support legacy apps. The following check is for those cases where it is mandatory
    // to use adreno alignment
    if (((usage & GRALLOC_USAGE_PRIVATE_VIDEO_HW) &&
          ((usage & BufferUsage::VIDEO_DECODER) ||
           (usage & BufferUsage::VIDEO_ENCODER))) ||
          !CpuCanAccess(usage)) {
      return true;
    }
  }
  return false;
}

bool CpuCanWrite(uint64_t usage) {
  if (usage & BufferUsage::CPU_WRITE_MASK) {
    // Application intends to use CPU for rendering
    return true;
  }

  return false;
}

uint32_t GetDataAlignment(int format, uint64_t usage) {
  uint32_t align = UINT(getpagesize());
  if (format == HAL_PIXEL_FORMAT_YCbCr_420_SP_TILED) {
    align = SIZE_8K;
  }

  if (usage & BufferUsage::PROTECTED) {
    if ((usage & BufferUsage::CAMERA_OUTPUT) || (usage & GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY)) {
      // The alignment here reflects qsee mmu V7L/V8L requirement
      align = SZ_2M;
    } else {
      align = SECURE_ALIGN;
    }
  }

  return align;
}

bool IsGPUFlagSupported(uint64_t usage) {
  bool ret = true;
  if ((usage & BufferUsage::GPU_MIPMAP_COMPLETE)) {
    ALOGE("GPU_MIPMAP_COMPLETE not supported");
    ret = false;
  }

  if ((usage & BufferUsage::GPU_CUBE_MAP)) {
    ALOGE("GPU_CUBE_MAP not supported");
    ret = false;
  }

  return ret;
}

int GetBpp(int format) {
  if (IsUncompressedRGBFormat(format)) {
    return GetBppForUncompressedRGB(format);
  }
  switch (format) {
    case HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_4x4_KHR:
    case HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:
    case HAL_PIXEL_FORMAT_RAW8:
    case static_cast<int>(PixelFormat::Y8):
      return 1;
    case static_cast<int>(PixelFormat::RAW16):
    case static_cast<int>(PixelFormat::Y16):
    case static_cast<int>(PixelFormat::YCBCR_422_SP):
    case HAL_PIXEL_FORMAT_YCrCb_422_SP:
    case static_cast<int>(PixelFormat::YCBCR_422_I):
    case HAL_PIXEL_FORMAT_YCrCb_422_I:
    case HAL_PIXEL_FORMAT_CbYCrY_422_I:
      return 2;
    case HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS:
    case HAL_PIXEL_FORMAT_YCbCr_420_P010:
      return 3;
    default:
      return -1;
  }
}

// Returns the final buffer size meant to be allocated with ion
unsigned int GetSize(const BufferInfo &info, unsigned int alignedw, unsigned int alignedh) {
  unsigned int size = 0;
  int format = info.format;
  int width = info.width;
  int height = info.height;
  uint64_t usage = info.usage;
  unsigned int y_plane, uv_plane;
  unsigned int mmm_color_format;
  unsigned int alignment = 16;

  if (!IsGPUFlagSupported(usage)) {
    ALOGE("Unsupported GPU usage flags present 0x%" PRIx64, usage);
    return 0;
  }

  if (IsCameraCustomFormat(format, usage) && CameraInfo::GetInstance()) {
    int result = CameraInfo::GetInstance()->GetBufferSize(format, width, height, &size);
    if (result != 0) {
      ALOGE("%s: Failed to get the buffer size through camera library. Error code: %d",
            __FUNCTION__, result);
      return 0;
    }
  } else if (IsUBwcEnabled(format, usage)) {
    size = GetUBwcSize(width, height, format, alignedw, alignedh);
  } else if (IsUncompressedRGBFormat(format)) {
    uint32_t bpp = GetBppForUncompressedRGB(format);
    size = alignedw * alignedh * bpp;
  } else if (IsCompressedRGBFormat(format)) {
    size = alignedw * alignedh * ASTC_BLOCK_SIZE;
  } else {
    // Below switch should be for only YUV/custom formats
    switch (format) {
      case static_cast<int>(PixelFormat::RAW16):
      case static_cast<int>(PixelFormat::Y16):
        size = alignedw * alignedh * 2;
        break;
      case static_cast<int>(PixelFormat::RAW10):
      case static_cast<int>(PixelFormat::RAW12):
        size = ALIGN(alignedw * alignedh, SIZE_4K);
        break;
      case HAL_PIXEL_FORMAT_RAW8:
      case static_cast<int>(PixelFormat::Y8):
        size = alignedw * alignedh * 1;
        break;
      // adreno formats
      case HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO:  // NV21
        size = ALIGN(alignedw * alignedh, SIZE_4K);
        size += (unsigned int)ALIGN(2 * ALIGN(width / 2, 32) * ALIGN(height / 2, 32), SIZE_4K);
        break;
      case HAL_PIXEL_FORMAT_YCbCr_420_SP_TILED:  // NV12
        // The chroma plane is subsampled,
        // but the pitch in bytes is unchanged
        // The GPU needs 4K alignment, but the video decoder needs 8K
        size = ALIGN(alignedw * alignedh, SIZE_8K);
        size += ALIGN(alignedw * (unsigned int)ALIGN(height / 2, 32), SIZE_8K);
        break;
      case static_cast<int>(PixelFormat::YV12):
        if ((width & 1) || (height & 1)) {
          ALOGE("w or h is odd for the YV12 format");
          return 0;
        }
        if (AdrenoAlignmentRequired(usage)) {
          if (AdrenoMemInfo::GetInstance() == nullptr) {
            ALOGE("Unable to get adreno instance");
            return 0;
          }
          alignment = AdrenoMemInfo::GetInstance()->GetGpuPixelAlignment();
        }
        size = alignedw * alignedh + (ALIGN(alignedw / 2, alignment) * (alignedh / 2)) * 2;
        size = ALIGN(size, (unsigned int)SIZE_4K);
        break;
      case HAL_PIXEL_FORMAT_YCbCr_420_SP:
      case static_cast<int>(PixelFormat::YCRCB_420_SP):
        size = ALIGN((alignedw * alignedh) + (alignedw * alignedh) / 2 + 1, SIZE_4K);
        break;
      case HAL_PIXEL_FORMAT_YCbCr_420_P010:
      case HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS:
        mmm_color_format =
            (usage & GRALLOC_USAGE_PRIVATE_HEIF) ? MMM_COLOR_FMT_P010_512 : MMM_COLOR_FMT_P010;
        size = MMM_COLOR_FMT_BUFFER_SIZE(mmm_color_format, width, height);
        break;
      case static_cast<int>(PixelFormat::YCBCR_422_SP):
      case HAL_PIXEL_FORMAT_YCrCb_422_SP:
      case static_cast<int>(PixelFormat::YCBCR_422_I):
      case HAL_PIXEL_FORMAT_YCrCb_422_I:
      case HAL_PIXEL_FORMAT_CbYCrY_422_I:
        if (width & 1) {
          ALOGE("width is odd for the YUV422_SP format");
          return 0;
        }
        size = ALIGN(alignedw * alignedh * 2, SIZE_4K);
        break;
      case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
      case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
        mmm_color_format =
            (usage & GRALLOC_USAGE_PRIVATE_HEIF) ? MMM_COLOR_FMT_NV12_512 : MMM_COLOR_FMT_NV12;
        size = MMM_COLOR_FMT_BUFFER_SIZE(mmm_color_format, width, height);
        break;
      case HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS:
      case HAL_PIXEL_FORMAT_NV21_ENCODEABLE:
        size = MMM_COLOR_FMT_BUFFER_SIZE(MMM_COLOR_FMT_NV21, width, height);
        break;
      case static_cast<int>(PixelFormat::BLOB):
        if (height != 1) {
          ALOGE("%s: Buffers with PixelFormat::BLOB must have height 1 ", __FUNCTION__);
          return 0;
        }
        size = (unsigned int)width;
        break;
      case HAL_PIXEL_FORMAT_NV12_HEIF:
        size = MMM_COLOR_FMT_BUFFER_SIZE(MMM_COLOR_FMT_NV12_512, width, height);
        break;
      case HAL_PIXEL_FORMAT_NV21_ZSL:
        size = ALIGN((alignedw * alignedh) + (alignedw * alignedh) / 2, SIZE_4K);
        break;
      default:
        ALOGE("%s: Unrecognized pixel format: 0x%x", __FUNCTION__, format);
        return 0;
    }
  }
  auto align = GetDataAlignment(format, usage);
  size = ALIGN(size, align) * info.layer_count;
  return size;
}

int GetBufferSizeAndDimensions(const BufferInfo &info, unsigned int *size, unsigned int *alignedw,
                               unsigned int *alignedh) {
  GraphicsMetadata graphics_metadata = {};
  if (info.width < 1 || info.height < 1) {
    *alignedw = 0;
    *alignedh = 0;
    *size = 0;
    ALOGW("%s: Invalid buffer info, Width: %d, Height: %d.", __FUNCTION__, info.width, info.height);
    return -1;
  }
  return GetBufferSizeAndDimensions(info, size, alignedw, alignedh, &graphics_metadata);
}

int GetBufferSizeAndDimensions(const BufferInfo &info, unsigned int *size, unsigned int *alignedw,
                               unsigned int *alignedh, GraphicsMetadata *graphics_metadata) {
  int buffer_type = GetBufferType(info.format);
  if (CanUseAdrenoForSize(buffer_type, info.usage)) {
    return GetGpuResourceSizeAndDimensions(info, size, alignedw, alignedh, graphics_metadata);
  } else {
    int err = GetAlignedWidthAndHeight(info, alignedw, alignedh);
    if (err) {
      *size = 0;
      return err;
    }
    *size = GetSize(info, *alignedw, *alignedh);
  }
  return 0;
}

void GetYuvUbwcSPPlaneInfo(uint32_t width, uint32_t height, int color_format,
                           PlaneLayoutInfo *plane_info) {
  // UBWC buffer has these 4 planes in the following sequence:
  // Y_Plane, UV_Plane, Y_Meta_Plane, UV_Meta_Plane
  unsigned int y_meta_stride = 0, y_meta_height = 0, y_meta_size = 0;
  unsigned int y_stride = 0, y_height = 0, y_size = 0;
  unsigned int c_meta_stride = 0, c_meta_height = 0, c_meta_size = 0;
  unsigned int alignment = 4096;
  unsigned int c_stride = 0, c_height = 0, c_size = 0;
  uint64_t yOffset = 0, cOffset = 0, yMetaOffset = 0, cMetaOffset = 0;

  y_meta_stride = MMM_COLOR_FMT_Y_META_STRIDE(color_format, INT(width));
  y_meta_height = MMM_COLOR_FMT_Y_META_SCANLINES(color_format, INT(height));
  y_meta_size = ALIGN((y_meta_stride * y_meta_height), alignment);

  y_stride = MMM_COLOR_FMT_Y_STRIDE(color_format, INT(width));
  y_height = MMM_COLOR_FMT_Y_SCANLINES(color_format, INT(height));
  y_size = ALIGN((y_stride * y_height), alignment);

  c_meta_stride = MMM_COLOR_FMT_UV_META_STRIDE(color_format, INT(width));
  c_meta_height = MMM_COLOR_FMT_UV_META_SCANLINES(color_format, INT(height));
  c_meta_size = ALIGN((c_meta_stride * c_meta_height), alignment);

  c_stride = MMM_COLOR_FMT_UV_STRIDE(color_format, INT(width));
  c_height = MMM_COLOR_FMT_UV_SCANLINES(color_format, INT(height));
  c_size = ALIGN((c_stride * c_height), alignment);
  yMetaOffset = 0;
  yOffset = y_meta_size;
  cMetaOffset = y_meta_size + y_size;
  cOffset = y_meta_size + y_size + c_meta_size;

  plane_info[0].component = (PlaneComponent)PLANE_COMPONENT_Y;
  plane_info[0].offset = (uint32_t)yOffset;
  plane_info[0].stride = static_cast<int32_t>(UINT(width));
  plane_info[0].stride_bytes = static_cast<int32_t>(y_stride);
  plane_info[0].scanlines = static_cast<int32_t>(y_height);
  plane_info[0].size = static_cast<uint32_t>(y_size);

  plane_info[1].component = (PlaneComponent)(PLANE_COMPONENT_Cb | PLANE_COMPONENT_Cr);
  plane_info[1].offset = (uint32_t)cOffset;
  plane_info[1].stride = static_cast<int32_t>(UINT(width));
  plane_info[1].stride_bytes = static_cast<int32_t>(c_stride);
  plane_info[1].scanlines = static_cast<int32_t>(c_height);
  plane_info[1].size = static_cast<uint32_t>(c_size);

  plane_info[2].component = (PlaneComponent)(PLANE_COMPONENT_META | PLANE_COMPONENT_Y);
  plane_info[2].offset = (uint32_t)yMetaOffset;
  plane_info[2].stride = static_cast<int32_t>(UINT(width));
  plane_info[2].stride_bytes = static_cast<int32_t>(y_meta_stride);
  plane_info[2].scanlines = static_cast<int32_t>(y_meta_height);
  plane_info[2].size = static_cast<uint32_t>(y_meta_size);

  plane_info[3].component =
      (PlaneComponent)(PLANE_COMPONENT_META | PLANE_COMPONENT_Cb | PLANE_COMPONENT_Cr);
  plane_info[3].offset = (uint32_t)cMetaOffset;
  plane_info[3].stride = static_cast<int32_t>(UINT(width));
  plane_info[3].stride_bytes = static_cast<int32_t>(c_meta_stride);
  plane_info[3].scanlines = static_cast<int32_t>(c_meta_height);
  plane_info[3].size = static_cast<uint32_t>(c_meta_size);
}

// This API gets information about 8 planes (Y_Plane, UV_Plane, Y_Meta_Plane, UV_Meta_Plane,
// Y_Plane, UV_Plane, Y_Meta_Plane, UV_Meta_Plane) and it stores the
// information in PlaneLayoutInfo array.
void GetYuvUbwcInterlacedSPPlaneInfo(uint32_t width, uint32_t height,
                                     PlaneLayoutInfo plane_info[8]) {
  // UBWC interlaced has top-bottom field layout with each field as
  // 8-plane (including meta plane also) NV12_UBWC with width = image_width
  // & height = image_height / 2.
  // Client passed plane_info argument is ptr to struct PlaneLayoutInfo[8].
  // Plane info to be filled for each field separately.
  height = (height + 1) >> 1;

  GetYuvUbwcSPPlaneInfo(width, height, MMM_COLOR_FMT_NV12_UBWC, &plane_info[0]);

  GetYuvUbwcSPPlaneInfo(width, height, MMM_COLOR_FMT_NV12_UBWC, &plane_info[4]);
}

// This API gets information about 2 planes (Y_Plane & UV_Plane).
// Here width and height are aligned width and aligned height.
// bpp: bits per pixel.
void GetYuvSPPlaneInfo(const BufferInfo &info, int format, uint32_t width, uint32_t height,
                       uint32_t bpp, PlaneLayoutInfo *plane_info) {
  int unaligned_width = info.width;
  int unaligned_height = info.height;
  unsigned int y_stride = 0, y_height = 0, y_size = 0;
  unsigned int c_stride = 0, c_height = 0, c_size = 0;
  uint64_t yOffset, cOffset;
  unsigned int mmm_color_format;

  y_stride = c_stride = UINT(width) * bpp;
  y_height = INT(height);
  y_size = y_stride * y_height;
  switch (format) {
    case HAL_PIXEL_FORMAT_YCbCr_420_SP:
    case static_cast<int>(PixelFormat::YCRCB_420_SP):
      c_size = (width * height) / 2 + 1;
      c_height = height >> 1;
      c_size = width * c_height;
      break;
    case static_cast<int>(PixelFormat::YCBCR_422_SP):
    case HAL_PIXEL_FORMAT_YCrCb_422_SP:
      if (unaligned_width & 1) {
        ALOGE("width is odd for the YUV422_SP format");
        return;
      }
      c_size = width * height;
      c_height = height;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
      mmm_color_format =
          (info.usage & GRALLOC_USAGE_PRIVATE_HEIF) ? MMM_COLOR_FMT_NV12_512 : MMM_COLOR_FMT_NV12;
      c_height = MMM_COLOR_FMT_UV_SCANLINES(mmm_color_format, height);
      c_size = c_stride * c_height;
      break;
    case HAL_PIXEL_FORMAT_NV12_HEIF:
      c_height = MMM_COLOR_FMT_UV_SCANLINES(MMM_COLOR_FMT_NV12_512, height);
      c_size = c_stride * c_height;
      break;
    case HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO:
      y_size = ALIGN(width * height, 4096);
      c_size = ALIGN(2 * ALIGN(unaligned_width / 2, 32) * ALIGN(unaligned_height / 2, 32), 4096);
      break;
    case HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS:
      c_height = MMM_COLOR_FMT_UV_SCANLINES(MMM_COLOR_FMT_NV21, height);
      c_size = c_stride * c_height;
      break;
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      c_height = height >> 1;
      c_size = width * c_height;
      break;
    case static_cast<int>(PixelFormat::Y16):
      c_size = c_stride = 0;
      c_height = 0;
      break;
    case static_cast<int>(PixelFormat::Y8):
      c_size = c_stride = 0;
      c_height = 0;
      break;
    default:
      break;
  }

  yOffset = 0;
  cOffset = y_size;

  plane_info[0].component = (PlaneComponent)PLANE_COMPONENT_Y;
  plane_info[0].offset = (uint32_t)yOffset;
  plane_info[0].step = 1;
  plane_info[0].stride = static_cast<int32_t>(UINT(width));
  plane_info[0].stride_bytes = static_cast<int32_t>(y_stride);
  plane_info[0].scanlines = static_cast<int32_t>(y_height);
  plane_info[0].size = static_cast<uint32_t>(y_size);

  plane_info[1].component = (PlaneComponent)(PLANE_COMPONENT_Cb | PLANE_COMPONENT_Cr);
  plane_info[1].offset = (uint32_t)cOffset;
  plane_info[1].step = 2 * bpp;
  plane_info[1].stride = static_cast<int32_t>(UINT(width));
  plane_info[1].stride_bytes = static_cast<int32_t>(c_stride);
  plane_info[1].scanlines = static_cast<int32_t>(c_height);
  plane_info[1].size = static_cast<uint32_t>(c_size);
}

int GetYUVPlaneInfo(const private_handle_t *hnd, struct android_ycbcr ycbcr[2]) {
  int err = 0;
  uint32_t width = UINT(hnd->width);
  uint32_t height = UINT(hnd->height);
  int format = hnd->format;
  uint64_t usage = hnd->usage;
  int32_t interlaced = 0;
  int plane_count = 0;
  int unaligned_width = INT(hnd->unaligned_width);
  int unaligned_height = INT(hnd->unaligned_height);
  BufferInfo info(unaligned_width, unaligned_height, format, usage);

  memset(ycbcr->reserved, 0, sizeof(ycbcr->reserved));

  // Check metadata for interlaced content.
  int interlace_flag = 0;
  if (GetMetaDataValue(const_cast<private_handle_t *>(hnd), QTI_PP_PARAM_INTERLACED,
                       &interlace_flag) == Error::NONE) {
    if (interlace_flag) {
      interlaced = LAYOUT_INTERLACED_FLAG;
    }
  }

  PlaneLayoutInfo plane_info[8] = {};
  // Get the chroma offsets from the handle width/height. We take advantage
  // of the fact the width _is_ the stride
  err = GetYUVPlaneInfo(info, format, width, height, interlaced, &plane_count, plane_info, hnd,
                        ycbcr);
  return err;
}

int GetRawPlaneInfo(int32_t format, int32_t width, int32_t height, PlaneLayoutInfo *plane_info) {
  int32_t step = 0;

  switch (format) {
    case static_cast<int32_t>(PixelFormat::RAW16):
      step = 2;
      break;
    case HAL_PIXEL_FORMAT_RAW8:
      step = 1;
      break;
    case static_cast<int32_t>(PixelFormat::RAW12):
    case static_cast<int32_t>(PixelFormat::RAW10):
      step = 0;
      break;
    default:
      ALOGW("RawPlaneInfo is unsupported for format 0x%x", format);
      return -EINVAL;
  }

  BufferInfo info(width, height, format);
  uint32_t alignedWidth, alignedHeight;
  GetAlignedWidthAndHeight(info, &alignedWidth, &alignedHeight);

  uint32_t size = GetSize(info, alignedWidth, alignedHeight);

  plane_info[0].component = (PlaneComponent)PLANE_COMPONENT_RAW;
  plane_info[0].h_subsampling = 0;
  plane_info[0].v_subsampling = 0;
  plane_info[0].offset = 0;
  plane_info[0].step = step;
  plane_info[0].stride = width;
  plane_info[0].stride_bytes = static_cast<int32_t>(alignedWidth);
  if (format == static_cast<int32_t>(PixelFormat::RAW16)) {
    plane_info[0].stride_bytes = static_cast<int32_t>(alignedWidth * GetBpp(format));
  }
  plane_info[0].scanlines = height;
  plane_info[0].size = size;

  return 0;
}

// Explicitly defined UBWC formats
bool IsUBwcFormat(int format) {
  switch (format) {
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
    case HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC:
    case HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC:
      return true;
    default:
      return IsUbwcFlexFormat(format);
  }
}

bool IsUBwcSupported(int format) {
  // Existing HAL formats with UBWC support
  switch (format) {
    case HAL_PIXEL_FORMAT_BGR_565:
    case static_cast<int>(PixelFormat::RGBA_8888):
    case static_cast<int>(PixelFormat::RGBX_8888):
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case static_cast<int>(PixelFormat::RGBA_1010102):
    case HAL_PIXEL_FORMAT_RGBX_1010102:
    case static_cast<int>(PixelFormat::DEPTH_16):
    case static_cast<int>(PixelFormat::DEPTH_24):
    case static_cast<int>(PixelFormat::DEPTH_24_STENCIL_8):
    case static_cast<int>(PixelFormat::DEPTH_32F):
    case static_cast<int>(PixelFormat::STENCIL_8):
    case static_cast<int>(PixelFormat::RGBA_FP16):
      return true;
    default:
      break;
  }

  return false;
}

// Check if the format must be macro-tiled. Later if the lists of tiled formats and Depth/Stencil
// formats gets updated, then handle it here appropriately.
bool IsTileRendered(int format) {
  return IsGpuDepthStencilFormat(format);
}

bool IsOnlyGpuUsage(uint64_t usage) {
  if (usage & NON_GPU_USAGE_MASK) {
    return false;
  }
  if (usage & ONLY_GPU_USAGE_MASK) {
    return true;
  }
  return false;
}

bool IsUBwcPISupported(int format, uint64_t usage) {
  // TODO(user): try and differentiate b/w mdp capability to support PI.
  if (!(usage & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC_PI)) {
    return false;
  }

  // As of now only two formats
  switch (format) {
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH:
    case HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC: {
      if ((usage & BufferUsage::GPU_TEXTURE) || (usage & BufferUsage::GPU_RENDER_TARGET)) {
        if (AdrenoMemInfo::GetInstance()) {
          return AdrenoMemInfo::GetInstance()->IsPISupportedByGPU(format, usage);
        }
      } else {
        return true;
      }
    }
  }

  return false;
}

bool IsUBwcEnabled(int format, uint64_t usage) {
  // Allow UBWC, if client is using an explicitly defined UBWC pixel format.
  if (IsUBwcFormat(format)) {
    return true;
  }

  // Allow UBWC, if an OpenGL client sets UBWC usage flag and GPU plus MDP
  // support the format. OR if a non-OpenGL client like Rotator, sets UBWC
  // usage flag and MDP supports the format. UBWC usage flag is not set by
  // App during buffer allocation via NDK API AHardwareBuffer_allocate for
  // DEPTH/STENCIL8 formats, so skip the check for it.
  if ((((usage & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC) ||
        (usage & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC_PI) ||
        (usage & BufferUsage::COMPOSER_CLIENT_TARGET)) &&
       IsUBwcSupported(format)) ||
      IsTileRendered(format)) {
    bool enable = true;
    // Query GPU for UBWC only if buffer is intended to be used by GPU.
    if ((usage & BufferUsage::GPU_TEXTURE) || (usage & BufferUsage::GPU_RENDER_TARGET)) {
      if (AdrenoMemInfo::GetInstance()) {
        enable = AdrenoMemInfo::GetInstance()->IsUBWCSupportedByGPU(format);
      }
    }

    // Allow UBWC, only if CPU usage flags are not set
    if (enable && !(CpuCanAccess(usage))) {
      return true;
    }
  }

  return false;
}

void GetYuvUBwcWidthAndHeight(int width, int height, int format, unsigned int *aligned_w,
                              unsigned int *aligned_h) {
  switch (format) {
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH:
      *aligned_w = MMM_COLOR_FMT_Y_STRIDE(MMM_COLOR_FMT_NV12_UBWC, width);
      *aligned_h = MMM_COLOR_FMT_Y_SCANLINES(MMM_COLOR_FMT_NV12_UBWC, height);
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC:
      // The macro returns the stride which is 4/3 times the width, hence * 3/4
      *aligned_w = (MMM_COLOR_FMT_Y_STRIDE(MMM_COLOR_FMT_NV12_BPP10_UBWC, width) * 3) / 4;
      *aligned_h = MMM_COLOR_FMT_Y_SCANLINES(MMM_COLOR_FMT_NV12_BPP10_UBWC, height);
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC:
      // The macro returns the stride which is 2 times the width, hence / 2
      *aligned_w = (MMM_COLOR_FMT_Y_STRIDE(MMM_COLOR_FMT_P010_UBWC, width) / 2);
      *aligned_h = MMM_COLOR_FMT_Y_SCANLINES(MMM_COLOR_FMT_P010_UBWC, height);
      break;
    default:
      ALOGE("%s: Unsupported pixel format: 0x%x", __FUNCTION__, format);
      break;
  }
}

void GetRgbUBwcBlockSize(uint32_t bpp, int *block_width, int *block_height) {
  *block_width = 0;
  *block_height = 0;

  switch (bpp) {
    case 2:
    case 4:
      *block_width = 16;
      *block_height = 4;
      break;
    case 8:
      *block_width = 8;
      *block_height = 4;
      break;
    case 16:
      *block_width = 4;
      *block_height = 4;
      break;
    default:
      ALOGE("%s: Unsupported bpp: %d", __FUNCTION__, bpp);
      break;
  }
}

unsigned int GetRgbUBwcMetaBufferSize(int width, int height, uint32_t bpp) {
  unsigned int size = 0;
  int meta_width, meta_height;
  int block_width, block_height;

  GetRgbUBwcBlockSize(bpp, &block_width, &block_height);
  if (!block_width || !block_height) {
    ALOGE("%s: Unsupported bpp: %d", __FUNCTION__, bpp);
    return size;
  }

  // Align meta buffer height to 16 blocks
  meta_height = ALIGN(((height + block_height - 1) / block_height), 16);

  // Align meta buffer width to 64 blocks
  meta_width = ALIGN(((width + block_width - 1) / block_width), 64);

  // Align meta buffer size to 4K
  size = (unsigned int)ALIGN((meta_width * meta_height), 4096);

  return size;
}

unsigned int GetUBwcSize(int width, int height, int format, unsigned int alignedw,
                         unsigned int alignedh) {
  unsigned int size = 0;
  uint32_t bpp = 0;
  switch (format) {
    case HAL_PIXEL_FORMAT_BGR_565:
    case static_cast<int>(PixelFormat::RGBA_8888):
    case static_cast<int>(PixelFormat::RGBX_8888):
    case static_cast<int>(PixelFormat::RGBA_1010102):
    case HAL_PIXEL_FORMAT_RGBX_1010102:
    case static_cast<int>(PixelFormat::RGBA_FP16):
      bpp = GetBppForUncompressedRGB(format);
      size = alignedw * alignedh * bpp;
      size += GetRgbUBwcMetaBufferSize(width, height, bpp);
      break;
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      size = MMM_COLOR_FMT_BUFFER_SIZE(MMM_COLOR_FMT_NV12_UBWC, width, height);
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC:
      size = MMM_COLOR_FMT_BUFFER_SIZE(MMM_COLOR_FMT_NV12_BPP10_UBWC, width, height);
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC:
      size = MMM_COLOR_FMT_BUFFER_SIZE(MMM_COLOR_FMT_P010_UBWC, width, height);
      break;
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH:
      size =
          GetBatchSize(format) * MMM_COLOR_FMT_BUFFER_SIZE(MMM_COLOR_FMT_NV12_UBWC, width, height);
      break;
    default:
      ALOGE("%s: Unsupported pixel format: 0x%x", __FUNCTION__, format);
      break;
  }

  return size;
}

unsigned int GetRgbMetaSize(int format, uint32_t width, uint32_t height, uint64_t usage) {
  unsigned int meta_size = 0;
  if (!IsUBwcEnabled(format, usage)) {
    return meta_size;
  }
  uint32_t bpp = GetBppForUncompressedRGB(format);
  switch (format) {
    case HAL_PIXEL_FORMAT_BGR_565:
    case static_cast<int>(PixelFormat::RGBA_8888):
    case static_cast<int>(PixelFormat::RGBX_8888):
    case static_cast<int>(PixelFormat::RGBA_1010102):
    case HAL_PIXEL_FORMAT_RGBX_1010102:
    case static_cast<int>(PixelFormat::RGBA_FP16):
      meta_size = GetRgbUBwcMetaBufferSize(width, height, bpp);
      break;
    default:
      ALOGE("%s:Unsupported RGB format: 0x%x", __FUNCTION__, format);
      break;
  }
  return meta_size;
}

int GetRgbDataAddress(private_handle_t *hnd, void **rgb_data) {
  int err = 0;

  // This api is for RGB* formats
  if (!IsUncompressedRGBFormat(hnd->format)) {
    return -EINVAL;
  }

  // linear buffer, nothing to do further
  if (!(hnd->flags & qtigralloc::PRIV_FLAGS_UBWC_ALIGNED)) {
    *rgb_data = reinterpret_cast<void *>(hnd->base);
    return err;
  }
  unsigned int meta_size = GetRgbMetaSize(hnd->format, hnd->width, hnd->height, hnd->usage);

  *rgb_data = reinterpret_cast<void *>(hnd->base + meta_size);

  return err;
}

int GetCustomDimensions(private_handle_t *hnd, int *stride, int *height) {
  CropRectangle_t crop;
  int interlaced = 0;
  *stride = hnd->width;
  *height = hnd->height;
  if (GetMetaDataValue(hnd, (int64_t)StandardMetadataType::CROP, &crop) == Error::NONE) {
    *stride = crop.right;
    *height = crop.bottom;
  } else if (GetMetaDataValue(hnd, QTI_PP_PARAM_INTERLACED, &interlaced) == Error::NONE) {
    if (interlaced && IsUBwcFormat(hnd->format)) {
      unsigned int alignedw = 0, alignedh = 0;
      // Get re-aligned height for single ubwc interlaced field and
      // multiply by 2 to get frame height.
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

// TODO(user): Collapse into GetColorSpaceFromMetadata with qdmetadata deprecation
Error GetColorSpaceFromColorMetaData(ColorMetaData color_metadata, uint32_t *color_space) {
  Error err = Error::NONE;
  switch (color_metadata.colorPrimaries) {
    case ColorPrimaries_BT709_5:
      *color_space = HAL_CSC_ITU_R_709;
      break;
    case ColorPrimaries_BT601_6_525:
    case ColorPrimaries_BT601_6_625:
      *color_space = ((color_metadata.range) ? HAL_CSC_ITU_R_601_FR : HAL_CSC_ITU_R_601);
      break;
    case ColorPrimaries_BT2020:
      *color_space = (color_metadata.range) ? HAL_CSC_ITU_R_2020_FR : HAL_CSC_ITU_R_2020;
      break;
    default:
      err = Error::UNSUPPORTED;
      *color_space = 0;
      ALOGW("Unknown Color primary = %d", color_metadata.colorPrimaries);
      break;
  }
  return err;
}

void GetColorSpaceFromMetadata(private_handle_t *hnd, int *color_space) {
  ColorMetaData color_metadata;
  if (GetMetaDataValue(hnd, QTI_COLOR_METADATA, &color_metadata) == Error::NONE) {
    switch (color_metadata.colorPrimaries) {
      case ColorPrimaries_BT709_5:
        *color_space = ((color_metadata.range) ? HAL_CSC_ITU_R_709_FR : HAL_CSC_ITU_R_709);
        break;
      case ColorPrimaries_BT601_6_525:
      case ColorPrimaries_BT601_6_625:
        *color_space = ((color_metadata.range) ? HAL_CSC_ITU_R_601_FR : HAL_CSC_ITU_R_601);
        break;
      case ColorPrimaries_BT2020:
        *color_space = (color_metadata.range) ? HAL_CSC_ITU_R_2020_FR : HAL_CSC_ITU_R_2020;
        break;
      default:
        ALOGW("Unknown Color primary = %d", color_metadata.colorPrimaries);
        break;
    }
  } else if (GetMetaDataValue(hnd, QTI_COLORSPACE, color_space) != Error::NONE) {
    *color_space = 0;
  }
}

int GetAlignedWidthAndHeight(const BufferInfo &info, unsigned int *alignedw,
                             unsigned int *alignedh) {
  int width = info.width;
  int height = info.height;
  int format = info.format;
  *alignedw = width;
  *alignedh = height;
  uint64_t usage = info.usage;
  if (width < 1 || height < 1) {
    *alignedw = 0;
    *alignedh = 0;
    ALOGW("%s: Invalid buffer info, Width: %d, Height: %d.", __FUNCTION__, width, height);
    return -1;
  }
  // Currently surface padding is only computed for RGB* surfaces.
  bool ubwc_enabled = IsUBwcEnabled(format, usage);
  int tile = ubwc_enabled;
  unsigned int mmm_color_format;
  // Use of aligned width and aligned height is to calculate the size of buffer,
  // but in case of camera custom format size is being calculated from given width
  // and given height.
  if (IsCameraCustomFormat(format, usage) && CameraInfo::GetInstance()) {
    int aligned_w = width;
    int aligned_h = height;
    int result = CameraInfo::GetInstance()->GetStrideInBytes(
        format, (PlaneComponent)PLANE_COMPONENT_Y, width, &aligned_w);
    if (result != 0) {
      ALOGE(
          "%s: Failed to get the aligned width for camera custom format. width: %d, height: %d,"
          "format: %d, Error code: %d",
          __FUNCTION__, width, height, format, result);
      *alignedw = width;
      *alignedh = aligned_h;
      return result;
    }

    result = CameraInfo::GetInstance()->GetScanline(format, (PlaneComponent)PLANE_COMPONENT_Y,
                                                    height, &aligned_h);
    if (result != 0) {
      ALOGE(
          "%s: Failed to get the aligned height for camera custom format. width: %d,"
          "height: %d, format: %d, Error code: %d",
          __FUNCTION__, width, height, format, result);
      *alignedw = aligned_w;
      *alignedh = height;
      return result;
    }

    *alignedw = aligned_w;
    *alignedh = aligned_h;
    return 0;
  }

  if (IsUncompressedRGBFormat(format)) {
    unsigned int aligned_w = width;
    unsigned int aligned_h = height;
    unsigned int alignment = 32;

    if (usage & ONLY_GPU_USAGE_MASK) {
      if (AdrenoMemInfo::GetInstance() == nullptr) {
        ALOGE("Unable to get adreno instance");
        return -1;
      }
      AdrenoMemInfo::GetInstance()->AlignUnCompressedRGB(width, height, format, tile, alignedw,
                                                         alignedh);
    } else {
      unsigned int bpp = GetBppForUncompressedRGB(format);
      alignment = 256;  // Based on mmm_color_fmt.h
      // When bpp is 3, width is directly aligned to 256
      if (bpp == 3) {
        *alignedw = ALIGN(width, alignment);
      } else {
        *alignedw = ALIGN((width * bpp), alignment) / bpp;
      }
      if (ubwc_enabled) {
        *alignedh = ALIGN(height, 16);
      } else {
        *alignedh = height;
      }
    }

    if (((usage & BufferUsage::VIDEO_ENCODER) || (usage & BufferUsage::VIDEO_DECODER) ||
        (usage & BufferUsage::COMPOSER_OVERLAY)) &&
        (format == static_cast<int>(PixelFormat::RGBA_8888))) {
      int mmm_format = MMM_COLOR_FMT_RGBA8888;
      if (ubwc_enabled) {
        mmm_format = MMM_COLOR_FMT_RGBA8888_UBWC;
      }
      aligned_w = MMM_COLOR_FMT_RGB_STRIDE_IN_PIXELS(mmm_format, *alignedw);
      aligned_h = MMM_COLOR_FMT_RGB_SCANLINES(mmm_format, *alignedh);

      *alignedw = aligned_w;
      *alignedh = aligned_h;
    }
    return 0;
  }

  if (IsGpuDepthStencilFormat(format)) {
    if (usage & ONLY_GPU_USAGE_MASK) {
      if (IsTileRendered(info.format)) {
        tile = true;
      }
      if (AdrenoMemInfo::GetInstance() == nullptr) {
        ALOGE("Unable to get adreno instance");
        return -1;
      }
      AdrenoMemInfo::GetInstance()->AlignGpuDepthStencilFormat(width, height, format, tile,
                                                               alignedw, alignedh);
      return 0;
    } else {
      ALOGE("DepthStencil format without GPU usage flags");
      return -1;
    }
  }

  if (ubwc_enabled) {
    GetYuvUBwcWidthAndHeight(width, height, format, alignedw, alignedh);
    return 0;
  }

  if (IsCompressedRGBFormat(format)) {
    if (usage & ONLY_GPU_USAGE_MASK) {
      if (AdrenoMemInfo::GetInstance() == nullptr) {
        ALOGE("Unable to get adreno instance");
        return -1;
      }
      AdrenoMemInfo::GetInstance()->AlignCompressedRGB(width, height, format, alignedw, alignedh);
    } else {
      ALOGE("CompressedRGB format without GPU usage flags");
      return -1;
    }
  }

  int aligned_w = width;
  int aligned_h = height;
  unsigned int alignment = 32;

  // Below should be only YUV family
  switch (format) {
    case static_cast<int>(PixelFormat::YCRCB_420_SP):
      /*
      * Todo: relook this alignment again
      * Change made to unblock the software EIS feature from camera
      * Currently using same alignment as camera doing
      */
      aligned_w = INT(MMM_COLOR_FMT_Y_STRIDE(MMM_COLOR_FMT_NV21, width));
      aligned_h = INT(MMM_COLOR_FMT_Y_SCANLINES(MMM_COLOR_FMT_NV21, height));
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP:
      if (usage & ONLY_GPU_USAGE_MASK) {
        if (AdrenoMemInfo::GetInstance() == nullptr) {
          ALOGE("Unable to get adreno instance");
          return -1;
        }
        alignment = AdrenoMemInfo::GetInstance()->GetGpuPixelAlignment();
        aligned_w = ALIGN(width, alignment);
      } else {
        // Keep default alignment as per current GpuPixelAlignment
        aligned_w = ALIGN(width, 64);
      }
      break;
    case HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO:
      aligned_w = ALIGN(width, alignment);
      break;
    case static_cast<int>(PixelFormat::RAW16):
    case static_cast<int>(PixelFormat::Y16):
    case static_cast<int>(PixelFormat::Y8):
      aligned_w = ALIGN(width, 16);
      break;
    case static_cast<int>(PixelFormat::RAW12):
      aligned_w = ALIGN(width * 12 / 8, 16);
      break;
    case static_cast<int>(PixelFormat::RAW10):
      aligned_w = ALIGN(width * 10 / 8, 16);
      break;
    case HAL_PIXEL_FORMAT_RAW8:
      aligned_w = ALIGN(width, 16);
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_TILED:
      aligned_w = ALIGN(width, 128);
      break;
    case static_cast<int>(PixelFormat::YV12):
      if (AdrenoAlignmentRequired(usage)) {
        if (AdrenoMemInfo::GetInstance() == nullptr) {
          ALOGE("Unable to get adreno instance");
          return -1;
        }
        alignment = AdrenoMemInfo::GetInstance()->GetGpuPixelAlignment();
        aligned_w = ALIGN(width, alignment);
      } else {
        aligned_w = ALIGN(width, 16);
      }
      break;
    case static_cast<int>(PixelFormat::YCBCR_422_SP):
    case HAL_PIXEL_FORMAT_YCrCb_422_SP:
    case static_cast<int>(PixelFormat::YCBCR_422_I):
    case HAL_PIXEL_FORMAT_YCrCb_422_I:
      aligned_w = ALIGN(width, 16);
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_P010:
    case HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS:
      mmm_color_format =
          (usage & GRALLOC_USAGE_PRIVATE_HEIF) ? MMM_COLOR_FMT_P010_512 : MMM_COLOR_FMT_P010;
      aligned_w = INT(MMM_COLOR_FMT_Y_STRIDE(mmm_color_format, width) / 2);
      aligned_h = INT(MMM_COLOR_FMT_Y_SCANLINES(mmm_color_format, height));
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
      mmm_color_format =
          (usage & GRALLOC_USAGE_PRIVATE_HEIF) ? MMM_COLOR_FMT_NV12_512 : MMM_COLOR_FMT_NV12;
      aligned_w = INT(MMM_COLOR_FMT_Y_STRIDE(mmm_color_format, width));
      aligned_h = INT(MMM_COLOR_FMT_Y_SCANLINES(mmm_color_format, height));
      break;
    case HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS:
      aligned_w = INT(MMM_COLOR_FMT_Y_STRIDE(MMM_COLOR_FMT_NV21, width));
      aligned_h = INT(MMM_COLOR_FMT_Y_SCANLINES(MMM_COLOR_FMT_NV21, height));
      break;
    case static_cast<int>(PixelFormat::BLOB):
      break;
    case HAL_PIXEL_FORMAT_NV12_HEIF:
      aligned_w = INT(MMM_COLOR_FMT_Y_STRIDE(MMM_COLOR_FMT_NV12_512, width));
      aligned_h = INT(MMM_COLOR_FMT_Y_SCANLINES(MMM_COLOR_FMT_NV12_512, height));
      break;
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      aligned_w = ALIGN(width, 64);
      aligned_h = ALIGN(height, 64);
      break;
    default:
      break;
  }

  *alignedw = (unsigned int)aligned_w;
  *alignedh = (unsigned int)aligned_h;
  return 0;
}

int GetGpuResourceSizeAndDimensions(const BufferInfo &info, unsigned int *size,
                                    unsigned int *alignedw, unsigned int *alignedh,
                                    GraphicsMetadata *graphics_metadata) {
  int err = GetAlignedWidthAndHeight(info, alignedw, alignedh);
  if (err) {
    return err;
  }

  AdrenoMemInfo *adreno_mem_info = AdrenoMemInfo::GetInstance();
  if (!adreno_mem_info) {
    return -ENOTSUP;
  }
  graphics_metadata->size = adreno_mem_info->AdrenoGetMetadataBlobSize();
  uint64_t adreno_usage = info.usage;
  // If gralloc disables UBWC based on any of the checks,
  // we pass modified usage flag to adreno to convey this.
  int is_ubwc_enabled = IsUBwcEnabled(info.format, info.usage);
  if (!is_ubwc_enabled) {
    adreno_usage &= ~(GRALLOC_USAGE_PRIVATE_ALLOC_UBWC);
  } else {
    adreno_usage |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
  }

  int tile_mode = is_ubwc_enabled;
  if (IsTileRendered(info.format)) {
    tile_mode = true;
  }

  // Call adreno api for populating metadata blob
  // Layer count is for 2D/Cubemap arrays and depth is used for 3D slice
  // Using depth to pass layer_count here
  int ret = adreno_mem_info->AdrenoInitMemoryLayout(graphics_metadata->data, info.width,
                                                    info.height, info.layer_count, /* depth */
                                                    info.format, 1, tile_mode, adreno_usage, 1);
  if (ret != 0) {
    ALOGW("%s Graphics metadata init failed", __FUNCTION__);
    *size = 0;
    return -ENOTSUP;
  }
  // Call adreno api with the metadata blob to get buffer size
  *size = adreno_mem_info->AdrenoGetAlignedGpuBufferSize(graphics_metadata->data);
  return 0;
}

bool CanUseAdrenoForSize(int buffer_type, uint64_t usage) {
  if (buffer_type == BUFFER_TYPE_VIDEO || !GetAdrenoSizeAPIStatus()) {
    return false;
  }

  if ((usage & BufferUsage::PROTECTED) &&
      ((usage & BufferUsage::CAMERA_OUTPUT) || (usage & GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY))) {
    return false;
  }

  if (!IsOnlyGpuUsage(usage)) {
    return false;
  }

  return true;
}

bool GetAdrenoSizeAPIStatus() {
  AdrenoMemInfo *adreno_mem_info = AdrenoMemInfo::GetInstance();
  if (adreno_mem_info) {
    return adreno_mem_info->AdrenoSizeAPIAvaliable();
  }
  return false;
}

bool UseUncached(int format, uint64_t usage) {
  if ((usage & GRALLOC_USAGE_PRIVATE_UNCACHED) || (usage & BufferUsage::PROTECTED)) {
    return true;
  }

  // CPU read rarely
  if ((usage & BufferUsage::CPU_READ_MASK) == static_cast<uint64_t>(BufferUsage::CPU_READ_RARELY)) {
    return true;
  }

  // CPU  write rarely
  if ((usage & BufferUsage::CPU_WRITE_MASK) ==
      static_cast<uint64_t>(BufferUsage::CPU_WRITE_RARELY)) {
    return true;
  }

  if ((usage & BufferUsage::SENSOR_DIRECT_DATA) || (usage & BufferUsage::GPU_DATA_BUFFER)) {
    return true;
  }

  if (format && IsUBwcEnabled(format, usage)) {
    return true;
  }

  return false;
}

uint64_t GetHandleFlags(int format, uint64_t usage) {
  uint64_t priv_flags = 0;

  if (usage & BufferUsage::VIDEO_ENCODER) {
    priv_flags |= qtigralloc::PRIV_FLAGS_VIDEO_ENCODER;
  }

  if (usage & BufferUsage::CAMERA_OUTPUT) {
    priv_flags |= qtigralloc::PRIV_FLAGS_CAMERA_WRITE;
  }

  if (usage & BufferUsage::CAMERA_INPUT) {
    priv_flags |= qtigralloc::PRIV_FLAGS_CAMERA_READ;
  }

  if (usage & BufferUsage::GPU_TEXTURE) {
    priv_flags |= qtigralloc::PRIV_FLAGS_HW_TEXTURE;
  }

  if (usage & GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY) {
    priv_flags |= qtigralloc::PRIV_FLAGS_SECURE_DISPLAY;
  }

  if (IsUBwcEnabled(format, usage)) {
    priv_flags |= qtigralloc::PRIV_FLAGS_UBWC_ALIGNED;
    priv_flags |= qtigralloc::PRIV_FLAGS_TILE_RENDERED;
    if (IsUBwcPISupported(format, usage)) {
      priv_flags |= qtigralloc::PRIV_FLAGS_UBWC_ALIGNED_PI;
    }
  }

  if ((usage & (BufferUsage::VIDEO_ENCODER | BufferUsage::VIDEO_DECODER |
                BufferUsage::CAMERA_OUTPUT | BufferUsage::GPU_RENDER_TARGET))) {
    priv_flags |= qtigralloc::PRIV_FLAGS_NON_CPU_WRITER;
  }

  if (!UseUncached(format, usage)) {
    priv_flags |= qtigralloc::PRIV_FLAGS_CACHED;
  }

  if (IsTileRendered(format)) {
    priv_flags |= qtigralloc::PRIV_FLAGS_TILE_RENDERED;
  }

  return priv_flags;
}

int GetImplDefinedFormat(uint64_t usage, int format) {
  int gr_format = format;

  // If input format is PixelFormat::IMPLEMENTATION_DEFINED then based on
  // the usage bits, gralloc assigns a format.
  if (format == static_cast<int>(PixelFormat::IMPLEMENTATION_DEFINED) ||
      format == static_cast<int>(PixelFormat::YCBCR_420_888)) {
    if ((usage & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC || usage & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC_PI) &&
        format != static_cast<int>(PixelFormat::YCBCR_420_888) &&
        !(usage & GRALLOC_USAGE_PRIVATE_10BIT)) {
      gr_format = HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC;
    } else if (usage & BufferUsage::VIDEO_ENCODER) {
      if (usage & GRALLOC_USAGE_PRIVATE_VIDEO_NV21_ENCODER) {
        gr_format = HAL_PIXEL_FORMAT_NV21_ENCODEABLE;  // NV21
      } else if (usage & GRALLOC_USAGE_PRIVATE_HEIF) {
        gr_format = HAL_PIXEL_FORMAT_NV12_HEIF;
      } else if (format == static_cast<int>(PixelFormat::YCBCR_420_888)) {
#ifdef USE_YCRCB_CAMERA_ENCODE
        if (usage & BufferUsage::CAMERA_OUTPUT) {
          gr_format = HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS;
        } else {
          gr_format = HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS;
        }
#else
       gr_format = HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS;
#endif
      } else {
        gr_format = HAL_PIXEL_FORMAT_NV12_ENCODEABLE;  // NV12
      }
    } else if (usage & BufferUsage::CAMERA_INPUT) {
      if (usage & BufferUsage::CAMERA_OUTPUT) {
        // Assumed ZSL if both producer and consumer camera flags set
        gr_format = HAL_PIXEL_FORMAT_NV21_ZSL;  // NV21
      } else {
        gr_format = static_cast<int>(PixelFormat::YCRCB_420_SP);  // NV21
      }
    } else if (usage & BufferUsage::CAMERA_OUTPUT) {
      if (format == static_cast<int>(PixelFormat::YCBCR_420_888)) {
        if ((usage & BufferUsage::PROTECTED) && (!CanAllocateZSLForSecureCamera())) {
          gr_format = static_cast<int>(PixelFormat::YCRCB_420_SP);  // NV21
        } else {
          gr_format = HAL_PIXEL_FORMAT_NV21_ZSL;  // NV21
        }
      } else {
        gr_format = HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS;  // NV12 preview
      }
    } else if (usage & GRALLOC_USAGE_PRIVATE_10BIT && format != HAL_PIXEL_FORMAT_YCbCr_420_888) {
      if (usage & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC) {
        gr_format = HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC;
      } else {
        gr_format = HAL_PIXEL_FORMAT_YCbCr_420_P010;
      }
    } else if (usage & BufferUsage::COMPOSER_OVERLAY) {
      // XXX: If we still haven't set a format, default to RGBA8888
      gr_format = static_cast<int>(PixelFormat::RGBA_8888);
    } else if (format == static_cast<int>(PixelFormat::YCBCR_420_888)) {
      // If no other usage flags are detected, default the
      // flexible YUV format to NV21_ZSL
      gr_format = HAL_PIXEL_FORMAT_NV21_ZSL;
      ALOGD(
          "Falling back to default YUV format - no camera/video specific format defined, usage "
          "0x%" PRIx64,
          usage);
    }
  }

  return gr_format;
}

int GetCustomFormatFlags(int format, uint64_t usage, int *custom_format, uint64_t *priv_flags) {
  *custom_format = GetImplDefinedFormat(usage, format);
  *priv_flags = GetHandleFlags(*custom_format, usage);

  if (usage & GRALLOC_USAGE_PROTECTED) {
    *priv_flags |= qtigralloc::PRIV_FLAGS_SECURE_BUFFER;
  }

  *priv_flags |= qtigralloc::PRIV_FLAGS_USES_ION;

  return 0;
}

int GetBufferType(int inputFormat) {
  return IsYuvFormat(inputFormat) ? BUFFER_TYPE_VIDEO : BUFFER_TYPE_UI;
}

// Here width and height are aligned width and aligned height.
int GetYUVPlaneInfo(const BufferInfo &info, int32_t format, int32_t width, int32_t height,
                    int32_t interlaced, int *plane_count, PlaneLayoutInfo *plane_info,
                    const private_handle_t *hnd, struct android_ycbcr *ycbcr) {
  int err = 0;
  unsigned int y_stride, c_stride, y_height, c_height, y_size, c_size, mmm_color_format;
  uint64_t yOffset, cOffset, crOffset, cbOffset;
  int h_subsampling = 0, v_subsampling = 0;
  unsigned int alignment = 16;
  uint64_t usage = info.usage;
  if (IsCameraCustomFormat(format, info.usage) && CameraInfo::GetInstance()) {
    int result = CameraInfo::GetInstance()->GetCameraFormatPlaneInfo(
        format, info.width, info.height, plane_count, plane_info);
    if (result == 0) {
      if (hnd != nullptr && ycbcr != nullptr) {
        CopyPlaneLayoutInfotoAndroidYcbcr(hnd->base, *plane_count, plane_info, ycbcr);
        if (format == HAL_PIXEL_FORMAT_NV21_ZSL) {
          std::swap(ycbcr->cb, ycbcr->cr);
        }
      }
    } else {
      ALOGE(
          "%s: Failed to get the plane info through camera library. width: %d, height: %d,"
          "format: %d, Error code: %d",
          __FUNCTION__, width, height, format, result);
    }
    return result;
  }

  if (hnd != nullptr) {
    // Check if UBWC buffer has been rendered in linear format.
    int linear_format = 0;
    if (GetMetaDataValue(const_cast<private_handle_t *>(hnd), QTI_LINEAR_FORMAT, &linear_format) ==
        Error::NONE) {
      format = INT(linear_format);
    }
  }

  switch (format) {
    // Semiplanar
    case HAL_PIXEL_FORMAT_YCbCr_420_SP:
    case static_cast<int32_t>(PixelFormat::YCBCR_422_SP):
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:  // Same as YCbCr_420_SP_VENUS
    case HAL_PIXEL_FORMAT_NV12_HEIF:
    case static_cast<int32_t>(PixelFormat::YCRCB_420_SP):
    case HAL_PIXEL_FORMAT_YCrCb_422_SP:
    case HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO:
    case HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      *plane_count = 2;
      GetYuvSPPlaneInfo(info, format, width, height, 1, plane_info);
      GetYuvSubSamplingFactor(format, &h_subsampling, &v_subsampling);
      plane_info[0].h_subsampling = 0;
      plane_info[0].v_subsampling = 0;
      plane_info[1].h_subsampling = h_subsampling;
      plane_info[1].v_subsampling = v_subsampling;
      break;

    case static_cast<int32_t>(PixelFormat::RAW16):
    case static_cast<int32_t>(PixelFormat::RAW12):
    case static_cast<int32_t>(PixelFormat::RAW10):
    case HAL_PIXEL_FORMAT_RAW8:
      *plane_count = 1;
      GetRawPlaneInfo(format, info.width, info.height, plane_info);
      break;

    case static_cast<int32_t>(PixelFormat::Y8):
      *plane_count = 1;
      GetYuvSPPlaneInfo(info, format, width, height, 1, plane_info);
      GetYuvSubSamplingFactor(format, &h_subsampling, &v_subsampling);
      plane_info[0].h_subsampling = h_subsampling;
      plane_info[0].v_subsampling = v_subsampling;
      break;

    case static_cast<int32_t>(PixelFormat::Y16):
      *plane_count = 1;
      GetYuvSPPlaneInfo(info, format, width, height, 2, plane_info);
      GetYuvSubSamplingFactor(format, &h_subsampling, &v_subsampling);
      plane_info[0].h_subsampling = h_subsampling;
      plane_info[0].v_subsampling = v_subsampling;
      break;

    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH:
      GetYuvSubSamplingFactor(format, &h_subsampling, &v_subsampling);
      if (interlaced & LAYOUT_INTERLACED_FLAG) {
        *plane_count = 8;
        GetYuvUbwcInterlacedSPPlaneInfo(width, height, plane_info);
        plane_info[0].step = plane_info[4].step = 1;
        plane_info[1].step = plane_info[5].step = 2;
        plane_info[0].h_subsampling = plane_info[4].h_subsampling = 0;
        plane_info[0].v_subsampling = plane_info[4].v_subsampling = 0;
        plane_info[1].h_subsampling = plane_info[5].h_subsampling = h_subsampling;
        plane_info[1].v_subsampling = plane_info[5].v_subsampling = v_subsampling;
        plane_info[2].h_subsampling = plane_info[3].h_subsampling = 0;
        plane_info[2].v_subsampling = plane_info[3].v_subsampling = 0;
        plane_info[2].step = plane_info[3].step = 0;
        plane_info[6].h_subsampling = plane_info[7].h_subsampling = 0;
        plane_info[6].v_subsampling = plane_info[7].v_subsampling = 0;
        plane_info[6].step = plane_info[7].step = 0;
      } else {
        *plane_count = 4;
        GetYuvUbwcSPPlaneInfo(width, height, MMM_COLOR_FMT_NV12_UBWC, plane_info);
        plane_info[0].h_subsampling = 0;
        plane_info[0].v_subsampling = 0;
        plane_info[0].step = 1;
        plane_info[1].h_subsampling = h_subsampling;
        plane_info[1].v_subsampling = v_subsampling;
        plane_info[1].step = 2;
        plane_info[2].h_subsampling = plane_info[3].h_subsampling = 0;
        plane_info[2].v_subsampling = plane_info[3].v_subsampling = 0;
        plane_info[2].step = plane_info[3].step = 0;
      }
      break;

    case HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC:
      *plane_count = 4;
      GetYuvUbwcSPPlaneInfo(width, height, MMM_COLOR_FMT_NV12_BPP10_UBWC, plane_info);
      GetYuvSubSamplingFactor(format, &h_subsampling, &v_subsampling);
      plane_info[0].h_subsampling = 0;
      plane_info[0].v_subsampling = 0;
      plane_info[1].step = 1;
      plane_info[1].h_subsampling = h_subsampling;
      plane_info[1].v_subsampling = v_subsampling;
      plane_info[1].step = 3;
      plane_info[2].h_subsampling = plane_info[3].h_subsampling = 0;
      plane_info[2].v_subsampling = plane_info[3].v_subsampling = 0;
      plane_info[2].step = plane_info[3].step = 0;
      break;

    case HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC:
      *plane_count = 4;
      GetYuvUbwcSPPlaneInfo(width, height, MMM_COLOR_FMT_P010_UBWC, plane_info);
      GetYuvSubSamplingFactor(format, &h_subsampling, &v_subsampling);
      plane_info[0].h_subsampling = 0;
      plane_info[0].v_subsampling = 0;
      plane_info[1].step = 1;
      plane_info[1].h_subsampling = h_subsampling;
      plane_info[1].v_subsampling = v_subsampling;
      plane_info[1].step = 4;
      plane_info[2].h_subsampling = plane_info[3].h_subsampling = 0;
      plane_info[2].v_subsampling = plane_info[3].v_subsampling = 0;
      plane_info[2].step = plane_info[3].step = 0;
      break;

    case HAL_PIXEL_FORMAT_YCbCr_420_P010:
    case HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS:
      *plane_count = 2;
      mmm_color_format =
          (info.usage & GRALLOC_USAGE_PRIVATE_HEIF) ? MMM_COLOR_FMT_P010_512 : MMM_COLOR_FMT_P010;
      y_stride = MMM_COLOR_FMT_Y_STRIDE(mmm_color_format, width);
      c_stride = MMM_COLOR_FMT_UV_STRIDE(mmm_color_format, width);
      y_height = MMM_COLOR_FMT_Y_SCANLINES(mmm_color_format, height);
      c_height = MMM_COLOR_FMT_UV_SCANLINES(mmm_color_format, INT(height));
      y_size = y_stride * y_height;
      yOffset = 0;
      cOffset = y_size;
      c_size = c_stride * c_height;
      GetYuvSubSamplingFactor(format, &h_subsampling, &v_subsampling);

      plane_info[0].component = (PlaneComponent)PLANE_COMPONENT_Y;
      plane_info[0].offset = (uint32_t)yOffset;
      plane_info[0].stride = static_cast<int32_t>(UINT(width));
      plane_info[0].stride_bytes = static_cast<int32_t>(y_stride);
      plane_info[0].scanlines = static_cast<int32_t>(y_height);
      plane_info[0].size = static_cast<uint32_t>(y_size);
      plane_info[0].step = 2;
      plane_info[0].h_subsampling = 0;
      plane_info[0].v_subsampling = 0;

      plane_info[1].component = (PlaneComponent)(PLANE_COMPONENT_Cb | PLANE_COMPONENT_Cr);
      plane_info[1].offset = (uint32_t)cOffset;
      plane_info[1].stride = static_cast<int32_t>(UINT(width));
      plane_info[1].stride_bytes = static_cast<int32_t>(c_stride);
      plane_info[1].scanlines = static_cast<int32_t>(c_height);
      plane_info[1].size = static_cast<uint32_t>(c_size);
      plane_info[1].step = 4;
      plane_info[1].h_subsampling = h_subsampling;
      plane_info[1].v_subsampling = v_subsampling;
      break;
    // Planar
    case static_cast<int32_t>(PixelFormat::YV12):
      if ((info.width & 1) || (info.height & 1)) {
        ALOGE("w or h is odd for the YV12 format");
        err = -EINVAL;
        return err;
      }
      if (AdrenoAlignmentRequired(usage)) {
        if (AdrenoMemInfo::GetInstance() == nullptr) {
          ALOGE("Unable to get adreno instance");
          return 0;
        }
        alignment = AdrenoMemInfo::GetInstance()->GetGpuPixelAlignment();
      }
      *plane_count = 3;
      y_stride = width;
      c_stride = ALIGN(width / 2, alignment);
      y_height = UINT(height);
      y_size = (y_stride * y_height);
      height = height >> 1;
      c_height = UINT(height);
      c_size = (c_stride * c_height);
      yOffset = 0;
      crOffset = y_size;
      cbOffset = (y_size + c_size);
      GetYuvSubSamplingFactor(format, &h_subsampling, &v_subsampling);

      plane_info[0].component = (PlaneComponent)PLANE_COMPONENT_Y;
      plane_info[0].offset = (uint32_t)yOffset;
      plane_info[0].stride = static_cast<int32_t>(UINT(width));
      plane_info[0].stride_bytes = static_cast<int32_t>(y_stride);
      plane_info[0].scanlines = static_cast<int32_t>(y_height);
      plane_info[0].size = static_cast<uint32_t>(y_size);
      plane_info[0].step = 1;
      plane_info[0].h_subsampling = 0;
      plane_info[0].v_subsampling = 0;

      plane_info[1].component = (PlaneComponent)PLANE_COMPONENT_Cb;
      plane_info[1].offset = (uint32_t)cbOffset;
      plane_info[2].component = (PlaneComponent)PLANE_COMPONENT_Cr;
      plane_info[2].offset = (uint32_t)crOffset;
      for (int i = 1; i < 3; i++) {
        plane_info[i].stride = static_cast<int32_t>(UINT(width));
        plane_info[i].stride_bytes = static_cast<int32_t>(c_stride);
        plane_info[i].scanlines = static_cast<int32_t>(c_height);
        plane_info[i].size = static_cast<uint32_t>(c_size);
        plane_info[i].step = 1;
        plane_info[i].h_subsampling = h_subsampling;
        plane_info[i].v_subsampling = v_subsampling;
      }
      break;
    case HAL_PIXEL_FORMAT_CbYCrY_422_I:
      if (info.width & 1) {
        ALOGE("width is odd for the YUV422_SP format");
        err = -EINVAL;
        return err;
      }
      *plane_count = 1;
      y_stride = width * 2;
      y_height = UINT(height);
      y_size = y_stride * y_height;
      yOffset = 0;
      plane_info[0].component = (PlaneComponent)PLANE_COMPONENT_Y;
      plane_info[0].offset = (uint32_t)yOffset;
      plane_info[0].stride = static_cast<int32_t>(UINT(width));
      plane_info[0].stride_bytes = static_cast<int32_t>(y_stride);
      plane_info[0].scanlines = static_cast<int32_t>(y_height);
      plane_info[0].size = static_cast<uint32_t>(y_size);
      plane_info[0].step = 1;
      plane_info[0].h_subsampling = 0;
      plane_info[0].v_subsampling = 0;
      break;

    // Unsupported formats
    case static_cast<int32_t>(PixelFormat::YCBCR_422_I):
    case HAL_PIXEL_FORMAT_YCrCb_422_I:
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_TILED:
    default:
      *plane_count = 0;
      ALOGD("%s: Invalid format passed: 0x%x", __FUNCTION__, format);
      err = -EINVAL;
  }
  if (err == 0 && hnd != nullptr && ycbcr != nullptr) {
    if ((interlaced & LAYOUT_INTERLACED_FLAG) &&
        (format == HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC || IsUbwcFlexFormat(format))) {
      CopyPlaneLayoutInfotoAndroidYcbcr(hnd->base, *plane_count, plane_info, ycbcr);
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
      CopyPlaneLayoutInfotoAndroidYcbcr(field_base, *plane_count, &plane_info[4], &ycbcr[1]);
    } else {
      CopyPlaneLayoutInfotoAndroidYcbcr(hnd->base, *plane_count, plane_info, ycbcr);
      switch (format) {
        case static_cast<int>(PixelFormat::YCRCB_420_SP):
        case HAL_PIXEL_FORMAT_YCrCb_422_SP:
        case HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO:
        case HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS:
        case HAL_PIXEL_FORMAT_NV21_ZSL:
          std::swap(ycbcr->cb, ycbcr->cr);
      }
    }
  }
  return err;
}

void GetYuvSubSamplingFactor(int32_t format, int *h_subsampling, int *v_subsampling) {
  switch (format) {
    case HAL_PIXEL_FORMAT_YCbCr_420_SP:
    case HAL_PIXEL_FORMAT_YCbCr_420_P010:
    case HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC:
    case HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC:
    case HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS:
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
    case HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO:
    case HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS:
    case static_cast<int32_t>(PixelFormat::YCRCB_420_SP):
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:  // Same as YCbCr_420_SP_VENUS
    case static_cast<int32_t>(PixelFormat::YV12):
    case HAL_PIXEL_FORMAT_NV12_HEIF:
    case HAL_PIXEL_FORMAT_NV21_ZSL:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH:
      *h_subsampling = 1;
      *v_subsampling = 1;
      break;
    case static_cast<int32_t>(PixelFormat::YCBCR_422_SP):
    case HAL_PIXEL_FORMAT_YCrCb_422_SP:
    case HAL_PIXEL_FORMAT_CbYCrY_422_I:
      *h_subsampling = 1;
      *v_subsampling = 0;
      break;
    case static_cast<int32_t>(PixelFormat::Y16):
    case static_cast<int32_t>(PixelFormat::Y8):
    case static_cast<int32_t>(PixelFormat::BLOB):
    default:
      *h_subsampling = 0;
      *v_subsampling = 0;
      break;
  }
}

void CopyPlaneLayoutInfotoAndroidYcbcr(uint64_t base, int plane_count, PlaneLayoutInfo *plane_info,
                                       struct android_ycbcr *ycbcr) {
  ycbcr->y = reinterpret_cast<void *>(base + plane_info[0].offset);
  ycbcr->ystride = plane_info[0].stride_bytes;
  if (plane_count == 1) {
    ycbcr->cb = NULL;
    ycbcr->cr = NULL;
    ycbcr->cstride = 0;
    ycbcr->chroma_step = 0;
  } else if (plane_count == 2 || plane_count == 4 || plane_count == 8) {
    /* For YUV semiplanar :-
     *   - In progressive & linear case plane count is 2 and plane_info[0] will
     *     contain info about Y plane and plane_info[1] will contain info about UV plane.
     *   - In progressive & compressed case plane count is 4 then plane_info[0] will
     *     contain info about Y plane and plane_info[1] will contain info about UV plane.
     *     Remaining two plane (plane_info[2] & plane_info[3]) contain info about the
     *     Y_Meta_Plane and UV_Meta_Plane.
     *   - In interlaced & compressed case plane count is 8 then plane_info[0], plane_info[1],
     *     plane_info[4] & plane_info[5] will contain info about Y_plane, UV_plane, Y_plane
     *     & UV_plane. Remaining plane will contain info about the meta planes. As in this case
     *     this API is called twice through GetYUVPlaneInfo() with address of plane_info[0] &
     *     plane_info[4], so this will calculate the information accordingly and will fill the
     *     ycbcr structure with interlaced plane info only.
     */
    ycbcr->cb = reinterpret_cast<void *>(base + plane_info[1].offset);
    ycbcr->cr = reinterpret_cast<void *>(base + plane_info[1].offset + 1);
    ycbcr->cstride = plane_info[1].stride_bytes;
    ycbcr->chroma_step = plane_info[1].step;
  } else if (plane_count == 3) {
    /* For YUV planar :-
     * Plane size is 3 and plane_info[0], plane_info[1], plane_info[2] will
     * contain info about y_plane, cb_plane and cr_plane accordingly.
     */
    ycbcr->cb = reinterpret_cast<void *>(base + plane_info[1].offset);
    ycbcr->cr = reinterpret_cast<void *>(base + plane_info[2].offset);
    ycbcr->cstride = plane_info[1].stride_bytes;
    ycbcr->chroma_step = plane_info[1].step;
  }
}

bool HasAlphaComponent(int32_t format) {
  switch (format) {
    case static_cast<int32_t>(PixelFormat::RGBA_8888):
    case static_cast<int32_t>(PixelFormat::BGRA_8888):
    case HAL_PIXEL_FORMAT_RGBA_5551:
    case HAL_PIXEL_FORMAT_RGBA_4444:
    case static_cast<int32_t>(PixelFormat::RGBA_1010102):
    case HAL_PIXEL_FORMAT_ARGB_2101010:
    case HAL_PIXEL_FORMAT_BGRA_1010102:
    case HAL_PIXEL_FORMAT_ABGR_2101010:
    case static_cast<int32_t>(PixelFormat::RGBA_FP16):
      return true;
    default:
      return false;
  }
}

void GetRGBPlaneInfo(const BufferInfo &info, int32_t format, int32_t width, int32_t height,
                     int32_t /* flags */, int *plane_count, PlaneLayoutInfo *plane_info) {
  uint64_t usage = info.usage;
  *plane_count = 1;
  plane_info->component =
      (PlaneComponent)(PLANE_COMPONENT_R | PLANE_COMPONENT_G | PLANE_COMPONENT_B);
  if (HasAlphaComponent(format)) {
    plane_info->component = (PlaneComponent)(plane_info->component | PLANE_COMPONENT_A);
  }
  GetBufferSizeAndDimensions(info, &(plane_info->size), (unsigned int *)&width,
                             (unsigned int *)&height);
  plane_info->step = GetBpp(format);
  plane_info->offset = GetRgbMetaSize(format, width, height, usage);
  plane_info->h_subsampling = 0;
  plane_info->v_subsampling = 0;
  plane_info->stride = width;
  plane_info->stride_bytes = width * plane_info->step;
  plane_info->scanlines = height;
}

// TODO(user): tile vs ubwc -- may need to find a diff way to differentiate
void GetDRMFormat(uint32_t format, uint32_t flags, uint32_t *drm_format,
                  uint64_t *drm_format_modifier) {
  bool compressed = (flags & qtigralloc::PRIV_FLAGS_UBWC_ALIGNED) ? true : false;
  switch (format) {
    case static_cast<uint32_t>(PixelFormat::RGBA_8888):
      *drm_format = DRM_FORMAT_ABGR8888;
      break;
    case HAL_PIXEL_FORMAT_RGBA_5551:
      *drm_format = DRM_FORMAT_ABGR1555;
      break;
    case HAL_PIXEL_FORMAT_RGBA_4444:
      *drm_format = DRM_FORMAT_ABGR4444;
      break;
    case static_cast<uint32_t>(PixelFormat::BGRA_8888):
      *drm_format = DRM_FORMAT_ARGB8888;
      break;
    case static_cast<uint32_t>(PixelFormat::RGBX_8888):
      *drm_format = DRM_FORMAT_XBGR8888;
      if (compressed)
        *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case HAL_PIXEL_FORMAT_BGRX_8888:
      *drm_format = DRM_FORMAT_XRGB8888;
      break;
    case static_cast<uint32_t>(PixelFormat::RGB_888):
      *drm_format = DRM_FORMAT_BGR888;
      break;
    case static_cast<uint32_t>(PixelFormat::RGB_565):
      *drm_format = DRM_FORMAT_BGR565;
      break;
    case HAL_PIXEL_FORMAT_BGR_565:
      *drm_format = DRM_FORMAT_BGR565;
      if (compressed)
        *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case static_cast<uint32_t>(PixelFormat::RGBA_1010102):
      *drm_format = DRM_FORMAT_ABGR2101010;
      if (compressed)
        *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case HAL_PIXEL_FORMAT_ARGB_2101010:
      *drm_format = DRM_FORMAT_BGRA1010102;
      break;
    case HAL_PIXEL_FORMAT_RGBX_1010102:
      *drm_format = DRM_FORMAT_XBGR2101010;
      if (compressed)
        *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case HAL_PIXEL_FORMAT_XRGB_2101010:
      *drm_format = DRM_FORMAT_BGRX1010102;
      break;
    case HAL_PIXEL_FORMAT_BGRA_1010102:
      *drm_format = DRM_FORMAT_ARGB2101010;
      break;
    case HAL_PIXEL_FORMAT_ABGR_2101010:
      *drm_format = DRM_FORMAT_RGBA1010102;
      break;
    case HAL_PIXEL_FORMAT_BGRX_1010102:
      *drm_format = DRM_FORMAT_XRGB2101010;
      break;
    case HAL_PIXEL_FORMAT_XBGR_2101010:
      *drm_format = DRM_FORMAT_RGBX1010102;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
      *drm_format = DRM_FORMAT_NV12;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH:
      *drm_format = DRM_FORMAT_NV12;
      if (compressed) {
        *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      } else {
        *drm_format_modifier = DRM_FORMAT_MOD_QCOM_TILE;
      }
      break;
    case static_cast<uint32_t>(PixelFormat::YCRCB_420_SP):
      *drm_format = DRM_FORMAT_NV21;
      break;
    case HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS:
      *drm_format = DRM_FORMAT_NV21;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_P010:
    case HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_DX;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC:
      *drm_format = DRM_FORMAT_NV12;
      if (compressed) {
        *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED | DRM_FORMAT_MOD_QCOM_DX;
      } else {
        *drm_format_modifier = DRM_FORMAT_MOD_QCOM_TILE | DRM_FORMAT_MOD_QCOM_DX;
      }
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC:
      *drm_format = DRM_FORMAT_NV12;
      if (compressed) {
        *drm_format_modifier =
            DRM_FORMAT_MOD_QCOM_COMPRESSED | DRM_FORMAT_MOD_QCOM_DX | DRM_FORMAT_MOD_QCOM_TIGHT;
      } else {
        *drm_format_modifier =
            DRM_FORMAT_MOD_QCOM_TILE | DRM_FORMAT_MOD_QCOM_DX | DRM_FORMAT_MOD_QCOM_TIGHT;
      }
      break;
    case static_cast<uint32_t>(PixelFormat::YCBCR_422_SP):
      *drm_format = DRM_FORMAT_NV16;
      break;
    /*
  TODO: No HAL_PIXEL_FORMAT equivalent?
  case kFormatYCrCb422H2V1SemiPlanar:
    *drm_format = DRM_FORMAT_NV61;
    break;*/
    case static_cast<uint32_t>(PixelFormat::YV12):
      *drm_format = DRM_FORMAT_YVU420;
      break;
    case static_cast<uint32_t>(PixelFormat::RGBA_FP16):
      *drm_format = DRM_FORMAT_ABGR16161616F;
      break;
    default:
      ALOGE("%s: Unsupported format %d", __FUNCTION__, format);
  }
}

bool CanAllocateZSLForSecureCamera() {
  static bool inited = false;
  static bool can_allocate = true;
  if (inited) {
    return can_allocate;
  }
  char property[PROPERTY_VALUE_MAX];
  property_get(SECURE_PREVIEW_BUFFER_FORMAT_PROP, property, "0");
  if (!(strncmp(property, "420_sp", PROPERTY_VALUE_MAX))) {
    can_allocate = false;
  }
  inited = true;
  ALOGI("CanAllocateZSLForSecureCamera: %d", can_allocate);

  return can_allocate;
}

uint64_t GetCustomContentMetadataSize(int format, uint64_t usage) {
  #ifndef METADATA_V2
    return 0;
  #else
    static uint64_t VALID_USAGES = (BufferUsage::VIDEO_ENCODER | BufferUsage::VIDEO_DECODER |
                                    BufferUsage::CAMERA_OUTPUT);

    if (IsYuvFormat(format) && (usage & VALID_USAGES)) {
      return sizeof(CustomContentMetadata);
    } else {
      return 0;
    }
  #endif
}

uint64_t GetMetaDataSize(uint64_t reserved_region_size, uint64_t custom_content_md_region_size) {
// Only include the reserved region size when using Metadata_t V2
#ifndef METADATA_V2
  reserved_region_size = 0;
#endif
  return static_cast<uint64_t>(ROUND_UP_PAGESIZE(sizeof(MetaData_t) + reserved_region_size +
                                                 custom_content_md_region_size));
}

void UnmapAndReset(private_handle_t *handle) {
  uint64_t reserved_region_size = handle->reserved_size;
  if (private_handle_t::validate(handle) == 0 && handle->base_metadata) {
    munmap(reinterpret_cast<void *>(handle->base_metadata),
           GetMetaDataSize(reserved_region_size, handle->custom_content_md_reserved_size));
    handle->base_metadata = 0;
  }
}

int ValidateAndMap(private_handle_t *handle) {
  if (private_handle_t::validate(handle)) {
    ALOGE("%s: Private handle is invalid - handle:%p", __func__, handle);
    return -1;
  }
  if (handle->fd_metadata < 0) {
    // Silently return, metadata cannot be used
    return -1;
  }

  if (!handle->base_metadata) {
    uint64_t reserved_region_size = handle->reserved_size;
    uint64_t size = GetMetaDataSize(reserved_region_size, handle->custom_content_md_reserved_size);
    void *base = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, handle->fd_metadata, 0);
    if (base == reinterpret_cast<void *>(MAP_FAILED)) {
      ALOGE("%s: metadata mmap failed - handle:%p fd: %d err: %s", __func__, handle,
            handle->fd_metadata, strerror(errno));
      return -1;
    }
    handle->base_metadata = (uintptr_t)base;
  }
  return 0;
}

int colorMetaDataToColorSpace(ColorMetaData in, int *out) {
  if (in.colorPrimaries == ColorPrimaries_BT601_6_525 ||
      in.colorPrimaries == ColorPrimaries_BT601_6_625) {
    if (in.range == Range_Full) {
      *out = HAL_CSC_ITU_R_601_FR;
    } else {
      *out = HAL_CSC_ITU_R_601;
    }
  } else if (in.colorPrimaries == ColorPrimaries_BT2020) {
    if (in.range == Range_Full) {
      *out = HAL_CSC_ITU_R_2020_FR;
    } else {
      *out = HAL_CSC_ITU_R_2020;
    }
  } else if (in.colorPrimaries == ColorPrimaries_BT709_5) {
    if (in.range == Range_Full) {
      *out = HAL_CSC_ITU_R_709_FR;
    } else {
      *out = HAL_CSC_ITU_R_709;
    }
  } else {
    ALOGE(
        "Cannot convert ColorMetaData to ColorSpace_t. "
        "Primaries = %d, Range = %d",
        in.colorPrimaries, in.range);
    return -1;
  }

  return 0;
}

int colorSpaceToColorMetadata(int in, ColorMetaData *out) {
  out->transfer = Transfer_sRGB;
  switch (in) {
    case HAL_CSC_ITU_R_601:
      out->colorPrimaries = ColorPrimaries_BT601_6_525;
      out->range = Range_Limited;
      break;
    case HAL_CSC_ITU_R_601_FR:
      out->colorPrimaries = ColorPrimaries_BT601_6_525;
      out->range = Range_Full;
      break;
    case HAL_CSC_ITU_R_709:
      out->colorPrimaries = ColorPrimaries_BT709_5;
      out->range = Range_Limited;
      break;
    case HAL_CSC_ITU_R_709_FR:
      out->colorPrimaries = ColorPrimaries_BT709_5;
      out->range = Range_Full;
      break;
    case HAL_CSC_ITU_R_2020:
      out->colorPrimaries = ColorPrimaries_BT2020;
      out->range = Range_Limited;
      break;
    case HAL_CSC_ITU_R_2020_FR:
      out->colorPrimaries = ColorPrimaries_BT2020;
      out->range = Range_Full;
      break;
    default:
      ALOGE("Cannot convert ColorSpace_t %d to ColorMetaData", in);
      return -1;
      break;
  }

  return 0;
}

bool getGralloc4Array(MetaData_t *metadata, int64_t paramType) {
  switch (paramType) {
    case (int64_t)StandardMetadataType::CROP:
      return metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(
          ::android::gralloc4::MetadataType_Crop.value)];
    case (int64_t)StandardMetadataType::BLEND_MODE:
      return metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(
          ::android::gralloc4::MetadataType_BlendMode.value)];
    case QTI_VT_TIMESTAMP:
    case QTI_PP_PARAM_INTERLACED:
    case QTI_VIDEO_PERF_MODE:
    case QTI_GRAPHICS_METADATA:
    case QTI_UBWC_CR_STATS_INFO:
    case QTI_REFRESH_RATE:
    case QTI_MAP_SECURE_BUFFER:
    case QTI_LINEAR_FORMAT:
    case QTI_SINGLE_BUFFER_MODE:
    case QTI_CVP_METADATA:
    case QTI_VIDEO_HISTOGRAM_STATS:
#ifdef QTI_VIDEO_TRANSCODE_STATS
    case QTI_VIDEO_TRANSCODE_STATS:
#endif
    case QTI_VIDEO_TS_INFO:
    case QTI_S3D_FORMAT:
    case QTI_BUFFER_PERMISSION:
      return metadata->isVendorMetadataSet[GET_VENDOR_METADATA_STATUS_INDEX(paramType)];
    case QTI_COLOR_METADATA:
      return metadata->isVendorMetadataSet[GET_VENDOR_METADATA_STATUS_INDEX(QTI_COLOR_METADATA)] ||
             metadata->isStandardMetadataSet[
             GET_STANDARD_METADATA_STATUS_INDEX((int64_t)StandardMetadataType::DATASPACE)] ||
             metadata->isStandardMetadataSet[
             GET_STANDARD_METADATA_STATUS_INDEX((int64_t)StandardMetadataType::SMPTE2086)] ||
             metadata->isStandardMetadataSet[
             GET_STANDARD_METADATA_STATUS_INDEX((int64_t)StandardMetadataType::CTA861_3)] ||
             metadata->isStandardMetadataSet[
             GET_STANDARD_METADATA_STATUS_INDEX((int64_t)StandardMetadataType::SMPTE2094_40)];
    case QTI_COLORSPACE:
      // QTI_COLORSPACE is derived from QTI_COLOR_METADATA
      return metadata->isVendorMetadataSet[GET_VENDOR_METADATA_STATUS_INDEX(QTI_COLOR_METADATA)];
  // Following metadata types are not changed after allocation - treat as set by default
    case (int64_t)StandardMetadataType::BUFFER_ID:
    case (int64_t)StandardMetadataType::NAME:
    case (int64_t)StandardMetadataType::WIDTH:
    case (int64_t)StandardMetadataType::HEIGHT:
    case (int64_t)StandardMetadataType::LAYER_COUNT:
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_REQUESTED:
    case (int64_t)StandardMetadataType::USAGE:
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_FOURCC:
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_MODIFIER:
    case (int64_t)StandardMetadataType::PROTECTED_CONTENT:
    case (int64_t)StandardMetadataType::ALLOCATION_SIZE:
    case QTI_FD:
    case QTI_PRIVATE_FLAGS:
    case QTI_ALIGNED_WIDTH_IN_PIXELS:
    case QTI_ALIGNED_HEIGHT_IN_PIXELS:
    case QTI_STANDARD_METADATA_STATUS:
    case QTI_VENDOR_METADATA_STATUS:
    case QTI_RGB_DATA_ADDRESS:
    case QTI_YUV_PLANE_INFO:
    case QTI_CUSTOM_DIMENSIONS_STRIDE:
    case QTI_CUSTOM_DIMENSIONS_HEIGHT:
    case QTI_BUFFER_TYPE:
    case (int64_t)StandardMetadataType::DATASPACE:
    case (int64_t)StandardMetadataType::PLANE_LAYOUTS:
#ifdef QTI_MEM_HANDLE
    case QTI_MEM_HANDLE:
#endif
      return true;
    default:
      ALOGE("paramType %d not supported", paramType);
      return false;
  }
}

Error GetMetaDataByReference(void *buffer, int64_t type, void **out) {
  return GetMetaDataInternal(buffer, type, nullptr, out);
}

Error GetMetaDataValue(void *buffer, int64_t type, void *in) {
  return GetMetaDataInternal(buffer, type, in, nullptr);
}

Error ColorMetadataToDataspace(ColorMetaData color_metadata, Dataspace *dataspace) {
  Dataspace primaries, transfer, range = Dataspace::UNKNOWN;

  switch (color_metadata.colorPrimaries) {
    case ColorPrimaries_BT709_5:
      primaries = Dataspace::STANDARD_BT709;
      break;
    // TODO(user): verify this is equivalent
    case ColorPrimaries_BT470_6M:
      primaries = Dataspace::STANDARD_BT470M;
      break;
    case ColorPrimaries_BT601_6_625:
      primaries = Dataspace::STANDARD_BT601_625;
      break;
    case ColorPrimaries_BT601_6_525:
      primaries = Dataspace::STANDARD_BT601_525;
      break;
    case ColorPrimaries_GenericFilm:
      primaries = Dataspace::STANDARD_FILM;
      break;
    case ColorPrimaries_BT2020:
      primaries = Dataspace::STANDARD_BT2020;
      break;
    case ColorPrimaries_AdobeRGB:
      primaries = Dataspace::STANDARD_ADOBE_RGB;
      break;
    case ColorPrimaries_DCIP3:
      primaries = Dataspace::STANDARD_DCI_P3;
      break;
    default:
      return Error::UNSUPPORTED;
      /*
       ColorPrimaries_SMPTE_240M;
       ColorPrimaries_SMPTE_ST428;
       ColorPrimaries_EBU3213;
      */
  }

  switch (color_metadata.transfer) {
    case Transfer_sRGB:
      transfer = Dataspace::TRANSFER_SRGB;
      break;
    case Transfer_Gamma2_2:
      transfer = Dataspace::TRANSFER_GAMMA2_2;
      break;
    case Transfer_Gamma2_8:
      transfer = Dataspace::TRANSFER_GAMMA2_8;
      break;
    case Transfer_SMPTE_170M:
      transfer = Dataspace::TRANSFER_SMPTE_170M;
      break;
    case Transfer_Linear:
      transfer = Dataspace::TRANSFER_LINEAR;
      break;
    case Transfer_HLG:
      transfer = Dataspace::TRANSFER_HLG;
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

  switch (color_metadata.range) {
    case Range_Full:
      range = Dataspace::RANGE_FULL;
      break;
    case Range_Limited:
      range = Dataspace::RANGE_LIMITED;
      break;
    case Range_Extended:
      range = Dataspace::RANGE_EXTENDED;
      break;
    default:
      return Error::UNSUPPORTED;
  }

  *dataspace = (Dataspace)((uint32_t)primaries | (uint32_t)transfer | (uint32_t)range);
  return Error::NONE;
}

static Error getComponentSizeAndOffset(int32_t format, PlaneLayoutComponent &comp) {
  switch (format) {
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGBA_8888):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGBX_8888):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGB_888):
      comp.sizeInBits = 8;
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.offsetInBits = 8;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.offsetInBits = 16;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value &&
                 format != HAL_PIXEL_FORMAT_RGB_888) {
        comp.offsetInBits = 24;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGB_565):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 5;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.offsetInBits = 5;
        comp.sizeInBits = 6;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.offsetInBits = 11;
        comp.sizeInBits = 5;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_BGR_565):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.offsetInBits = 11;
        comp.sizeInBits = 5;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.offsetInBits = 5;
        comp.sizeInBits = 6;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 5;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_BGRA_8888):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_BGRX_8888):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_BGR_888):
      comp.sizeInBits = 8;
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.offsetInBits = 16;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.offsetInBits = 8;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value &&
                 format != HAL_PIXEL_FORMAT_BGR_888) {
        comp.offsetInBits = 24;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGBA_5551):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.sizeInBits = 5;
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.sizeInBits = 5;
        comp.offsetInBits = 5;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.sizeInBits = 5;
        comp.offsetInBits = 10;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value) {
        comp.sizeInBits = 1;
        comp.offsetInBits = 15;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGBA_4444):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.sizeInBits = 4;
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.sizeInBits = 4;
        comp.offsetInBits = 4;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.sizeInBits = 4;
        comp.offsetInBits = 8;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value) {
        comp.sizeInBits = 4;
        comp.offsetInBits = 12;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_R_8):
    case static_cast<int32_t>(aidl::android::hardware::graphics::common::PixelFormat::R_8):
      comp.sizeInBits = 8;
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.offsetInBits = 0;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RG_88):
      comp.sizeInBits = 8;
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.offsetInBits = 8;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGBA_1010102):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGBX_1010102):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 10;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 20;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value) {
        comp.sizeInBits = 2;
        comp.offsetInBits = 30;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_ARGB_2101010):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_XRGB_2101010):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 2;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 12;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 22;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value) {
        comp.sizeInBits = 2;
        comp.offsetInBits = 0;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_BGRA_1010102):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_BGRX_1010102):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 20;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 10;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value) {
        comp.sizeInBits = 2;
        comp.offsetInBits = 30;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_ABGR_2101010):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_XBGR_2101010):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 22;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 12;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 2;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value) {
        comp.sizeInBits = 2;
        comp.offsetInBits = 0;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGBA_FP16):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.sizeInBits = 16;
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.sizeInBits = 16;
        comp.offsetInBits = 16;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.sizeInBits = 16;
        comp.offsetInBits = 32;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value) {
        comp.sizeInBits = 16;
        comp.offsetInBits = 48;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCbCr_420_SP):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCbCr_422_SP):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_NV12_ENCODEABLE):
      comp.sizeInBits = 8;
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_Y.value ||
          comp.type.value == android::gralloc4::PlaneLayoutComponentType_CB.value) {
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_CR.value) {
        comp.offsetInBits = 8;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCrCb_420_SP):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCrCb_422_SP):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_NV21_ZSL):
      comp.sizeInBits = 8;
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_Y.value ||
          comp.type.value == android::gralloc4::PlaneLayoutComponentType_CR.value) {
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_CB.value) {
        comp.offsetInBits = 8;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_Y16):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_Y.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 16;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YV12):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_Y.value ||
          comp.type.value == android::gralloc4::PlaneLayoutComponentType_CB.value ||
          comp.type.value == android::gralloc4::PlaneLayoutComponentType_CR.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 8;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_Y8):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_Y.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 8;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCbCr_420_P010):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_Y.value ||
          comp.type.value == android::gralloc4::PlaneLayoutComponentType_CB.value) {
        comp.offsetInBits = 6;
        comp.sizeInBits = 10;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_CR.value) {
        comp.offsetInBits = 22;
        comp.sizeInBits = 10;
      } else {
        return Error::BAD_VALUE;
      }
      break;

    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RAW16):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_RAW.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 16;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RAW12):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RAW10):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_RAW.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = -1;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RAW8):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_RAW.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 8;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    default:
      ALOGD_IF(DEBUG, "Offset and size in bits unknown for format %d", format);
      return Error::UNSUPPORTED;
  }
  return Error::NONE;
}

static void grallocToStandardPlaneLayoutComponentType(uint32_t in,
                                                      std::vector<PlaneLayoutComponent> *components,
                                                      int32_t format) {
  PlaneLayoutComponent comp;
  comp.offsetInBits = -1;
  comp.sizeInBits = -1;

  if (in & PLANE_COMPONENT_Y) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_Y;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_Cb) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_CB;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_Cr) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_CR;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_R) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_R;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_G) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_G;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_B) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_B;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_A) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_A;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_RAW) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_RAW;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_META) {
    comp.type = qtigralloc::PlaneLayoutComponentType_Meta;
    components->push_back(comp);
  }
}

Error GetPlaneLayout(private_handle_t *handle,
                     std::vector<aidl::android::hardware::graphics::common::PlaneLayout> *out) {
  std::vector<PlaneLayout> plane_info;
  int plane_count = 0;
  BufferInfo info(handle->unaligned_width, handle->unaligned_height, handle->format, handle->usage);

  gralloc::PlaneLayoutInfo plane_layout[8] = {};
  if (gralloc::IsYuvFormat(handle->format)) {
    gralloc::GetYUVPlaneInfo(info, handle->format, handle->width, handle->height, handle->flags,
                             &plane_count, plane_layout, handle);
  } else if (gralloc::IsUncompressedRGBFormat(handle->format) ||
             gralloc::IsCompressedRGBFormat(handle->format)) {
    gralloc::GetRGBPlaneInfo(info, handle->format, handle->width, handle->height, handle->flags,
                             &plane_count, plane_layout);
  } else {
    return Error::BAD_BUFFER;
  }
  plane_info.resize(plane_count);
  for (int i = 0; i < plane_count; i++) {
    std::vector<PlaneLayoutComponent> components;
    grallocToStandardPlaneLayoutComponentType(plane_layout[i].component, &plane_info[i].components,
                                              handle->format);
    plane_info[i].horizontalSubsampling = (1ull << plane_layout[i].h_subsampling);
    plane_info[i].verticalSubsampling = (1ull << plane_layout[i].v_subsampling);
    plane_info[i].offsetInBytes = static_cast<int64_t>(plane_layout[i].offset);
    plane_info[i].sampleIncrementInBits = static_cast<int64_t>(plane_layout[i].step * 8);
    plane_info[i].strideInBytes = static_cast<int64_t>(plane_layout[i].stride_bytes);
    plane_info[i].totalSizeInBytes = static_cast<int64_t>(plane_layout[i].size);
    plane_info[i].widthInSamples = handle->unaligned_width >> plane_layout[i].h_subsampling;
    plane_info[i].heightInSamples = handle->unaligned_height >> plane_layout[i].v_subsampling;
  }
  *out = plane_info;
  return Error::NONE;
}

Error GetMetaDataInternal(void *buffer, int64_t type, void *in, void **out) {
  if (!in && !out) {
    ALOGE("Invalid input params");
    return Error::UNSUPPORTED;
  }

  // Make sure we send 0 only if the operation queried is present
  auto ret = Error::BAD_VALUE;
  if (buffer == nullptr) {
    return ret;
  }

  private_handle_t *handle = static_cast<private_handle_t *>(buffer);
  MetaData_t *data = reinterpret_cast<MetaData_t *>(handle->base_metadata);

  int err = ValidateAndMap(handle);
  if (err != 0) {
    return Error::UNSUPPORTED;
  }

  if (data == nullptr) {
    return ret;
  }

  bool copy = true;
  if (in == nullptr) {
    // Get by reference - do not check if metadata has been set
    copy = false;
  } else {
    if (!getGralloc4Array(data, type)) {
      return ret;
    }
  }

  ret = Error::NONE;

  switch (type) {
    case QTI_PP_PARAM_INTERLACED:
      if (copy) {
        *(reinterpret_cast<int32_t *>(in)) = data->interlaced;
      } else {
        *out = &data->interlaced;
      }
      break;
    case (int64_t)StandardMetadataType::CROP:
      if (copy) {
        *(reinterpret_cast<CropRectangle_t *>(in)) = data->crop;
      }
      break;
    case QTI_REFRESH_RATE:
      if (copy) {
        *(reinterpret_cast<float *>(in)) = data->refreshrate;
      } else {
        *out = &data->refreshrate;
      }
      break;
    case QTI_MAP_SECURE_BUFFER:
      if (copy) {
        *(reinterpret_cast<int32_t *>(in)) = data->mapSecureBuffer;
      } else {
        *out = &data->mapSecureBuffer;
      }
      break;
    case QTI_LINEAR_FORMAT:
      if (copy) {
        *(reinterpret_cast<uint32_t *>(in)) = data->linearFormat;
      } else {
        *out = &data->linearFormat;
      }
      break;
    case QTI_SINGLE_BUFFER_MODE:
      if (copy) {
        *(reinterpret_cast<uint32_t *>(in)) = data->isSingleBufferMode;
      } else {
        *out = &data->isSingleBufferMode;
      }
      break;
    case QTI_VT_TIMESTAMP:
      if (copy) {
        *(reinterpret_cast<uint64_t *>(in)) = data->vtTimeStamp;
      } else {
        *out = &data->vtTimeStamp;
      }
      break;
    case QTI_COLOR_METADATA:
      if (copy) {
        *(reinterpret_cast<ColorMetaData *>(in)) = data->color;
      } else {
        *out = &data->color;
      }
      break;
    case QTI_UBWC_CR_STATS_INFO: {
      if (copy) {
        struct UBWCStats *stats = (struct UBWCStats *)in;
        int numelems = sizeof(data->ubwcCRStats) / sizeof(struct UBWCStats);
        for (int i = 0; i < numelems; i++) {
          stats[i] = data->ubwcCRStats[i];
        }
      } else {
        *out = &data->ubwcCRStats[0];
      }
      break;
    }
    case QTI_VIDEO_PERF_MODE:
      if (copy) {
        *(reinterpret_cast<uint32_t *>(in)) = data->isVideoPerfMode;
      } else {
        *out = &data->isVideoPerfMode;
      }
      break;
    case QTI_GRAPHICS_METADATA:
      if (copy) {
        memcpy(in, data->graphics_metadata.data, sizeof(data->graphics_metadata.data));
      } else {
        *out = &data->graphics_metadata;
      }
      break;
    case QTI_CVP_METADATA: {
      if (copy) {
        struct CVPMetadata *cvpMetadata = (struct CVPMetadata *)in;
        cvpMetadata->size = 0;
        if (data->cvpMetadata.size <= CVP_METADATA_SIZE) {
          cvpMetadata->size = data->cvpMetadata.size;
          memcpy(cvpMetadata->payload, data->cvpMetadata.payload, data->cvpMetadata.size);
          cvpMetadata->capture_frame_rate = data->cvpMetadata.capture_frame_rate;
          cvpMetadata->cvp_frame_rate = data->cvpMetadata.cvp_frame_rate;
          cvpMetadata->flags = data->cvpMetadata.flags;
          memcpy(cvpMetadata->reserved, data->cvpMetadata.reserved, (8 * sizeof(uint32_t)));
        } else {
          ret = Error::BAD_VALUE;
        }
      } else {
        *out = &data->cvpMetadata;
      }
      break;
    }
    case QTI_VIDEO_HISTOGRAM_STATS: {
      if (copy) {
        struct VideoHistogramMetadata *vidstats = (struct VideoHistogramMetadata *)in;
        vidstats->stat_len = 0;
        if (data->video_histogram_stats.stat_len <= VIDEO_HISTOGRAM_STATS_SIZE) {
          memcpy(vidstats->stats_info, data->video_histogram_stats.stats_info,
                 VIDEO_HISTOGRAM_STATS_SIZE);
          vidstats->stat_len = data->video_histogram_stats.stat_len;
          vidstats->frame_type = data->video_histogram_stats.frame_type;
          vidstats->display_width = data->video_histogram_stats.display_width;
          vidstats->display_height = data->video_histogram_stats.display_height;
          vidstats->decode_width = data->video_histogram_stats.decode_width;
          vidstats->decode_height = data->video_histogram_stats.decode_height;
        } else {
          ret = Error::BAD_VALUE;
        }
      } else {
        *out = &data->video_histogram_stats;
      }
      break;
    }
#ifdef QTI_VIDEO_TRANSCODE_STATS
    case QTI_VIDEO_TRANSCODE_STATS: {
      if (copy) {
        struct VideoTranscodeStatsMetadata *vidtranscodestats =
            (struct VideoTranscodeStatsMetadata *)in;
        vidtranscodestats->stat_len = 0;
        if (data->video_transcode_stats.stat_len <= VIDEO_TRANSCODE_STATS_SIZE) {
          memcpy(vidtranscodestats->stats_info, data->video_transcode_stats.stats_info,
                 VIDEO_TRANSCODE_STATS_SIZE);
          vidtranscodestats->stat_len = data->video_transcode_stats.stat_len;
        } else {
          ret = Error::BAD_VALUE;
        }
      } else {
        *out = &data->video_transcode_stats;
      }
      break;
    }
#endif
    case QTI_VIDEO_TS_INFO:
      if (copy) {
        *(reinterpret_cast<VideoTimestampInfo *>(in)) = data->videoTsInfo;
      } else {
        *out = &data->videoTsInfo;
      }
      break;
#ifdef QTI_TIMED_RENDERING
    case QTI_TIMED_RENDERING:
      if (copy) {
        *(reinterpret_cast<uint32_t *>(in)) = data->timedRendering;
      } else {
        *out = &data->timedRendering;
      }
      break;
#endif
    case (int64_t)StandardMetadataType::BUFFER_ID:
      if (copy) {
        *(reinterpret_cast<uint64_t *>(in)) = (uint64_t)handle->id;
      } else {
        *out = &handle->id;
      }
      break;
    case (int64_t)StandardMetadataType::NAME: {
      if (copy) {
        std::string name(data->name);
        *(reinterpret_cast<std::string *>(in)) = name;
      } else {
        *out = &data->name;
      }
      break;
    }
    case (int64_t)StandardMetadataType::WIDTH:
      if (copy) {
        *(reinterpret_cast<uint64_t *>(in)) = (uint64_t)handle->unaligned_width;
      } else {
        *out = &handle->unaligned_width;
      }
      break;
    case (int64_t)StandardMetadataType::HEIGHT:
      if (copy) {
        *(reinterpret_cast<uint64_t *>(in)) = (uint64_t)handle->unaligned_height;
      } else {
        *out = &handle->unaligned_height;
      }
      break;
    case (int64_t)StandardMetadataType::LAYER_COUNT:
      if (copy) {
        *(reinterpret_cast<uint64_t *>(in)) = (uint64_t)handle->layer_count;
      } else {
        *out = &handle->layer_count;
      }
      break;
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_REQUESTED:
      if (copy) {
        *(reinterpret_cast<PixelFormat *>(in)) = (PixelFormat)handle->format;
      } else {
        *out = &handle->format;
      }
      break;
    case (int64_t)StandardMetadataType::USAGE:
      if (copy) {
        *(reinterpret_cast<uint64_t *>(in)) = (uint64_t)handle->usage;
      } else {
        *out = &handle->usage;
      }
      break;
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_FOURCC: {
      if (copy) {
        uint32_t drm_format = 0;
        uint64_t drm_format_modifier = 0;
        GetDRMFormat(handle->format, handle->flags, &drm_format, &drm_format_modifier);
        *(reinterpret_cast<uint32_t *>(in)) = drm_format;
      }
      break;
    }
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_MODIFIER: {
      if (copy) {
        uint32_t drm_format = 0;
        uint64_t drm_format_modifier = 0;
        GetDRMFormat(handle->format, handle->flags, &drm_format, &drm_format_modifier);
        *(reinterpret_cast<uint64_t *>(in)) = drm_format_modifier;
      }
      break;
    }
    case (int64_t)StandardMetadataType::PROTECTED_CONTENT: {
      if (copy) {
        uint64_t protected_content = (handle->flags & qtigralloc::PRIV_FLAGS_SECURE_BUFFER) ? 1 : 0;
        *(reinterpret_cast<uint64_t *>(in)) = protected_content;
      }
      break;
    }
    case (int64_t)StandardMetadataType::ALLOCATION_SIZE:
      if (copy) {
        *(reinterpret_cast<uint32_t *>(in)) = (uint32_t)handle->size;
      } else {
        *out = &handle->size;
      }
      break;
    case QTI_FD:
      if (copy) {
        *(reinterpret_cast<int32_t *>(in)) = handle->fd;
      } else {
        *out = &handle->fd;
      }
      break;
    case QTI_PRIVATE_FLAGS:
      if (copy) {
        *(reinterpret_cast<uint32_t *>(in)) = handle->flags;
      } else {
        *out = &handle->flags;
      }
      break;
    case QTI_ALIGNED_WIDTH_IN_PIXELS:
      if (copy) {
        *(reinterpret_cast<uint32_t *>(in)) = handle->width;
      } else {
        *out = &handle->width;
      }
      break;
    case QTI_ALIGNED_HEIGHT_IN_PIXELS:
      if (copy) {
        *(reinterpret_cast<uint32_t *>(in)) = handle->height;
      } else {
        *out = &handle->height;
      }
      break;
    case QTI_COLORSPACE: {
      if (copy) {
        uint32_t colorspace;
        auto error = GetColorSpaceFromColorMetaData(data->color, &colorspace);
        if (error == Error::NONE) {
          *(reinterpret_cast<uint32_t *>(in)) = colorspace;
        } else {
          ret = Error::BAD_VALUE;
        }
      }
      break;
    }
    case QTI_YUV_PLANE_INFO: {
      if (copy) {
        qti_ycbcr layout[2];
        android_ycbcr yuv_plane_info[2];
        int error = GetYUVPlaneInfo(handle, yuv_plane_info);
        if (!error) {
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
          ALOGD(" layout: y: %" PRIu64 " , cr: %" PRIu64 " , cb: %" PRIu64
                " , yStride: %d, cStride: %d, chromaStep: %d ",
                yOffset, crOffset, cbOffset, layout[0].yStride, layout[0].cStride,
                layout[0].chromaStep);

          memcpy(in, layout, YCBCR_LAYOUT_ARRAY_SIZE * sizeof(qti_ycbcr));
        } else {
          ret = Error::BAD_BUFFER;
        }
      }
      break;
    }
    case QTI_CUSTOM_DIMENSIONS_STRIDE: {
      if (copy) {
        int32_t stride;
        int32_t height;
        if (GetCustomDimensions(handle, &stride, &height) == 0) {
          *(reinterpret_cast<int32_t *>(in)) = stride;
        } else {
          ret = Error::BAD_VALUE;
        }
      }
      break;
    }
    case QTI_CUSTOM_DIMENSIONS_HEIGHT: {
      if (copy) {
        int32_t stride = handle->width;
        int32_t height = handle->height;
        if (GetCustomDimensions(handle, &stride, &height) == 0) {
          *(reinterpret_cast<int32_t *>(in)) = height;
        } else {
          ret = Error::BAD_VALUE;
        }
      }
      break;
    }
    case QTI_RGB_DATA_ADDRESS: {
      if (copy) {
        void *rgb_data = nullptr;
        if (GetRgbDataAddress(handle, &rgb_data) == 0) {
          *(reinterpret_cast<void **>(in)) = rgb_data;
        } else {
          ret = Error::BAD_BUFFER;
        }
      }
      break;
    }
    case QTI_BUFFER_TYPE:
      if (copy) {
        *(reinterpret_cast<uint32_t *>(in)) = handle->buffer_type;
      } else {
        *out = &handle->buffer_type;
      }
      break;
    case QTI_STANDARD_METADATA_STATUS:
      if (copy) {
        memcpy(in, data->isStandardMetadataSet, sizeof(bool) * METADATA_SET_SIZE);
      } else {
        *out = &data->isStandardMetadataSet;
      }
      break;
    case QTI_VENDOR_METADATA_STATUS:
      if (copy) {
        memcpy(in, data->isVendorMetadataSet, sizeof(bool) * METADATA_SET_SIZE);
      } else {
        *out = &data->isVendorMetadataSet;
      }
      break;
    case (int64_t)StandardMetadataType::DATASPACE: {
      if (copy) {
        if (data->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(type)]) {
          Dataspace dataspace;
          ColorMetadataToDataspace(data->color, &dataspace);
          *(reinterpret_cast<Dataspace *>(in)) = dataspace;
        } else {
          *(reinterpret_cast<Dataspace *>(in)) = Dataspace::UNKNOWN;
        }
      }
      break;
    }
    case (int64_t)StandardMetadataType::BLEND_MODE:
      if (copy) {
        *(reinterpret_cast<int32_t *>(in)) = data->blendMode;
      } else {
        *out = &data->blendMode;
      }
      break;
    case (int64_t)StandardMetadataType::PLANE_LAYOUTS:
      if (copy) {
        std::vector<PlaneLayout> *plane_layouts = reinterpret_cast<std::vector<PlaneLayout> *>(in);
        GetPlaneLayout(handle, plane_layouts);
      }
      break;
#ifdef QTI_BUFFER_PERMISSION
    case QTI_BUFFER_PERMISSION:
      if (copy) {
        BufferPermission *buf_perm = reinterpret_cast<BufferPermission *>(in);
        int numelems = sizeof(data->bufferPerm) / sizeof(BufferPermission);
        for (int i = 0; i < numelems; i++) {
          buf_perm[i] = data->bufferPerm[i];
        }
      } else {
        *out = &data->bufferPerm[0];
      }
      break;
#endif
#ifdef QTI_MEM_HANDLE
    case QTI_MEM_HANDLE:
      if (copy) {
        *(reinterpret_cast<int64_t *>(in)) = data->memHandle;
      } else {
        *out = &data->memHandle;
      }
      break;
#endif
    default:
      ALOGD_IF(DEBUG, "Unsupported metadata type %d", type);
      ret = Error::BAD_VALUE;
      break;
  }

  return ret;
}

void setGralloc4Array(MetaData_t *metadata, int64_t paramType, bool isSet) {
  switch (paramType) {
    case (int64_t)StandardMetadataType::CROP:
      metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(
          ::android::gralloc4::MetadataType_Crop.value)] = isSet;
      break;
    case QTI_VT_TIMESTAMP:
    case QTI_COLOR_METADATA:
    case QTI_PP_PARAM_INTERLACED:
    case QTI_VIDEO_PERF_MODE:
    case QTI_GRAPHICS_METADATA:
    case QTI_UBWC_CR_STATS_INFO:
    case QTI_REFRESH_RATE:
    case QTI_COLORSPACE:
    case QTI_MAP_SECURE_BUFFER:
    case QTI_LINEAR_FORMAT:
    case QTI_SINGLE_BUFFER_MODE:
    case QTI_CVP_METADATA:
    case QTI_VIDEO_HISTOGRAM_STATS:
#ifdef QTI_VIDEO_TRANSCODE_STATS
    case QTI_VIDEO_TRANSCODE_STATS:
#endif
    case QTI_VIDEO_TS_INFO:
    case QTI_S3D_FORMAT:
    case QTI_BUFFER_PERMISSION:
      metadata->isVendorMetadataSet[GET_VENDOR_METADATA_STATUS_INDEX(paramType)] = isSet;
      break;
    // Following metadata types are not changed after allocation - treat as set by default
    case (int64_t)StandardMetadataType::BUFFER_ID:
    case (int64_t)StandardMetadataType::NAME:
    case (int64_t)StandardMetadataType::WIDTH:
    case (int64_t)StandardMetadataType::HEIGHT:
    case (int64_t)StandardMetadataType::LAYER_COUNT:
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_REQUESTED:
    case (int64_t)StandardMetadataType::USAGE:
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_FOURCC:
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_MODIFIER:
    case (int64_t)StandardMetadataType::PROTECTED_CONTENT:
    case (int64_t)StandardMetadataType::ALLOCATION_SIZE:
    case QTI_FD:
    case QTI_PRIVATE_FLAGS:
    case QTI_ALIGNED_WIDTH_IN_PIXELS:
    case QTI_ALIGNED_HEIGHT_IN_PIXELS:
    case QTI_MEM_HANDLE:
      break;
    default:
      ALOGE("paramType %d not supported in Gralloc4", paramType);
  }
}

Error SetMetaData(private_handle_t *handle, uint64_t paramType, void *param) {
  auto err = ValidateAndMap(handle);
  if (err != 0) {
    return Error::UNSUPPORTED;
  }

  MetaData_t *data = reinterpret_cast<MetaData_t *>(handle->base_metadata);

  if (data == nullptr) {
    return Error::UNSUPPORTED;
  }

  // If parameter is NULL reset the specific MetaData Key
  if (!param) {
    setGralloc4Array(data, paramType, false);
    switch (paramType) {
      case QTI_VIDEO_PERF_MODE:
        data->isVideoPerfMode = 0;
        break;
      case QTI_CVP_METADATA:
        data->cvpMetadata.size = 0;
        break;
      case QTI_VIDEO_HISTOGRAM_STATS:
        data->video_histogram_stats.stat_len = 0;
        break;
#ifdef QTI_VIDEO_TRANSCODE_STATS
      case QTI_VIDEO_TRANSCODE_STATS:
        data->video_transcode_stats.stat_len = 0;
        break;
#endif
      default:
        ALOGE("Unknown paramType %d", paramType);
        break;
    }
    // param unset
    return Error::NONE;
  }

  setGralloc4Array(data, paramType, true);
  switch (paramType) {
    case QTI_PP_PARAM_INTERLACED:
      data->interlaced = *(reinterpret_cast<int32_t *>(param));
      break;
    case (int64_t)StandardMetadataType::CROP: {
      CropRectangle_t in = *(reinterpret_cast<CropRectangle_t *>(param));
      data->crop = {0, 0, in.right, in.bottom};
      break;
    }
    case QTI_REFRESH_RATE:
      data->refreshrate = *(reinterpret_cast<float *>(param));
      break;
    case QTI_COLORSPACE: {
      ColorMetaData color = {};
      if (!colorSpaceToColorMetadata(*(reinterpret_cast<int *>(param)), &color)) {
        data->color = color;
        break;
      }
      return Error::BAD_VALUE;
    }
    case QTI_MAP_SECURE_BUFFER:
      data->mapSecureBuffer = *(reinterpret_cast<int32_t *>(param));
      break;
    case QTI_LINEAR_FORMAT:
      data->linearFormat = *(reinterpret_cast<uint32_t *>(param));
      break;
    case QTI_SINGLE_BUFFER_MODE:
      data->isSingleBufferMode = *(reinterpret_cast<uint32_t *>(param));
      break;
    case QTI_VT_TIMESTAMP:
      data->vtTimeStamp = *(reinterpret_cast<uint64_t *>(param));
      break;
    case QTI_COLOR_METADATA:
      data->color = *(reinterpret_cast<ColorMetaData *>(param));
      break;
    case QTI_UBWC_CR_STATS_INFO: {
      struct UBWCStats *stats = (struct UBWCStats *)param;
      int numelems = sizeof(data->ubwcCRStats) / sizeof(struct UBWCStats);
      for (int i = 0; i < numelems; i++) {
        data->ubwcCRStats[i] = stats[i];
      }
      break;
    }
    case QTI_VIDEO_PERF_MODE:
      data->isVideoPerfMode = *(reinterpret_cast<uint32_t *>(param));
      break;
    case QTI_GRAPHICS_METADATA: {
      GraphicsMetadata payload = *(reinterpret_cast<GraphicsMetadata *>(param));
      data->graphics_metadata.size = payload.size;
      memcpy(data->graphics_metadata.data, payload.data, sizeof(data->graphics_metadata.data));
      break;
    }
    case QTI_CVP_METADATA: {
      struct CVPMetadata *cvpMetadata = (struct CVPMetadata *)param;
      if (cvpMetadata->size <= CVP_METADATA_SIZE) {
        data->cvpMetadata.size = cvpMetadata->size;
        memcpy(data->cvpMetadata.payload, cvpMetadata->payload, cvpMetadata->size);
        data->cvpMetadata.capture_frame_rate = cvpMetadata->capture_frame_rate;
        data->cvpMetadata.cvp_frame_rate = cvpMetadata->cvp_frame_rate;
        data->cvpMetadata.flags = cvpMetadata->flags;
        memcpy(data->cvpMetadata.reserved, cvpMetadata->reserved, (8 * sizeof(uint32_t)));
      } else {
        setGralloc4Array(data, paramType, false);
        ALOGE("%s: cvp metadata length %d is more than max size %d", __func__, cvpMetadata->size,
              CVP_METADATA_SIZE);
        return Error::BAD_VALUE;
      }
      break;
    }
    case QTI_VIDEO_HISTOGRAM_STATS: {
      struct VideoHistogramMetadata *vidstats = (struct VideoHistogramMetadata *)param;
      if (vidstats->stat_len <= VIDEO_HISTOGRAM_STATS_SIZE) {
        memcpy(data->video_histogram_stats.stats_info, vidstats->stats_info,
               VIDEO_HISTOGRAM_STATS_SIZE);
        data->video_histogram_stats.stat_len = vidstats->stat_len;
        data->video_histogram_stats.frame_type = vidstats->frame_type;
        data->video_histogram_stats.display_width = vidstats->display_width;
        data->video_histogram_stats.display_height = vidstats->display_height;
        data->video_histogram_stats.decode_width = vidstats->decode_width;
        data->video_histogram_stats.decode_height = vidstats->decode_height;
      } else {
        setGralloc4Array(data, paramType, false);
        ALOGE("%s: video stats length %u is more than max size %u", __func__, vidstats->stat_len,
              VIDEO_HISTOGRAM_STATS_SIZE);
        return Error::BAD_VALUE;
      }
      break;
    }
#ifdef QTI_VIDEO_TRANSCODE_STATS
    case QTI_VIDEO_TRANSCODE_STATS: {
      struct VideoTranscodeStatsMetadata *vidtranscodestats =
          (struct VideoTranscodeStatsMetadata *)param;
      if (vidtranscodestats->stat_len <= VIDEO_TRANSCODE_STATS_SIZE) {
        memcpy(data->video_transcode_stats.stats_info, vidtranscodestats->stats_info,
               VIDEO_TRANSCODE_STATS_SIZE);
        data->video_transcode_stats.stat_len = vidtranscodestats->stat_len;
      } else {
        setGralloc4Array(data, paramType, false);
        ALOGE("%s: video transcode stats length %u is more than max size %u", __func__,
               vidtranscodestats->stat_len, VIDEO_TRANSCODE_STATS_SIZE);
        return Error::BAD_VALUE;
      }
      break;
    }
#endif
    case QTI_VIDEO_TS_INFO:
      data->videoTsInfo = *(reinterpret_cast<VideoTimestampInfo *>(param));
      break;
    case QTI_BUFFER_PERMISSION: {
      BufferPermission*buf_perm = reinterpret_cast<BufferPermission *>(param);
      int numelems = sizeof(data->bufferPerm) / sizeof(BufferPermission);
      for (int i = 0; i < numelems; i++) {
        data->bufferPerm[i] = buf_perm[i];
      }
      break;
    }
    case QTI_MEM_HANDLE:
      data->memHandle = *(reinterpret_cast<int64_t *>(param));
      break;
    default:
      ALOGE("Unknown paramType %d", paramType);
      break;
  }
  return Error::NONE;
}
}  // namespace gralloc
