/*
 * Copyright (c) 2019-2021 The Linux Foundation. All rights reserved.
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
 *
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 */

#include "QtiGrallocMetadata.h"
#include "gr_buf_descriptor.h"
#define ATRACE_TAG (ATRACE_TAG_GRAPHICS | ATRACE_TAG_HAL)
#include "QtiMapperExtensions.h"
#include <cutils/properties.h>
#include <cutils/trace.h>
#include <sync/sync.h>
#include "gr_utils.h"
#include <QtiGralloc.h>

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace mapperextensions {
namespace V1_1 {
namespace implementation {

using gralloc::BufferInfo;
using MetadataType = ::android::hardware::graphics::mapper::V4_0::IMapper::MetadataType;

QtiMapperExtensions::QtiMapperExtensions() {
  buf_mgr_ = BufferManager::GetInstance();
  enable_logs_ = property_get_bool(ENABLE_LOGS_PROP, 0);
  snap_helper_ = GrallocSnapHelperLegacy::GetInstance();
}

Return<void> QtiMapperExtensions::getMapSecureBufferFlag(void *buffer,
                                                         getMapSecureBufferFlag_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  int map_secure_buffer = 0;
  if (snap_helper_->IsSnapAllocEnabled()) {
    int snap_ret = snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer),
                                             QTI_MAP_SECURE_BUFFER, &map_secure_buffer, false);
    err = (snap_ret == 0) ? Error::NONE : Error::UNSUPPORTED;
  } else {
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = static_cast<Error>(
          gralloc::GetMetaDataValue(hnd, QTI_MAP_SECURE_BUFFER, &map_secure_buffer));
    }
  }
  if (err != Error::NONE) {
    map_secure_buffer = 0;
  }
  hidl_cb(err, map_secure_buffer != 0);
  return Void();
}

Return<void> QtiMapperExtensions::getInterlacedFlag(void *buffer, getInterlacedFlag_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto ret = Error::NONE;
  auto hnd = static_cast<private_handle_t *>(buffer);
  int interlaced_flag = 0;
  if (snap_helper_->IsSnapAllocEnabled()) {
    err = Error::NONE;
    int snap_ret = snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer),
                                             QTI_PP_PARAM_INTERLACED, &interlaced_flag, false);
    ret = (snap_ret == 0) ? Error::NONE : Error::UNSUPPORTED;
  } else {
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = Error::NONE;
      ret = static_cast<Error>(
          gralloc::GetMetaDataValue(hnd, QTI_PP_PARAM_INTERLACED, &interlaced_flag));
    }
  }
  if (ret != Error::NONE) {
    interlaced_flag = 0;
    ALOGW(
        "%s: getMetaData returned %d, defaulting to "
        "interlaced_flag = %d",
        __FUNCTION__, ret, interlaced_flag);
  }
  hidl_cb(err, interlaced_flag != 0);
  return Void();
}

Return<void> QtiMapperExtensions::getCustomDimensions(void *buffer,
                                                      getCustomDimensions_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int stride = 0;
  int height = 0;
  if (buffer != nullptr) {
    if (snap_helper_->IsSnapAllocEnabled()) {
      // TODO: implement this
      //int snap_ret = snap_helper_->GetCustomDimensions(static_cast<native_handle_t *>(buffer), &stride, &height);

      int snap_ret = snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer),
                                               QTI_CUSTOM_DIMENSIONS_STRIDE, &stride,
                                               false);  // false -> convert_to_hidl_bytestream
      if (snap_ret == 0) {
        snap_ret = snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer),
                                             QTI_CUSTOM_DIMENSIONS_HEIGHT, &height, false);
      }
      err = (snap_ret == 0) ? Error::NONE : Error::BAD_BUFFER;
    } else {
      auto hnd = static_cast<private_handle_t *>(buffer);

      if (private_handle_t::validate(hnd) == 0) {
        stride = hnd->width;
        height = hnd->height;
        int ret = gralloc::GetCustomDimensions(hnd, &stride, &height);
        if (ret) {
          ALOGW(
              "%s: Error during GetCustomDimensions API call. "
              "stride: %d, height: %d",
              __FUNCTION__, stride, height);
          err = Error::BAD_BUFFER;
        } else {
          err = Error::NONE;
        }
      }
    }
  }
  hidl_cb(err, stride, height);
  return Void();
}

Return<void> QtiMapperExtensions::getRgbDataAddress(void *buffer, getRgbDataAddress_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  void *rgb_data = nullptr;
  if (snap_helper_->IsSnapAllocEnabled()) {
    int snap_ret = snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer),
                                             QTI_RGB_DATA_ADDRESS, &rgb_data, false);
    err = (snap_ret == 0) ? Error::NONE : Error::BAD_BUFFER;
  } else {
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      if (gralloc::GetRgbDataAddress(hnd, &rgb_data) == 0) {
        err = Error::NONE;
      }
    }
  }
  ALOGD_IF(enable_logs_, "RGB data address %" PRIu64, reinterpret_cast<uint64_t>(rgb_data));
  hidl_cb(err, rgb_data);
  return Void();
}

Return<void> QtiMapperExtensions::calculateBufferAttributes(int32_t width, int32_t height,
                                                            int32_t format, uint64_t usage,
                                                            calculateBufferAttributes_cb hidl_cb) {
  auto err = Error::NONE;
  bool ubwc_enabled = false;
  if (snap_helper_->IsSnapAllocEnabled()) {
    uint64_t alignedw = 0;
    uint64_t alignedh = 0;
    gralloc::BufferDescriptor desc;
    desc.SetUsage(usage);
    desc.SetColorFormat(format);
    desc.SetDimensions(width, height);

    int snap_ret =
        snap_helper_->GetFromBufferDescriptor(desc, QTI_ALIGNED_WIDTH_IN_PIXELS, &alignedw,
                                              false);  // false -> convert_to_hidl_bytestream
    if (snap_ret == 0) {
      snap_ret = snap_helper_->GetFromBufferDescriptor(desc, QTI_ALIGNED_HEIGHT_IN_PIXELS,
                                                       &alignedh, false);
    }
    if (snap_ret != 0) {
      err = Error::BAD_BUFFER;
    }
    // TODO: query UBWC status - add metadata type and add to GetFromBufferDescriptor
    hidl_cb(err, alignedw, alignedh, ubwc_enabled);
  } else {
    unsigned int alignedw, alignedh;
    BufferInfo info(width, height, format, usage);
    int ret = gralloc::GetAlignedWidthAndHeight(info, &alignedw, &alignedh);
    if (ret) {
      err = Error::BAD_BUFFER;
    }
    ubwc_enabled = gralloc::IsUBwcEnabled(format, usage);
    hidl_cb(err, alignedw, alignedh, ubwc_enabled);
  }
  return Void();
}

Return<void> QtiMapperExtensions::getCustomFormatFlags(int32_t format, uint64_t usage,
                                                       getCustomFormatFlags_cb hidl_cb) {
  uint64_t priv_flags = 0;
  auto err = Error::NONE;
  int32_t custom_format = format;
  if (snap_helper_->IsSnapAllocEnabled()) {
    ALOGE("%s is deprecated for SnapAlloc", __FUNCTION__);
  } else {
    if (gralloc::GetCustomFormatFlags(format, usage, &custom_format, &priv_flags) != 0) {
      err = Error::UNSUPPORTED;
    }
  }
  hidl_cb(err, custom_format, priv_flags);
  return Void();
}

Return<void> QtiMapperExtensions::getColorSpace(void *buffer, getColorSpace_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int color_space = 0;
  if (snap_helper_->IsSnapAllocEnabled()) {
    int snap_ret = snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer), QTI_COLORSPACE,
                                             &color_space, false);
    err = (snap_ret == 0) ? Error::NONE : Error::BAD_BUFFER;
  } else {
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      gralloc::GetColorSpaceFromMetadata(hnd, &color_space);
      err = Error::NONE;
    }
  }
  hidl_cb(err, color_space);
  return Void();
}

Return<void> QtiMapperExtensions::getYuvPlaneInfo(void *buffer, getYuvPlaneInfo_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  hidl_vec<YCbCrLayout> layout;
  layout.resize(2);
  if (snap_helper_->IsSnapAllocEnabled()) {
    qti_ycbcr yuv_plane_info[2];
    int snap_ret = snap_helper_->GetMetadata(reinterpret_cast<native_handle_t *>(buffer),
                                             QTI_YUV_PLANE_INFO, &yuv_plane_info, false);
    err = (snap_ret == 0) ? Error::NONE : Error::BAD_BUFFER;
    for (int i = 0; i < 2; i++) {
      layout[i].y = yuv_plane_info[i].y;
      layout[i].cr = yuv_plane_info[i].cr;
      layout[i].cb = yuv_plane_info[i].cb;
      layout[i].yStride = static_cast<uint32_t>(yuv_plane_info[i].yStride);
      layout[i].cStride = static_cast<uint32_t>(yuv_plane_info[i].cStride);
      layout[i].chromaStep = static_cast<uint32_t>(yuv_plane_info[i].chromaStep);
    }
  } else {
    auto hnd = static_cast<private_handle_t *>(buffer);
    android_ycbcr yuv_plane_info[2];
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      if (gralloc::GetYUVPlaneInfo(hnd, yuv_plane_info) == 0) {
        err = Error::NONE;
        for (int i = 0; i < 2; i++) {
          layout[i].y = yuv_plane_info[i].y;
          layout[i].cr = yuv_plane_info[i].cr;
          layout[i].cb = yuv_plane_info[i].cb;
          layout[i].yStride = static_cast<uint32_t>(yuv_plane_info[i].ystride);
          layout[i].cStride = static_cast<uint32_t>(yuv_plane_info[i].cstride);
          layout[i].chromaStep = static_cast<uint32_t>(yuv_plane_info[i].chroma_step);
        }
      }
    }
  }
  hidl_cb(err, layout);
  return Void();
}

Return<Error> QtiMapperExtensions::setSingleBufferMode(void *buffer, bool enable) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);

  if (snap_helper_->IsSnapAllocEnabled()) {
    ALOGE("%s is not yet implemented for SnapAlloc", __FUNCTION__);
    // TODO: Dheepthi - change this to encode helper or update once SetMetadata uploaded [Done]
    int snap_ret = snap_helper_->SetMetadata(static_cast<native_handle_t *>(buffer),
                                             QTI_SINGLE_BUFFER_MODE, &enable, 0);
    err = (snap_ret == 0) ? Error::NONE : Error::UNSUPPORTED;
  } else {
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = static_cast<Error>(gralloc::SetMetaData(hnd, QTI_SINGLE_BUFFER_MODE, &enable));
      err = (err != Error::NONE) ? Error::UNSUPPORTED : err;
    }
  }
  return err;
}

Return<void> QtiMapperExtensions::getFd(void *buffer, getFd_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int fd = 0;

  if (snap_helper_->IsSnapAllocEnabled()) {
    int snap_ret =
        snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer), QTI_FD, &fd, false);
    err = (snap_ret == 0) ? Error::NONE : Error::BAD_BUFFER;
  } else {
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = Error::NONE;
      fd = hnd->fd;
    }
  }
  hidl_cb(err, fd);
  return Void();
}

Return<void> QtiMapperExtensions::getWidth(void *buffer, getWidth_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int width = 0;
  if (snap_helper_->IsSnapAllocEnabled()) {
    int snap_ret = snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer),
                                             QTI_ALIGNED_WIDTH_IN_PIXELS, &width, false);
    err = (snap_ret == 0) ? Error::NONE : Error::BAD_BUFFER;
  } else {
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = Error::NONE;
      width = hnd->width;
    }
  }
  hidl_cb(err, width);
  return Void();
}

Return<void> QtiMapperExtensions::getHeight(void *buffer, getHeight_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int height = 0;
  if (snap_helper_->IsSnapAllocEnabled()) {
    int snap_ret = snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer),
                                             QTI_ALIGNED_HEIGHT_IN_PIXELS, &height, false);
    err = (snap_ret == 0) ? Error::NONE : Error::BAD_BUFFER;
  } else {
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = Error::NONE;
      height = hnd->height;
    }
  }
  hidl_cb(err, height);
  return Void();
}

Return<void> QtiMapperExtensions::getFormat(void *buffer, getFormat_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int format = 0;
  if (snap_helper_->IsSnapAllocEnabled()) {
    int snap_ret = snap_helper_->GetMetadata(
        static_cast<native_handle_t *>(buffer),
        static_cast<int64_t>(StandardMetadataType::PIXEL_FORMAT_REQUESTED), &format, false);
    err = (snap_ret == 0) ? Error::NONE : Error::BAD_BUFFER;
  } else {
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = Error::NONE;
      format = hnd->format;
    }
  }
  hidl_cb(err, format);
  return Void();
}

Return<void> QtiMapperExtensions::getPrivateFlags(void *buffer, getPrivateFlags_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int flags = 0;
  if (snap_helper_->IsSnapAllocEnabled()) {
    int snap_ret = snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer),
                                             QTI_PRIVATE_FLAGS, &flags, false);
    err = (snap_ret == 0) ? Error::NONE : Error::BAD_BUFFER;
  } else {
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = Error::NONE;
      flags = hnd->flags;
    }
  }
  hidl_cb(err, flags);
  return Void();
}

Return<void> QtiMapperExtensions::getUnalignedWidth(void *buffer, getUnalignedWidth_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  if (snap_helper_->IsSnapAllocEnabled()) {
    uint64_t unaligned_width = 0;
    int snap_ret = snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer),
                                             static_cast<int64_t>(StandardMetadataType::WIDTH),
                                             &unaligned_width, false);
    err = (snap_ret == 0) ? Error::NONE : Error::UNSUPPORTED;
    hidl_cb(err, unaligned_width);
  } else {
    int unaligned_width = 0;
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = Error::NONE;
      unaligned_width = hnd->unaligned_width;
    }
    hidl_cb(err, unaligned_width);
  }
  return Void();
}

Return<void> QtiMapperExtensions::getUnalignedHeight(void *buffer, getUnalignedHeight_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  if (snap_helper_->IsSnapAllocEnabled()) {
    uint64_t unaligned_height = 0;
    int snap_ret = snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer),
                                             static_cast<int64_t>(StandardMetadataType::HEIGHT),
                                             &unaligned_height, false);
    err = (snap_ret == 0) ? Error::NONE : Error::UNSUPPORTED;
    hidl_cb(err, unaligned_height);
  } else {
    int unaligned_height = 0;
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = Error::NONE;
      unaligned_height = hnd->unaligned_height;
    }
    hidl_cb(err, unaligned_height);
  }
  return Void();
}

Return<void> QtiMapperExtensions::getLayerCount(void *buffer, getLayerCount_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  if (snap_helper_->IsSnapAllocEnabled()) {
    uint64_t layer_count = 0;
    int snap_ret = snap_helper_->GetMetadata(
        static_cast<native_handle_t *>(buffer),
        static_cast<int64_t>(StandardMetadataType::LAYER_COUNT), &layer_count, false);
    err = (snap_ret == 0) ? Error::NONE : Error::UNSUPPORTED;
    hidl_cb(err, layer_count);
  } else {
    unsigned int layer_count = 0;
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = Error::NONE;
      layer_count = hnd->layer_count;
    }
    hidl_cb(err, layer_count);
  }
  return Void();
}

Return<void> QtiMapperExtensions::getId(void *buffer, getId_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  uint64_t id = 0;
  if (snap_helper_->IsSnapAllocEnabled()) {
    int snap_ret = snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer),
                                             static_cast<int64_t>(StandardMetadataType::BUFFER_ID),
                                             &id, false);
    err = (snap_ret == 0) ? Error::NONE : Error::UNSUPPORTED;
  } else {
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = Error::NONE;
      id = hnd->id;
    }
  }
  hidl_cb(err, id);
  return Void();
}

Return<void> QtiMapperExtensions::getUsageFlags(void *buffer, getUsageFlags_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;

  uint64_t usage = 0;
  if (snap_helper_->IsSnapAllocEnabled()) {
    int snap_ret =
        snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer),
                                  static_cast<int64_t>(StandardMetadataType::USAGE), &usage, false);
    err = (snap_ret == 0) ? Error::NONE : Error::UNSUPPORTED;
  } else {
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = Error::NONE;
      usage = hnd->usage;
    }
  }
  hidl_cb(err, usage);
  return Void();
}

Return<void> QtiMapperExtensions::getSize(void *buffer, getSize_cb hidl_cb) {
  ALOGD_IF(enable_logs_, __FUNCTION__);

  auto err = Error::BAD_BUFFER;
  if (snap_helper_->IsSnapAllocEnabled()) {
    uint64_t size = 0;
    int snap_ret = snap_helper_->GetMetadata(
        static_cast<native_handle_t *>(buffer),
        static_cast<int64_t>(StandardMetadataType::ALLOCATION_SIZE), &size, false);
    err = (snap_ret == 0) ? Error::NONE : Error::UNSUPPORTED;
    hidl_cb(err, size);
  } else {
    unsigned int size = 0;
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = Error::NONE;
      size = hnd->size;
    }
    hidl_cb(err, size);
  }
  return Void();
}

Return<void> QtiMapperExtensions::getOffset(void *buffer, getOffset_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  unsigned int offset = 0;
  if (snap_helper_->IsSnapAllocEnabled()) {
    ALOGW("%s is deprecated for SnapAlloc - returning offset 0", __FUNCTION__);
    err = Error::NONE;
    offset = 0;
  } else {
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = Error::NONE;
      offset = hnd->offset;
    }
  }
  hidl_cb(err, offset);
  return Void();
}

Return<void> QtiMapperExtensions::getSurfaceMetadata(void *buffer, getSurfaceMetadata_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  GraphicsMetadata surface_metadata;
  if (snap_helper_->IsSnapAllocEnabled()) {
    int snap_ret = snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer),
                                             QTI_GRAPHICS_METADATA, &surface_metadata, false);
    err = (snap_ret == 0) ? Error::NONE : Error::UNSUPPORTED;
  } else {
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = static_cast<Error>(
          gralloc::GetMetaDataValue(hnd, QTI_GRAPHICS_METADATA, &surface_metadata));
    }
  }
  if (err != Error::NONE) {
    hidl_cb(err, nullptr);
  } else {
    hidl_cb(err, &surface_metadata);
  }
  return Void();
}

// It will return size for single layer only i.e. layer count is always 1.
Return<void> QtiMapperExtensions::getFormatLayout(int32_t format, uint64_t usage, int32_t flags,
                                                  int32_t width, int32_t height,
                                                  getFormatLayout_cb hidl_cb) {
  auto err = Error::NONE;
  hidl_vec<PlaneLayout> plane_info;
  unsigned int alignedw = 0, alignedh = 0;
  int plane_count = 0;
  uint32_t size = 0;

  ALOGD_IF(enable_logs_, "%s: input format %d usage %" PRIu64 " width %d height %d interlaced %d",
           __FUNCTION__, format, usage, width, height, flags);

  if (snap_helper_->IsSnapAllocEnabled()) {
    BufferInfo info(width, height, format, usage);
    // TODO: reduce code duplication here
    std::vector<gralloc::PlaneLayoutInfo> plane_layout_info;

    if (!snap_helper_->GetFormatLayout(info, &plane_layout_info, &size, flags)) {
      plane_info.resize(plane_layout_info.size());

      ALOGD_IF(enable_logs_, "%s: Number of plane - %zu gralloc format %d", __FUNCTION__,
               plane_layout_info.size(), format);
      for (int i = 0; i < plane_layout_info.size(); i++) {
        plane_info[i].component = plane_layout_info[i].component;
        plane_info[i].h_subsampling = plane_layout_info[i].h_subsampling;
        plane_info[i].v_subsampling = plane_layout_info[i].v_subsampling;
        plane_info[i].offset = plane_layout_info[i].offset;
        plane_info[i].pixel_increment = plane_layout_info[i].step;
        plane_info[i].stride = plane_layout_info[i].stride;
        plane_info[i].stride_bytes = plane_layout_info[i].stride_bytes;
        plane_info[i].scanlines = plane_layout_info[i].scanlines;
        plane_info[i].size = plane_layout_info[i].size;
        ALOGD_IF(enable_logs_, "%s: plane info: component - %d", __FUNCTION__,
                 plane_info[i].component);
        ALOGD_IF(enable_logs_,
                 "h_subsampling - %u, v_subsampling - %u, offset - %u, pixel_increment - %d",
                 plane_info[i].h_subsampling, plane_info[i].v_subsampling, plane_info[i].offset,
                 plane_info[i].pixel_increment);
        ALOGD_IF(enable_logs_, "stride_pixel - %d, stride_bytes - %d, scanlines - %d, size - %u",
                 plane_info[i].stride, plane_info[i].stride_bytes, plane_info[i].scanlines,
                 plane_info[i].size);
      }
      hidl_cb(err, size, plane_info);
      return Void();
    } else {
      err = Error::UNSUPPORTED;
      hidl_cb(err, size, plane_info);
      return Void();
    }
  } else {
    int custom_format = gralloc::GetImplDefinedFormat(usage, format);
    BufferInfo info(width, height, custom_format, usage);

    int ret = gralloc::GetBufferSizeAndDimensions(info, &size, &alignedw, &alignedh);
    if (ret) {
      err = Error::BAD_BUFFER;
      hidl_cb(err, size, plane_info);
      return Void();
    }
    gralloc::PlaneLayoutInfo plane_layout[8] = {};
    ALOGD_IF(enable_logs_, "%s: Aligned width and height - wxh: %ux%u custom_format = %d",
             __FUNCTION__, alignedw, alignedh, custom_format);
    if (gralloc::IsYuvFormat(custom_format)) {
      // flags here only refers to layout (interlaced) flags, not private or buffer usage flags
      gralloc::GetYUVPlaneInfo(info, custom_format, alignedw, alignedh, flags, &plane_count,
                               plane_layout);
    } else if (gralloc::IsUncompressedRGBFormat(custom_format) ||
               gralloc::IsCompressedRGBFormat(custom_format)) {
      gralloc::GetRGBPlaneInfo(info, custom_format, alignedw, alignedh, flags, &plane_count,
                               plane_layout);
    } else {
      err = Error::BAD_BUFFER;
      hidl_cb(err, size, plane_info);
      return Void();
    }
    ALOGD_IF(enable_logs_, "%s: Number of plane - %d, custom_format - %d", __FUNCTION__,
             plane_count, custom_format);
    plane_info.resize(plane_count);
    for (int i = 0; i < plane_count; i++) {
      plane_info[i].component = plane_layout[i].component;
      plane_info[i].h_subsampling = plane_layout[i].h_subsampling;
      plane_info[i].v_subsampling = plane_layout[i].v_subsampling;
      plane_info[i].offset = plane_layout[i].offset;
      plane_info[i].pixel_increment = plane_layout[i].step;
      plane_info[i].stride = plane_layout[i].stride;
      plane_info[i].stride_bytes = plane_layout[i].stride_bytes;
      plane_info[i].scanlines = plane_layout[i].scanlines;
      plane_info[i].size = plane_layout[i].size;
      ALOGD_IF(enable_logs_, "%s: plane info: component - %d", __FUNCTION__,
               plane_info[i].component);
      ALOGD_IF(enable_logs_,
               "h_subsampling - %u, v_subsampling - %u, offset - %u, pixel_increment - %d",
               plane_info[i].h_subsampling, plane_info[i].v_subsampling, plane_info[i].offset,
               plane_info[i].pixel_increment);
      ALOGD_IF(enable_logs_, "stride_pixel - %d, stride_bytes - %d, scanlines - %d, size - %u",
               plane_info[i].stride, plane_info[i].stride_bytes, plane_info[i].scanlines,
               plane_info[i].size);
    }
  }
  hidl_cb(err, size, plane_info);
  return Void();
}

Return<Error> QtiMapperExtensions::getSurfaceMetadata_V1(void *buffer, void *metadata) {
  auto err = Error::BAD_BUFFER;

  if (snap_helper_->IsSnapAllocEnabled()) {
    int snap_ret = snap_helper_->GetMetadata(static_cast<native_handle_t *>(buffer),
                                             QTI_GRAPHICS_METADATA, metadata, false);
    err = (snap_ret == 0) ? Error::NONE : Error::UNSUPPORTED;
  } else {
    auto hnd = static_cast<private_handle_t *>(buffer);
    if (metadata != nullptr && buffer != nullptr && private_handle_t::validate(hnd) == 0) {
      err = static_cast<Error>(gralloc::GetMetaDataValue(hnd, QTI_GRAPHICS_METADATA, metadata));
      err = (err != Error::NONE) ? Error::UNSUPPORTED : err;
    }
  }

  return err;
}

Return<Error> QtiMapperExtensions::copyMetaData(void *src, void *dst) {
  auto error = Error::BAD_BUFFER;
  auto src_hnd = static_cast<private_handle_t *>(src);
  auto dst_hnd = static_cast<private_handle_t *>(dst);

  if (src != nullptr && dst != nullptr && private_handle_t::validate(src_hnd) == 0 &&
      private_handle_t::validate(dst_hnd) == 0) {
    if (static_cast<IMapperExtensions_1_0_Error>(buf_mgr_->IsBufferImported(src_hnd)) ==
            Error::NONE &&
        static_cast<IMapperExtensions_1_0_Error>(buf_mgr_->IsBufferImported(dst_hnd)) ==
            Error::NONE) {
      MetaData_t *src_data = reinterpret_cast<MetaData_t *>(src_hnd->base_metadata);
      MetaData_t *dst_data = reinterpret_cast<MetaData_t *>(dst_hnd->base_metadata);
      *dst_data = *src_data;
      error = Error::NONE;
    }
  } else {
    ALOGE("%s: Copy Failed - src buffer: %p, dst buffer: %p", __FUNCTION__, src, dst);
  }
  return error;
}

Return<Error> QtiMapperExtensions::setMetadataBlob(const hidl_vec<uint8_t> &src, void *dst) {
  auto error = Error::BAD_BUFFER;
  if (src.data() == nullptr) {
    return error;
  }
  auto dst_hnd = static_cast<private_handle_t *>(dst);

  if (dst != nullptr && private_handle_t::validate(dst_hnd) == 0) {
    if (static_cast<IMapperExtensions_1_0_Error>(buf_mgr_->IsBufferImported(dst_hnd)) ==
        Error::NONE) {
      const MetaData_t *src_data = reinterpret_cast<const MetaData_t *>(src.data());
      MetaData_t *dst_data = reinterpret_cast<MetaData_t *>(dst_hnd->base_metadata);
      *dst_data = *src_data;
      error = Error::NONE;
    }
  } else {
    ALOGE("%s: Copy Failed - src buffer: %p, dst pointer: %p", __FUNCTION__, src.data(), dst);
  }
  return error;
}

Return<void> QtiMapperExtensions::getMetadataBlob(void *src, getMetadataBlob_cb _hidl_cb) {
  auto error = Error::BAD_BUFFER;
  hidl_vec<uint8_t> out;
  auto src_hnd = static_cast<private_handle_t *>(src);
  out.resize(sizeof(MetaData_t));

  if (src != nullptr && private_handle_t::validate(src_hnd) == 0) {
    if (static_cast<IMapperExtensions_1_0_Error>(buf_mgr_->IsBufferImported(src_hnd)) ==
        Error::NONE) {
      MetaData_t *src_data = reinterpret_cast<MetaData_t *>(src_hnd->base_metadata);
      MetaData_t *dst_data = reinterpret_cast<MetaData_t *>(out.data());
      memcpy(dst_data, src_data, sizeof(MetaData_t));
      error = Error::NONE;
      _hidl_cb(error, out);
    }
  } else {
    ALOGE("%s: Get Failed - src buffer: %p", __FUNCTION__, src);
  }
  _hidl_cb(error, out);
  return Void();
}

Return<Error> QtiMapperExtensions::getMetaDataValue(void *src, const MetadataType &type, void *in) {
  auto error = Error::BAD_BUFFER;

  if (src != nullptr) {
    if (snap_helper_->IsSnapAllocEnabled()) {
      int snap_ret =
          snap_helper_->GetMetadata(static_cast<native_handle_t *>(src), type.value, in, false);
      error = (snap_ret == 0) ? Error::NONE : Error::UNSUPPORTED;
    } else {
      if (type.name != GRALLOC4_STANDARD_METADATA_TYPE && type.name != qtigralloc::VENDOR_QTI) {
        return Error::UNSUPPORTED;
      }
      error = static_cast<IMapperExtensions_1_0_Error>(
          buf_mgr_->GetMetadataValue(static_cast<private_handle_t *>(src), type.value, in));
    }
  }
  return error;
}

}  // namespace implementation
}  // namespace V1_1
}  // namespace mapperextensions
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
