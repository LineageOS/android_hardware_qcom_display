/*
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
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

#define ATRACE_TAG (ATRACE_TAG_GRAPHICS | ATRACE_TAG_HAL)
#define DEBUG 0
#include "QtiMapper.h"
#include <cutils/trace.h>
#include <qdMetaData.h>
#include <sync/sync.h>
#include "gr_utils.h"

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace mapper {
namespace V1_1 {
namespace implementation {

using gralloc::BufferInfo;

QtiMapper::QtiMapper() {
  buf_mgr_ = BufferManager::GetInstance();
  ALOGD_IF(DEBUG, "Created QtiMapper instance");
}

bool QtiMapper::ValidDescriptor(const BufferDescriptorInfo_2_1 &bd) {
  if (bd.width == 0 || bd.height == 0 || (static_cast<int32_t>(bd.format) <= 0) ||
      bd.layerCount <= 0) {
    return false;
  }

  return true;
}

Error QtiMapper::CreateDescriptor(const BufferDescriptorInfo_2_1& descriptor_info,
                                  IMapperBufferDescriptor *descriptor) {
  ALOGD_IF(DEBUG,
           "BufferDescriptorInfo: wxh: %dx%d usage: 0x%" PRIu64 " format: %d layer_count: %d",
           descriptor_info.width, descriptor_info.height, descriptor_info.usage,
           static_cast<uint32_t>(descriptor_info.format), descriptor_info.layerCount);

  if (ValidDescriptor(descriptor_info)) {
    auto vec = gralloc::BufferDescriptor::Encode(descriptor_info);
    *descriptor = vec;
    return Error::NONE;
  } else {
    return Error::BAD_VALUE;
  }
}

// Methods from ::android::hardware::graphics::mapper::V2_0::IMapper follow.
Return<void> QtiMapper::createDescriptor(const BufferDescriptorInfo_2_0 &descriptor_info,
                                         createDescriptor_cb hidl_cb) {
  IMapperBufferDescriptor descriptor;
  auto info_2_1 = BufferDescriptorInfo_2_1 {
      descriptor_info.width,
      descriptor_info.height,
      descriptor_info.layerCount,
      static_cast<PixelFormat>(descriptor_info.format),
      descriptor_info.usage,
  };
  auto err = CreateDescriptor(info_2_1, &descriptor);
  hidl_cb(err, descriptor);
  return Void();
}

Return<void> QtiMapper::importBuffer(const hidl_handle &raw_handle, importBuffer_cb hidl_cb) {
  if (!raw_handle.getNativeHandle()) {
    ALOGE("%s: Unable to import handle", __FUNCTION__);
    hidl_cb(Error::BAD_BUFFER, nullptr);
    return Void();
  }

  native_handle_t *buffer_handle = native_handle_clone(raw_handle.getNativeHandle());
  if (!buffer_handle) {
    ALOGE("%s: Unable to clone handle", __FUNCTION__);
    hidl_cb(Error::NO_RESOURCES, nullptr);
    return Void();
  }

  auto error = buf_mgr_->RetainBuffer(PRIV_HANDLE_CONST(buffer_handle));
  if (error != Error::NONE) {
    ALOGE("%s: Unable to retain handle: %p", __FUNCTION__, buffer_handle);
    native_handle_close(buffer_handle);
    native_handle_delete(buffer_handle);

    hidl_cb(error, nullptr);
    return Void();
  }
  ALOGD_IF(DEBUG, "Imported handle: %p id: %" PRIu64, buffer_handle,
           PRIV_HANDLE_CONST(buffer_handle)->id);
  hidl_cb(Error::NONE, buffer_handle);
  return Void();
}

Return<Error> QtiMapper::freeBuffer(void *buffer) {
  if (!buffer) {
    return Error::BAD_BUFFER;
  }
  return buf_mgr_->ReleaseBuffer(PRIV_HANDLE_CONST(buffer));
}

bool QtiMapper::GetFenceFd(const hidl_handle &fence_handle, int *outFenceFd) {
  auto handle = fence_handle.getNativeHandle();
  if (handle && handle->numFds > 1) {
    ALOGE("invalid fence handle with %d fds", handle->numFds);
    return false;
  }

  *outFenceFd = (handle && handle->numFds == 1) ? handle->data[0] : -1;
  return true;
}

void QtiMapper::WaitFenceFd(int fence_fd) {
  if (fence_fd < 0) {
    return;
  }

  const int timeout = 3000;
  ATRACE_BEGIN("fence wait");
  const int error = sync_wait(fence_fd, timeout);
  ATRACE_END();
  if (error < 0) {
    ALOGE("QtiMapper: lock fence %d didn't signal in %u ms -  error: %s", fence_fd, timeout,
          strerror(errno));
  }
}

Error QtiMapper::LockBuffer(void *buffer, uint64_t usage, const hidl_handle &acquire_fence) {
  if (!buffer) {
    return Error::BAD_BUFFER;
  }

  int fence_fd;
  if (!GetFenceFd(acquire_fence, &fence_fd)) {
    return Error::BAD_VALUE;
  }

  if (fence_fd > 0) {
    WaitFenceFd(fence_fd);
  }

  auto hnd = PRIV_HANDLE_CONST(buffer);

  return buf_mgr_->LockBuffer(hnd, usage);
}

Return<void> QtiMapper::lock(void *buffer, uint64_t cpu_usage,
                             const IMapper::Rect & /*access_region*/,
                             const hidl_handle &acquire_fence, lock_cb hidl_cb) {
  auto err = LockBuffer(buffer, cpu_usage, acquire_fence);
  if (err != Error::NONE) {
    hidl_cb(err, nullptr);
    return Void();
  }

  auto hnd = PRIV_HANDLE_CONST(buffer);
  auto *out_data = reinterpret_cast<void *>(hnd->base);
  hidl_cb(Error::NONE, out_data);
  return Void();
}

Return<void> QtiMapper::lockYCbCr(void *buffer, uint64_t cpu_usage,
                                  const IMapper::Rect & /*access_region*/,
                                  const hidl_handle &acquire_fence, lockYCbCr_cb hidl_cb) {
  YCbCrLayout layout = {};
  auto err = LockBuffer(buffer, cpu_usage, acquire_fence);
  if (err != Error::NONE) {
    hidl_cb(err, layout);
    return Void();
  }

  auto hnd = PRIV_HANDLE_CONST(buffer);
  android_ycbcr yuv_plane_info[2];
  if (gralloc::GetYUVPlaneInfo(hnd, yuv_plane_info) != 0) {
    hidl_cb(Error::BAD_VALUE, layout);
  }
  layout.y = yuv_plane_info[0].y;
  layout.cr = yuv_plane_info[0].cr;
  layout.cb = yuv_plane_info[0].cb;
  layout.yStride = static_cast<uint32_t>(yuv_plane_info[0].ystride);
  layout.cStride = static_cast<uint32_t>(yuv_plane_info[0].cstride);
  layout.chromaStep = static_cast<uint32_t>(yuv_plane_info[0].chroma_step);
  hidl_cb(Error::NONE, layout);
  return Void();
}

Return<void> QtiMapper::unlock(void *buffer, unlock_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  if (buffer != nullptr) {
    err = buf_mgr_->UnlockBuffer(PRIV_HANDLE_CONST(buffer));
  }
  // We don't have a release fence
  hidl_cb(err, hidl_handle(nullptr));
  return Void();
}

Return<Error> QtiMapper::validateBufferSize(void* buffer,
                                            const BufferDescriptorInfo_2_1& descriptor_info,
                                            uint32_t /*stride*/) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (buf_mgr_->IsBufferImported(hnd) != Error::NONE) {
      return Error::BAD_BUFFER;
    }
    auto info = gralloc::BufferInfo(descriptor_info.width, descriptor_info.height,
                                    static_cast<uint32_t>(descriptor_info.format),
                                    static_cast<uint64_t>(descriptor_info.usage));
    info.layer_count = descriptor_info.layerCount;
    err = buf_mgr_->ValidateBufferSize(hnd, info);
  }
  return err;
}

Return<void> QtiMapper::getTransportSize(void *buffer,
                                         IMapper_2_1::getTransportSize_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  uint32_t num_fds = 0, num_ints = 0;
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (buf_mgr_->IsBufferImported(hnd) != Error::NONE) {
      hidl_cb(err, num_fds, num_ints);
      return Void();
    }
    num_fds = 2;
    // TODO(user): reduce to transported values;
    num_ints = static_cast<uint32_t >(hnd->numInts);
    err = Error::NONE;
  }
  ALOGD_IF(DEBUG, "GetTransportSize: num fds: %d num ints: %d err:%d", num_fds, num_ints, err);
  hidl_cb(err, num_fds, num_ints);
  return Void();
}

Return<void> QtiMapper::createDescriptor_2_1(const BufferDescriptorInfo_2_1& descriptor_info,
                                             IMapper_2_1::createDescriptor_2_1_cb hidl_cb) {
  IMapperBufferDescriptor descriptor;
  auto err = CreateDescriptor(descriptor_info, &descriptor);
  hidl_cb(err, descriptor);
  return Void();
}

Return<void> QtiMapper::getMapSecureBufferFlag(void *buffer, getMapSecureBufferFlag_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  int *map_secure_buffer = 0;
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (getMetaData(hnd, GET_MAP_SECURE_BUFFER, map_secure_buffer) != 0) {
      *map_secure_buffer = 0;
    } else {
      err = Error::NONE;
    }
  }
  hidl_cb(err, *map_secure_buffer != 0);
  return Void();
}

Return<void> QtiMapper::getInterlacedFlag(void *buffer, getInterlacedFlag_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  int *interlaced_flag = nullptr;
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (getMetaData(hnd, GET_PP_PARAM_INTERLACED, interlaced_flag) != 0) {
      *interlaced_flag = 0;
    } else {
      err = Error::NONE;
    }
  }
  hidl_cb(err, *interlaced_flag != 0);
  return Void();
}

Return<void> QtiMapper::getCustomDimensions(void *buffer, getCustomDimensions_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  int stride = 0;
  int height = 0;
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    stride = hnd->width;
    height = hnd->height;
    gralloc::GetCustomDimensions(hnd, &stride, &height);
    err = Error::NONE;
  }
  hidl_cb(err, stride, height);
  return Void();
}

Return<void> QtiMapper::getRgbDataAddress(void *buffer, getRgbDataAddress_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  void *rgb_data = nullptr;
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (gralloc::GetRgbDataAddress(hnd, &rgb_data) == 0) {
      err = Error::NONE;
    }
  }
  hidl_cb(err, rgb_data);
  return Void();
}

Return<void> QtiMapper::calculateBufferAttributes(int32_t width, int32_t height, int32_t format,
                                                  uint64_t usage,
                                                  calculateBufferAttributes_cb hidl_cb) {
  unsigned int alignedw, alignedh;
  BufferInfo info(width, height, format, usage);
  gralloc::GetAlignedWidthAndHeight(info, &alignedw, &alignedh);
  bool ubwc_enabled = gralloc::IsUBwcEnabled(format, usage);
  hidl_cb(Error::NONE, alignedw, alignedh, ubwc_enabled);
  return Void();
}

Return<void> QtiMapper::getColorSpace(void *buffer, getColorSpace_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  int color_space = 0;
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    gralloc::GetColorSpaceFromMetadata(hnd, &color_space);
    err = Error::NONE;
  }
  hidl_cb(err, color_space);
  return Void();
}

Return<void> QtiMapper::getYuvPlaneInfo(void *buffer, getYuvPlaneInfo_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  hidl_vec<YCbCrLayout> layout;
  layout.resize(2);
  android_ycbcr yuv_plane_info[2];
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (gralloc::GetYUVPlaneInfo(hnd, yuv_plane_info) == 0) {
      err = Error::NONE;
      for (int i=0; i < 2; i++) {
        layout[i].y = yuv_plane_info[i].y;
        layout[i].cr = yuv_plane_info[i].cr;
        layout[i].cb = yuv_plane_info[i].cb;
        layout[i].yStride = static_cast<uint32_t>(yuv_plane_info[i].ystride);
        layout[i].cStride = static_cast<uint32_t>(yuv_plane_info[i].cstride);
        layout[i].chromaStep = static_cast<uint32_t>(yuv_plane_info[i].chroma_step);
      }
    }
  }
  hidl_cb(err, layout);
  return Void();
}

Return<Error> QtiMapper::setSingleBufferMode(void *buffer, bool enable) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (setMetaData(hnd, SET_SINGLE_BUFFER_MODE, &enable) != 0) {
      err = Error::UNSUPPORTED;
    } else {
      err = Error::NONE;
    }
  }
  return err;
}

Return<void> QtiMapper::getCustomFormatFlags(int32_t format, uint64_t /*usage*/,
                                             getCustomFormatFlags_cb hidl_cb) {
  uint64_t priv_flags = 0;
  auto err = Error::UNSUPPORTED;
  int32_t custom_format = format;
  hidl_cb(err, custom_format, priv_flags);
  return Void();
}

Return<void> QtiMapper::getFd(void *buffer, getFd_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int fd = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    fd = hnd->fd;
  }
  hidl_cb(err, fd);
  return Void();
}

Return<void> QtiMapper::getWidth(void *buffer, getWidth_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int width = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    width = hnd->width;
  }
  hidl_cb(err, width);
  return Void();
}

Return<void> QtiMapper::getHeight(void *buffer, getHeight_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int height = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    height = hnd->height;
  }
  hidl_cb(err, height);
  return Void();
}

Return<void> QtiMapper::getFormat(void *buffer, getFormat_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int format = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    format = hnd->format;
  }
  hidl_cb(err, format);
  return Void();
}

Return<void> QtiMapper::getPrivateFlags(void *buffer, getPrivateFlags_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int flags = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    flags = hnd->flags;
  }
  hidl_cb(err, flags);
  return Void();
}

Return<void> QtiMapper::getUnalignedWidth(void *buffer, getUnalignedWidth_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int unaligned_width = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    unaligned_width = hnd->unaligned_width;
  }
  hidl_cb(err, unaligned_width);
  return Void();
}

Return<void> QtiMapper::getUnalignedHeight(void *buffer, getUnalignedHeight_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int unaligned_height = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    unaligned_height = hnd->unaligned_height;
  }
  hidl_cb(err, unaligned_height);
  return Void();
}

Return<void> QtiMapper::getLayerCount(void *buffer, getLayerCount_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  unsigned int layer_count = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    layer_count = hnd->layer_count;
  }
  hidl_cb(err, layer_count);
  return Void();
}

Return<void> QtiMapper::getId(void *buffer, getId_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  uint64_t id = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    id = hnd->id;
  }
  hidl_cb(err, id);
  return Void();
}

Return<void> QtiMapper::getUsageFlags(void *buffer, getUsageFlags_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  uint64_t usage = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    usage = hnd->usage;
  }
  hidl_cb(err, usage);
  return Void();
}

Return<void> QtiMapper::getSize(void *buffer, getSize_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  unsigned int size = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    size = hnd->size;
  }
  hidl_cb(err, size);
  return Void();
}

Return<void> QtiMapper::getOffset(void *buffer, getOffset_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  unsigned int offset = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    offset = hnd->offset;
  }
  hidl_cb(err, offset);
  return Void();
}

// Methods from ::android::hidl::base::V1_0::IBase follow.

// When we are in passthrough mode, this method is used
// by hidl to obtain the SP HAL object
IMapper_2_1 *HIDL_FETCH_IMapper(const char * /* name */) {
  ALOGD_IF(DEBUG, "Fetching IMapper from QtiMapper");
  auto mapper = new QtiMapper();
  return static_cast<IMapper_2_1 *>(mapper);
}

IQtiMapper *HIDL_FETCH_IQtiMapper(const char * /* name */) {
  ALOGD_IF(DEBUG, "Fetching QtiMapper");
  return new QtiMapper();
}

}  // namespace implementation
}  // namespace V1_1
}  // namespace mapper
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
