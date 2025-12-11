/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <log/log.h>
#include <cutils/properties.h>
#include <dlfcn.h>
#include <mutex>
#include "gr_ubwcp_utils.h"

using std::lock_guard;
using std::mutex;

namespace gralloc {

UbwcpUtils *UbwcpUtils::s_instance = nullptr;

UbwcpUtils *UbwcpUtils::GetInstance() {
  static mutex s_lock;
  lock_guard<mutex> obj(s_lock);
  if (!s_instance) {
    s_instance = new UbwcpUtils();
  }

  return s_instance;
}

UbwcpUtils::UbwcpUtils() {
  libUbwcpUtils_ = ::dlopen("libubwcp.so", RTLD_NOW);
  if (libUbwcpUtils_) {
    *reinterpret_cast<void **>(&LINK_UBWCPLib_create_session) =
        ::dlsym(libUbwcpUtils_, "UBWCPLib_create_session");
    *reinterpret_cast<void **>(&LINK_UBWCPLib_destroy_session) =
        ::dlsym(libUbwcpUtils_, "UBWCPLib_destroy_session");
#ifdef TARGET_USES_UBWCP
    *reinterpret_cast<void **>(&LINK_UBWCPLib_get_stride_alignment) =
        ::dlsym(libUbwcpUtils_, "UBWCPLib_get_stride_alignment");
    *reinterpret_cast<void **>(&LINK_UBWCPLib_validate_stride) =
        ::dlsym(libUbwcpUtils_, "UBWCPLib_validate_stride");
    *reinterpret_cast<void **>(&LINK_UBWCPLib_set_buf_attrs) =
        ::dlsym(libUbwcpUtils_, "UBWCPLib_set_buf_attrs");
#endif
  } else {
    ALOGW("Failed to load libubwcp.so");
  }
}

UbwcpUtils::~UbwcpUtils() {
  if (libUbwcpUtils_) {
    ::dlclose(libUbwcpUtils_);
  }
}
#ifdef TARGET_USES_UBWCP
UBWCPLib_Image_Format UbwcpUtils::GetUbwcpPixelFormat(int hal_format) {
  switch (hal_format) {
    case HAL_PIXEL_FORMAT_RGBA_8888:
      return UBWCPLib_RGBA8888;
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
      return UBWCPLib_NV12;
    case HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC:
      return UBWCPLib_TP10;
    case HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC:
      return UBWCPLib_P010;
    default:
      ALOGE("%s: No map for format: 0x%x", __FUNCTION__, hal_format);
      break;
  }
  return UBWCPLib_NUM_FORMATS;
}

bool UbwcpUtils::IsStrideAligned(int hal_format, int width) {
  UBWCPLib_Image_Format ubwcp_format = GetUbwcpPixelFormat(hal_format);
  size_t stride_req;
  void *session;
  int ret = false;

  session = LINK_UBWCPLib_create_session();
  if (!session) {
    ALOGW("Unable to create UBWCPLib session\n");
    return ret;
  }

  LINK_UBWCPLib_get_stride_alignment(session, ubwcp_format, &stride_req);

  // if width is aligned to stride_req
  if (ALIGN(width, stride_req))
    ret = true;
  else {
    ALOGW("Stride is not aligned for format: %d width:%d stride_requirement: %zu", hal_format, width,
          stride_req);
    ret = false;
  }

  LINK_UBWCPLib_destroy_session(session);
  return ret;
}

#endif

void UbwcpUtils::ConfigUBWCPAttributes(private_handle_t const *handle) {
#ifdef TARGET_USES_UBWCP
  private_handle_t *hnd = const_cast<private_handle_t *>(handle);
  int ret = 0;
  BufferInfo info(hnd->unaligned_width, hnd->unaligned_height, hnd->format,
                  hnd->usage | GRALLOC_USAGE_PRIVATE_NO_UBWC_P);

  if (IsUBwcPEnabled(hnd->format, hnd->usage)) {
    void *session;
    hnd->ubwcp_format = true;
    size_t planar_padding;

    session = LINK_UBWCPLib_create_session();
    if (!session) {
      ALOGW("Unable to create UBWCPLib session\n");
      return;
    }
    UBWCPLib_buf_attrs attrsUBWCP;
    memset(&attrsUBWCP, 0, sizeof(UBWCPLib_buf_attrs));

    int plane_count = 0;

    gralloc::PlaneLayoutInfo plane_layout[8] = {};
    if (gralloc::IsYuvFormat(handle->format)) {
      gralloc::GetYUVPlaneInfo(info, handle->format, handle->unaligned_width,
                               handle->unaligned_height, handle->flags, &plane_count, plane_layout,
                               handle);
      if (handle->format == HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC) {
        planar_padding = plane_layout[3].offset - plane_layout[0].offset - plane_layout[0].size;
      } else {
        planar_padding = plane_layout[1].offset - plane_layout[0].offset - plane_layout[0].size;
      }
      attrsUBWCP.planar_padding = planar_padding;
      hnd->linear_size = plane_layout[0].size + plane_layout[1].size;
    } else {
      gralloc::GetRGBPlaneInfo(info, handle->format, handle->unaligned_width,
                               handle->unaligned_height, handle->flags, &plane_count, plane_layout);
      attrsUBWCP.planar_padding = 0;
      hnd->linear_size = plane_layout[0].size;
    }

    attrsUBWCP.image_format = GetUbwcpPixelFormat(hnd->format);
    attrsUBWCP.width = hnd->unaligned_width;
    attrsUBWCP.height = hnd->unaligned_height;
    attrsUBWCP.stride = plane_layout[0].stride_bytes;
    attrsUBWCP.scanlines = plane_layout[0].scanlines;

    ALOGD("attrsUBWCP f:0x%x, w: %d, h: %d stride: %d scan_lines: %d planar_padding: %d size:%d \n",
          attrsUBWCP.image_format, attrsUBWCP.width, attrsUBWCP.height, attrsUBWCP.stride,
          attrsUBWCP.scanlines, attrsUBWCP.planar_padding, hnd->linear_size);

    ret = LINK_UBWCPLib_set_buf_attrs(session, hnd->fd, &attrsUBWCP);
    if (ret) {
      ALOGE("UBWCPLib_set_buf_attrs failed and ret is %d \n", ret);
      return;
    }

    LINK_UBWCPLib_destroy_session(session);
  }
#endif
}
}  // namespace gralloc
