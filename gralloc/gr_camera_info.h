/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.

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
 */

#ifndef __GR_CAMERA_INFO_H__
#define __GR_CAMERA_INFO_H__

#include <camx/camxformatutilexternal.h>

#include "gr_utils.h"

namespace gralloc {

class CameraInfo {
 public:
  int GetUBWCInfo(int format, bool *is_Supported, bool *is_PI, int *version);

  int GetPlaneAlignment(int format, int plane_type, unsigned int *alignment);

  int IsPerPlaneFdNeeded(int format, bool *is_per_plane_fd_needed);

  int GetBpp(int format, int *bpp);

  int GetPerPlaneBpp(int format, int plane_type, int *bpp);

  int GetPlaneStartAddressAlignment(int format, int plane_type, int *alignment);

  int GetBufferSize(int format, int width, int height, unsigned int *size);

  int GetStrideInBytes(int format, int plane_type, int width, int *stride_bytes);

  int GetStrideInPixels(int format, int plane_type, int width, float *stride_pixel);

  int GetPixelIncrement(int format, int plane_type, int *pixel_increment);

  int GetPlaneOffset(int format, int plane_type, int *offset);

  int GetSubsamplingFactor(int format, int plane_type, bool isHorizontal, int *subsampling_factor);

  int GetPlaneTypes(int format, PlaneComponent *plane_component_array, int *plane_count);

  int GetScanline(int format, int plane_type, int height, int *scanlines);

  int GetPlaneSize(int format, int plane_type, int width, int height, unsigned int *size);

  int GetCameraFormatPlaneInfo(int format, int width, int height, int *plane_count,
                               PlaneLayoutInfo *plane_info);

  CamxPixelFormat GetCameraPixelFormat(int hal_format);

  static CameraInfo *GetInstance();

 private:
  CameraInfo();
  ~CameraInfo();

  PlaneComponent GetPlaneComponent(CamxPlaneType plane_type);

  CamxPlaneType GetCamxPlaneType(int plane_type);

  CamxFormatResult (*LINK_camera_get_stride_in_bytes)(CamxPixelFormat format,
                                                      CamxPlaneType plane_type, int width,
                                                      int *stride) = nullptr;

  CamxFormatResult (*LINK_camera_get_stride_in_pixels)(CamxPixelFormat format,
                                                       CamxPlaneType plane_type, int width,
                                                       float *stride) = nullptr;

  CamxFormatResult (*LINK_camera_get_scanline)(CamxPixelFormat format, CamxPlaneType plane_type,
                                               int height, int *scanLine) = nullptr;

  CamxFormatResult (*LINK_camera_get_plane_size)(CamxPixelFormat format, CamxPlaneType plane_type,
                                                 int width, int height,
                                                 unsigned int *aligned_size) = nullptr;

  CamxFormatResult (*LINK_camera_get_buffer_size)(CamxPixelFormat format, int width, int height,
                                                  unsigned int *buffer_size) = nullptr;

  CamxFormatResult (*LINK_camera_get_ubwc_info)(CamxPixelFormat format, bool *isSupported,
                                                bool *isPI, int *version) = nullptr;

  CamxFormatResult (*LINK_camera_get_plane_alignment)(CamxPixelFormat format,
                                                      CamxPlaneType plane_type,
                                                      unsigned int *alignment) = nullptr;

  CamxFormatResult (*LINK_camera_get_plane_offset)(CamxPixelFormat format, CamxPlaneType plane_type,
                                                   int *offset) = nullptr;

  CamxFormatResult (*LINK_camera_get_plane_types)(CamxPixelFormat format,
                                                  CamxPlaneType *plane_types_array,
                                                  int *plane_count) = nullptr;

  CamxFormatResult (*LINK_camera_is_per_plane_fd_needed)(CamxPixelFormat format,
                                                         bool *is_perplane_fd_needed) = nullptr;

  CamxFormatResult (*LINK_camera_get_bpp)(CamxPixelFormat format, int *bpp) = nullptr;

  CamxFormatResult (*LINK_camera_get_per_plane_bpp)(CamxPixelFormat format,
                                                    CamxPlaneType plane_type, int *bpp) = nullptr;

  CamxFormatResult (*LINK_camera_get_subsampling_factor)(CamxPixelFormat format,
                                                         CamxPlaneType plane_type,
                                                         bool is_horizontal,
                                                         int *subsampling_factor) = nullptr;

  CamxFormatResult (*LINK_camera_get_plane_count)(CamxPixelFormat format,
                                                  int *plane_count) = nullptr;

  CamxFormatResult (*LINK_camera_get_pixel_increment)(CamxPixelFormat format,
                                                      CamxPlaneType plane_type,
                                                      int *pixel_increment) = nullptr;

  CamxFormatResult (*LINK_camera_get_plane_start_address_alignment)(CamxPixelFormat format,
                                                                    CamxPlaneType planeType,
                                                                    int *pAlignment) = nullptr;

  void *libcamera_utils_ = nullptr;
  static CameraInfo *s_instance;
};

}  // namespace gralloc

#endif  // __GR_CAMERA_INFO_H__
