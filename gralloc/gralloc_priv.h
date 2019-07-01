/*
 * Copyright (c) 2011-2019, The Linux Foundation. All rights reserved.
 * Not a Contribution
 *
 * Copyright (C) 2008 The Android Open Source Project
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

#ifndef __GRALLOC_PRIV_H__
#define __GRALLOC_PRIV_H__

#include <errno.h>
#include <unistd.h>
#include "gr_priv_handle.h"

#define GRALLOC_PROP_PREFIX  "vendor.gralloc."
#define GRALLOC_PROP(prop_name) GRALLOC_PROP_PREFIX prop_name

#define DISABLE_UBWC_PROP                    GRALLOC_PROP("disable_ubwc")
#define DISABLE_AHARDWAREBUFFER_PROP         GRALLOC_PROP("disable_ahardware_buffer")
#define ENABLE_FB_UBWC_PROP                  GRALLOC_PROP("enable_fb_ubwc")
#define MAP_FB_MEMORY_PROP                   GRALLOC_PROP("map_fb_memory")
#define USE_SYSTEM_HEAP_FOR_SENSORS          GRALLOC_PROP("use_system_heap_for_sensors")

#define ROUND_UP_PAGESIZE(x) roundUpToPageSize(x)
inline int roundUpToPageSize(int x) {
  return (x + (getpagesize() - 1)) & ~(getpagesize() - 1);
}

/* Gralloc usage bits indicating the type of allocation that should be used */
/* Refer to BufferUsage in hardware/interfaces/graphics/common/<ver>/types.hal */

/* The bits below are in officially defined vendor space
 * i.e bits 28-31 and 48-63*/
/* Non linear, Universal Bandwidth Compression */
#define GRALLOC_USAGE_PRIVATE_ALLOC_UBWC (UINT32_C(1) << 28)

/* Set this for allocating uncached memory (using O_DSYNC),
 * cannot be used with noncontiguous heaps */
#define GRALLOC_USAGE_PRIVATE_UNCACHED (UINT32_C(1) << 29)

/* This flag is used to indicate 10 bit format.
 * When both GRALLOC_USAGE_PRIVATE_ALLOC_UBWC & GRALLOC_USAGE_PRIVATE_10BIT
 * are set then it will indicate UBWC_TP10 format.
 * When only GRALLOC_USAGE_PRIVATE_10BIT is set it will indicate linear P010 format.
 */
#define GRALLOC_USAGE_PRIVATE_10BIT (UINT32_C(1) << 30)

/* This flag is used for SECURE display usecase */
#define GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY (UINT32_C(1) << 31)

/* This flag is used to indicate video NV21 format */
#define GRALLOC_USAGE_PRIVATE_VIDEO_NV21_ENCODER 1ULL << 48

/* unused legacy flags */
#define GRALLOC_USAGE_PRIVATE_MM_HEAP 0
#define GRALLOC_USAGE_PRIVATE_IOMMU_HEAP 0

/* TODO(user): move these to use sanctioned vendor bits
 * once end to end 64-bit support is available */
/* This flag is set for WFD usecase */
#define GRALLOC_USAGE_PRIVATE_WFD (UINT32_C(1) << 21)

/* TODO(user): move these to use sanctioned vendor bits
 * once end to end 64-bit support is available */
/* This flag is set for HEIF usecase */
#define GRALLOC_USAGE_PRIVATE_HEIF (UINT32_C(1) << 27)

/* TODO(user): Remove when clients stop referencing this flag */
#define GRALLOC_USAGE_PRIVATE_10BIT_TP 0

/* This flag indicates PI format is being used */
#define GRALLOC_USAGE_PRIVATE_ALLOC_UBWC_PI 1ULL << 49

/* This flag is set while CDSP accesses the buffer */
#define GRALLOC_USAGE_PRIVATE_CDSP 1ULL << 50

/* Legacy gralloc1 definitions */
/* Some clients may still be using the old flags */
#define GRALLOC1_PRODUCER_USAGE_PRIVATE_ADSP_HEAP GRALLOC_USAGE_PRIVATE_ADSP_HEAP
#define GRALLOC1_PRODUCER_USAGE_PRIVATE_ALLOC_UBWC GRALLOC_USAGE_PRIVATE_ALLOC_UBWC
#define GRALLOC1_PRODUCER_USAGE_PRIVATE_UNCACHED GRALLOC_USAGE_PRIVATE_UNCACHED
#define GRALLOC1_CONSUMER_USAGE_PRIVATE_SECURE_DISPLAY GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY
#define GRALLOC1_PRODUCER_USAGE_PRIVATE_MM_HEAP GRALLOC_USAGE_PRIVATE_MM_HEAP
#define GRALLOC1_PRODUCER_USAGE_PRIVATE_10BIT GRALLOC_USAGE_PRIVATE_10BIT
#define GRALLOC1_PRODUCER_USAGE_PRIVATE_10BIT_TP GRALLOC_USAGE_PRIVATE_10BIT_TP
#define GRALLOC1_CONSUMER_USAGE_PRIVATE_10BIT_TP GRALLOC_USAGE_PRIVATE_10BIT_TP
#define GRALLOC1_PRODUCER_USAGE_PRIVATE_VIDEO_NV21_ENCODER GRALLOC_USAGE_PRIVATE_VIDEO_NV21_ENCODER

// for PERFORM API :
#define GRALLOC_MODULE_PERFORM_CREATE_HANDLE_FROM_BUFFER 1
#define GRALLOC_MODULE_PERFORM_GET_STRIDE 2
#define GRALLOC_MODULE_PERFORM_GET_CUSTOM_STRIDE_FROM_HANDLE 3
#define GRALLOC_MODULE_PERFORM_GET_CUSTOM_STRIDE_AND_HEIGHT_FROM_HANDLE 4
#define GRALLOC_MODULE_PERFORM_GET_ATTRIBUTES 5
#define GRALLOC_MODULE_PERFORM_GET_COLOR_SPACE_FROM_HANDLE 6
#define GRALLOC_MODULE_PERFORM_GET_YUV_PLANE_INFO 7
#define GRALLOC_MODULE_PERFORM_GET_MAP_SECURE_BUFFER_INFO 8
#define GRALLOC_MODULE_PERFORM_GET_UBWC_FLAG 9
#define GRALLOC_MODULE_PERFORM_GET_RGB_DATA_ADDRESS 10
#define GRALLOC_MODULE_PERFORM_GET_IGC 11
#define GRALLOC_MODULE_PERFORM_SET_IGC 12
#define GRALLOC_MODULE_PERFORM_SET_SINGLE_BUFFER_MODE 13
#define GRALLOC1_MODULE_PERFORM_GET_BUFFER_SIZE_AND_DIMENSIONS 14
#define GRALLOC1_MODULE_PERFORM_GET_INTERLACE_FLAG 15
#define GRALLOC_MODULE_PERFORM_GET_GRAPHICS_METADATA 16

// OEM specific HAL formats
#define HAL_PIXEL_FORMAT_RGBA_5551 6
#define HAL_PIXEL_FORMAT_RGBA_4444 7
#define HAL_PIXEL_FORMAT_NV12_ENCODEABLE 0x102
#define HAL_PIXEL_FORMAT_NV21_ENCODEABLE 0x7FA30C00
#define HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS 0x7FA30C04
#define HAL_PIXEL_FORMAT_YCbCr_420_SP_TILED 0x7FA30C03
#define HAL_PIXEL_FORMAT_YCbCr_420_SP 0x109
#define HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO 0x7FA30C01
#define HAL_PIXEL_FORMAT_YCrCb_422_SP 0x10B
#define HAL_PIXEL_FORMAT_R_8 0x10D
#define HAL_PIXEL_FORMAT_RG_88 0x10E
#define HAL_PIXEL_FORMAT_YCbCr_444_SP 0x10F
#define HAL_PIXEL_FORMAT_YCrCb_444_SP 0x110
#define HAL_PIXEL_FORMAT_YCrCb_422_I 0x111
#define HAL_PIXEL_FORMAT_BGRX_8888 0x112
#define HAL_PIXEL_FORMAT_NV21_ZSL 0x113
#define HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS 0x114
#define HAL_PIXEL_FORMAT_BGR_565 0x115
#define HAL_PIXEL_FORMAT_RAW8 0x123
#define HAL_PIXEL_FORMAT_NV12_HEIF 0x116

// 10 bit
#define HAL_PIXEL_FORMAT_ARGB_2101010 0x117
#define HAL_PIXEL_FORMAT_RGBX_1010102 0x118
#define HAL_PIXEL_FORMAT_XRGB_2101010 0x119
#define HAL_PIXEL_FORMAT_BGRA_1010102 0x11A
#define HAL_PIXEL_FORMAT_ABGR_2101010 0x11B
#define HAL_PIXEL_FORMAT_BGRX_1010102 0x11C
#define HAL_PIXEL_FORMAT_XBGR_2101010 0x11D
#define HAL_PIXEL_FORMAT_YCbCr_420_P010 0x11F
#define HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC 0x124
#define HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS 0x7FA30C0A

#define HAL_PIXEL_FORMAT_CbYCrY_422_I 0x120
#define HAL_PIXEL_FORMAT_BGR_888 0x121

#define HAL_PIXEL_FORMAT_INTERLACE 0x180

// Camera utils format
#define HAL_PIXEL_FORMAT_NV12_LINEAR_FLEX  0x125
#define HAL_PIXEL_FORMAT_NV12_UBWC_FLEX 0x126
#define HAL_PIXEL_FORMAT_MULTIPLANAR_FLEX 0x127

// v4l2_fourcc('Y', 'U', 'Y', 'L'). 24 bpp YUYV 4:2:2 10 bit per component
#define HAL_PIXEL_FORMAT_YCbCr_422_I_10BIT 0x4C595559

// v4l2_fourcc('Y', 'B', 'W', 'C'). 10 bit per component. This compressed
// format reduces the memory access bandwidth
#define HAL_PIXEL_FORMAT_YCbCr_422_I_10BIT_COMPRESSED 0x43574259

// UBWC aligned Venus format
#define HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC 0x7FA30C06
#define HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC 0x7FA30C09

// Khronos ASTC formats
#define HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_4x4_KHR 0x93B0
#define HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_5x4_KHR 0x93B1
#define HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_5x5_KHR 0x93B2
#define HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_6x5_KHR 0x93B3
#define HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_6x6_KHR 0x93B4
#define HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x5_KHR 0x93B5
#define HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x6_KHR 0x93B6
#define HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x8_KHR 0x93B7
#define HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x5_KHR 0x93B8
#define HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x6_KHR 0x93B9
#define HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x8_KHR 0x93BA
#define HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x10_KHR 0x93BB
#define HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_12x10_KHR 0x93BC
#define HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_12x12_KHR 0x93BD
#define HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR 0x93D0
#define HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR 0x93D1
#define HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR 0x93D2
#define HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR 0x93D3
#define HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR 0x93D4
#define HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR 0x93D5
#define HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR 0x93D6
#define HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR 0x93D7
#define HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR 0x93D8
#define HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR 0x93D9
#define HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR 0x93DA
#define HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR 0x93DB
#define HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR 0x93DC
#define HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR 0x93DD

/* possible values for inverse gamma correction */
#define HAL_IGC_NOT_SPECIFIED 0
#define HAL_IGC_s_RGB 1

/* Color Space: Values maps to ColorSpace_t in qdMetadata.h */
#define HAL_CSC_ITU_R_601 0
#define HAL_CSC_ITU_R_601_FR 1
#define HAL_CSC_ITU_R_709 2
#define HAL_CSC_ITU_R_2020 3
#define HAL_CSC_ITU_R_2020_FR 4

/* possible formats for 3D content*/
enum {
  HAL_NO_3D = 0x0,
  HAL_3D_SIDE_BY_SIDE_L_R = 0x1,
  HAL_3D_SIDE_BY_SIDE_R_L = 0x2,
  HAL_3D_TOP_BOTTOM = 0x4,
  HAL_3D_IN_SIDE_BY_SIDE_L_R = 0x10000,  // unused legacy format
};

enum { BUFFER_TYPE_UI = 0, BUFFER_TYPE_VIDEO };

/* Flag to determine interlaced content
 * Value maps to Flags presents in types.hal of QtiMapperextensions
 */
enum {
  LAYOUT_INTERLACED_FLAG = 1 << 0,
};

#endif  // __GRALLOC_PRIV_H__
