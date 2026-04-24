/*
 * Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
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
 * Copyright (c) 2023-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <aidl/android/hardware/graphics/allocator/BufferDescriptorInfo.h>
#include <aidl/android/hardware/graphics/common/BufferUsage.h>
#include <aidl/android/hardware/graphics/common/PixelFormat.h>
#include <aidl/android/hardware/graphics/common/StandardMetadataType.h>
#include <android/hardware/graphics/mapper/IMapper.h>
#include <android/hardware/graphics/mapper/utils/IMapperMetadataTypes.h>
#include <android/hardware/graphics/mapper/utils/IMapperProvider.h>
#include <cutils/native_handle.h>
#include <dlfcn.h>
#include <vndksupport/linker.h>

#include <QtiGralloc.h>
#include <gralloctypes/Gralloc4.h>

#include <algorithm>
#include <string>

#include "gr_buf_mgr.h"
#include "gr_snap_helper.h"
#include "mapper_utils.h"

namespace stablec {
namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace mapper5 {

using namespace ::aidl::android::hardware::graphics::common;
using namespace ::android::hardware::graphics::mapper;
using ::aidl::android::hardware::graphics::allocator::BufferDescriptorInfo;
using ::android::base::unique_fd;
using Error = AIMapper_Error;

using gralloc::BufferManager;
using mapper::isStandardMetadata;
using mapper::isVendorMetadata;
using mapper::STANDARD_METADATA_NAME;
using mapper::VENDOR_QTI_METADATA_NAME;

#define REQUIRE_DRIVER()                                       \
  if (!snap_helper_ || !snap_alloc_enable_) {                  \
    ALOGE("Failed to %s. Driver is uninitialized.", __func__); \
    return AIMAPPER_ERROR_NO_RESOURCES;                        \
  }

#define VALIDATE_BUFFER_HANDLE(bufferHandle)                \
  if (!(bufferHandle)) {                                    \
    ALOGW("Failed to %s. Null buffer_handle_t.", __func__); \
    return AIMAPPER_ERROR_BAD_BUFFER;                       \
  }

constexpr unsigned int METADATA_BUFFERSIZE_INITIAL = 10000;

#define VALIDATE_DRIVER_AND_BUFFER_HANDLE(bufferHandle) \
  REQUIRE_DRIVER()                                      \
  VALIDATE_BUFFER_HANDLE(bufferHandle)

class QtiMapper5 final : public ::vendor::mapper::IMapperV5Impl {
 public:
  QtiMapper5();
  ~QtiMapper5() override = default;
  Error importBuffer(const native_handle_t *_Nonnull handle,
                     buffer_handle_t _Nullable *_Nonnull outBufferHandle) override;
  Error freeBuffer(buffer_handle_t _Nonnull buffer) override;
  Error getTransportSize(buffer_handle_t _Nonnull buffer, uint32_t *_Nonnull outNumFds,
                         uint32_t *_Nonnull outNumInts) override;
  Error lock(buffer_handle_t _Nonnull buffer, uint64_t cpuUsage, ARect accessRegion,
             int acquireFence, void *_Nullable *_Nonnull outData) override;
  Error unlock(buffer_handle_t _Nonnull buffer, int *_Nonnull releaseFence) override;
  Error flushLockedBuffer(buffer_handle_t _Nonnull buffer) override;
  Error rereadLockedBuffer(buffer_handle_t _Nonnull buffer) override;
  int32_t getMetadata(buffer_handle_t _Nonnull buffer, AIMapper_MetadataType metadataType,
                      void *_Nonnull outData, size_t outDataSize) override;
  int32_t getStandardMetadata(buffer_handle_t _Nonnull buffer, int64_t standardMetadataType,
                              void *_Nonnull outData, size_t outDataSize) override;
  Error setMetadata(buffer_handle_t _Nonnull buffer, AIMapper_MetadataType metadataType,
                    const void *_Nonnull metadata, size_t metadataSize) override;
  /**
  *  Sets global values for given standard metadata types
  *
  *  For std::optional metadata types, nullopt can be provided as a valid metadata buffer to
  *  explicitly invalidate the existing metadata value
  */
  Error setStandardMetadata(buffer_handle_t _Nonnull buffer, int64_t standardMetadataType,
                            const void *_Nonnull metadata, size_t metadataSize) override;
  Error listSupportedMetadataTypes(
      const AIMapper_MetadataTypeDescription *_Nullable *_Nonnull outDescriptionList,
      size_t *_Nonnull outNumberOfDescriptions) override;
  Error dumpBuffer(buffer_handle_t _Nonnull bufferHandle,
                   AIMapper_DumpBufferCallback _Nonnull dumpBufferCallback,
                   void *_Null_unspecified context) override;
  Error dumpAllBuffers(AIMapper_BeginDumpBufferCallback _Nonnull beginDumpBufferCallback,
                       AIMapper_DumpBufferCallback _Nonnull dumpBufferCallback,
                       void *_Null_unspecified context) override;
  Error getReservedRegion(buffer_handle_t _Nonnull buffer,
                          void *_Nullable *_Nonnull outReservedRegion,
                          uint64_t *_Nonnull outReservedSize) override;

 private:
  Error DumpBufferMetadata(buffer_handle_t _Nonnull buffer,
                           AIMapper_DumpBufferCallback _Nonnull dumpBufferCallback,
                           void *_Null_unspecified context);
  int32_t GetMetadataPrivate(buffer_handle_t _Nonnull bufferHandle, int64_t metadataType,
                             void *_Nonnull outData, size_t outDataSize, bool isStandard);
  Error SetMetadataPrivate(buffer_handle_t _Nonnull bufferHandle, int64_t metadataType,
                           const void *_Nonnull metadata, size_t metadataSize, bool isStandard);
  size_t GetExpectedSize(uint64_t metadata_type);

  gralloc::GrallocSnapHelper *_Nullable snap_helper_ = nullptr;
  bool snap_alloc_enable_ = false;

  std::unordered_map<uint64_t, size_t> type_to_size_{
      {static_cast<uint64_t>(SnapMetadataType::BUFFER_ID), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::NAME), sizeof(std::string)},
      {static_cast<uint64_t>(SnapMetadataType::WIDTH), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::HEIGHT), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::LAYER_COUNT), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::PIXEL_FORMAT_REQUESTED), sizeof(SnapPixelFormat)},
      {static_cast<uint64_t>(SnapMetadataType::PIXEL_FORMAT_FOURCC), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::DRM_PIXEL_FORMAT_MODIFIER), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::USAGE), sizeof(SnapUsage)},
      {static_cast<uint64_t>(SnapMetadataType::ALLOCATION_SIZE), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::PROTECTED_CONTENT), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::COMPRESSION), sizeof(int64_t)},
      {static_cast<uint64_t>(SnapMetadataType::INTERLACED), sizeof(int64_t)},
      {static_cast<uint64_t>(SnapMetadataType::CHROMA_SITING), sizeof(int64_t)},
      {static_cast<uint64_t>(SnapMetadataType::PLANE_LAYOUTS), sizeof(SnapBufferLayout)},
      {static_cast<uint64_t>(SnapMetadataType::CROP), sizeof(SnapRect)},
      {static_cast<uint64_t>(SnapMetadataType::DATASPACE), sizeof(SnapDataspace)},
      {static_cast<uint64_t>(SnapMetadataType::BLEND_MODE), sizeof(SnapBlendMode)},
      {static_cast<uint64_t>(SnapMetadataType::VT_TIMESTAMP), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::PP_PARAM_INTERLACED), sizeof(int32_t)},
      {static_cast<uint64_t>(SnapMetadataType::VIDEO_PERF_MODE), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::GRAPHICS_METADATA), sizeof(SnapGraphicsMetadata)},
      {static_cast<uint64_t>(SnapMetadataType::UBWC_CR_STATS_INFO),
       (sizeof(SnapUBWCStats) * QTI_UBWC_STATS_ARRAY_SIZE)},
      {static_cast<uint64_t>(SnapMetadataType::REFRESH_RATE), sizeof(float)},
      {static_cast<uint64_t>(SnapMetadataType::MAP_SECURE_BUFFER), sizeof(int32_t)},
      {static_cast<uint64_t>(SnapMetadataType::LINEAR_FORMAT), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::SINGLE_BUFFER_MODE), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::CVP_METADATA), sizeof(SnapCVPMetadata)},
      {static_cast<uint64_t>(SnapMetadataType::VIDEO_HISTOGRAM_STATS),
       sizeof(SnapVideoHistogramMetadata)},
      {static_cast<uint64_t>(SnapMetadataType::FD), sizeof(int32_t)},
      {static_cast<uint64_t>(SnapMetadataType::ALIGNED_WIDTH_IN_PIXELS), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::STRIDE), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::ALIGNED_HEIGHT_IN_PIXELS), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::STANDARD_METADATA_STATUS),
       (sizeof(bool) * METADATA_SET_SIZE)},
      {static_cast<uint64_t>(SnapMetadataType::VENDOR_METADATA_STATUS),
       (sizeof(bool) * METADATA_SET_SIZE)},
      {static_cast<uint64_t>(SnapMetadataType::BUFFER_TYPE), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::VIDEO_TS_INFO), sizeof(SnapVideoTimestampInfo)},
      {static_cast<uint64_t>(SnapMetadataType::CUSTOM_DIMENSIONS_STRIDE), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::CUSTOM_DIMENSIONS_HEIGHT), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::RGB_DATA_ADDRESS), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::BUFFER_PERMISSION),
       sizeof(SnapBufferPermission) * BUFFERCLIENT_MAX},
      {static_cast<uint64_t>(SnapMetadataType::MEM_HANDLE), sizeof(int64_t)},
      {static_cast<uint64_t>(SnapMetadataType::TIMED_RENDERING), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::CUSTOM_CONTENT_METADATA),
       sizeof(SnapCustomContentMetadata)},
      {static_cast<uint64_t>(SnapMetadataType::VIDEO_TRANSCODE_STATS),
       sizeof(SnapVideoTranscodeStatsMetadata)},
      {static_cast<uint64_t>(SnapMetadataType::MASTERING_DISPLAY), sizeof(SnapMasteringDisplay)},
      {static_cast<uint64_t>(StandardMetadataType::SMPTE2086), sizeof(SnapMasteringDisplay)},
      {static_cast<uint64_t>(SnapMetadataType::CONTENT_LIGHT_LEVEL), sizeof(SnapContentLightLevel)},
      {static_cast<uint64_t>(StandardMetadataType::CTA861_3), sizeof(SnapContentLightLevel)},
      {static_cast<uint64_t>(SnapMetadataType::DYNAMIC_METADATA), sizeof(SnapDynamicMetadata)},
      {static_cast<uint64_t>(StandardMetadataType::SMPTE2094_40), sizeof(SnapDynamicMetadata)},
      {static_cast<uint64_t>(StandardMetadataType::SMPTE2094_10),
       sizeof(SnapCustomContentMetadata)},
      {static_cast<uint64_t>(SnapMetadataType::COLOR_REMAPPING_INFO),
       sizeof(SnapColorRemappingInfo)},
      {static_cast<uint64_t>(SnapMetadataType::MATRIX_COEFFICIENTS),
       sizeof(SnapMatrixCoEfficients)},
      {static_cast<uint64_t>(SnapMetadataType::HEAP_NAME), sizeof(std::string)},
      {static_cast<uint64_t>(SnapMetadataType::IS_UBWC), sizeof(int64_t)},
      {static_cast<uint64_t>(SnapMetadataType::IS_TILE_RENDERED), sizeof(int64_t)},
      {static_cast<uint64_t>(SnapMetadataType::IS_CACHED), sizeof(int64_t)},
      {static_cast<uint64_t>(SnapMetadataType::PIXEL_FORMAT_ALLOCATED), sizeof(SnapPixelFormat)},
      {static_cast<uint64_t>(SnapMetadataType::EARLYNOTIFY_LINECOUNT), sizeof(int32_t)},
      {static_cast<uint64_t>(SnapMetadataType::BASE_ADDRESS), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::BUFFER_DEQUEUE_DURATION), sizeof(int64_t)},
      {static_cast<uint64_t>(SnapMetadataType::ANAMORPHIC_COMPRESSION_METADATA),
       sizeof(SnapAnamorphicMetadata)},
      // TODO: Remove the legacy type below once HWC has moved to Snap defs
      {static_cast<uint64_t>(QTI_COLOR_METADATA), sizeof(ColorMetaData)},
      {static_cast<uint64_t>(SnapMetadataType::THREE_DIMENSIONAL_REF_INFO),
       sizeof(SnapThreeDimensionalRefInfo)},
      {static_cast<uint64_t>(SnapMetadataType::VIEW_ID), sizeof(uint32_t)},
  };
};

class QtiMapper5Legacy final : public ::vendor::mapper::IMapperV5Impl {
 public:
  QtiMapper5Legacy();
  ~QtiMapper5Legacy() override = default;
  Error importBuffer(const native_handle_t *_Nonnull handle,
                     buffer_handle_t _Nullable *_Nonnull outBufferHandle) override;
  Error freeBuffer(buffer_handle_t _Nonnull buffer) override;
  Error getTransportSize(buffer_handle_t _Nonnull buffer, uint32_t *_Nonnull outNumFds,
                         uint32_t *_Nonnull outNumInts) override;
  Error lock(buffer_handle_t _Nonnull buffer, uint64_t cpuUsage, ARect accessRegion,
             int acquireFence, void *_Nullable *_Nonnull outData) override;
  Error unlock(buffer_handle_t _Nonnull buffer, int *_Nonnull releaseFence) override;
  Error flushLockedBuffer(buffer_handle_t _Nonnull buffer) override;
  Error rereadLockedBuffer(buffer_handle_t _Nonnull buffer) override;
  int32_t getMetadata(buffer_handle_t _Nonnull buffer, AIMapper_MetadataType metadataType,
                      void *_Nonnull outData, size_t outDataSize) override;
  int32_t getStandardMetadata(buffer_handle_t _Nonnull buffer, int64_t standardMetadataType,
                              void *_Nonnull outData, size_t outDataSize) override;
  Error setMetadata(buffer_handle_t _Nonnull buffer, AIMapper_MetadataType metadataType,
                    const void *_Nonnull metadata, size_t metadataSize) override;
  Error setStandardMetadata(buffer_handle_t _Nonnull buffer, int64_t standardMetadataType,
                            const void *_Nonnull metadata, size_t metadataSize) override;
  Error listSupportedMetadataTypes(
      const AIMapper_MetadataTypeDescription *_Nullable *_Nonnull outDescriptionList,
      size_t *_Nonnull outNumberOfDescriptions) override;
  Error dumpBuffer(buffer_handle_t _Nonnull bufferHandle,
                   AIMapper_DumpBufferCallback _Nonnull dumpBufferCallback,
                   void *_Null_unspecified context) override;
  Error dumpAllBuffers(AIMapper_BeginDumpBufferCallback _Nonnull beginDumpBufferCallback,
                       AIMapper_DumpBufferCallback _Nonnull dumpBufferCallback,
                       void *_Null_unspecified context) override;
  Error getReservedRegion(buffer_handle_t _Nonnull buffer,
                          void *_Nullable *_Nonnull outReservedRegion,
                          uint64_t *_Nonnull outReservedSize) override;

 private:
  void WaitFenceFd(int fence_fd);
  Error DumpBufferMetadata(buffer_handle_t _Nonnull buffer,
                           AIMapper_DumpBufferCallback _Nonnull dumpBufferCallback,
                           void *_Null_unspecified context);
  int32_t GetMetadataPrivate(buffer_handle_t _Nonnull bufferHandle, int64_t metadataType,
                             void *_Nonnull outData, size_t outDataSize, bool isStandard);
  Error SetMetadataPrivate(buffer_handle_t _Nonnull bufferHandle, int64_t metadataType,
                           const void *_Nonnull metadata, size_t metadataSize, bool isStandard);

  BufferManager *_Nullable buf_mgr_ = nullptr;

  std::unordered_map<uint64_t, size_t> type_to_size_{
      {static_cast<uint64_t>(SnapMetadataType::BUFFER_ID), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::NAME), sizeof(std::string)},
      {static_cast<uint64_t>(SnapMetadataType::WIDTH), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::HEIGHT), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::LAYER_COUNT), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::PIXEL_FORMAT_REQUESTED), sizeof(GrallocPixelFormat)},
      {static_cast<uint64_t>(SnapMetadataType::PIXEL_FORMAT_FOURCC), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::DRM_PIXEL_FORMAT_MODIFIER), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::USAGE), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::ALLOCATION_SIZE), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::PROTECTED_CONTENT), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::COMPRESSION), sizeof(GrallocExtendableType)},
      {static_cast<uint64_t>(SnapMetadataType::INTERLACED), sizeof(GrallocExtendableType)},
      {static_cast<uint64_t>(SnapMetadataType::CHROMA_SITING), sizeof(GrallocExtendableType)},
      {static_cast<uint64_t>(SnapMetadataType::PLANE_LAYOUTS), sizeof(SnapBufferLayout)},
      {static_cast<uint64_t>(SnapMetadataType::CROP), sizeof(Rect)},
      {static_cast<uint64_t>(SnapMetadataType::DATASPACE), sizeof(GrallocDataspace)},
      {static_cast<uint64_t>(SnapMetadataType::BLEND_MODE), sizeof(BlendMode)},
      {static_cast<uint64_t>(SnapMetadataType::VT_TIMESTAMP), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::PP_PARAM_INTERLACED), sizeof(int32_t)},
      {static_cast<uint64_t>(SnapMetadataType::VIDEO_PERF_MODE), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::GRAPHICS_METADATA),
       sizeof(((GraphicsMetadata *)(0))->data)},
      {static_cast<uint64_t>(SnapMetadataType::UBWC_CR_STATS_INFO),
       (sizeof(UBWCStats) * QTI_UBWC_STATS_ARRAY_SIZE)},
      {static_cast<uint64_t>(SnapMetadataType::REFRESH_RATE), sizeof(float)},
      {static_cast<uint64_t>(SnapMetadataType::MAP_SECURE_BUFFER), sizeof(int32_t)},
      {static_cast<uint64_t>(SnapMetadataType::LINEAR_FORMAT), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::SINGLE_BUFFER_MODE), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::CVP_METADATA), sizeof(CVPMetadata)},
      {static_cast<uint64_t>(SnapMetadataType::VIDEO_HISTOGRAM_STATS),
       sizeof(VideoHistogramMetadata)},
      {static_cast<uint64_t>(SnapMetadataType::FD), sizeof(int32_t)},
      {static_cast<uint64_t>(SnapMetadataType::ALIGNED_WIDTH_IN_PIXELS), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::STRIDE), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::ALIGNED_HEIGHT_IN_PIXELS), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::STANDARD_METADATA_STATUS),
       (sizeof(bool) * METADATA_SET_SIZE)},
      {static_cast<uint64_t>(SnapMetadataType::VENDOR_METADATA_STATUS),
       (sizeof(bool) * METADATA_SET_SIZE)},
      {static_cast<uint64_t>(SnapMetadataType::BUFFER_TYPE), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::VIDEO_TS_INFO), sizeof(VideoTimestampInfo)},
      {static_cast<uint64_t>(SnapMetadataType::CUSTOM_DIMENSIONS_STRIDE), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::CUSTOM_DIMENSIONS_HEIGHT), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::RGB_DATA_ADDRESS), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::BUFFER_PERMISSION),
       sizeof(BufferPermission) * BUFFERCLIENT_MAX},
      {static_cast<uint64_t>(SnapMetadataType::MEM_HANDLE), sizeof(int64_t)},
      {static_cast<uint64_t>(SnapMetadataType::TIMED_RENDERING), sizeof(uint32_t)},
      {static_cast<uint64_t>(SnapMetadataType::CUSTOM_CONTENT_METADATA),
       sizeof(CustomContentMetadata)},
      {static_cast<uint64_t>(SnapMetadataType::VIDEO_TRANSCODE_STATS),
       sizeof(VideoTranscodeStatsMetadata)},
      {static_cast<uint64_t>(SnapMetadataType::MASTERING_DISPLAY),
       sizeof(std::optional<GrallocSmpte2086>)},
      {static_cast<uint64_t>(StandardMetadataType::SMPTE2086),
       sizeof(std::optional<GrallocSmpte2086>)},
      {static_cast<uint64_t>(SnapMetadataType::CONTENT_LIGHT_LEVEL),
       sizeof(std::optional<GrallocCta861_3>)},
      {static_cast<uint64_t>(StandardMetadataType::CTA861_3),
       sizeof(std::optional<GrallocCta861_3>)},
      {static_cast<uint64_t>(SnapMetadataType::DYNAMIC_METADATA),
       sizeof(((SnapDynamicMetadata *)(0))->dynamicMetaDataPayload)},
      {static_cast<uint64_t>(StandardMetadataType::SMPTE2094_40),
       sizeof(((SnapDynamicMetadata *)(0))->dynamicMetaDataPayload)},
      {static_cast<uint64_t>(StandardMetadataType::SMPTE2094_10),
       sizeof(SnapCustomContentMetadata)},
      {static_cast<uint64_t>(SnapMetadataType::COLOR_REMAPPING_INFO), sizeof(ColorRemappingInfo)},
      {static_cast<uint64_t>(SnapMetadataType::MATRIX_COEFFICIENTS),
       sizeof(SnapMatrixCoEfficients)},
      {static_cast<uint64_t>(SnapMetadataType::HEAP_NAME), sizeof(std::string)},
      {static_cast<uint64_t>(SnapMetadataType::IS_UBWC), sizeof(int32_t)},
      {static_cast<uint64_t>(SnapMetadataType::IS_TILE_RENDERED), sizeof(int32_t)},
      {static_cast<uint64_t>(SnapMetadataType::IS_CACHED), sizeof(int32_t)},
      {static_cast<uint64_t>(QTI_COLOR_METADATA), sizeof(ColorMetaData)},
      {static_cast<uint64_t>(QTI_PRIVATE_FLAGS), sizeof(int32_t)},
      {static_cast<uint64_t>(QTI_COLORSPACE), sizeof(uint32_t)},
      {static_cast<uint64_t>(QTI_YUV_PLANE_INFO), (YCBCR_LAYOUT_ARRAY_SIZE * sizeof(qti_ycbcr))},
      {static_cast<uint64_t>(SnapMetadataType::EARLYNOTIFY_LINECOUNT), sizeof(int32_t)},
      {static_cast<uint64_t>(SnapMetadataType::BASE_ADDRESS), sizeof(uint64_t)},
      {static_cast<uint64_t>(SnapMetadataType::BUFFER_DEQUEUE_DURATION), sizeof(int64_t)},
  };
};

}  // namespace mapper5
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
}  // namespace stablec
