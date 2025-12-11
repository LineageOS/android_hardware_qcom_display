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
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define ATRACE_TAG (ATRACE_TAG_GRAPHICS | ATRACE_TAG_HAL)
#include "QtiMapper5.h"
#include "gr_snap_helper.h"

#include <cutils/properties.h>
#include <cutils/trace.h>
#include <sync/sync.h>

#include <vector>
#include <mutex>

using std::lock_guard;
using std::mutex;

static bool enable_logs = true;

namespace stablec {
namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace mapper5 {

using aidl::android::hardware::graphics::common::StandardMetadataType;
using gralloc::BufferInfo;

[[clang::no_destroy]] static std::mutex handles_heap_lock_;

QtiMapper5::QtiMapper5() {
  enable_logs = property_get_bool(ENABLE_LOGS_PROP, 0);
  ALOGD_IF(enable_logs, "Created QtiMapper5 instance");
  snap_helper_ = gralloc::GrallocSnapHelper::GetInstance();
  if (snap_helper_) {
    snap_alloc_enable_ = snap_helper_->IsSnapAllocEnabled();
  }
}

Error QtiMapper5::importBuffer(const native_handle_t *_Nonnull bufferHandle,
                               buffer_handle_t _Nullable *_Nonnull outBufferHandle) {
  REQUIRE_DRIVER()
  if (!bufferHandle || !outBufferHandle || bufferHandle->numFds == 0) {
    ALOGE("Failed to importBuffer. Bad handle.");
    return AIMAPPER_ERROR_BAD_BUFFER;
  }
  native_handle_t *importedBufferHandle = native_handle_clone(bufferHandle);
  if (!importedBufferHandle) {
    ALOGE("Failed to importBuffer. Handle clone failed: %s.", strerror(errno));
    return AIMAPPER_ERROR_NO_RESOURCES;
  }

  std::lock_guard<std::mutex> lock(handles_heap_lock_);
  int snap_ret = snap_helper_->Import(importedBufferHandle);
  if (snap_ret) {
    ALOGE("%s: Unable to retain handle: %p", __FUNCTION__, importedBufferHandle);
    native_handle_close(importedBufferHandle);
    native_handle_delete(importedBufferHandle);
    return AIMAPPER_ERROR_NO_RESOURCES;
  }

  ALOGD_IF(enable_logs, "Imported handle: %p id: %" PRIu64, importedBufferHandle,
           QTI_HANDLE_CONST(importedBufferHandle)->id);
  *outBufferHandle = importedBufferHandle;
  return AIMAPPER_ERROR_NONE;
}

Error QtiMapper5::freeBuffer(buffer_handle_t _Nonnull buffer) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(buffer)
  std::lock_guard<std::mutex> lock(handles_heap_lock_);
  int ret = snap_helper_->Free(const_cast<native_handle *>(buffer));
  if (ret) {
    ALOGW("%s: Unable to free buffer: %p", __FUNCTION__, buffer);
    return AIMAPPER_ERROR_BAD_BUFFER;
  }

  ALOGD_IF(enable_logs, "Freed handle: %p id: %" PRIu64, buffer, QTI_HANDLE_CONST(buffer)->id);
  return AIMAPPER_ERROR_NONE;
}

Error QtiMapper5::getTransportSize(buffer_handle_t _Nonnull bufferHandle,
                                   uint32_t *_Nonnull outNumFds, uint32_t *_Nonnull outNumInts) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(bufferHandle)
  // No local process data is currently stored on the native handle.
  *outNumFds = bufferHandle->numFds;
  *outNumInts = bufferHandle->numInts;
  ALOGD_IF(enable_logs, "getTransportSize: num fds: %d num ints: %d", *outNumFds, *outNumInts);
  return AIMAPPER_ERROR_NONE;
}

Error QtiMapper5::lock(buffer_handle_t _Nonnull bufferHandle, uint64_t cpuUsage, ARect region,
                       int acquireFenceRawFd, void *_Nullable *_Nonnull outData) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(bufferHandle)
  if (cpuUsage == 0) {
    ALOGE("Failed to lock. Bad cpu usage: %" PRIu64 ".", cpuUsage);
    return AIMAPPER_ERROR_BAD_VALUE;
  }

  uint64_t snap_base = 0;
  CropRectangle_t gr_access_region = {
      .left = region.left, .top = region.top, .right = region.right, .bottom = region.bottom};
  int ret_val = snap_helper_->Lock(const_cast<native_handle_t *>(bufferHandle), cpuUsage,
                                   gr_access_region, acquireFenceRawFd, &snap_base);

  if (ret_val != 0) {
    ALOGE("Snap failed to lock buffer");
  } else {
    ALOGD_IF(enable_logs, "QtiMapper5::lock address %lu", snap_base);
    *outData = reinterpret_cast<void *>(snap_base);
  }
  return static_cast<Error>(ret_val);
}

Error QtiMapper5::unlock(buffer_handle_t _Nonnull buffer, int *_Nonnull releaseFence) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(buffer)
  auto err = AIMAPPER_ERROR_BAD_BUFFER;
  int ret_val = snap_helper_->Unlock(const_cast<native_handle_t *>(buffer), releaseFence);
  if (ret_val != 0) {
    ALOGE("Snap failed to unlock buffer");
  } else {
    err = AIMAPPER_ERROR_NONE;
  }

  if (err) {
    ALOGE("Failed to unlock.");
  }
  return err;
}

Error QtiMapper5::flushLockedBuffer(buffer_handle_t _Nonnull buffer) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(buffer)
  auto err = AIMAPPER_ERROR_BAD_BUFFER;
  if (!snap_helper_->FlushLockedBuffer(const_cast<native_handle_t *>(buffer))) {
    err = AIMAPPER_ERROR_NONE;
  }

  if (err) {
    ALOGE("Failed to flushLockedBuffer. Flush failed.");
  }
  return err;
}

Error QtiMapper5::rereadLockedBuffer(buffer_handle_t _Nonnull buffer) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(buffer)
  auto err = AIMAPPER_ERROR_BAD_BUFFER;
  if (!snap_helper_->RereadLockedBuffer(const_cast<native_handle_t *>(buffer))) {
    err = AIMAPPER_ERROR_NONE;
  }

  if (err) {
    ALOGE("Failed to rereadLockedBuffer. Failed to invalidate.");
  }
  return err;
}

size_t QtiMapper5::GetExpectedSize(uint64_t metadata_type) {
  if (type_to_size_.find(metadata_type) != type_to_size_.end()) {
    return type_to_size_.at(metadata_type);
  }
  ALOGW("Can't find expected metadata size, invalid metadata type: %ld", metadata_type);
  return 0;
}

int32_t QtiMapper5::GetMetadataPrivate(buffer_handle_t _Nonnull bufferHandle, int64_t metadataType,
                                       void *_Nonnull outData, size_t outDataSize,
                                       bool isStandard) {
  if (!(bufferHandle)) {
    ALOGW("Failed to %s. Null buffer_handle_t.", __func__);
    return -AIMAPPER_ERROR_BAD_BUFFER;
  }

  if (!snap_helper_ || !snap_alloc_enable_) {
    return -AIMAPPER_ERROR_NO_RESOURCES;
  }
  int32_t size_required = outDataSize;
  auto snap_error =
      snap_helper_->GetMetadata(const_cast<native_handle_t *>(bufferHandle), metadataType, outData,
                                false, false, isStandard, &size_required);
  if (snap_error == SnapError::NONE) {
    return size_required;
  } else {
    return -(static_cast<int32_t>(snap_error));
  }
}

int32_t QtiMapper5::getMetadata(buffer_handle_t _Nonnull buffer, AIMapper_MetadataType metadataType,
                                void *_Nonnull outData, size_t outDataSize) {
  if (isStandardMetadata(metadataType)) {
    return getStandardMetadata(buffer, metadataType.value, outData, outDataSize);
  } else if (isVendorMetadata(metadataType)) {
    auto expected_size = GetExpectedSize(metadataType.value);
    if (expected_size == 0) {
      return -AIMAPPER_ERROR_UNSUPPORTED;
    }
    if (expected_size != outDataSize) {
      ALOGW(
          "Metadata output size %zu not equal to expected size %zu. Returning without fetching "
          "metadata: %ld",
          outDataSize, expected_size, metadataType.value);
      return expected_size;
    }
    ALOGD_IF(enable_logs, "%s: Buffer: %p" PRIu64 " MetadataType(vendor): %ld ExpectedSize: %ld",
             __FUNCTION__, buffer, metadataType.value, expected_size);
    return (GetMetadataPrivate(buffer, metadataType.value, outData, outDataSize, false));
  }
  return -AIMAPPER_ERROR_UNSUPPORTED;
}

int32_t QtiMapper5::getStandardMetadata(buffer_handle_t _Nonnull bufferHandle, int64_t standardType,
                                        void *_Nonnull outData, size_t outDataSize) {
  ALOGD_IF(enable_logs, "%s: Buffer: %p" PRIu64 " MetadataType(standard): %ld ExpectedSize: %ld",
           __FUNCTION__, bufferHandle, standardType, outDataSize);
  // For cases where client sends in nullptr intentionally to know bytestream size, set outData to
  // a valid vector but keep outDataSize to be 0 as a hint to gr_snap_helper so we end up returning
  // the size without copying
  std::vector<uint8_t> bytestream(1);
  if (outData == nullptr) {
    outData = bytestream.data();
  }
  return (GetMetadataPrivate(bufferHandle, standardType, outData, outDataSize, true));
}

Error QtiMapper5::SetMetadataPrivate(buffer_handle_t _Nonnull bufferHandle, int64_t metadataType,
                                     const void *_Nonnull metadata, size_t metadataSize,
                                     bool isStandard) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(bufferHandle)

  if (!isStandard) {
    metadataSize = 0;
  }
  return (static_cast<Error>(snap_helper_->SetMetadata(const_cast<native_handle_t *>(bufferHandle),
                                                       metadataType, const_cast<void *>(metadata),
                                                       metadataSize)));
}

Error QtiMapper5::setMetadata(buffer_handle_t _Nonnull buffer, AIMapper_MetadataType metadataType,
                              const void *_Nonnull metadata, size_t metadataSize) {
  // Divert to setStandardMetadata for standard metadata requests
  if (isStandardMetadata(metadataType)) {
    return setStandardMetadata(buffer, metadataType.value, metadata, metadataSize);
  } else if (metadataType.name == qtigralloc::VENDOR_QTI) {
    auto expected_size = GetExpectedSize(metadataType.value);
    if (expected_size == 0) {
      return AIMAPPER_ERROR_UNSUPPORTED;
    }
    if (expected_size != metadataSize) {
      ALOGW(
          "Metadata size %zu not equal to expected size %zu. Returning without setting "
          "metadata: %ld",
          metadataSize, expected_size, metadataType.value);
      return AIMAPPER_ERROR_BAD_VALUE;
    }
    ALOGD_IF(enable_logs, "%s: Buffer: %p" PRIu64 " MetadataType(vendor): %ld MetadataSize: %ld",
             __FUNCTION__, buffer, metadataType.value, metadataSize);
    return (SetMetadataPrivate(buffer, metadataType.value, metadata, metadataSize, false));
  }
  return AIMAPPER_ERROR_UNSUPPORTED;
}

Error QtiMapper5::setStandardMetadata(buffer_handle_t _Nonnull bufferHandle,
                                      int64_t standardTypeRaw, const void *_Nonnull metadata,
                                      size_t metadataSize) {
  ALOGD_IF(enable_logs, "%s: Buffer: %p" PRIu64 " MetadataType(standard): %ld MetadataSize: %ld",
           __FUNCTION__, bufferHandle, standardTypeRaw, metadataSize);
  metadataSize = (metadataSize == 0 && metadata == nullptr) ? 1 : metadataSize;
  return (SetMetadataPrivate(bufferHandle, standardTypeRaw, metadata, metadataSize, true));
}

constexpr AIMapper_MetadataTypeDescription describeStandard(StandardMetadataType type,
                                                            bool isGettable, bool isSettable) {
  return {
      {STANDARD_METADATA_NAME, static_cast<int64_t>(type)}, nullptr, isGettable, isSettable, {0}};
}

constexpr AIMapper_MetadataTypeDescription describeQTI(int64_t type, const char *desc,
                                                       bool isGettable, bool isSettable) {
  return {{VENDOR_QTI_METADATA_NAME, type}, desc, isGettable, isSettable, {0}};
}

Error QtiMapper5::listSupportedMetadataTypes(
    const AIMapper_MetadataTypeDescription *_Nullable *_Nonnull outDescriptionList,
    size_t *_Nonnull outNumberOfDescriptions) {
  static constexpr std::array<AIMapper_MetadataTypeDescription, 64> sSupportedMetadaTypes{
      describeStandard(StandardMetadataType::BUFFER_ID, true, false),
      describeStandard(StandardMetadataType::NAME, true, false),
      describeStandard(StandardMetadataType::WIDTH, true, false),
      describeStandard(StandardMetadataType::HEIGHT, true, false),
      describeStandard(StandardMetadataType::LAYER_COUNT, true, false),
      describeStandard(StandardMetadataType::PIXEL_FORMAT_REQUESTED, true, false),
      describeStandard(StandardMetadataType::PIXEL_FORMAT_FOURCC, true, false),
      describeStandard(StandardMetadataType::PIXEL_FORMAT_MODIFIER, true, false),
      describeStandard(StandardMetadataType::USAGE, true, false),
      describeStandard(StandardMetadataType::ALLOCATION_SIZE, true, false),
      describeStandard(StandardMetadataType::PROTECTED_CONTENT, true, false),
      describeStandard(StandardMetadataType::COMPRESSION, true, false),
      describeStandard(StandardMetadataType::INTERLACED, true, false),
      describeStandard(StandardMetadataType::CHROMA_SITING, true, false),
      describeStandard(StandardMetadataType::PLANE_LAYOUTS, true, false),
      describeStandard(StandardMetadataType::CROP, true, true),
      describeStandard(StandardMetadataType::DATASPACE, true, true),
      describeStandard(StandardMetadataType::COMPRESSION, true, false),
      describeStandard(StandardMetadataType::BLEND_MODE, true, true),
      describeStandard(StandardMetadataType::SMPTE2086, true, true),
      describeStandard(StandardMetadataType::CTA861_3, true, true),
      describeStandard(StandardMetadataType::SMPTE2094_40, true, true),
      describeStandard(StandardMetadataType::SMPTE2094_10, true, true),
      describeStandard(StandardMetadataType::STRIDE, true, false),
      describeQTI(SnapMetadataType::VT_TIMESTAMP, "VT Timestamp", true, true),
      describeQTI(SnapMetadataType::MATRIX_COEFFICIENTS, "Color metadata - Matrix coefficients",
                  true, true),
      describeQTI(SnapMetadataType::MASTERING_DISPLAY, "Color metadata - Mastering display", true,
                  true),
      describeQTI(SnapMetadataType::CONTENT_LIGHT_LEVEL, "Color metadata - Content light level",
                  true, true),
      describeQTI(SnapMetadataType::COLOR_REMAPPING_INFO, "Color metadata - Color remapping info",
                  true, true),
      describeQTI(SnapMetadataType::DYNAMIC_METADATA, "Color metadata - Dynamic metadata", true,
                  true),
      describeQTI(SnapMetadataType::PP_PARAM_INTERLACED, "Interlaced", true, true),
      describeQTI(SnapMetadataType::VIDEO_PERF_MODE, "Video perf mode", true, true),
      describeQTI(SnapMetadataType::GRAPHICS_METADATA, "Graphics metadata", true, true),
      describeQTI(SnapMetadataType::UBWC_CR_STATS_INFO, "UBWC stats", true, true),
      describeQTI(SnapMetadataType::REFRESH_RATE, "Refresh rate", true, true),
      describeQTI(SnapMetadataType::MAP_SECURE_BUFFER, "Secure buffer mappable", true, true),
      describeQTI(SnapMetadataType::LINEAR_FORMAT, "Linear format", true, true),
      describeQTI(SnapMetadataType::SINGLE_BUFFER_MODE, "Single buffer mode flag", true, true),
      describeQTI(SnapMetadataType::CVP_METADATA, "CVP metadata", true, true),
      describeQTI(SnapMetadataType::VIDEO_HISTOGRAM_STATS, "Video histogram stats", true, true),
      describeQTI(SnapMetadataType::VIDEO_TRANSCODE_STATS, "Video transcode stats", true, true),
      describeQTI(SnapMetadataType::FD, "FD in internal handle", true, false),
      describeQTI(SnapMetadataType::IS_UBWC, "UBWC flag", true, false),
      describeQTI(SnapMetadataType::IS_TILE_RENDERED, "Tile rendered flag", true, false),
      describeQTI(SnapMetadataType::IS_CACHED, "Cached flag", true, false),
      describeQTI(SnapMetadataType::ALIGNED_WIDTH_IN_PIXELS, "width in internal handle", true,
                  false),
      describeQTI(SnapMetadataType::ALIGNED_HEIGHT_IN_PIXELS, "height in internal handle", true,
                  false),
      describeQTI(SnapMetadataType::STANDARD_METADATA_STATUS, "Is standard metadata set", true,
                  false),
      describeQTI(SnapMetadataType::VENDOR_METADATA_STATUS, "Is vendor metadata set", true, false),
      describeQTI(SnapMetadataType::BUFFER_TYPE, "Buffer type from internal handle", true, false),
      describeQTI(SnapMetadataType::VIDEO_TS_INFO, "Video timestamp info", true, true),
      describeQTI(SnapMetadataType::CUSTOM_DIMENSIONS_STRIDE,
                  "Custom (factors in crop/interlaced height) width", true, false),
      describeQTI(SnapMetadataType::CUSTOM_DIMENSIONS_HEIGHT,
                  "Custom (factors in crop/interlaced height) height", true, false),
      describeQTI(SnapMetadataType::RGB_DATA_ADDRESS,
                  "RGB data address, factors in offset for UBWC buffer", true, false),
      describeQTI(SnapMetadataType::BUFFER_PERMISSION, "BufferPermission", true, true),
      describeQTI(SnapMetadataType::MEM_HANDLE, "MemHandle", true, false),
      describeQTI(SnapMetadataType::TIMED_RENDERING, "timed rendering", true, true),
      describeQTI(SnapMetadataType::CUSTOM_CONTENT_METADATA, "Custom content metadata", true, true),
      describeQTI(SnapMetadataType::EARLYNOTIFY_LINECOUNT,
                  "Early notify line count - used by video", true, true),
      describeQTI(SnapMetadataType::HEAP_NAME, "Heap name", true, false),
      describeQTI(SnapMetadataType::BASE_ADDRESS, "Buffer data base address", true, false),
      describeQTI(SnapMetadataType::PIXEL_FORMAT_ALLOCATED, "Pixel format post allocation", true,
                  false),
      describeQTI(SnapMetadataType::BUFFER_DEQUEUE_DURATION, "Last buffer dequeue duration", true,
                  true),
      describeQTI(SnapMetadataType::ANAMORPHIC_COMPRESSION_METADATA, "anamorphic compression data",
                  true, true),
  };
  *outDescriptionList = sSupportedMetadaTypes.data();
  *outNumberOfDescriptions = sSupportedMetadaTypes.size();
  return AIMAPPER_ERROR_NONE;
}

Error QtiMapper5::DumpBufferMetadata(buffer_handle_t _Nonnull buffer,
                                     AIMapper_DumpBufferCallback _Nonnull dumpBufferCallback,
                                     void *_Null_unspecified context) {
  const AIMapper_MetadataTypeDescription *descriptions = nullptr;
  size_t descriptionCount = 0;
  listSupportedMetadataTypes(&descriptions, &descriptionCount);
  std::vector<uint8_t> tempBuffer;
  size_t bufferSize;
  tempBuffer.resize(METADATA_BUFFERSIZE_INITIAL);

  for (int i = 0; i < descriptionCount; i++) {
    const auto it = descriptions[i];
    const auto type = it.metadataType;
    if (isVendorMetadata(type)) {
      bufferSize = tempBuffer.size();
      bufferSize = type_to_size_.find(static_cast<uint64_t>(type.value)) != type_to_size_.end()
                       ? type_to_size_.at(type.value)
                       : bufferSize;
      tempBuffer.resize(bufferSize);
    }
    int32_t size = getMetadata(buffer, type, tempBuffer.data(), tempBuffer.size());
    if (size < 0) {
      ALOGD_IF(enable_logs,
               "%s: Failed to retrieve Metadata: %" PRIu64 " error: %d Handle:%p" PRIu64,
               __FUNCTION__, type.value, size, buffer);
      // If buffer is deleted during metadata dump, return BAD_BUFFER
      if (size == -AIMAPPER_ERROR_BAD_BUFFER) {
        return AIMAPPER_ERROR_BAD_BUFFER;
      }
    } else if (size > tempBuffer.size()) {
      // The initial size should always be large enough, but just in case...
      tempBuffer.resize(size * 2);
      size = getMetadata(buffer, type, tempBuffer.data(), tempBuffer.size());
    }

    if (size >= 0 && size <= tempBuffer.size()) {
      dumpBufferCallback(context, it.metadataType, tempBuffer.data(), tempBuffer.size());
    } else {
      continue;
    }
  }
  return AIMAPPER_ERROR_NONE;
}

Error QtiMapper5::dumpBuffer(buffer_handle_t _Nonnull bufferHandle,
                             AIMapper_DumpBufferCallback _Nonnull dumpBufferCallback,
                             void *_Null_unspecified context) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(bufferHandle)
  std::lock_guard<std::mutex> lock(handles_heap_lock_);
  return DumpBufferMetadata(bufferHandle, dumpBufferCallback, context);
}

Error QtiMapper5::dumpAllBuffers(AIMapper_BeginDumpBufferCallback _Nonnull beginDumpBufferCallback,
                                 AIMapper_DumpBufferCallback _Nonnull dumpBufferCallback,
                                 void *_Null_unspecified context) {
  REQUIRE_DRIVER()
  std::vector<buffer_handle_t> handle_list{};
  std::lock_guard<std::mutex> lock(handles_heap_lock_);
  if (snap_helper_->GetAllHandles(&handle_list)) {
    return AIMAPPER_ERROR_UNSUPPORTED;
  }

  Error error = AIMAPPER_ERROR_NONE;
  for (auto handle : handle_list) {
    beginDumpBufferCallback(context);
    // Ignore other errors since some vendor metadata types like RGB address and custom metadata
    // aren't supported for all cases
    if (DumpBufferMetadata(handle, dumpBufferCallback, context) == AIMAPPER_ERROR_BAD_BUFFER) {
      error = AIMAPPER_ERROR_BAD_BUFFER;
    }
  }

  return error;
}

Error QtiMapper5::getReservedRegion(buffer_handle_t _Nonnull buffer,
                                    void *_Nullable *_Nonnull outReservedRegion,
                                    uint64_t *_Nonnull outReservedSize) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(buffer)
  Error error = AIMAPPER_ERROR_UNSUPPORTED;

  if (!snap_helper_->GetReservedRegion(const_cast<native_handle_t *>(buffer), outReservedRegion,
                                       outReservedSize)) {
    error = AIMAPPER_ERROR_NONE;
  } else {
    error = AIMAPPER_ERROR_UNSUPPORTED;
  }

  if (error != AIMAPPER_ERROR_NONE) {
    ALOGE("Failed to getReservedRegion. Failed to getReservedRegionArea.");
    return AIMAPPER_ERROR_BAD_BUFFER;
  }
  return AIMAPPER_ERROR_NONE;
}

extern "C" uint32_t ANDROID_HAL_MAPPER_VERSION = AIMAPPER_VERSION_5;
extern "C" Error AIMapper_loadIMapper(AIMapper *_Nullable *_Nonnull outImplementation) {
  ALOGD_IF(enable_logs, "Fetching IMapper5 from QtiMapper5");
  if (gralloc::GrallocSnapHelper::GetInstance()->IsSnapAllocEnabled()) {
    static ::vendor::mapper::IMapperProvider<QtiMapper5> provider;
    return provider.load(outImplementation);
  }

  // Return Gralloc4 back-end based implementation if SnapAlloc is unavailable
  ALOGE(
      "Gralloc4 is deprecated, please enable SnapAlloc and switch to the supported QtiMapper5 "
      "instance");
  static ::vendor::mapper::IMapperProvider<QtiMapper5Legacy> provider;
  return provider.load(outImplementation);
}

// LEGACY (GRALLOC4 BACK-END) IMPLEMENTATION
// ___________1¶¶¶¶¶¶¶¶¶¶1__________1¶¶¶¶¶¶¶¶1_______
// ________¶11____________¶11______¶1________¶¶1_____
// ______¶1___¶¶1_____1¶¶___1¶¶___¶____________¶¶____
// ____¶1____1¶1¶_____¶1¶¶_____1¶¶¶____111111111¶¶___
// __1¶______1¶¶¶_____¶¶¶1_____1¶¶____¶¶¶¶¶¶¶¶¶¶¶¶¶1_
// __¶_______1¶¶¶_____¶¶¶1____¶¶___________________¶¶
// _¶__________1_______1______¶¶___________________1¶
// ¶__________________________1¶1_________________1¶¶
// ¶___________________________1¶¶____¶¶¶¶¶¶¶¶¶¶¶¶¶__
// ¶__________111_____111_______1¶_______________1¶__
// ¶________¶¶¶¶¶¶¶¶¶¶¶¶¶¶1_____1¶________________1¶_
// ¶_______¶¶____1¶¶¶____1¶¶_____1¶1_____________1¶1_
// ¶______11______________¶¶_______¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶___
// _¶__________________________11__¶1¶_____¶1________
// _1¶_________________________1__¶1_¶_____¶¶________
// __¶¶_____________________111__¶¶__¶______¶¶_______
// ____¶¶________________111___¶¶____¶______¶¶_______
// _____1¶1________111111____1¶1_____¶¶¶¶¶¶¶_________
// _______¶¶¶11___________1¶¶¶_________1111__________
// __________111¶111111¶111__________________________
#undef REQUIRE_DRIVER
#define REQUIRE_DRIVER()                                       \
  if (!buf_mgr_) {                                             \
    ALOGE("Failed to %s. Driver is uninitialized.", __func__); \
    return AIMAPPER_ERROR_NO_RESOURCES;                        \
  }

QtiMapper5Legacy::QtiMapper5Legacy() {
  buf_mgr_ = BufferManager::GetInstance();
  enable_logs = property_get_bool(ENABLE_LOGS_PROP, 0);
  ALOGD_IF(enable_logs,
           "Created QtiMapper5Legacy (uses deprecated gralloc4 back-end) instance, please enable "
           "SnapAlloc to switch to the currently supported QtiMapper5 instance.");
}

Error QtiMapper5Legacy::importBuffer(const native_handle_t *_Nonnull bufferHandle,
                                     buffer_handle_t _Nullable *_Nonnull outBufferHandle) {
  REQUIRE_DRIVER()
  if (!bufferHandle || bufferHandle->numFds == 0) {
    ALOGE("Failed to importBuffer. Bad handle.");
    return AIMAPPER_ERROR_BAD_BUFFER;
  }
  native_handle_t *importedBufferHandle = native_handle_clone(bufferHandle);
  if (!importedBufferHandle) {
    ALOGE("Failed to importBuffer. Handle clone failed: %s.", strerror(errno));
    return AIMAPPER_ERROR_NO_RESOURCES;
  }

  int ret = static_cast<Error>(buf_mgr_->RetainBuffer(QTI_HANDLE_CONST(importedBufferHandle)));
  if (ret) {
    ALOGE("%s: Unable to retain handle: %p", __FUNCTION__, importedBufferHandle);
    native_handle_close(importedBufferHandle);
    native_handle_delete(importedBufferHandle);
    return AIMAPPER_ERROR_NO_RESOURCES;
  }

  ALOGD_IF(enable_logs, "Imported handle: %p id: %" PRIu64, importedBufferHandle,
           QTI_HANDLE_CONST(importedBufferHandle)->id);
  *outBufferHandle = importedBufferHandle;
  return AIMAPPER_ERROR_NONE;
}

Error QtiMapper5Legacy::freeBuffer(buffer_handle_t _Nonnull buffer) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(buffer)
  int ret;
  ret = static_cast<Error>(buf_mgr_->ReleaseBuffer(QTI_HANDLE_CONST(buffer)));
  if (ret) {
    ALOGW("%s: Unable to free buffer: %p", __FUNCTION__, buffer);
    return AIMAPPER_ERROR_BAD_BUFFER;
  }

  ALOGD_IF(enable_logs, "Freed handle: %p id: %" PRIu64, buffer, QTI_HANDLE_CONST(buffer)->id);
  return AIMAPPER_ERROR_NONE;
}

void QtiMapper5Legacy::WaitFenceFd(int fence_fd) {
  if (fence_fd < 0) {
    return;
  }

  const int timeout = 3000;
  ATRACE_BEGIN("fence wait");
  const int error = sync_wait(fence_fd, timeout);
  ATRACE_END();
  if (error < 0) {
    ALOGE("QtiMapper5Legacy: lock fence %d didn't signal in %u ms -  error: %s", fence_fd, timeout,
          strerror(errno));
  }
}

Error QtiMapper5Legacy::getTransportSize(buffer_handle_t _Nonnull bufferHandle,
                                         uint32_t *_Nonnull outNumFds,
                                         uint32_t *_Nonnull outNumInts) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(bufferHandle)
  // No local process data is currently stored on the native handle.
  *outNumFds = bufferHandle->numFds;
  *outNumInts = bufferHandle->numInts;
  ALOGD_IF(enable_logs, "getTransportSize: num fds: %d num ints: %d", *outNumFds, *outNumInts);
  return AIMAPPER_ERROR_NONE;
}

Error QtiMapper5Legacy::lock(buffer_handle_t _Nonnull bufferHandle, uint64_t cpuUsage, ARect region,
                             int acquireFenceRawFd, void *_Nullable *_Nonnull outData) {
  // We take ownership of the FD in all cases, even for errors
  if (acquireFenceRawFd > 0) {
    WaitFenceFd(acquireFenceRawFd);
  }
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(bufferHandle)
  if (cpuUsage == 0) {
    ALOGE("Failed to lock. Bad cpu usage: %" PRIu64 ".", cpuUsage);
    return AIMAPPER_ERROR_BAD_VALUE;
  }

  auto hnd = QTI_HANDLE_CONST(bufferHandle);

  if (region.top < 0 || region.left < 0 || region.right < 0 || region.bottom < 0 ||
      region.right > hnd->width || region.bottom > hnd->height) {
    return AIMAPPER_ERROR_BAD_VALUE;
  }

  auto err = static_cast<Error>(buf_mgr_->LockBuffer(hnd, cpuUsage));
  if (err != AIMAPPER_ERROR_NONE) {
    ALOGW("Failed to lock buffer");
    return err;
  }
  ALOGD_IF(enable_logs, "QtiMapper5Legacy::lock address %lu", hnd->base);
  *outData = reinterpret_cast<void *>(hnd->base);
  return AIMAPPER_ERROR_NONE;
}

Error QtiMapper5Legacy::unlock(buffer_handle_t _Nonnull buffer, int *_Nonnull releaseFence) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(buffer)
  auto err = AIMAPPER_ERROR_BAD_BUFFER;
  err = static_cast<Error>(buf_mgr_->UnlockBuffer(QTI_HANDLE_CONST(buffer)));
  *releaseFence = -1;

  if (err) {
    ALOGE("Failed to unlock.");
  }
  return err;
}

Error QtiMapper5Legacy::flushLockedBuffer(buffer_handle_t _Nonnull buffer) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(buffer)
  auto err = AIMAPPER_ERROR_BAD_BUFFER;
  err = static_cast<Error>(buf_mgr_->FlushBuffer(QTI_HANDLE_CONST(buffer)));

  if (err) {
    ALOGE("Failed to flushLockedBuffer. Flush failed.");
  }
  return err;
}

Error QtiMapper5Legacy::rereadLockedBuffer(buffer_handle_t _Nonnull buffer) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(buffer)
  auto err = AIMAPPER_ERROR_BAD_BUFFER;
  err = static_cast<Error>(buf_mgr_->RereadBuffer(QTI_HANDLE_CONST(buffer)));

  if (err) {
    ALOGE("Failed to rereadLockedBuffer. Failed to invalidate.");
  }
  return err;
}

int32_t QtiMapper5Legacy::GetMetadataPrivate(buffer_handle_t _Nonnull bufferHandle,
                                             int64_t metadataType, void *_Nonnull outData,
                                             size_t outDataSize, bool isStandard) {
  if (!(bufferHandle)) {
    ALOGW("Failed to %s. Null buffer_handle_t.", __func__);
    return -AIMAPPER_ERROR_BAD_BUFFER;
  }

  if (!buf_mgr_) {
    ALOGE("Failed to %s. Driver is uninitialized.", __func__);
    return -AIMAPPER_ERROR_NO_RESOURCES;
  }
  auto hnd = QTI_HANDLE_CONST(bufferHandle);
  return (buf_mgr_->GetMetadata(const_cast<private_handle_t *>(hnd), metadataType, outData,
                                outDataSize));
}

int32_t QtiMapper5Legacy::getMetadata(buffer_handle_t _Nonnull buffer,
                                      AIMapper_MetadataType metadataType, void *_Nonnull outData,
                                      size_t outDataSize) {
  if (isStandardMetadata(metadataType)) {
    return getStandardMetadata(buffer, metadataType.value, outData, outDataSize);
  } else if (isVendorMetadata(metadataType)) {
    auto expected_size =
        (type_to_size_.find(static_cast<uint64_t>(metadataType.value)) != type_to_size_.end())
            ? type_to_size_.at(metadataType.value)
            : outDataSize;
    ALOGD_IF(enable_logs, "%s: Buffer: %p" PRIu64 " MetadataType(vendor): %ld ExpectedSize: %ld",
             __FUNCTION__, buffer, metadataType.value, expected_size);
    return (GetMetadataPrivate(buffer, metadataType.value, outData, outDataSize, false));
  }
  return -AIMAPPER_ERROR_UNSUPPORTED;
}

int32_t QtiMapper5Legacy::getStandardMetadata(buffer_handle_t _Nonnull bufferHandle,
                                              int64_t standardType, void *_Nonnull outData,
                                              size_t outDataSize) {
  ALOGD_IF(enable_logs, "%s: Buffer: %p" PRIu64 " MetadataType(standard): %ld ExpectedSize: %ld",
           __FUNCTION__, bufferHandle, standardType, outDataSize);
  return (GetMetadataPrivate(bufferHandle, standardType, outData, outDataSize, true));
}

Error QtiMapper5Legacy::SetMetadataPrivate(buffer_handle_t _Nonnull bufferHandle,
                                           int64_t metadataType, const void *_Nonnull metadata,
                                           size_t metadataSize, bool isStandard) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(bufferHandle)

  auto hnd = QTI_HANDLE_CONST(bufferHandle);
  return static_cast<Error>(buf_mgr_->SetMetadata(const_cast<private_handle_t *>(hnd), metadataType,
                                                  metadata, metadataSize));
}

Error QtiMapper5Legacy::setMetadata(buffer_handle_t _Nonnull buffer,
                                    AIMapper_MetadataType metadataType,
                                    const void *_Nonnull metadata, size_t metadataSize) {
  // Divert to setStandardMetadata for standard metadata requests
  if (isStandardMetadata(metadataType)) {
    return setStandardMetadata(buffer, metadataType.value, metadata, metadataSize);
  } else if (metadataType.name == qtigralloc::VENDOR_QTI) {
    ALOGD_IF(enable_logs, "%s: Buffer: %p" PRIu64 " MetadataType(vendor): %ld MetadataSize: %ld",
             __FUNCTION__, buffer, metadataType.value, metadataSize);
    return (SetMetadataPrivate(buffer, metadataType.value, metadata, metadataSize, false));
  }
  return AIMAPPER_ERROR_UNSUPPORTED;
}

Error QtiMapper5Legacy::setStandardMetadata(buffer_handle_t _Nonnull bufferHandle,
                                            int64_t standardTypeRaw, const void *_Nonnull metadata,
                                            size_t metadataSize) {
  ALOGD_IF(enable_logs, "%s: Buffer: %p" PRIu64 " MetadataType(standard): %ld MetadataSize: %ld",
           __FUNCTION__, bufferHandle, standardTypeRaw, metadataSize);
  return (SetMetadataPrivate(bufferHandle, standardTypeRaw, metadata, metadataSize, true));
}

Error QtiMapper5Legacy::listSupportedMetadataTypes(
    const AIMapper_MetadataTypeDescription *_Nullable *_Nonnull outDescriptionList,
    size_t *_Nonnull outNumberOfDescriptions) {
  static constexpr std::array<AIMapper_MetadataTypeDescription, 63> sSupportedMetadaTypes{
      describeStandard(StandardMetadataType::BUFFER_ID, true, false),
      describeStandard(StandardMetadataType::NAME, true, false),
      describeStandard(StandardMetadataType::WIDTH, true, false),
      describeStandard(StandardMetadataType::HEIGHT, true, false),
      describeStandard(StandardMetadataType::LAYER_COUNT, true, false),
      describeStandard(StandardMetadataType::PIXEL_FORMAT_REQUESTED, true, false),
      describeStandard(StandardMetadataType::PIXEL_FORMAT_FOURCC, true, false),
      describeStandard(StandardMetadataType::PIXEL_FORMAT_MODIFIER, true, false),
      describeStandard(StandardMetadataType::USAGE, true, false),
      describeStandard(StandardMetadataType::ALLOCATION_SIZE, true, false),
      describeStandard(StandardMetadataType::PROTECTED_CONTENT, true, false),
      describeStandard(StandardMetadataType::COMPRESSION, true, false),
      describeStandard(StandardMetadataType::INTERLACED, true, false),
      describeStandard(StandardMetadataType::CHROMA_SITING, true, false),
      describeStandard(StandardMetadataType::PLANE_LAYOUTS, true, false),
      describeStandard(StandardMetadataType::CROP, true, true),
      describeStandard(StandardMetadataType::DATASPACE, true, true),
      describeStandard(StandardMetadataType::COMPRESSION, true, false),
      describeStandard(StandardMetadataType::BLEND_MODE, true, true),
      describeStandard(StandardMetadataType::SMPTE2086, true, true),
      describeStandard(StandardMetadataType::CTA861_3, true, true),
      describeStandard(StandardMetadataType::SMPTE2094_40, true, true),
      describeStandard(StandardMetadataType::SMPTE2094_10, true, true),
      describeStandard(StandardMetadataType::STRIDE, true, false),
      describeQTI(SnapMetadataType::VT_TIMESTAMP, "VT Timestamp", true, true),
      describeQTI(SnapMetadataType::MATRIX_COEFFICIENTS, "Color metadata - Matrix coefficients",
                  true, true),
      describeQTI(SnapMetadataType::MASTERING_DISPLAY, "Color metadata - Mastering display", true,
                  true),
      describeQTI(SnapMetadataType::CONTENT_LIGHT_LEVEL, "Color metadata - Content light level",
                  true, true),
      describeQTI(SnapMetadataType::COLOR_REMAPPING_INFO, "Color metadata - Color remapping info",
                  true, true),
      describeQTI(SnapMetadataType::DYNAMIC_METADATA, "Color metadata - Dynamic metadata", true,
                  true),
      describeQTI(SnapMetadataType::PP_PARAM_INTERLACED, "Interlaced", true, true),
      describeQTI(SnapMetadataType::VIDEO_PERF_MODE, "Video perf mode", true, true),
      describeQTI(SnapMetadataType::GRAPHICS_METADATA, "Graphics metadata", true, true),
      describeQTI(SnapMetadataType::UBWC_CR_STATS_INFO, "UBWC stats", true, true),
      describeQTI(SnapMetadataType::REFRESH_RATE, "Refresh rate", true, true),
      describeQTI(SnapMetadataType::MAP_SECURE_BUFFER, "Secure buffer mappable", true, true),
      describeQTI(SnapMetadataType::LINEAR_FORMAT, "Linear format", true, true),
      describeQTI(SnapMetadataType::SINGLE_BUFFER_MODE, "Single buffer mode flag", true, true),
      describeQTI(SnapMetadataType::CVP_METADATA, "CVP metadata", true, true),
      describeQTI(SnapMetadataType::VIDEO_HISTOGRAM_STATS, "Video histogram stats", true, true),
      describeQTI(SnapMetadataType::VIDEO_TRANSCODE_STATS, "Video transcode stats", true, true),
      describeQTI(SnapMetadataType::FD, "FD in internal handle", true, false),
      describeQTI(SnapMetadataType::IS_UBWC, "UBWC flag", true, false),
      describeQTI(SnapMetadataType::IS_TILE_RENDERED, "Tile rendered flag", true, false),
      describeQTI(SnapMetadataType::IS_CACHED, "Cached flag", true, false),
      describeQTI(SnapMetadataType::ALIGNED_WIDTH_IN_PIXELS, "width in internal handle", true,
                  false),
      describeQTI(SnapMetadataType::ALIGNED_HEIGHT_IN_PIXELS, "height in internal handle", true,
                  false),
      describeQTI(SnapMetadataType::STANDARD_METADATA_STATUS, "Is standard metadata set", true,
                  false),
      describeQTI(SnapMetadataType::VENDOR_METADATA_STATUS, "Is vendor metadata set", true, false),
      describeQTI(SnapMetadataType::BUFFER_TYPE, "Buffer type from internal handle", true, false),
      describeQTI(SnapMetadataType::VIDEO_TS_INFO, "Video timestamp info", true, true),
      describeQTI(SnapMetadataType::CUSTOM_DIMENSIONS_STRIDE,
                  "Custom (factors in crop/interlaced height) width", true, false),
      describeQTI(SnapMetadataType::CUSTOM_DIMENSIONS_HEIGHT,
                  "Custom (factors in crop/interlaced height) height", true, false),
      describeQTI(SnapMetadataType::RGB_DATA_ADDRESS,
                  "RGB data address, factors in offset for UBWC buffer", true, false),
      describeQTI(SnapMetadataType::BUFFER_PERMISSION, "BufferPermission", true, true),
      describeQTI(SnapMetadataType::MEM_HANDLE, "MemHandle", true, false),
      describeQTI(SnapMetadataType::TIMED_RENDERING, "timed rendering", true, true),
      describeQTI(SnapMetadataType::CUSTOM_CONTENT_METADATA, "Custom content metadata", true, true),
      describeQTI(SnapMetadataType::EARLYNOTIFY_LINECOUNT,
                  "Early notify line count - used by video", true, true),
      describeQTI(SnapMetadataType::HEAP_NAME, "Heap name", true, false),
      describeQTI(SnapMetadataType::BASE_ADDRESS, "Buffer data base address", true, false),
      describeQTI(SnapMetadataType::PIXEL_FORMAT_ALLOCATED, "Format Post allocation", true, false),
      describeQTI(SnapMetadataType::BUFFER_DEQUEUE_DURATION, "Last buffer dequeue duration", true,
                  true),
  };
  *outDescriptionList = sSupportedMetadaTypes.data();
  *outNumberOfDescriptions = sSupportedMetadaTypes.size();
  return AIMAPPER_ERROR_NONE;
}

Error QtiMapper5Legacy::DumpBufferMetadata(buffer_handle_t _Nonnull buffer,
                                           AIMapper_DumpBufferCallback _Nonnull dumpBufferCallback,
                                           void *_Null_unspecified context) {
  const AIMapper_MetadataTypeDescription *descriptions = nullptr;
  size_t descriptionCount = 0;
  listSupportedMetadataTypes(&descriptions, &descriptionCount);
  std::vector<uint8_t> tempBuffer;
  size_t bufferSize;
  tempBuffer.resize(METADATA_BUFFERSIZE_INITIAL);

  for (int i = 0; i < descriptionCount; i++) {
    const auto it = descriptions[i];
    const auto type = it.metadataType;
    int32_t size = getMetadata(buffer, type, tempBuffer.data(), tempBuffer.size());
    if (size < 0) {
      ALOGD_IF(enable_logs,
               "%s: Failed to retrieve Metadata: %" PRIu64 " error: %d Handle:%p" PRIu64,
               __FUNCTION__, type.value, size, buffer);
      // If buffer is deleted during metadata dump, return BAD_BUFFER
      if (size == -AIMAPPER_ERROR_BAD_BUFFER) {
        return AIMAPPER_ERROR_BAD_BUFFER;
      }
    } else if (size > tempBuffer.size()) {
      // The initial size should always be large enough, but just in case...
      tempBuffer.resize(size * 2);
      size = getMetadata(buffer, type, tempBuffer.data(), tempBuffer.size());
    }

    if (size >= 0 && size <= tempBuffer.size()) {
      bufferSize = tempBuffer.size();
      bufferSize =
          (isVendorMetadata(it.metadataType) &&
           type_to_size_.find(static_cast<uint64_t>(it.metadataType.value)) != type_to_size_.end())
              ? type_to_size_.at(it.metadataType.value)
              : bufferSize;
      dumpBufferCallback(context, it.metadataType, tempBuffer.data(), bufferSize);
    } else {
      ALOGW("%s: Failed to retrieve Metadata on 2nd attempt: %" PRIu64 " error: %d Handle:%p" PRIu64,
            __FUNCTION__, type.value, size, buffer);
      continue;
    }
  }
  return AIMAPPER_ERROR_NONE;
}

Error QtiMapper5Legacy::dumpBuffer(buffer_handle_t _Nonnull bufferHandle,
                                   AIMapper_DumpBufferCallback _Nonnull dumpBufferCallback,
                                   void *_Null_unspecified context) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(bufferHandle)
  return DumpBufferMetadata(bufferHandle, dumpBufferCallback, context);
}

Error QtiMapper5Legacy::dumpAllBuffers(
    AIMapper_BeginDumpBufferCallback _Nonnull beginDumpBufferCallback,
    AIMapper_DumpBufferCallback _Nonnull dumpBufferCallback, void *_Null_unspecified context) {
  REQUIRE_DRIVER()
  std::vector<buffer_handle_t> handle_list;
  std::vector<const private_handle_t *> private_handle_list;
  if (static_cast<Error>(buf_mgr_->GetAllHandles(&private_handle_list))) {
    return AIMAPPER_ERROR_NO_RESOURCES;
  }
  for (auto handle : private_handle_list) {
    handle_list.push_back(static_cast<buffer_handle_t>(handle));
  }

  for (auto handle : handle_list) {
    beginDumpBufferCallback(context);
    DumpBufferMetadata(handle, dumpBufferCallback, context);
  }

  return AIMAPPER_ERROR_NONE;
}

Error QtiMapper5Legacy::getReservedRegion(buffer_handle_t _Nonnull buffer,
                                          void *_Nullable *_Nonnull outReservedRegion,
                                          uint64_t *_Nonnull outReservedSize) {
  VALIDATE_DRIVER_AND_BUFFER_HANDLE(buffer)
  Error error = AIMAPPER_ERROR_UNSUPPORTED;

  auto hnd = QTI_HANDLE_CONST(buffer);

  if (static_cast<Error>(buf_mgr_->IsBufferImported(hnd)) != AIMAPPER_ERROR_NONE) {
    ALOGE("Failed to getReservedRegion. Buffer not imported.");
    return AIMAPPER_ERROR_BAD_BUFFER;
  }

  error = static_cast<Error>(buf_mgr_->GetReservedRegion(const_cast<private_handle_t *>(hnd),
                                                         outReservedRegion, outReservedSize));

  if (error != AIMAPPER_ERROR_NONE) {
    ALOGE("Failed to getReservedRegion. Failed to getReservedRegionArea.");
    return AIMAPPER_ERROR_BAD_BUFFER;
  }
  return AIMAPPER_ERROR_NONE;
}

}  // namespace mapper5
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
}  // namespace stablec
