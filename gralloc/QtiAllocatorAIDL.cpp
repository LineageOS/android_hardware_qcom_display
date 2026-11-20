/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Backported from the LineageOS lineage-24.0-caf-sm8650 display HAL
 * (gralloc/QtiAllocatorAIDL.cpp) to the sdm660 gralloc: same BufferManager,
 * same IMapper@4.0 descriptor decoding as the HIDL QtiAllocator 4.0.
 */

#define DEBUG 0
#include "QtiAllocatorAIDL.h"

#include <aidlcommonsupport/NativeHandle.h>
#include <cutils/properties.h>
#include <log/log.h>

#include <vector>

#include "QtiMapper4.h"
#include "gr_utils.h"

using gralloc::Error;

namespace aidl {
namespace android {
namespace hardware {
namespace graphics {
namespace allocator {
namespace impl {

static void GetProperties(gralloc::GrallocProperties *props) {
  props->use_system_heap_for_sensors =
      property_get_bool("vendor.gralloc.use_system_heap_for_sensors", 1);
  props->ubwc_disable = property_get_bool("vendor.gralloc.disable_ubwc", 0);
  props->ahardware_buffer_disable = property_get_bool("vendor.gralloc.disable_ahardware_buffer", 0);
}

static inline ndk::ScopedAStatus ToBinderStatus(Error error) {
  AllocationError ret;
  switch (error) {
    case Error::NONE:
      return ndk::ScopedAStatus::ok();
    case Error::BAD_DESCRIPTOR:
      ret = AllocationError::BAD_DESCRIPTOR;
      break;
    case Error::UNSUPPORTED:
      ret = AllocationError::UNSUPPORTED;
      break;
    case Error::NO_RESOURCES:
    default:
      // Never report success for an unknown failure.
      ret = AllocationError::NO_RESOURCES;
      break;
  }
  return ndk::ScopedAStatus::fromServiceSpecificError(static_cast<int32_t>(ret));
}

QtiAllocatorAIDL::QtiAllocatorAIDL() {
  gralloc::GrallocProperties properties;
  GetProperties(&properties);
  buf_mgr_ = BufferManager::GetInstance();
  buf_mgr_->SetGrallocDebugProperties(properties);
}

ndk::ScopedAStatus QtiAllocatorAIDL::allocate(const std::vector<uint8_t> &descriptor,
                                              int32_t count, AllocationResult *result) {
  ALOGD_IF(DEBUG, "AIDL allocate: count %d", count);
  if (count <= 0) {
    return ToBinderStatus(Error::BAD_DESCRIPTOR);
  }

  gralloc::BufferDescriptor desc;
  ::android::hardware::hidl_vec<uint8_t> hidl_desc(descriptor);
  auto err = ::vendor::qti::hardware::display::mapper::V4_0::implementation::QtiMapper::Decode(
      hidl_desc, &desc);
  if (err != Error::NONE) {
    ALOGE("AIDL allocate: cannot decode the buffer descriptor: %d", static_cast<int>(err));
    return ToBinderStatus(err);
  }

  const size_t n = static_cast<size_t>(count);  // count > 0 checked above
  std::vector<buffer_handle_t> buffers;
  buffers.reserve(n);
  for (size_t i = 0; i < n; i++) {
    buffer_handle_t buffer;
    err = buf_mgr_->AllocateBuffer(desc, &buffer);
    if (err != Error::NONE) {
      break;
    }
    buffers.emplace_back(buffer);
  }

  if (err != Error::NONE) {
    for (auto b : buffers) {
      buf_mgr_->ReleaseBuffer(PRIV_HANDLE_CONST(b));
    }
    return ToBinderStatus(err);
  }

  result->stride = static_cast<int32_t>(PRIV_HANDLE_CONST(buffers[0])->width);
  result->buffers.resize(n);
  for (size_t i = 0; i < n; i++) {
    // dupToAidl dups the fds; the binder parcel owns the copies.
    result->buffers[i] = ::android::dupToAidl(buffers[i]);
    buf_mgr_->ReleaseBuffer(PRIV_HANDLE_CONST(buffers[i]));
  }

  return ndk::ScopedAStatus::ok();
}

}  // namespace impl
}  // namespace allocator
}  // namespace graphics
}  // namespace hardware
}  // namespace android
}  // namespace aidl
