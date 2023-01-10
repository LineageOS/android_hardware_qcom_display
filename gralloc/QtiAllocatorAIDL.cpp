/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "QtiAllocatorAIDL.h"

#include <cutils/properties.h>
#include <log/log.h>
#include <aidlcommonsupport/NativeHandle.h>
#include <vendor/qti/hardware/display/mapper/4.0/IQtiMapper.h>

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
  props->use_system_heap_for_sensors = property_get_bool("vendor.gralloc.use_system_heap_for_sensors", 1);

  props->ubwc_disable = property_get_bool("vendor.gralloc.disable_ubwc", 0);

  props->ahardware_buffer_disable = property_get_bool("vendor.gralloc.disable_ahardware_buffer", 0);
}

static inline ndk::ScopedAStatus ToBinderStatus(Error error) {
  AllocationError ret;

  switch (error) {
    case Error::BAD_DESCRIPTOR:
      ret = AllocationError::BAD_DESCRIPTOR;
      break;
    case Error::NO_RESOURCES:
      ret = AllocationError::NO_RESOURCES;
      break;
    case Error::UNSUPPORTED:
      ret = AllocationError::UNSUPPORTED;
      break;
    default:
      return ndk::ScopedAStatus::ok();
  }

  return ndk::ScopedAStatus::fromServiceSpecificError(static_cast<int32_t>(ret));
}

QtiAllocatorAIDL::QtiAllocatorAIDL() {
  gralloc::GrallocProperties properties;
  GetProperties(&properties);
  buf_mgr_ = BufferManager::GetInstance();
  buf_mgr_->SetGrallocDebugProperties(properties);
  enable_logs_ = property_get_bool("vendor.gralloc.enable_logs", 0);
}

ndk::ScopedAStatus QtiAllocatorAIDL::allocate(const std::vector<uint8_t>& descriptor, int32_t count,
                                                 AllocationResult* result) {
  ALOGD_IF(enable_logs_, "Allocating buffers count: %d", count);
  gralloc::BufferDescriptor desc;
  ::android::hardware::hidl_vec<uint8_t> hidl_desc(descriptor);

  auto err = ::vendor::qti::hardware::display::mapper::V4_0::implementation::QtiMapper::Decode(
      hidl_desc, &desc);
  if (err != Error::NONE) {
    ALOGE("Failed to allocate. Can't decode buffer descriptor: %d", static_cast<int>(err));
    return ToBinderStatus(err);
  }

  const size_t n = static_cast<size_t>(count);
  std::vector<buffer_handle_t> buffers;
  buffers.reserve(n);

  for (size_t i = 0; i < n; i++) {
    buffer_handle_t buffer;
    err = buf_mgr_->AllocateBuffer(desc, &buffer);
    if (err != Error::NONE) {
      return ToBinderStatus(err);
    }
    buffers.emplace_back(buffer);
  }

  if (buffers.size() > 0) {
    result->stride = static_cast<int32_t>(PRIV_HANDLE_CONST(buffers[0])->width);
  }

  result->buffers.resize(n);
  for (size_t i = 0; i < n; i++) {
    auto buffer = buffers[i];
    result->buffers[i] = ::android::dupToAidl(buffer);
    buf_mgr_->ReleaseBuffer(PRIV_HANDLE_CONST(buffer));
  }

  return ndk::ScopedAStatus::ok();
}

}  // namespace impl
}  // namespace allocator
}  // namespace graphics
}  // namespace hardware
}  // namespace android
}  // namespace aidl
