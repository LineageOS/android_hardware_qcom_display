/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Backported from the LineageOS lineage-24.0-caf-sm8650 display HAL
 * (gralloc/QtiAllocatorAIDL.h) to the sdm660 gralloc.
 */

#ifndef __QTIALLOCATORAIDL_H__
#define __QTIALLOCATORAIDL_H__

#include <aidl/android/hardware/graphics/allocator/AllocationError.h>
#include <aidl/android/hardware/graphics/allocator/AllocationResult.h>
#include <aidl/android/hardware/graphics/allocator/BnAllocator.h>

#include <vector>

#include "gr_buf_mgr.h"
#include "gr_utils.h"

namespace aidl {
namespace android {
namespace hardware {
namespace graphics {
namespace allocator {
namespace impl {

using gralloc::BufferManager;

// android.hardware.graphics.allocator-V1 (Android 12 "gralloc 4.1"): the
// descriptor is the IMapper@4.0 encoded BufferDescriptor, so this needs the
// HIDL mapper 4.0 (QtiMapper4) and nothing else from newer display HALs.
class QtiAllocatorAIDL : public BnAllocator {
 public:
  QtiAllocatorAIDL();

  ndk::ScopedAStatus allocate(const std::vector<uint8_t> &descriptor, int32_t count,
                              AllocationResult *result) override;

 private:
  BufferManager *buf_mgr_ = nullptr;
};

}  // namespace impl
}  // namespace allocator
}  // namespace graphics
}  // namespace hardware
}  // namespace android
}  // namespace aidl

#endif  // __QTIALLOCATORAIDL_H__
