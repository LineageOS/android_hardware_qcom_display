/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <cutils/properties.h>
#include <cutils/trace.h>

#include "QtiMapperExtensions2.h"

static bool enable_logs = true;

namespace stablec {
namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace mapperextensions2 {

extern "C" uint32_t QTI_MAPPER_EXTENSION_VERSION = IQTIMAPPEREXT_VERSION_2;
extern "C" Error IQtiMapperExt_loadIMapperExt(
    IQtiMapperExt *_Nullable *_Nonnull outImplementation) {
  ALOGD_IF(enable_logs, "Fetching IQtiMapperExt from QtiMapperExtensions2");
  if (gralloc::GrallocSnapHelper::GetInstance()->IsSnapAllocEnabled()) {
    static ::vendor::qtimapperext::IQtiMapperExtProvider<QtiMapperExtensions2> provider;
    return provider.load(outImplementation);
  }

  ALOGE("Enable SnapAlloc to use stable-c IQtiMapperExtV2");
  return IQTIMAPPEREXT_ERROR_UNSUPPORTED;
}

QtiMapperExtensions2::QtiMapperExtensions2() {
  enable_logs = property_get_bool(ENABLE_LOGS_PROP, 0);
  ALOGD_IF(enable_logs, "Created QtiMapperExtensions2 instance");
  snap_helper_ = gralloc::GrallocSnapHelper::GetInstance();
  if (snap_helper_) {
    snap_alloc_enable_ = snap_helper_->IsSnapAllocEnabled();
  }
}

Error QtiMapperExtensions2::getMultiViewInfo(buffer_handle_t _Nonnull buffer,
                                             uint32_t *_Nonnull views) {
  auto snap_error = snap_helper_->GetMetadata(const_cast<native_handle_t *>(buffer),
                                              SnapMetadataType::MULTI_VIEW_INFO, views, false,
                                              false, false, nullptr);
  return static_cast<Error>(snap_error);
}

Error QtiMapperExtensions2::getBaseView(buffer_handle_t _Nonnull buffer, uint32_t *_Nonnull view) {
  native_handle_t *gr_handle = const_cast<native_handle_t *>(buffer);
  auto snap_error = snap_helper_->GetBaseView(gr_handle, view);
  return static_cast<Error>(snap_error);
}

Error QtiMapperExtensions2::importViewBuffer(buffer_handle_t _Nonnull metaHandle, uint32_t view,
                                             buffer_handle_t _Nullable *_Nonnull outBufferHandle) {
  native_handle_t *meta_handle = const_cast<native_handle_t *>(metaHandle);
  auto snap_error = snap_helper_->ImportViewBuffer(meta_handle, view, outBufferHandle);
  return static_cast<Error>(snap_error);
}

}  // namespace mapperextensions2
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
}  // namespace stablec
