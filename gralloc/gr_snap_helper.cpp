/*
 * Copyright (c) 2023-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include <QtiGralloc.h>
#include "BufferUsage.h"
#include "QtiGrallocDefs.h"
#include "QtiGrallocMetadata.h"
#include "gr_snap_helper.h"

#include <dlfcn.h>
#include <log/log.h>
#include <cstdint>
#include <mutex>
#include <cutils/properties.h>
#include <utils/debug.h>
#include "gr_utils.h"
#include "android/binder_auto_utils.h"
#include "gralloctypes/Gralloc4.h"
#include <aidl/android/hardware/graphics/allocator/AllocationResult.h>
#include <android/hardware/graphics/mapper/utils/IMapperMetadataTypes.h>

using SnapFence = vendor_qti_hardware_display_common_Fence;
using SnapAddress = vendor_qti_hardware_display_common_Address;

using android::hardware::graphics::mapper::StandardMetadata;

using std::lock_guard;
using std::mutex;

namespace gralloc {

[[clang::no_destroy]] static std::unordered_map<native_handle_t *, SnapHandle *> handles_map_;
[[clang::no_destroy]] static std::mutex map_lock_;
GrallocSnapHelper *GrallocSnapHelper::s_instance = nullptr;

static NativeHandle AIDLNativeHandleFromSnapHandle(SnapHandle *snap_buffer_handle,
                                                   bool pass_fd_ownership) {
  NativeHandle aidl_native_handle;

  aidl_native_handle.fds = std::vector<ndk::ScopedFileDescriptor>(snap_buffer_handle->num_fds);
  for (size_t i = 0; i < snap_buffer_handle->num_fds; i++) {
    int fd = snap_buffer_handle->buffer_data[i];
    aidl_native_handle.fds.at(i).set(pass_fd_ownership ? fd : fcntl(fd, F_DUPFD_CLOEXEC, 0));
  }

  aidl_native_handle.ints = std::vector<int32_t>(
      snap_buffer_handle->buffer_data + snap_buffer_handle->num_fds,
      snap_buffer_handle->buffer_data + snap_buffer_handle->num_fds + snap_buffer_handle->num_ints);

  return aidl_native_handle;
}

static SnapHandle *SnapHandleFromCNativeHandle(native_handle_t *native_handle,
                                               bool pass_fd_ownership) {
  if (!native_handle)
    return nullptr;

  SnapHandle *snap_handle = vendor::qti::hardware::display::snapalloc::snap_handle_create(
      native_handle->numFds, native_handle->numInts);

  if (!snap_handle)
    return nullptr;

  for (size_t i = 0; i < native_handle->numFds; i++) {
    int fd = native_handle->data[i];
    snap_handle->buffer_data[i] = pass_fd_ownership ? fd : fcntl(fd, F_DUPFD_CLOEXEC, 0);
  }

  memcpy((snap_handle->buffer_data + snap_handle->num_fds),
         (native_handle->data + native_handle->numFds), native_handle->numInts * sizeof(int));

  return snap_handle;
}

static native_handle_t *CNativeHandleFromSnapHandle(SnapHandle *snap_handle,
                                                    bool pass_fd_ownership) {
  if (!snap_handle)
    return nullptr;

  native_handle_t *native_handle =
      native_handle_create(snap_handle->num_fds, snap_handle->num_ints);

  if (!native_handle)
    return nullptr;

  for (size_t i = 0; i < snap_handle->num_fds; i++) {
    int fd = snap_handle->buffer_data[i];
    native_handle->data[i] = pass_fd_ownership ? fd : fcntl(fd, F_DUPFD_CLOEXEC, 0);
  }

  memcpy((native_handle->data + native_handle->numFds),
         (snap_handle->buffer_data + snap_handle->num_fds), snap_handle->num_ints * sizeof(int));

  return native_handle;
}

static void *SnapAddressToPointer(SnapAddress &addr) {
  return reinterpret_cast<void *>(addr.addressPointer);
}

GrallocSnapHelper *GrallocSnapHelper::GetInstance() {
  static mutex s_lock;
  lock_guard<mutex> obj(s_lock);
  if (!s_instance) {
    s_instance = new GrallocSnapHelper();
  }

  return s_instance;
}

GrallocSnapHelper::GrallocSnapHelper() {
  snap_alloc_enable_ = false;

  char property[PROPERTY_VALUE_MAX];
  property_get(ENABLE_SNAPALLOC_PROP, property, "0");
  if (!(strncmp(property, "1", PROPERTY_VALUE_MAX)) ||
      !(strncmp(property, "true", PROPERTY_VALUE_MAX))) {
    snap_alloc_enable_ = true;
  }

  enable_logs_ = property_get_bool(ENABLE_LOGS_PROP, 0);

  if (!snap_alloc_enable_) {
    ALOGD("SnapAlloc is disabled");
    return;
  }

  const std::string snapalloc_lib_name = "vendor.qti.hardware.display.snapalloc-impl.so";
  snap_impl_lib_ = ::dlopen(snapalloc_lib_name.c_str(), RTLD_NOW);
  if (!snap_impl_lib_) {
    ALOGE("Dlopen error for snapalloc impl: %s", dlerror());
    snap_alloc_enable_ = false;
    return;
  }

  *reinterpret_cast<void **>(&LINK_FETCH_ISnapAlloc) = ::dlsym(snap_impl_lib_, "FETCH_ISnapAlloc");
  if (LINK_FETCH_ISnapAlloc) {
    snapallocator_ = LINK_FETCH_ISnapAlloc(&debugger_impl_);
  }

  if (!LINK_FETCH_ISnapAlloc || snapallocator_ == nullptr) {
    ALOGE("%s: Failed to link FETCH_ISnapAlloc - %s", __FUNCTION__, strerror(errno));
    snap_alloc_enable_ = false;
    return;
  }

  *reinterpret_cast<void **>(&LINK_FETCH_ISnapMapper) =
      ::dlsym(snap_impl_lib_, "FETCH_ISnapMapper");
  if (LINK_FETCH_ISnapMapper) {
    snapmapper_ = LINK_FETCH_ISnapMapper(&debugger_impl_);
  }

  if (!LINK_FETCH_ISnapMapper || snapmapper_ == nullptr) {
    ALOGE("%s: Failed to link FETCH_ISnapAlloc - %s", __FUNCTION__, strerror(errno));
    snap_alloc_enable_ = false;
    return;
  }

  // Inverse of the hard coded maps to translate in the other direction
  for (auto entry : snap_to_gralloc_format_) {
    gralloc_to_snap_format_.emplace(std::make_pair(entry.second, entry.first));
  }
  for (auto entry : snap_to_gralloc_ubwc_format_) {
    gralloc_ubwc_to_snap_format_.emplace(std::make_pair(entry.second, entry.first));
  }
  for (auto entry : gralloc_to_snap_usage_) {
    snap_to_gralloc_usage_.emplace(std::make_pair(entry.second, entry.first));
  }
  for (auto entry : cpu_gralloc_to_snap_usage_) {
    cpu_snap_to_gralloc_usage_.emplace(std::make_pair(entry.second, entry.first));
  }
  for (auto entry : gralloc_to_snap_ubwc_version_) {
    snap_to_gralloc_ubwc_version_.emplace(std::make_pair(entry.second, entry.first));
  }
}

GrallocSnapHelper::~GrallocSnapHelper() {
  std::lock_guard<std::mutex> lock(map_lock_);

  for (auto &entry : handles_map_) {
    native_handle_delete(entry.first);
  }
  handles_map_.clear();

  if (snap_impl_lib_)
    dlclose(snap_impl_lib_);
}

int GrallocSnapHelper::Allocate(
    gralloc::BufferDescriptor gr_desc, int buffer_count,
    aidl::android::hardware::graphics::allocator::AllocationResult *result) {
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  if (result == nullptr) {
    ALOGE("%s: Invalid AllocationResult pointer passed in", __FUNCTION__);
    return SnapError::BAD_VALUE;
  }

  SnapDescriptor snap_desc = {};
  auto err = GetSnapDescriptor(gr_desc, snap_desc);
  if (err) {
    ALOGE("%s: Failed to get Snap Descriptor", __FUNCTION__);
    return err;
  }
  SnapAllocationResult snap_result;

  auto status = snapallocator_->Allocate(snap_desc, buffer_count, &snap_result);

  if (status != SnapError::NONE) {
    ALOGE("%s: Failed to allocate via SnapAlloc. Error code: %d", __FUNCTION__, status);
    return status;
  }

  result->stride = snap_result.stride;
  result->buffers.resize(snap_result.handles.size());
  for (int i = 0; i < snap_result.handles.size(); i++) {
    result->buffers[i] = AIDLNativeHandleFromSnapHandle(snap_result.handles[i], false);
    snapmapper_->Release(*snap_result.handles[i]);
  }

  return status;
}

int GrallocSnapHelper::Import(native_handle_t *gr_hnd) {
  if (gr_hnd == nullptr) {
    ALOGE("Invalid gralloc handle");
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }
  std::lock_guard<std::mutex> lock(map_lock_);

  SnapError status = SnapError::BAD_BUFFER;
  if (handles_map_.find(gr_hnd) == handles_map_.end()) {
    SnapHandle *handle = SnapHandleFromCNativeHandle(const_cast<native_handle_t *>(gr_hnd), false);
    if (handle != nullptr) {
      auto status = snapmapper_->Retain(*handle);
      if (status == SnapError::NONE) {
        // Maintain map so that native_handle_t doesn't need to be duped during calls after import
        handles_map_.emplace(std::make_pair(gr_hnd, handle));
        ALOGD_IF(enable_logs_,
                 "gr_snap_helper Import - handles_map_.size() %d after emplace into map",
                 handles_map_.size());
        return SnapError::NONE;
      } else {
        ALOGE("%s: Failed to import via SnapAlloc. Error code: %d", __FUNCTION__, status);
        return status;
      }
    } else {
      ALOGE("Failed to create snap handle from native_handle_t %p", gr_hnd);
      return status;
    }
  }

  // Handle already in map
  return SnapError::NONE;
}

int GrallocSnapHelper::ImportViewBuffer(native_handle_t *meta_handle, uint32_t view,
                                        buffer_handle_t *out_buffer_handle) {
  if (meta_handle == nullptr) {
    ALOGE("Invalid gralloc handle");
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }
  std::lock_guard<std::mutex> lock(map_lock_);

  SnapError status = SnapError::BAD_BUFFER;
  if (handles_map_.find(meta_handle) == handles_map_.end()) {
    ALOGE("Meta Handle should be imported before importing auxillary view buffer");
    return SnapError::UNSUPPORTED;
  } else {
    SnapHandle *snap_meta_handle = handles_map_.at(meta_handle);
    SnapHandle *view_handle = nullptr;
    auto status = snapmapper_->RetainViewBuffer(*snap_meta_handle, view, &view_handle);

    if (status == SnapError::NONE) {
      native_handle_t *native_handle = CNativeHandleFromSnapHandle(view_handle, false);
      handles_map_.emplace(std::make_pair(native_handle, view_handle));
      ALOGD_IF(enable_logs_,
               "gr_snap_helper ImportViewBuffer - handles_map_.size() %d"
               "after emplace into map",
               handles_map_.size());
      *out_buffer_handle = native_handle;
      return SnapError::NONE;
    } else {
      ALOGE("%s: Failed to import via SnapAlloc. Error code: %d", __FUNCTION__, status);
      return status;
    }
  }
}

int GrallocSnapHelper::Free(native_handle_t *gr_hnd) {
  if (gr_hnd == nullptr) {
    ALOGE("Invalid gralloc handle");
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }
  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    auto status = snapmapper_->Release(*hnd);
    if (status == SnapError::NONE || status == SnapError::BUF_NOT_FREED) {
      // Only free fds if SnapHandle has been freed
      if (status == SnapError::NONE) {
        handles_map_.erase(gr_hnd);
        native_handle_close(gr_hnd);
        native_handle_delete(gr_hnd);
      }
      return SnapError::NONE;
    } else {
      ALOGE("%s: Failed to free via SnapAlloc. Error code: %d", __FUNCTION__, status);
    }
  }
  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelper::Lock(native_handle_t *gr_hnd, uint64_t gr_usage,
                            CropRectangle_t gr_access_region, int fence_fd, uint64_t *base_addr) {
  if (gr_hnd == nullptr) {
    ALOGE("Invalid gralloc handle");
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    // TODO: Get pixel format requested here to determine if it's explicit UBWC format
    SnapUsage usage = GetSnapUsage(gr_usage, 0);
    SnapRect access_region = {.left = gr_access_region.left,
                              .top = gr_access_region.top,
                              .right = gr_access_region.right,
                              .bottom = gr_access_region.bottom};

    SnapFence acquire_fence;
    acquire_fence.fence_fd = fence_fd;
    SnapAddress ret_addr;

    auto status = snapmapper_->Lock(*hnd, static_cast<SnapUsage>(usage), access_region,
                                    acquire_fence, &ret_addr);

    if (status == SnapError::NONE) {
      *base_addr = ret_addr.addressPointer;
      return SnapError::NONE;
    } else {
      ALOGE("%s: Failed to lock via SnapAlloc. Error code: %d", __FUNCTION__, status);
      return status;
    }
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }

  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelper::Unlock(native_handle_t *gr_hnd, void *in_fence) {
  if (gr_hnd == nullptr) {
    ALOGE("Invalid gralloc handle");
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    SnapFence release_fence;
    auto status = snapmapper_->Unlock(*hnd, &release_fence);
    if (status == SnapError::NONE) {
      in_fence = nullptr;
    } else {
      ALOGE("%s: Failed to unlock via SnapAlloc. Error code: %d", __FUNCTION__, status);
    }
    return status;
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }

  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelper::ValidateBufferSize(native_handle_t *gr_hnd, gralloc::BufferInfo gr_desc) {
  if (gr_hnd == nullptr) {
    ALOGE("Invalid gralloc handle");
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    SnapDescriptor snap_desc = {};
    auto err = GetSnapDescriptor(gr_desc, snap_desc);
    if (err) {
      return err;
    }
    auto status = snapmapper_->ValidateBufferSize(*hnd, snap_desc);
    if (status != SnapError::NONE) {
      ALOGE("%s: Failed to validate buffer size via SnapAlloc. Error code: %d", __FUNCTION__,
            status);
    }
    return status;
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }

  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelper::FlushLockedBuffer(native_handle_t *gr_hnd) {
  if (gr_hnd == nullptr) {
    ALOGE("Invalid gralloc handle");
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    auto status = snapmapper_->FlushLockedBuffer(*hnd);
    if (status != SnapError::NONE) {
      ALOGE("%s: Failed to flush locked buffer via SnapAlloc. Error code: %d", __FUNCTION__,
            status);
    }
    return status;
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }

  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelper::RereadLockedBuffer(native_handle_t *gr_hnd) {
  if (gr_hnd == nullptr) {
    ALOGE("Invalid gralloc handle");
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    auto status = snapmapper_->RereadLockedBuffer(*hnd);
    if (status != SnapError::NONE) {
      ALOGE("%s: Failed to reread locked buffer via SnapAlloc. Error code: %d", __FUNCTION__,
            status);
    }
    return status;
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }

  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelper::GetReservedRegion(native_handle_t *gr_hnd, void **reserved_region,
                                         uint64_t *reserved_region_size) {
  if (gr_hnd == nullptr) {
    ALOGE("Invalid gralloc handle");
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    SnapReservedRegion snap_reserved_region;
    auto status =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::RESERVED_REGION, &snap_reserved_region);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGE("Unable to get reserved region from snap");
      return status;
    }
    *reserved_region =
        reinterpret_cast<void *>(snap_reserved_region.reserved_region_addr.addressPointer);
    *reserved_region_size = static_cast<uint64_t>(snap_reserved_region.size);
    return SnapError::NONE;
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }

  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelper::IsSupported(gralloc::BufferDescriptor gr_desc, bool *is_supported) {
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  SnapDescriptor snap_desc = {};
  auto err = GetSnapDescriptor(gr_desc, snap_desc);
  if (err) {
    return err;
  }

  snapallocator_->IsSupported(snap_desc, is_supported);

  return SnapError::NONE;
}

template <aidl::android::hardware::graphics::common::StandardMetadataType T>
int32_t Mapper5Encode(const typename StandardMetadata<T>::value_type &value, void *out_buffer,
                      size_t out_size) {
  using Value = typename StandardMetadata<T>::value;

  auto size_required = Value::encode(value, nullptr, 0);
  if (size_required < 0) {
    ALOGW_IF(-AIMAPPER_ERROR_UNSUPPORTED != size_required,
             "%s: Unexpected error %d during size calculation for encode (%d) call", __FUNCTION__,
             -size_required, static_cast<int64_t>(T));
    return -AIMAPPER_ERROR_UNSUPPORTED;
  }

  if (out_buffer != nullptr && size_required <= out_size) {
    size_required = Value::encode(value, out_buffer, out_size);
    if (size_required < 0 || (size_t)size_required > out_size) {
      ALOGW("Mapper5Encode (%d) failed, calculated size %d with buffer size %zd",
            static_cast<int64_t>(T), size_required, out_size);
    }
  }

  return size_required;
}

template <aidl::android::hardware::graphics::common::StandardMetadataType T>
auto Mapper5Decode(void *bytestream, size_t size)
    -> decltype(StandardMetadata<T>::value::decode(nullptr, 0)) {
  using Value = typename StandardMetadata<T>::value;
  return Value::decode(bytestream, size);
}

int GrallocSnapHelper::GetAllHandles(std::vector<buffer_handle_t> *out_handle_list) {
  std::lock_guard<std::mutex> lock(map_lock_);
  if (handles_map_.empty()) {
    return SnapError::NO_RESOURCES;
  }
  out_handle_list->reserve(handles_map_.size());
  for (auto handle : handles_map_) {
    out_handle_list->push_back(static_cast<buffer_handle_t>(handle.first));
  }
  return SnapError::NONE;
}

SnapError GrallocSnapHelper::BufferIDHelper(SnapHandle *hnd, uint32_t aidl_size,
                                            void *gralloc_in_set, void *gralloc_out_get,
                                            SnapDescriptor *buf_des, bool check_metadata_set,
                                            int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    if (aidl_size) {
      uint64_t snap_buffer_id = 0;
      error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BUFFER_ID, &snap_buffer_id);
      if (error) {
        ALOGW("%s - Error while getting the metadata type %d from snapmapper", __FUNCTION__,
              static_cast<int>(SnapMetadataType::BUFFER_ID));
        return error;
      }
      *mapper_return = Mapper5Encode<StandardMetadataType::BUFFER_ID>(
          snap_buffer_id, gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    } else {
      // Both snap and gralloc use uint64_t for buffer ID - write directly to gralloc output to reduce # copies
      error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BUFFER_ID, gralloc_out_get);
      if (error) {
        ALOGW("%s - Error while getting the metadata type %d from snapmapper", __FUNCTION__,
              static_cast<int>(SnapMetadataType::BUFFER_ID));
        return error;
      }
    }
  } else if (gralloc_in_set != nullptr) {
    error = SnapError::BAD_VALUE;
  }
  return error;
}

SnapError GrallocSnapHelper::UsageHelper(SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set,
                                         void *gralloc_out_get, SnapDescriptor *buf_des,
                                         bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapUsage snap_usage = static_cast<SnapUsage>(0);
    void *snap_out_get = aidl_size ? &snap_usage : gralloc_out_get;
    if (buf_des != nullptr) {
      error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::USAGE, snap_out_get);
    } else {
      error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::USAGE, snap_out_get);
    }
    if (aidl_size) {
      uint64_t gr_usage = GetGrallocUsage(snap_usage);
      *mapper_return = Mapper5Encode<StandardMetadataType::USAGE>(
          static_cast<GrallocBufferUsage>(gr_usage), gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    }
  } else if (gralloc_in_set != nullptr) {
    error = SnapError::BAD_VALUE;
  }
  return error;
}

SnapError GrallocSnapHelper::DataspaceHelper(SnapHandle *hnd, uint32_t aidl_size,
                                             void *gralloc_in_set, void *gralloc_out_get,
                                             SnapDescriptor *buf_des, bool check_metadata_set,
                                             int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapDataspace snap_dataspace = {};
    void *snap_out_get = aidl_size ? &snap_dataspace : gralloc_out_get;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::DATASPACE, snap_out_get);
    if (aidl_size) {
      GrallocDataspace gr_dataspace = {};
      ConvertSnapDataspaceToGrallocDataspace(snap_dataspace, &gr_dataspace);
      *mapper_return = Mapper5Encode<StandardMetadataType::DATASPACE>(gr_dataspace, gralloc_out_get,
                                                                      *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    }
  } else if (gralloc_in_set != nullptr) {
    SnapDataspace *snap_dataspace = static_cast<SnapDataspace *>(gralloc_in_set);
    SnapDataspace dataspace = {};
    if (aidl_size) {
      auto decoded_result =
          Mapper5Decode<StandardMetadataType::DATASPACE>(gralloc_in_set, aidl_size);
      if (!decoded_result.has_value()) {
        return SnapError::UNSUPPORTED;
      }
      int err = ConvertGrallocDataspaceToSnapDataspace(*decoded_result, &dataspace);
      if (err != SnapError::NONE && static_cast<int>(*decoded_result) != 0) {
        ALOGW("%s: Attempting to set invalid gralloc dataspace - %d", __FUNCTION__,
              *decoded_result);
        return SnapError::UNSUPPORTED;
      }
      snap_dataspace = static_cast<SnapDataspace *>(&dataspace);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::DATASPACE, snap_dataspace);
  }
  return error;
}

SnapError GrallocSnapHelper::NameHelper(SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set,
                                        void *gralloc_out_get, SnapDescriptor *buf_des,
                                        bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  std::string name = "";
  if (gralloc_in_set != nullptr) {
    return error;
  }
  void *snap_out_get = aidl_size ? &name : gralloc_out_get;
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::NAME, snap_out_get);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::NAME, snap_out_get);
  }

  if (aidl_size) {
    *mapper_return =
        Mapper5Encode<StandardMetadataType::NAME>(name, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  }
  return error;
}

SnapError GrallocSnapHelper::WidthHelper(SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set,
                                         void *gralloc_out_get, SnapDescriptor *buf_des,
                                         bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint64_t snap_width = 0;
  if (gralloc_in_set != nullptr) {
    return error;
  }
  void *snap_out_get = aidl_size ? &snap_width : gralloc_out_get;
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::WIDTH, snap_out_get);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::WIDTH, snap_out_get);
  }

  if (aidl_size) {
    *mapper_return =
        Mapper5Encode<StandardMetadataType::WIDTH>(snap_width, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  }
  return error;
}

SnapError GrallocSnapHelper::HeightHelper(SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set,
                                          void *gralloc_out_get, SnapDescriptor *buf_des,
                                          bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint64_t snap_height = 0;
  if (gralloc_in_set != nullptr) {
    return error;
  }
  void *snap_out_get = aidl_size ? &snap_height : gralloc_out_get;
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::HEIGHT, snap_out_get);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::HEIGHT, snap_out_get);
  }

  if (aidl_size) {
    *mapper_return =
        Mapper5Encode<StandardMetadataType::HEIGHT>(snap_height, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  }
  return error;
}

SnapError GrallocSnapHelper::LayerCountHelper(SnapHandle *hnd, uint32_t aidl_size,
                                              void *gralloc_in_set, void *gralloc_out_get,
                                              SnapDescriptor *buf_des, bool check_metadata_set,
                                              int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint64_t layer_count = 0;
  if (gralloc_in_set != nullptr) {
    return SnapError::BAD_VALUE;
  }
  void *snap_out_get = aidl_size ? &layer_count : gralloc_out_get;
  if (buf_des != nullptr) {
    error =
        snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::LAYER_COUNT, snap_out_get);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::LAYER_COUNT, snap_out_get);
  }

  if (aidl_size) {
    *mapper_return = Mapper5Encode<StandardMetadataType::LAYER_COUNT>(layer_count, gralloc_out_get,
                                                                      *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  }
  return error;
}

SnapError GrallocSnapHelper::PixelFormatRequestedHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                        void *gralloc_in_set, void *gralloc_out_get,
                                                        SnapDescriptor *buf_des,
                                                        bool check_metadata_set,
                                                        int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  SnapPixelFormat snap_pixel_format = SnapPixelFormat::PIXEL_FORMAT_UNSPECIFIED;
  SnapUsage snap_usage = static_cast<SnapUsage>(0);
  uint64_t modifier = 0;

  if (gralloc_in_set != nullptr) {
    return SnapError::BAD_VALUE;
  }
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::PIXEL_FORMAT_REQUESTED,
                                                 &snap_pixel_format);
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::USAGE, &snap_usage);
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::FORMAT_MODIFIER,
                                                 &modifier);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::PIXEL_FORMAT_REQUESTED,
                                     &snap_pixel_format);
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::USAGE, &snap_usage);
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::FORMAT_MODIFIER, &modifier);
  }
  SnapFormatDescriptor snap_fmt_desc = {.format = snap_pixel_format,
                                        .modifier = static_cast<SnapPixelFormatModifier>(modifier)};
  int gr_format = 0;
  SnapPixelFormat snap_flat_pixel_format = SnapPixelFormat::PIXEL_FORMAT_UNSPECIFIED;

  if (!aidl_size) {
    GetSnapFlatFormat(snap_fmt_desc, snap_usage, &snap_flat_pixel_format);
    if (snap_flat_pixel_format == SnapPixelFormat::PIXEL_FORMAT_UNSPECIFIED) {
      snap_flat_pixel_format = snap_pixel_format;
    }
    *static_cast<SnapPixelFormat *>(gralloc_out_get) = snap_flat_pixel_format;
    return error;
  }

  GetGrallocFormat(snap_fmt_desc, snap_usage, &gr_format);
  if (!gr_format) {
    gr_format = static_cast<int>(snap_pixel_format);
  }
  *mapper_return = Mapper5Encode<StandardMetadataType::PIXEL_FORMAT_REQUESTED>(
      static_cast<GrallocPixelFormat>(gr_format), gralloc_out_get, *mapper_return);
  if (*mapper_return < 0) {
    return SnapError::BAD_VALUE;
  }
  return error;
}

SnapError GrallocSnapHelper::PixelFormatAllocatedHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                        void *gralloc_in_set, void *gralloc_out_get,
                                                        SnapDescriptor *buf_des,
                                                        bool check_metadata_set,
                                                        int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  SnapPixelFormat snap_pixel_format = SnapPixelFormat::PIXEL_FORMAT_UNSPECIFIED;
  SnapUsage snap_usage = static_cast<SnapUsage>(0);
  uint64_t modifier = 0;

  if (gralloc_in_set != nullptr) {
    return SnapError::BAD_VALUE;
  }
  // This type is only supported as a vendor metadata type in Gralloc5
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::PIXEL_FORMAT_ALLOCATED,
                                                 &snap_pixel_format);
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::USAGE, &snap_usage);
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::FORMAT_MODIFIER,
                                                 &modifier);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::PIXEL_FORMAT_ALLOCATED,
                                     &snap_pixel_format);
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::USAGE, &snap_usage);
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::FORMAT_MODIFIER, &modifier);
  }
  SnapFormatDescriptor snap_fmt_desc = {.format = snap_pixel_format,
                                        .modifier = static_cast<SnapPixelFormatModifier>(modifier)};
  SnapPixelFormat snap_flat_pixel_format = SnapPixelFormat::PIXEL_FORMAT_UNSPECIFIED;

  GetSnapFlatFormat(snap_fmt_desc, snap_usage, &snap_flat_pixel_format);
  if (snap_flat_pixel_format == SnapPixelFormat::PIXEL_FORMAT_UNSPECIFIED) {
    snap_flat_pixel_format = snap_pixel_format;
  }

  *static_cast<SnapPixelFormat *>(gralloc_out_get) = snap_flat_pixel_format;

  return error;
}

SnapError GrallocSnapHelper::PixelFormatFourCCHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                     void *gralloc_in_set, void *gralloc_out_get,
                                                     SnapDescriptor *buf_des,
                                                     bool check_metadata_set,
                                                     int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint32_t pixel_format_fourcc = 0;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  void *snap_out_get = aidl_size ? &pixel_format_fourcc : gralloc_out_get;
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::PIXEL_FORMAT_FOURCC,
                                                 snap_out_get);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::PIXEL_FORMAT_FOURCC, snap_out_get);
  }

  if (aidl_size) {
    *mapper_return = Mapper5Encode<StandardMetadataType::PIXEL_FORMAT_FOURCC>(
        pixel_format_fourcc, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  }
  return error;
}

SnapError GrallocSnapHelper::DRMPixelFormatModifierHelper(
    SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set, void *gralloc_out_get,
    SnapDescriptor *buf_des, bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint64_t pixel_format_modifier = 0;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  void *snap_out_get = aidl_size ? &pixel_format_modifier : gralloc_out_get;
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(
        *buf_des, SnapMetadataType::DRM_PIXEL_FORMAT_MODIFIER, snap_out_get);
  } else if (gralloc_out_get != nullptr) {
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::DRM_PIXEL_FORMAT_MODIFIER, snap_out_get);
  }

  if (aidl_size) {
    *mapper_return = Mapper5Encode<StandardMetadataType::PIXEL_FORMAT_MODIFIER>(
        pixel_format_modifier, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  }
  return error;
}

SnapError GrallocSnapHelper::AllocationSizeHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                  void *gralloc_in_set, void *gralloc_out_get,
                                                  SnapDescriptor *buf_des, bool check_metadata_set,
                                                  int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint32_t allocation_size = 0;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  void *snap_out_get = aidl_size ? &allocation_size : gralloc_out_get;
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::ALLOCATION_SIZE,
                                                 snap_out_get);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::ALLOCATION_SIZE, snap_out_get);
  }

  if (aidl_size) {
    *mapper_return = Mapper5Encode<StandardMetadataType::ALLOCATION_SIZE>(
        static_cast<uint64_t>(allocation_size), gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  }
  return error;
}

SnapError GrallocSnapHelper::BaseViewHelper(SnapHandle *hnd, uint32_t aidl_size,
                                            void *gralloc_in_set, void *gralloc_out_get,
                                            SnapDescriptor *buf_des, bool check_metadata_set,
                                            int32_t *mapper_return) {
  (void)aidl_size;
  auto error = SnapError::BAD_VALUE;
  void *snap_out_get = gralloc_out_get;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BASE_VIEW, snap_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = SnapError::UNSUPPORTED;
  }
  return error;
}

SnapError GrallocSnapHelper::MultiViewHelper(SnapHandle *hnd, uint32_t aidl_size,
                                             void *gralloc_in_set, void *gralloc_out_get,
                                             SnapDescriptor *buf_des, bool check_metadata_set,
                                             int32_t *mapper_return) {
  (void)aidl_size;
  auto error = SnapError::BAD_VALUE;
  void *snap_out_get = gralloc_out_get;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::MULTI_VIEW_INFO, snap_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = SnapError::UNSUPPORTED;
  }
  return error;
}

SnapError GrallocSnapHelper::ThreeDimensionalRefInfoHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                 void *gralloc_in_set, void *gralloc_out_get,
                                                 SnapDescriptor *buf_des, bool check_metadata_set,
                                                 int32_t *mapper_return) {
  (void)aidl_size;
  auto error = SnapError::BAD_VALUE;
  void *snap_out_get = gralloc_out_get;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::THREE_DIMENSIONAL_REF_INFO, snap_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::THREE_DIMENSIONAL_REF_INFO, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::ProtectedContentHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                    void *gralloc_in_set, void *gralloc_out_get,
                                                    SnapDescriptor *buf_des,
                                                    bool check_metadata_set,
                                                    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint64_t protect_content = 0;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  void *snap_out_get = aidl_size ? &protect_content : gralloc_out_get;
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::PROTECTED_CONTENT,
                                                 snap_out_get);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::PROTECTED_CONTENT, snap_out_get);
  }

  if (aidl_size) {
    *mapper_return = Mapper5Encode<StandardMetadataType::PROTECTED_CONTENT>(
        protect_content, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  }
  return error;
}

SnapError GrallocSnapHelper::CompressionHelper(SnapHandle *hnd, uint32_t aidl_size,
                                               void *gralloc_in_set, void *gralloc_out_get,
                                               SnapDescriptor *buf_des, bool check_metadata_set,
                                               int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  int64_t snap_compression = 0;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  void *snap_out_get = aidl_size ? &snap_compression : gralloc_out_get;
  if (buf_des != nullptr) {
    error =
        snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::COMPRESSION, snap_out_get);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::COMPRESSION, snap_out_get);
  }

  if (aidl_size) {
    GrallocExtendableType gr_compression = {};

    if (snap_compression == vendor_qti_hardware_display_common_Compression::COMPRESSION_NONE) {
      gr_compression = android::gralloc4::Compression_None;
    } else {
      gr_compression = {"QTI", snap_compression};
    }
    *mapper_return = Mapper5Encode<StandardMetadataType::COMPRESSION>(
        gr_compression, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  }
  return error;
}

SnapError GrallocSnapHelper::InterlacedHelper(SnapHandle *hnd, uint32_t aidl_size,
                                              void *gralloc_in_set, void *gralloc_out_get,
                                              SnapDescriptor *buf_des, bool check_metadata_set,
                                              int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (gralloc_out_get != nullptr) {
    int64_t snap_interlaced = 0;
    void *snap_out_get = aidl_size ? &snap_interlaced : gralloc_out_get;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::INTERLACED, snap_out_get);
    if (aidl_size) {
      GrallocExtendableType gr_interlaced = {};
      if (snap_interlaced == vendor_qti_hardware_display_common_Interlaced::INTERLACED_NONE) {
        gr_interlaced = android::gralloc4::Interlaced_None;
      } else {
        gr_interlaced = qtigralloc::Interlaced_Qti;
      }
      *mapper_return = Mapper5Encode<StandardMetadataType::INTERLACED>(
          gr_interlaced, gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    }
  }
  return error;
}

SnapError GrallocSnapHelper::ChromaSitingHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                void *gralloc_in_set, void *gralloc_out_get,
                                                SnapDescriptor *buf_des, bool check_metadata_set,
                                                int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (gralloc_out_get != nullptr) {
    int64_t snap_chroma_siting = 0;
    void *snap_out_get = aidl_size ? &snap_chroma_siting : gralloc_out_get;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CHROMA_SITING, snap_out_get);
    if (aidl_size) {
      GrallocExtendableType gr_chroma_siting = {};
      if (snap_chroma_siting ==
          vendor_qti_hardware_display_common_ChromaSiting::CHROMA_SITING_NONE) {
        gr_chroma_siting = android::gralloc4::ChromaSiting_None;
      }
      *mapper_return = Mapper5Encode<StandardMetadataType::CHROMA_SITING>(
          gr_chroma_siting, gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    }
  }
  return error;
}

SnapError GrallocSnapHelper::PlaneLayoutsHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                void *gralloc_in_set, void *gralloc_out_get,
                                                SnapDescriptor *buf_des, bool check_metadata_set,
                                                int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  SnapBufferLayout snap_buffer_layout = {};
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  void *snap_out_get = aidl_size ? &snap_buffer_layout : gralloc_out_get;
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::PLANE_LAYOUTS,
                                                 snap_out_get);
    if (!error) {
      int64_t ubwc_enabled_in_snap;
      error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::IS_UBWC,
                                                   &ubwc_enabled_in_snap);
      // Added to keep parity with getFormatLayout since sdm, composer and gralloc don't expect
      // meta planes for this usecase.
      if ((IsUncompressedRGBFormat(static_cast<int>(buf_des->format)) ||
           IsCompressedRGBFormat(static_cast<int>(buf_des->format))) &&
          ubwc_enabled_in_snap) {
        (*static_cast<SnapBufferLayout *>(snap_out_get)).plane_count /= 2;
      }
    }
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::PLANE_LAYOUTS, snap_out_get);
  }
  if (aidl_size) {
    std::vector<GrallocPlaneLayout> gr_plane_layouts;
    ConvertSnapBufferlayoutToGrallocPlaneLayout(hnd, buf_des, snap_buffer_layout,
                                                &gr_plane_layouts);
    *mapper_return = Mapper5Encode<StandardMetadataType::PLANE_LAYOUTS>(
        gr_plane_layouts, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  }
  return error;
}

SnapError GrallocSnapHelper::CropHelper(SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set,
                                        void *gralloc_out_get, SnapDescriptor *buf_des,
                                        bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapRect snap_rect = {};
    void *snap_out_get = aidl_size ? &snap_rect : gralloc_out_get;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CROP, snap_out_get);
    if (aidl_size) {
      std::vector<Rect> out_crop = {
          {snap_rect.left, snap_rect.top, snap_rect.right, snap_rect.bottom}};
      *mapper_return =
          Mapper5Encode<StandardMetadataType::CROP>(out_crop, gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    }
  } else if (gralloc_in_set != nullptr) {
    SnapRect *snap_rect = static_cast<SnapRect *>(gralloc_in_set);
    SnapRect rect = {};
    if (aidl_size) {
      auto decoded_result = Mapper5Decode<StandardMetadataType::CROP>(gralloc_in_set, aidl_size);
      if (!decoded_result.has_value() || decoded_result->size() != 1) {
        return SnapError::UNSUPPORTED;
      }
      rect = {.left = decoded_result->at(0).left,
              .top = decoded_result->at(0).top,
              .right = decoded_result->at(0).right,
              .bottom = decoded_result->at(0).bottom};
      snap_rect = &rect;
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::CROP, snap_rect);
  }
  return error;
}

SnapError GrallocSnapHelper::BlendModeHelper(SnapHandle *hnd, uint32_t aidl_size,
                                             void *gralloc_in_set, void *gralloc_out_get,
                                             SnapDescriptor *buf_des, bool check_metadata_set,
                                             int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapBlendMode snap_blendmode = {};
    void *snap_out_get = aidl_size ? &snap_blendmode : gralloc_out_get;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BLEND_MODE, snap_out_get);

    if (aidl_size) {
      *mapper_return = Mapper5Encode<StandardMetadataType::BLEND_MODE>(
          static_cast<BlendMode>(snap_blendmode), gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    }
  } else if (gralloc_in_set != nullptr) {
    SnapBlendMode *snap_blendmode = static_cast<SnapBlendMode *>(gralloc_in_set);
    SnapBlendMode blendmode = {};
    if (aidl_size) {
      auto decoded_result =
          Mapper5Decode<StandardMetadataType::BLEND_MODE>(gralloc_in_set, aidl_size);
      if (!decoded_result.has_value()) {
        return SnapError::UNSUPPORTED;
      }
      blendmode = static_cast<SnapBlendMode>(*decoded_result);
      snap_blendmode = &blendmode;
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::BLEND_MODE, snap_blendmode);
  }
  return error;
}

SnapError GrallocSnapHelper::VTTimestampHelper(SnapHandle *hnd, uint32_t aidl_size,
                                               void *gralloc_in_set, void *gralloc_out_get,
                                               SnapDescriptor *buf_des, bool check_metadata_set,
                                               int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::VT_TIMESTAMP, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::VT_TIMESTAMP, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::BufferDequeueDurationHelper(
    SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set, void *gralloc_out_get,
    SnapDescriptor *buf_des, bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::BUFFER_DEQUEUE_DURATION, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::BUFFER_DEQUEUE_DURATION, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::CompressionMetadataHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                       void *gralloc_in_set, void *gralloc_out_get,
                                                       SnapDescriptor *buf_des,
                                                       bool check_metadata_set,
                                                       int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::ANAMORPHIC_COMPRESSION_METADATA,
                                     gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::ANAMORPHIC_COMPRESSION_METADATA,
                                     gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::PPParamInterlacedHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                     void *gralloc_in_set, void *gralloc_out_get,
                                                     SnapDescriptor *buf_des,
                                                     bool check_metadata_set,
                                                     int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::PP_PARAM_INTERLACED, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::PP_PARAM_INTERLACED, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::VideoPerfModeHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                 void *gralloc_in_set, void *gralloc_out_get,
                                                 SnapDescriptor *buf_des, bool check_metadata_set,
                                                 int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::VIDEO_PERF_MODE, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::VIDEO_PERF_MODE, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::GraphicsMetadataHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                    void *gralloc_in_set, void *gralloc_out_get,
                                                    SnapDescriptor *buf_des,
                                                    bool check_metadata_set,
                                                    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::GRAPHICS_METADATA, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::GRAPHICS_METADATA, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::RefreshRateHelper(SnapHandle *hnd, uint32_t aidl_size,
                                               void *gralloc_in_set, void *gralloc_out_get,
                                               SnapDescriptor *buf_des, bool check_metadata_set,
                                               int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::REFRESH_RATE, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::REFRESH_RATE, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::MapSecureBufferHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                   void *gralloc_in_set, void *gralloc_out_get,
                                                   SnapDescriptor *buf_des, bool check_metadata_set,
                                                   int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::MAP_SECURE_BUFFER, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::MAP_SECURE_BUFFER, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::LinearFormatHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                void *gralloc_in_set, void *gralloc_out_get,
                                                SnapDescriptor *buf_des, bool check_metadata_set,
                                                int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::LINEAR_FORMAT, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::LINEAR_FORMAT, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::SingleBufferModeHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                    void *gralloc_in_set, void *gralloc_out_get,
                                                    SnapDescriptor *buf_des,
                                                    bool check_metadata_set,
                                                    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::SINGLE_BUFFER_MODE, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::SINGLE_BUFFER_MODE, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::FDHelper(SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set,
                                      void *gralloc_out_get, SnapDescriptor *buf_des,
                                      bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::FD, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::FD, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::AlignedWidthInPixelsHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                        void *gralloc_in_set, void *gralloc_out_get,
                                                        SnapDescriptor *buf_des,
                                                        bool check_metadata_set,
                                                        int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint32_t aligned_width = 0;
  if (gralloc_out_get != nullptr) {
    void *snap_out_get = aidl_size ? &aligned_width : gralloc_out_get;
    if (buf_des != nullptr) {
      error = snapmapper_->GetFromBufferDescriptor(
          *buf_des, SnapMetadataType::ALIGNED_WIDTH_IN_PIXELS, snap_out_get);
    } else {
      error =
          snapmapper_->GetMetadata(*hnd, SnapMetadataType::ALIGNED_WIDTH_IN_PIXELS, snap_out_get);
    }
    if (aidl_size) {
      *mapper_return = Mapper5Encode<StandardMetadataType::STRIDE>(aligned_width, gralloc_out_get,
                                                                   *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    }
  } else if (gralloc_in_set != nullptr) {
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::ALIGNED_WIDTH_IN_PIXELS, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::AlignedHeightInPixelsHelper(
    SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set, void *gralloc_out_get,
    SnapDescriptor *buf_des, bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    if (buf_des != nullptr) {
      error = snapmapper_->GetFromBufferDescriptor(
          *buf_des, SnapMetadataType::ALIGNED_HEIGHT_IN_PIXELS, gralloc_out_get);
    } else {
      error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::ALIGNED_HEIGHT_IN_PIXELS,
                                       gralloc_out_get);
    }
  } else if (gralloc_in_set != nullptr) {
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::ALIGNED_HEIGHT_IN_PIXELS, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::VendorMetadataStatusHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                        void *gralloc_in_set, void *gralloc_out_get,
                                                        SnapDescriptor *buf_des,
                                                        bool check_metadata_set,
                                                        int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::VENDOR_METADATA_STATUS, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::VENDOR_METADATA_STATUS, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::StandardMetadataStatusHelper(
    SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set, void *gralloc_out_get,
    SnapDescriptor *buf_des, bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::STANDARD_METADATA_STATUS, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::STANDARD_METADATA_STATUS, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::BufferTypeHelper(SnapHandle *hnd, uint32_t aidl_size,
                                              void *gralloc_in_set, void *gralloc_out_get,
                                              SnapDescriptor *buf_des, bool check_metadata_set,
                                              int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BUFFER_TYPE, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::BUFFER_TYPE, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::CustomDimensionsStrideHelper(
    SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set, void *gralloc_out_get,
    SnapDescriptor *buf_des, bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::CUSTOM_DIMENSIONS_STRIDE, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::CUSTOM_DIMENSIONS_STRIDE, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::CustomDimensionsHeightHelper(
    SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set, void *gralloc_out_get,
    SnapDescriptor *buf_des, bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::CUSTOM_DIMENSIONS_HEIGHT, gralloc_out_get);
    if (error) {
      ALOGW("%s - Error while getting the metadata type %d from snapmapper", __FUNCTION__,
            static_cast<int>(SnapMetadataType::CUSTOM_DIMENSIONS_HEIGHT));
      return error;
    }
  } else if (gralloc_in_set != nullptr) {
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::CUSTOM_DIMENSIONS_HEIGHT, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::RGBDataAddressHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                  void *gralloc_in_set, void *gralloc_out_get,
                                                  SnapDescriptor *buf_des, bool check_metadata_set,
                                                  int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::RGB_DATA_ADDRESS, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::RGB_DATA_ADDRESS, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::BufferPermissionHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                    void *gralloc_in_set, void *gralloc_out_get,
                                                    SnapDescriptor *buf_des,
                                                    bool check_metadata_set,
                                                    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BUFFER_PERMISSION, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::BUFFER_PERMISSION, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::MemHandleHelper(SnapHandle *hnd, uint32_t aidl_size,
                                             void *gralloc_in_set, void *gralloc_out_get,
                                             SnapDescriptor *buf_des, bool check_metadata_set,
                                             int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::MEM_HANDLE, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::MEM_HANDLE, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::TimedRenderingHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                  void *gralloc_in_set, void *gralloc_out_get,
                                                  SnapDescriptor *buf_des, bool check_metadata_set,
                                                  int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::TIMED_RENDERING, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::TIMED_RENDERING, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::UBWCCRStatsInfoHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                   void *gralloc_in_set, void *gralloc_out_get,
                                                   SnapDescriptor *buf_des, bool check_metadata_set,
                                                   int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::UBWC_CR_STATS_INFO, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::UBWC_CR_STATS_INFO, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::CVPMetadataHelper(SnapHandle *hnd, uint32_t aidl_size,
                                               void *gralloc_in_set, void *gralloc_out_get,
                                               SnapDescriptor *buf_des, bool check_metadata_set,
                                               int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CVP_METADATA, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::CVP_METADATA, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::VideoTranscodeStatsHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                       void *gralloc_in_set, void *gralloc_out_get,
                                                       SnapDescriptor *buf_des,
                                                       bool check_metadata_set,
                                                       int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::VIDEO_TRANSCODE_STATS, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::VIDEO_TRANSCODE_STATS, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::VideoTSInfoHelper(SnapHandle *hnd, uint32_t aidl_size,
                                               void *gralloc_in_set, void *gralloc_out_get,
                                               SnapDescriptor *buf_des, bool check_metadata_set,
                                               int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::VIDEO_TS_INFO, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::VIDEO_TS_INFO, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::VideoHistogramStatsHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                       void *gralloc_in_set, void *gralloc_out_get,
                                                       SnapDescriptor *buf_des,
                                                       bool check_metadata_set,
                                                       int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::VIDEO_HISTOGRAM_STATS, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::VIDEO_HISTOGRAM_STATS, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::CustomContentMetadataHelper(
    SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set, void *gralloc_out_get,
    SnapDescriptor *buf_des, bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::CUSTOM_CONTENT_METADATA, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::CUSTOM_CONTENT_METADATA, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::SMPTE2094_10Helper(SnapHandle *hnd, uint32_t aidl_size,
                                                void *gralloc_in_set, void *gralloc_out_get,
                                                SnapDescriptor *buf_des, bool check_metadata_set,
                                                int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapCustomContentMetadata snap_custom_metadata = {};
    void *snap_out_get = aidl_size ? &snap_custom_metadata : gralloc_out_get;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::SMPTE2094_10, snap_out_get);
    if (aidl_size && snap_custom_metadata.size) {
      std::vector<uint8_t> custom_metadata_payload;
      custom_metadata_payload.resize(sizeof(snap_custom_metadata.metadataPayload));
      memcpy(custom_metadata_payload.data(), &snap_custom_metadata.metadataPayload,
             sizeof(snap_custom_metadata.metadataPayload));
      *mapper_return = Mapper5Encode<StandardMetadataType::SMPTE2094_10>(
          custom_metadata_payload, gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    }
  } else if (gralloc_in_set != nullptr) {
    SnapCustomContentMetadata *snap_custom_metadata =
        static_cast<SnapCustomContentMetadata *>(gralloc_in_set);
    SnapCustomContentMetadata snap_converted_custom_metadata = {};
    if (aidl_size) {
      std::optional<std::vector<uint8_t>> custom_metadata_payload = {};
      auto decoded_result =
          Mapper5Decode<StandardMetadataType::SMPTE2094_10>(gralloc_in_set, aidl_size);
      if (!decoded_result.has_value()) {
        return SnapError::UNSUPPORTED;
      }
      custom_metadata_payload = *decoded_result;
      snap_converted_custom_metadata.size = static_cast<int>(custom_metadata_payload->size());
      memcpy(&snap_converted_custom_metadata.metadataPayload, custom_metadata_payload->data(),
             custom_metadata_payload->size());

      snap_custom_metadata = &snap_converted_custom_metadata;
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::SMPTE2094_10, snap_custom_metadata);
  }
  return error;
}

SnapError GrallocSnapHelper::MasteringDisplayHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                    void *gralloc_in_set, void *gralloc_out_get,
                                                    SnapDescriptor *buf_des,
                                                    bool check_metadata_set,
                                                    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  // Conversion factors for Snap <=> AIDL/AOSP conversion
  // AIDL equivalent struct uses 1:1 units where as Snap uses 1/50k for primaries and whitepoint,
  // and 1/10k for minDisplayLuminance
  constexpr float snap_units[2] = {50000.0f, 10000.0f};
  if (gralloc_out_get != nullptr) {
    SnapMasteringDisplay snap_mastering_display_values = {};
    void *snap_out_get = aidl_size ? &snap_mastering_display_values : gralloc_out_get;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::MASTERING_DISPLAY, snap_out_get);
    if (aidl_size) {
      std::optional<GrallocSmpte2086> mastering_display_values;
      GrallocSmpte2086 smpte2086;
      if (snap_mastering_display_values.colorVolumeSEIEnabled) {
        smpte2086.primaryRed = {
            static_cast<float>(snap_mastering_display_values.primaryRed.x) / snap_units[0],
            static_cast<float>(snap_mastering_display_values.primaryRed.y) / snap_units[0]};
        smpte2086.primaryGreen = {
            static_cast<float>(snap_mastering_display_values.primaryGreen.x) / snap_units[0],
            static_cast<float>(snap_mastering_display_values.primaryGreen.y) / snap_units[0]};
        smpte2086.primaryBlue = {
            static_cast<float>(snap_mastering_display_values.primaryBlue.x) / snap_units[0],
            static_cast<float>(snap_mastering_display_values.primaryBlue.y) / snap_units[0]};
        smpte2086.whitePoint = {
            static_cast<float>(snap_mastering_display_values.whitePoint.x) / snap_units[0],
            static_cast<float>(snap_mastering_display_values.whitePoint.y) / snap_units[0]};
        smpte2086.maxLuminance =
            static_cast<float>(snap_mastering_display_values.maxDisplayLuminance);
        smpte2086.minLuminance =
            static_cast<float>(snap_mastering_display_values.minDisplayLuminance) / snap_units[1];
        mastering_display_values = std::move(smpte2086);
      }

      *mapper_return = Mapper5Encode<StandardMetadataType::SMPTE2086>(
          mastering_display_values, gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    }
  } else if (gralloc_in_set != nullptr) {
    SnapMasteringDisplay *snap_mastering_display_values =
        static_cast<SnapMasteringDisplay *>(gralloc_in_set);
    SnapMasteringDisplay snap_converted_mastering_display_values = {};
    if (aidl_size) {
      std::optional<GrallocSmpte2086> mastering_display_values = {};
      auto decoded_result =
          Mapper5Decode<StandardMetadataType::SMPTE2086>(gralloc_in_set, aidl_size);
      if (!decoded_result.has_value()) {
        return SnapError::UNSUPPORTED;
      }
      mastering_display_values = *decoded_result;
      if (mastering_display_values != std::nullopt) {
        snap_converted_mastering_display_values.colorVolumeSEIEnabled = true;
        snap_converted_mastering_display_values.primaryRed = {
            static_cast<uint32_t>(mastering_display_values->primaryRed.x * snap_units[0]),
            static_cast<uint32_t>(mastering_display_values->primaryRed.y * snap_units[0])};
        snap_converted_mastering_display_values.primaryGreen = {
            static_cast<uint32_t>(mastering_display_values->primaryGreen.x * snap_units[0]),
            static_cast<uint32_t>(mastering_display_values->primaryGreen.y * snap_units[0])};
        snap_converted_mastering_display_values.primaryBlue = {
            static_cast<uint32_t>(mastering_display_values->primaryBlue.x * snap_units[0]),
            static_cast<uint32_t>(mastering_display_values->primaryBlue.y * snap_units[0])};
        snap_converted_mastering_display_values.whitePoint = {
            static_cast<uint32_t>(mastering_display_values->whitePoint.x * snap_units[0]),
            static_cast<uint32_t>(mastering_display_values->whitePoint.y * snap_units[0])};
        snap_converted_mastering_display_values.maxDisplayLuminance =
            static_cast<uint32_t>(mastering_display_values->maxLuminance);
        snap_converted_mastering_display_values.minDisplayLuminance =
            static_cast<uint32_t>(mastering_display_values->minLuminance * snap_units[1]);
      } else {
        snap_converted_mastering_display_values.colorVolumeSEIEnabled = false;
      }
      snap_mastering_display_values = &snap_converted_mastering_display_values;
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::MASTERING_DISPLAY,
                                     snap_mastering_display_values);
  }
  // Handling for when std::nullopt is passed in with expectation to invalidate the metadata
  else if (gralloc_in_set == nullptr && aidl_size == 1) {
    SnapMasteringDisplay snap_mastering_display_values = {};
    snap_mastering_display_values.colorVolumeSEIEnabled = false;
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::MASTERING_DISPLAY,
                                     &snap_mastering_display_values);
  }
  return error;
}

SnapError GrallocSnapHelper::ContentLightLevelHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                     void *gralloc_in_set, void *gralloc_out_get,
                                                     SnapDescriptor *buf_des,
                                                     bool check_metadata_set,
                                                     int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapContentLightLevel snap_content_light_level = {};
    void *snap_out_get = aidl_size ? &snap_content_light_level : gralloc_out_get;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CONTENT_LIGHT_LEVEL, snap_out_get);
    if (aidl_size) {
      std::optional<GrallocCta861_3> content_light_level = {};
      GrallocCta861_3 cta861_3;
      if (snap_content_light_level.lightLevelSEIEnabled) {
        cta861_3.maxContentLightLevel =
            static_cast<float>(snap_content_light_level.maxContentLightLevel);
        cta861_3.maxFrameAverageLightLevel =
            static_cast<float>(snap_content_light_level.maxFrameAverageLightLevel);
        content_light_level = std::move(cta861_3);
      }
      *mapper_return = Mapper5Encode<StandardMetadataType::CTA861_3>(
          content_light_level, gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    }
  } else if (gralloc_in_set != nullptr) {
    SnapContentLightLevel *snap_content_light_level =
        static_cast<SnapContentLightLevel *>(gralloc_in_set);
    SnapContentLightLevel snap_converted_content_light_level = {};
    if (aidl_size) {
      std::optional<GrallocCta861_3> content_light_level = {};
      auto decoded_result =
          Mapper5Decode<StandardMetadataType::CTA861_3>(gralloc_in_set, aidl_size);
      if (!decoded_result.has_value()) {
        return SnapError::UNSUPPORTED;
      }
      content_light_level = *decoded_result;
      if (content_light_level != std::nullopt) {
        snap_converted_content_light_level.lightLevelSEIEnabled = true;
        snap_converted_content_light_level.maxContentLightLevel =
            static_cast<uint32_t>(content_light_level->maxContentLightLevel);
        snap_converted_content_light_level.maxFrameAverageLightLevel =
            static_cast<uint32_t>(content_light_level->maxFrameAverageLightLevel);
      } else {
        snap_converted_content_light_level.lightLevelSEIEnabled = false;
      }
      snap_content_light_level = &snap_converted_content_light_level;
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::CONTENT_LIGHT_LEVEL,
                                     snap_content_light_level);
  } else if (gralloc_in_set == nullptr && aidl_size == 1) {
    SnapContentLightLevel snap_content_light_level = {};
    snap_content_light_level.lightLevelSEIEnabled = false;
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::CONTENT_LIGHT_LEVEL,
                                     &snap_content_light_level);
  }
  return error;
}

SnapError GrallocSnapHelper::DynamicMetadataHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                   void *gralloc_in_set, void *gralloc_out_get,
                                                   SnapDescriptor *buf_des, bool check_metadata_set,
                                                   int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapDynamicMetadata snap_dynamic_metadata = {};
    void *snap_out_get = aidl_size ? &snap_dynamic_metadata : gralloc_out_get;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::DYNAMIC_METADATA, snap_out_get);
    if (aidl_size) {
      if (snap_dynamic_metadata.dynamicMetaDataValid) {
        std::vector<uint8_t> dynamic_metadata_payload;
        dynamic_metadata_payload.resize(sizeof(snap_dynamic_metadata.dynamicMetaDataPayload));
        memcpy(dynamic_metadata_payload.data(), &snap_dynamic_metadata.dynamicMetaDataPayload,
               sizeof(snap_dynamic_metadata.dynamicMetaDataPayload));
        *mapper_return = Mapper5Encode<StandardMetadataType::SMPTE2094_40>(
            dynamic_metadata_payload, gralloc_out_get, *mapper_return);
        if (*mapper_return < 0) {
          return SnapError::BAD_VALUE;
        }
      }
    }
  } else if (gralloc_in_set != nullptr) {
    SnapDynamicMetadata *snap_dynamic_metadata = static_cast<SnapDynamicMetadata *>(gralloc_in_set);
    SnapDynamicMetadata snap_converted_dynamic_metadata = {};
    if (aidl_size) {
      std::optional<std::vector<uint8_t>> dynamic_metadata_payload = {};
      auto decoded_result =
          Mapper5Decode<StandardMetadataType::SMPTE2094_40>(gralloc_in_set, aidl_size);
      if (!decoded_result.has_value()) {
        return SnapError::UNSUPPORTED;
      }
      dynamic_metadata_payload = *decoded_result;
      if (dynamic_metadata_payload != std::nullopt) {
        snap_converted_dynamic_metadata.dynamicMetaDataLen =
            static_cast<int>(dynamic_metadata_payload->size());
        memcpy(&snap_converted_dynamic_metadata.dynamicMetaDataPayload,
               dynamic_metadata_payload->data(), dynamic_metadata_payload->size());
        snap_converted_dynamic_metadata.dynamicMetaDataValid = true;
      } else {
        snap_converted_dynamic_metadata.dynamicMetaDataValid = false;
      }
      snap_dynamic_metadata = &snap_converted_dynamic_metadata;
    }
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::DYNAMIC_METADATA, snap_dynamic_metadata);
  } else if (gralloc_in_set == nullptr && aidl_size == 1) {
    // Handling for when std::nullopt is passed in with expectation to invalidate the metadata
    SnapDynamicMetadata snap_dynamic_metadata = {};
    snap_dynamic_metadata.dynamicMetaDataValid = false;
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::DYNAMIC_METADATA, &snap_dynamic_metadata);
  }
  return error;
}

SnapError GrallocSnapHelper::ColorRemappingInfoHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                      void *gralloc_in_set, void *gralloc_out_get,
                                                      SnapDescriptor *buf_des,
                                                      bool check_metadata_set,
                                                      int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::COLOR_REMAPPING_INFO, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::COLOR_REMAPPING_INFO, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::HeapNameHelper(SnapHandle *hnd, uint32_t aidl_size,
                                            void *gralloc_in_set, void *gralloc_out_get,
                                            SnapDescriptor *buf_des, bool check_metadata_set,
                                            int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::HEAP_NAME, gralloc_out_get);
  }
  return error;
}

int GrallocSnapHelper::GetMetadata(native_handle_t *gr_hnd, uint64_t gr_metadata_type, void *out,
                                   bool convert_bytestream, bool check_metadata_set,
                                   uint32_t aidl_size, int32_t *mapper_return) {
  if (gr_hnd == nullptr) {
    ALOGE("Invalid gralloc handle");
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    auto snap_metadata_type = static_cast<SnapMetadataType>(gr_metadata_type);
    // TODO: Cleanup this block and legacy color metadata code once HWC moves to Snap defs
    if (snap_metadata_type == static_cast<SnapMetadataType>(QTI_COLOR_METADATA)) {
      return ColorMetadataHelper(hnd, 0, nullptr, out, nullptr, check_metadata_set, mapper_return);
    }

    if (metadata_conversion_helper_function_map.find(snap_metadata_type) !=
        metadata_conversion_helper_function_map.end()) {
      MetadataHelper metadata_helper_func =
          metadata_conversion_helper_function_map[snap_metadata_type];
      auto error = ((this->*metadata_helper_func)(hnd, aidl_size, nullptr, out, nullptr,
                                                  check_metadata_set, mapper_return));
      if (error == SnapError::METADATA_NOT_SET && !check_metadata_set) {
        ALOGI("Metadata type %d is not set.Returning default values as check_metadata_set is %d",
              gr_metadata_type, check_metadata_set);
        return SnapError::NONE;
      }
      return error;
    } else {
      return SnapError::UNSUPPORTED;
    }
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
    return SnapError::BAD_BUFFER;
  }

  return SnapError::UNSUPPORTED;
}

int GrallocSnapHelper::GetMetadataState(native_handle_t *gr_hnd, SnapMetadataType metadata_type,
                                        bool *out) {
  if (gr_hnd == nullptr) {
    ALOGE("Invalid gralloc handle");
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    auto status = snapmapper_->GetMetadataState(*hnd, metadata_type, out);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGW("%s: Failed to get metadata state for metadata type %d via SnapAlloc. Error code: %d",
            __FUNCTION__, metadata_type, status);
    }
    return status;
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }

  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelper::SetMetadata(native_handle_t *gr_hnd, uint64_t gr_metadata_type, void *in,
                                   uint32_t aidl_size) {
  if (gr_hnd == nullptr) {
    ALOGE("Invalid gralloc handle");
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    auto snap_metadata_type = static_cast<SnapMetadataType>(gr_metadata_type);
    // TODO: Cleanup this block and legacy color metadata code once HWC moves to Snap defs
    if (snap_metadata_type == static_cast<SnapMetadataType>(QTI_COLOR_METADATA)) {
      return ColorMetadataHelper(hnd, aidl_size, in, nullptr, nullptr, false, nullptr);
    }

    if (metadata_conversion_helper_function_map.find(snap_metadata_type) !=
        metadata_conversion_helper_function_map.end()) {
      MetadataHelper metadata_helper_func =
          metadata_conversion_helper_function_map[snap_metadata_type];
      auto error =
          ((this->*metadata_helper_func)(hnd, aidl_size, in, nullptr, nullptr, false, nullptr));
      if (error == SnapError::BAD_VALUE || error == SnapError::UNSUPPORTED) {
        ALOGW("Unable to set metadata - metadata type %d", snap_metadata_type);
      }
      return error;
    } else {
      return SnapError::UNSUPPORTED;
    }
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }
  return SnapError::BAD_BUFFER;
}

SnapError GrallocSnapHelper::ColorMetadataHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                 void *gralloc_in_set, void *gralloc_out_get,
                                                 SnapDescriptor *buf_des, bool check_metadata_set,
                                                 int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    ColorMetaData color_metadata;
    SnapDataspace dataspace;
    auto status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::DATASPACE, &dataspace);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGW("%s Unable to get DATASPACE from snap", __FUNCTION__);
    } else {
      color_metadata.colorPrimaries =
          static_cast<ColorPrimaries>(static_cast<uint32_t>(dataspace.colorPrimaries));
      color_metadata.range = static_cast<ColorRange>(static_cast<uint32_t>(dataspace.range));
      color_metadata.transfer =
          static_cast<GammaTransfer>(static_cast<uint32_t>(dataspace.transfer));
    }

    SnapMasteringDisplay snap_mastering_display_values;
    status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::MASTERING_DISPLAY,
                                      &snap_mastering_display_values);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGW("%s Unable to get MASTERING_DISPLAY from snap", __FUNCTION__);
    } else {
      color_metadata.masteringDisplayInfo.colorVolumeSEIEnabled = true;
      color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[0][0] =
          static_cast<uint32_t>(snap_mastering_display_values.primaryRed.x);
      color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[0][1] =
          static_cast<uint32_t>(snap_mastering_display_values.primaryRed.y);

      color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[1][0] =
          static_cast<uint32_t>(snap_mastering_display_values.primaryGreen.x);
      color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[1][1] =
          static_cast<uint32_t>(snap_mastering_display_values.primaryGreen.y);

      color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[2][0] =
          static_cast<uint32_t>(snap_mastering_display_values.primaryBlue.x);
      color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[2][1] =
          static_cast<uint32_t>(snap_mastering_display_values.primaryBlue.y);

      color_metadata.masteringDisplayInfo.primaries.whitePoint[0] =
          static_cast<uint32_t>(snap_mastering_display_values.whitePoint.x);
      color_metadata.masteringDisplayInfo.primaries.whitePoint[1] =
          static_cast<uint32_t>(snap_mastering_display_values.whitePoint.y);

      color_metadata.masteringDisplayInfo.maxDisplayLuminance =
          static_cast<uint32_t>(snap_mastering_display_values.maxDisplayLuminance);
      color_metadata.masteringDisplayInfo.minDisplayLuminance =
          static_cast<uint32_t>(snap_mastering_display_values.minDisplayLuminance);
    }

    SnapContentLightLevel snap_content_light_level;
    status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CONTENT_LIGHT_LEVEL,
                                      &snap_content_light_level);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGW("%s Unable to get CONTENT_LIGHT_LEVEL from snap", __FUNCTION__);
    } else {
      color_metadata.contentLightLevel.lightLevelSEIEnabled = true;
      color_metadata.contentLightLevel.maxContentLightLevel =
          static_cast<uint32_t>(snap_content_light_level.maxContentLightLevel);
      color_metadata.contentLightLevel.minPicAverageLightLevel =
          static_cast<uint32_t>(snap_content_light_level.maxFrameAverageLightLevel);
    }

    SnapDynamicMetadata snap_dynamic_metadata;
    status =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::DYNAMIC_METADATA, &snap_dynamic_metadata);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGW("%s Unable to get DYNAMIC_METADATA from snap", __FUNCTION__);
    } else {
      color_metadata.dynamicMetaDataLen = snap_dynamic_metadata.dynamicMetaDataLen;
      color_metadata.dynamicMetaDataValid = true;
      memcpy(&color_metadata.dynamicMetaDataPayload, &snap_dynamic_metadata.dynamicMetaDataPayload,
             snap_dynamic_metadata.dynamicMetaDataLen);
    }

    SnapColorRemappingInfo snap_color_remapping_info;
    status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::COLOR_REMAPPING_INFO,
                                      &snap_color_remapping_info);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGW("Unable to get COLOR_REMAPPING_INFO from snap");
    } else {
      ColorRemappingInfo gr_crI;
      gr_crI.criEnabled = static_cast<bool>(snap_color_remapping_info.criEnabled);
      gr_crI.crId = static_cast<uint32_t>(snap_color_remapping_info.crId);
      gr_crI.crCancelFlag = static_cast<uint32_t>(snap_color_remapping_info.crCancelFlag);
      gr_crI.crPersistenceFlag = static_cast<uint32_t>(snap_color_remapping_info.crPersistenceFlag);
      gr_crI.crVideoSignalInfoPresentFlag =
          static_cast<uint32_t>(snap_color_remapping_info.crVideoSignalInfoPresentFlag);
      gr_crI.crRange = static_cast<uint32_t>(snap_color_remapping_info.crRange);
      gr_crI.crInputBitDepth = static_cast<uint32_t>(snap_color_remapping_info.crInputBitDepth);
      gr_crI.crOutputBitDepth = static_cast<uint32_t>(snap_color_remapping_info.crOutputBitDepth);
      gr_crI.crMatrixPresentFlag =
          static_cast<uint32_t>(snap_color_remapping_info.crMatrixPresentFlag);
      gr_crI.crLog2MatrixDenom = static_cast<uint32_t>(snap_color_remapping_info.crLog2MatrixDenom);
      std::copy(std::begin(snap_color_remapping_info.crPreLutNumValMinusOne),
                std::end(snap_color_remapping_info.crPreLutNumValMinusOne),
                std::begin(gr_crI.crPreLutNumValMinusOne));
      std::copy(std::begin(snap_color_remapping_info.crPreLutCodedValue),
                std::end(snap_color_remapping_info.crPreLutCodedValue),
                std::begin(gr_crI.crPreLutCodedValue));
      std::copy(std::begin(snap_color_remapping_info.crPreLutTargetValue),
                std::end(snap_color_remapping_info.crPreLutTargetValue),
                std::begin(gr_crI.crPreLutTargetValue));
      std::copy(std::begin(snap_color_remapping_info.crCoefficients),
                std::end(snap_color_remapping_info.crCoefficients),
                std::begin(gr_crI.crCoefficients));
      std::copy(std::begin(snap_color_remapping_info.crPostLutNumValMinusOne),
                std::end(snap_color_remapping_info.crPostLutNumValMinusOne),
                std::begin(gr_crI.crPostLutNumValMinusOne));
      std::copy(std::begin(snap_color_remapping_info.crPostLutCodedValue),
                std::end(snap_color_remapping_info.crPostLutCodedValue),
                std::begin(gr_crI.crPostLutCodedValue));
      gr_crI.crMatrixCoefficients = static_cast<MatrixCoEfficients>(
          static_cast<uint32_t>(snap_color_remapping_info.crMatrixCoefficients));
      gr_crI.crTransferFunction = static_cast<GammaTransfer>(
          static_cast<uint32_t>(snap_color_remapping_info.crTransferFunction));
      gr_crI.crPrimaries =
          static_cast<ColorPrimaries>(static_cast<uint32_t>(snap_color_remapping_info.crPrimaries));
      color_metadata.cRI = gr_crI;
    }

    SnapMatrixCoEfficients snap_matrix_coefficients;
    status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::MATRIX_COEFFICIENTS,
                                      &snap_matrix_coefficients);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGW("Unable to get MATRIX_COEFFICIENTS from snap");
    } else {
      color_metadata.matrixCoefficients =
          static_cast<MatrixCoEfficients>(static_cast<int32_t>(snap_matrix_coefficients));
    }
    *static_cast<ColorMetaData *>(gralloc_out_get) = color_metadata;
    return SnapError::NONE;
  } else if (gralloc_in_set != nullptr) {
    ColorMetaData color_metadata;
    color_metadata = *static_cast<ColorMetaData *>(gralloc_in_set);

    SnapDataspace snap_dataspace;
    snap_dataspace.colorPrimaries = static_cast<SnapColorPrimaries>(color_metadata.colorPrimaries);
    snap_dataspace.range = static_cast<SnapColorRange>(color_metadata.range);
    snap_dataspace.transfer = static_cast<SnapGammaTransfer>(color_metadata.transfer);
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::DATASPACE, &snap_dataspace);
    if (error != SnapError::NONE) {
      ALOGE(
          "%s: Failed to set snap metadata type - DATASPACE via SnapAlloc. Error code: "
          "%d",
          __FUNCTION__, error);
      return error;
    }

    SnapMasteringDisplay snap_mastering_display_values;
    // Only convert values if enabled - otherwise send empty struct
    if (color_metadata.masteringDisplayInfo.colorVolumeSEIEnabled) {
      snap_mastering_display_values.primaryRed = {
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[0][0]),
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[0][1])};
      snap_mastering_display_values.primaryGreen = {
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[1][0]),
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[1][1])};
      snap_mastering_display_values.primaryBlue = {
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[2][0]),
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[2][1])};
      snap_mastering_display_values.whitePoint = {
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.whitePoint[0]),
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.whitePoint[1])};
      snap_mastering_display_values.maxDisplayLuminance =
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.maxDisplayLuminance);
      snap_mastering_display_values.minDisplayLuminance =
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.minDisplayLuminance);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::MASTERING_DISPLAY,
                                     &snap_mastering_display_values);
    if (error != SnapError::NONE) {
      ALOGE(
          "%s: Failed to set snap metadata type - MASTERING_DISPLAY via SnapAlloc. Error code: "
          "%d",
          __FUNCTION__, error);
      return error;
    }

    SnapContentLightLevel snap_content_light_level;
    // Only convert values if enabled - otherwise send empty struct
    if (color_metadata.contentLightLevel.lightLevelSEIEnabled) {
      snap_content_light_level.maxContentLightLevel =
          static_cast<float>(color_metadata.contentLightLevel.maxContentLightLevel);
      snap_content_light_level.maxFrameAverageLightLevel =
          static_cast<float>(color_metadata.contentLightLevel.minPicAverageLightLevel);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::CONTENT_LIGHT_LEVEL,
                                     &snap_content_light_level);
    if (error != SnapError::NONE) {
      ALOGE(
          "%s: Failed to set snap metadata type - CONTENT_LIGHT_LEVEL via SnapAlloc. Error "
          "code: %d",
          __FUNCTION__, error);
      return error;
    }
    SnapDynamicMetadata snap_dynamic_metadata;
    if (color_metadata.dynamicMetaDataValid &&
        color_metadata.dynamicMetaDataLen <= HDR_DYNAMIC_META_DATA_SZ) {
      snap_dynamic_metadata.dynamicMetaDataLen = color_metadata.dynamicMetaDataLen;
      snap_dynamic_metadata.dynamicMetaDataValid = color_metadata.dynamicMetaDataValid;
      memcpy(&snap_dynamic_metadata.dynamicMetaDataPayload, &color_metadata.dynamicMetaDataPayload,
             color_metadata.dynamicMetaDataLen);
    }
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::DYNAMIC_METADATA, &snap_dynamic_metadata);
    if (error != SnapError::NONE) {
      ALOGE(
          "%s: Failed to set snap metadata type - DYNAMIC_METADATA via SnapAlloc. Error code: "
          "%d",
          __FUNCTION__, error);
    }

    SnapColorRemappingInfo snap_color_remapping_info;
    if (static_cast<bool>(color_metadata.cRI.criEnabled)) {
      ColorRemappingInfo gr_crI = color_metadata.cRI;
      snap_color_remapping_info.criEnabled = static_cast<bool>(gr_crI.criEnabled);
      snap_color_remapping_info.crId = static_cast<int>(gr_crI.crId);
      snap_color_remapping_info.crCancelFlag = static_cast<int>(gr_crI.crCancelFlag);
      snap_color_remapping_info.crPersistenceFlag = static_cast<int>(gr_crI.crPersistenceFlag);
      snap_color_remapping_info.crVideoSignalInfoPresentFlag =
          static_cast<int>(gr_crI.crVideoSignalInfoPresentFlag);
      snap_color_remapping_info.crRange = static_cast<int>(gr_crI.crRange);
      snap_color_remapping_info.crInputBitDepth = static_cast<int>(gr_crI.crInputBitDepth);
      snap_color_remapping_info.crOutputBitDepth = static_cast<int>(gr_crI.crOutputBitDepth);
      snap_color_remapping_info.crMatrixPresentFlag = static_cast<int>(gr_crI.crMatrixPresentFlag);
      snap_color_remapping_info.crLog2MatrixDenom = static_cast<int>(gr_crI.crLog2MatrixDenom);
      std::copy(std::begin(gr_crI.crPreLutNumValMinusOne), std::end(gr_crI.crPreLutNumValMinusOne),
                std::begin(snap_color_remapping_info.crPreLutNumValMinusOne));
      std::copy(std::begin(gr_crI.crPreLutCodedValue), std::end(gr_crI.crPreLutCodedValue),
                std::begin(snap_color_remapping_info.crPreLutCodedValue));
      std::copy(std::begin(gr_crI.crPreLutTargetValue), std::end(gr_crI.crPreLutTargetValue),
                std::begin(snap_color_remapping_info.crPreLutTargetValue));
      std::copy(std::begin(gr_crI.crCoefficients), std::end(gr_crI.crCoefficients),
                std::begin(snap_color_remapping_info.crCoefficients));
      std::copy(std::begin(gr_crI.crPostLutNumValMinusOne),
                std::end(gr_crI.crPostLutNumValMinusOne),
                std::begin(snap_color_remapping_info.crPostLutNumValMinusOne));
      std::copy(std::begin(gr_crI.crPostLutCodedValue), std::end(gr_crI.crPostLutCodedValue),
                std::begin(snap_color_remapping_info.crPostLutCodedValue));
      snap_color_remapping_info.crMatrixCoefficients =
          static_cast<SnapMatrixCoEfficients>(static_cast<int>(gr_crI.crMatrixCoefficients));
      snap_color_remapping_info.crTransferFunction =
          static_cast<SnapGammaTransfer>(static_cast<int>(gr_crI.crTransferFunction));
      snap_color_remapping_info.crPrimaries =
          static_cast<SnapColorPrimaries>(static_cast<int>(gr_crI.crPrimaries));
      error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::COLOR_REMAPPING_INFO,
                                       &snap_color_remapping_info);
      if (error != SnapError::NONE) {
        ALOGE(
            "%s: Failed to set snap metadata type - COLOR_REMAPPING_INFO via SnapAlloc. Error "
            "code: %d",
            __FUNCTION__, error);
      }
    }

    SnapMatrixCoEfficients snap_matrix_coefficients;
    if (color_metadata.matrixCoefficients > 0) {
      int64_t snap_matrix_coefficients = static_cast<int64_t>(color_metadata.matrixCoefficients);
      error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::MATRIX_COEFFICIENTS,
                                       &snap_matrix_coefficients);
      if (error != SnapError::NONE) {
        ALOGE(
            "%s: Failed to set snap metadata type - MATRIX_COEFFICIENTS via SnapAlloc. Error "
            "code: %d",
            __FUNCTION__, error);
      }
    }
  }
  return error;
}

SnapError GrallocSnapHelper::IsUBWCHelper(SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set,
                                          void *gralloc_out_get, SnapDescriptor *buf_des,
                                          bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::IS_UBWC, gralloc_out_get);
  }

  return error;
}

SnapError GrallocSnapHelper::IsTileRenderedHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                  void *gralloc_in_set, void *gralloc_out_get,
                                                  SnapDescriptor *buf_des, bool check_metadata_set,
                                                  int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::IS_TILE_RENDERED, gralloc_out_get);
  }

  return error;
}

SnapError GrallocSnapHelper::IsCachedHelper(SnapHandle *hnd, uint32_t aidl_size,
                                            void *gralloc_in_set, void *gralloc_out_get,
                                            SnapDescriptor *buf_des, bool check_metadata_set,
                                            int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::IS_CACHED, gralloc_out_get);
  }

  return error;
}

SnapError GrallocSnapHelper::BaseAddressHelper(SnapHandle *hnd, uint32_t aidl_size,
                                               void *gralloc_in_set, void *gralloc_out_get,
                                               SnapDescriptor *buf_des, bool check_metadata_set,
                                               int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    // This type is only supported as a vendor metadata type in Gralloc5
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BASE_ADDRESS, gralloc_out_get);
  }
  return error;
}

SnapError GrallocSnapHelper::MatrixCoefficientsHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                      void *gralloc_in_set, void *gralloc_out_get,
                                                      SnapDescriptor *buf_des,
                                                      bool check_metadata_set,
                                                      int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    // This type is only supported as a vendor metadata type in Gralloc5
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::MATRIX_COEFFICIENTS, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    // This type is only supported as a vendor metadata type in Gralloc5
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::MATRIX_COEFFICIENTS, gralloc_in_set);
  }
  return error;
}

SnapError GrallocSnapHelper::EarlyNotifyLineCountHelper(SnapHandle *hnd, uint32_t aidl_size,
                                                        void *gralloc_in_set, void *gralloc_out_get,
                                                        SnapDescriptor *buf_des,
                                                        bool check_metadata_set,
                                                        int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::EARLYNOTIFY_LINECOUNT, gralloc_out_get);
  } else if (gralloc_in_set != nullptr) {
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::EARLYNOTIFY_LINECOUNT, gralloc_in_set);
  }
  return error;
}

int GrallocSnapHelper::GetFromBufferDescriptor(gralloc::BufferDescriptor gr_desc,
                                               uint64_t gr_metadata_type, void *out,
                                               bool convert_to_hidl_bytestream) {
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  SnapDescriptor snap_desc = {};
  auto err = GetSnapDescriptor(gr_desc, snap_desc);
  if (err) {
    return err;
  }
  auto snap_metadata_type = static_cast<SnapMetadataType>(gr_metadata_type);
  if (bufferdescription_conversion_helper_function_map.find(snap_metadata_type) !=
      bufferdescription_conversion_helper_function_map.end()) {
    MetadataHelper metadata_helper_func =
        bufferdescription_conversion_helper_function_map[snap_metadata_type];
    return ((this->*metadata_helper_func)(nullptr, 0, nullptr, out, &snap_desc, false, nullptr));
  } else {
    return SnapError::UNSUPPORTED;
  }

  return SnapError::UNSUPPORTED;
}

int GrallocSnapHelper::ConvertSnapBufferlayoutToGrallocPlaneLayout(
    SnapHandle *hnd, SnapDescriptor *buf_des, const SnapBufferLayout snap_buffer_layout,
    std::vector<GrallocPlaneLayout> *gr_plane_layouts) {
  uint64_t width, height;
  SnapPixelFormat snap_pixel_format = SnapPixelFormat::PIXEL_FORMAT_UNSPECIFIED;
  bool is_raw = false;
  bool individually_packed = true;

  if (hnd != nullptr) {
    // Get unaligned width
    auto status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::WIDTH, &width);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGE("Unable to get unaligned width");
      return status;
    }
    //Get unaligned height
    status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::HEIGHT, &height);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGE("Unable to get unaligned height");
      return status;
    }
    // Get pixel format
    status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::PIXEL_FORMAT_ALLOCATED,
                                      &snap_pixel_format);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGE("Unable to get pixel format");
      return status;
    }
  } else if (buf_des != nullptr) {
    auto error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::WIDTH, &width);
    if (error != SnapError::NONE) {
      ALOGE("Unable to get unaligned width");
      return error;
    }
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::HEIGHT, &height);
    if (error != SnapError::NONE) {
      ALOGE("Unable to get unaligned height");
      return error;
    }
    snap_pixel_format = buf_des->format;
  }
  auto snap_plane_layout = snap_buffer_layout.planes;
  int plane_count = snap_buffer_layout.plane_count;
  gr_plane_layouts->resize(plane_count);
  int bpp = snap_buffer_layout.bpp;

  // For RAW formats update information to meet IMapper5 VTS / Gralloc4 specs
  // Sets sampleIncrementInBits to 0 and component sizeInBits to -1 if sampleIncrementInBits is not
  // divisible by 8. These values aren't valid for formats such as RAW10 and RAW12 which aren't
  // individually packed due to their size not being in multiples of 8-bits
  switch (snap_pixel_format) {
    case SnapPixelFormat::RAW10:
    case SnapPixelFormat::RAW12:
    case SnapPixelFormat::RAW14:
      individually_packed = false;
      [[fallthrough]];
    case SnapPixelFormat::RAW8:
    case SnapPixelFormat::RAW16:
      is_raw = true;
      break;
    default:
      is_raw = false;
  }
  for (int i = 0; i < plane_count; i++) {
    (*gr_plane_layouts)[i].sampleIncrementInBits =
        is_raw && !individually_packed
            ? 0
            : static_cast<int64_t>(snap_plane_layout[i].sample_increment_bits);
    (*gr_plane_layouts)[i].strideInBytes =
        static_cast<int64_t>(snap_plane_layout[i].horizontal_stride_in_bytes);
    (*gr_plane_layouts)[i].totalSizeInBytes =
        static_cast<int64_t>(snap_plane_layout[i].size_in_bytes);
    (*gr_plane_layouts)[i].horizontalSubsampling =
        static_cast<int64_t>(snap_plane_layout[i].horizontal_subsampling);
    (*gr_plane_layouts)[i].verticalSubsampling =
        static_cast<int64_t>(snap_plane_layout[i].vertical_subsampling);
    // Convert horizontal and vertical subsampling into factor (e.g., 1 >> 2 = 0)
    (*gr_plane_layouts)[i].widthInSamples =
        width >> (snap_plane_layout[i].horizontal_subsampling >> 1ull);
    (*gr_plane_layouts)[i].heightInSamples =
        height >> (snap_plane_layout[i].vertical_subsampling >> 1ull);
    (*gr_plane_layouts)[i].offsetInBytes =
        static_cast<int64_t>(snap_plane_layout[i].offset_in_bytes);

    ALOGD_IF(
        enable_logs_,
        "Plane No: %d, sampleIncrementInBits %d, strideInBytes %d, totalSizeInBytes %d, "
        "horizontalSubsampling %d, verticalSubsampling %d, widthInSamples %d,  heightInSamples %d, "
        "offsetInBytes %d",
        i, (*gr_plane_layouts)[i].sampleIncrementInBits, (*gr_plane_layouts)[i].strideInBytes,
        (*gr_plane_layouts)[i].totalSizeInBytes, (*gr_plane_layouts)[i].horizontalSubsampling,
        (*gr_plane_layouts)[i].verticalSubsampling, (*gr_plane_layouts)[i].widthInSamples,
        (*gr_plane_layouts)[i].heightInSamples, (*gr_plane_layouts)[i].offsetInBytes);

    std::vector<GrallocPlaneLayoutComponent> gr_plane_layout_components;
    int snap_component_count = snap_plane_layout[i].component_count;
    auto snap_plane_layout_components = snap_plane_layout[i].components;
    for (int j = 0; j < snap_component_count; j++) {
      GrallocPlaneLayoutComponent gr_plane_layout_component;
      ConvertSnapToGrallocPlaneComponentType(snap_plane_layout_components[j].type,
                                             &gr_plane_layout_component.type);
      gr_plane_layout_component.offsetInBits = snap_plane_layout_components[j].offset_in_bits;
      gr_plane_layout_component.sizeInBits =
          (is_raw && !individually_packed) ? -1 : snap_plane_layout_components[j].size_in_bits;
      (*gr_plane_layouts)[i].components.push_back(gr_plane_layout_component);
    }
  }
  return SnapError::NONE;
}

int GrallocSnapHelper::ConvertGrallocPlaneLayoutToAndroidYCbCr(
    uint64_t base_addr, const std::vector<GrallocPlaneLayout> gr_plane_layouts,
    struct android_ycbcr *outYCbCr) {
  outYCbCr->y = nullptr;
  outYCbCr->cb = nullptr;
  outYCbCr->cr = nullptr;
  outYCbCr->ystride = 0;
  outYCbCr->cstride = 0;
  outYCbCr->chroma_step = 0;
  int next_plane = 0;
  for (const auto &planeLayout : gr_plane_layouts) {
    bool contains_meta = false;
    for (const auto &planeLayoutComponent : planeLayout.components) {
      if (planeLayoutComponent.type.value == qtigralloc::PlaneLayoutComponentType_Meta.value) {
        contains_meta = true;
      }
    }
    if (!contains_meta) {
      for (const auto &planeLayoutComponent : planeLayout.components) {
        auto tmpData =
            base_addr + planeLayout.offsetInBytes + (planeLayoutComponent.offsetInBits / 8);
        uint64_t sampleIncrementInBytes;
        auto type = static_cast<GrallocPlaneLayoutComponentType>(planeLayoutComponent.type.value);
        switch (type) {
          case GrallocPlaneLayoutComponentType::Y: {
            outYCbCr->y = reinterpret_cast<void *>(tmpData);
            outYCbCr->ystride = planeLayout.strideInBytes;
            break;
          }
          case GrallocPlaneLayoutComponentType::CB:
          case GrallocPlaneLayoutComponentType::CR: {
            sampleIncrementInBytes = planeLayout.sampleIncrementInBits / 8;
            outYCbCr->cstride = planeLayout.strideInBytes;
            outYCbCr->chroma_step = sampleIncrementInBytes;
            if (type == GrallocPlaneLayoutComponentType::CB) {
              outYCbCr->cb = reinterpret_cast<void *>(tmpData);
            } else {
              outYCbCr->cr = reinterpret_cast<void *>(tmpData);
            }
            break;
          }
          default:
            break;
        }
      }
    }
    // Interlaced UBWC formats have 8 Planes
    if (gr_plane_layouts.size() == 8) {
      // Planes 0-3 fills top field in android_ycbcr & planes 4-7 fills bottom field
      if (next_plane == 3) {
        outYCbCr = outYCbCr + 1;
      }
    }
    next_plane++;
  }
  ALOGD_IF(
      enable_logs_,
      "%s: base_addr %d, outYCbCr->y %d, outYCbCr->cb %d, outYCbCr->cr %d, outYCbCr->ystride %d, "
      "outYCbCr->cstride %d, outYCbCr->chroma_step %d",
      __FUNCTION__, base_addr, outYCbCr->y, outYCbCr->cb, outYCbCr->cr, outYCbCr->ystride,
      outYCbCr->cstride, outYCbCr->chroma_step);
  return SnapError::NONE;
}

bool GrallocSnapHelper::IsBufferImported(native_handle_t *gr_hnd) {
  if (gr_hnd == nullptr) {
    ALOGE("Invalid gralloc handle");
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    SnapHandle *hnd = handles_map_.at(gr_hnd);
    if (hnd != nullptr) {
      ALOGI("Gralloc handle %p has been imported", gr_hnd);
      return true;
    }
  }
  ALOGE("Gralloc handle %p has not been imported", gr_hnd);
  return false;
}

int GrallocSnapHelper::GetCustomDimensions(native_handle_t *gr_hnd, int *stride, int *height) {
  if (gr_hnd == nullptr) {
    ALOGE("Invalid gralloc handle");
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  ALOGE("GetCustomDimensions not implemented for Snapalloc");
  return SnapError::UNSUPPORTED;
}

int GrallocSnapHelper::ConvertSnapPlaneLayoutComponentToGralloc(SnapPlaneLayout *layout) {
  int gralloc_component = 0;
  for (int i = 0; i < layout->component_count; i++) {
    if (snap_to_gralloc_plane_layout_component_.find(layout->components[i].type) !=
        snap_to_gralloc_plane_layout_component_.end()) {
      gralloc_component |= snap_to_gralloc_plane_layout_component_.at(layout->components[i].type);
    }
  }
  return gralloc_component;
}

int GrallocSnapHelper::GetFormatLayout(gralloc::BufferInfo gr_desc, void *out, uint32_t *size,
                                       int interlaced) {
  if (!IsSnapAllocEnabled()) {
    ALOGW("SnapAlloc is disabled");
    return SnapError::UNSUPPORTED;
  }

  SnapDescriptor snap_desc = {};
  auto err = GetSnapDescriptor(gr_desc, snap_desc);
  if (err) {
    return err;
  }
  if (interlaced) {
    static SnapKeyValuePair modifier = {.key = "interlaced", .value = static_cast<uint64_t>(1)};
    snap_desc.additionalOptions.emplace_back(modifier);
  }
  SnapBufferLayout snap_plane_layouts;
  auto status = snapmapper_->GetFromBufferDescriptor(snap_desc, SnapMetadataType::PLANE_LAYOUTS,
                                                     &snap_plane_layouts);

  if (status == SnapError::NONE) {
    unsigned int alloc_size;
    status = snapmapper_->GetFromBufferDescriptor(snap_desc, SnapMetadataType::ALLOCATION_SIZE,
                                                  &alloc_size);
    *size = static_cast<uint32_t>(alloc_size);

    int64_t ubwc_enabled_in_snap;
    status = snapmapper_->GetFromBufferDescriptor(snap_desc, SnapMetadataType::IS_UBWC,
                                                  &ubwc_enabled_in_snap);

    std::vector<gralloc::PlaneLayoutInfo> *plane_info =
        reinterpret_cast<std::vector<gralloc::PlaneLayoutInfo> *>(out);
    // TODO: need to properly filter out meta plane for UBWC RGBA case - hacking for now to unblock
    if ((IsUncompressedRGBFormat(static_cast<int>(snap_desc.format)) ||
         IsCompressedRGBFormat(static_cast<int>(snap_desc.format))) &&
        ubwc_enabled_in_snap) {
      plane_info->resize(snap_plane_layouts.plane_count / 2);
    } else {
      plane_info->resize(snap_plane_layouts.plane_count);
    }
    for (int i = 0; i < plane_info->size(); i++) {
      (*plane_info)[i].component = static_cast<gralloc::PlaneComponent>(
          ConvertSnapPlaneLayoutComponentToGralloc(&snap_plane_layouts.planes[i]));
      (*plane_info)[i].h_subsampling = snap_plane_layouts.planes[i].horizontal_subsampling >> 1ull;
      (*plane_info)[i].v_subsampling = snap_plane_layouts.planes[i].vertical_subsampling >> 1ull;
      (*plane_info)[i].offset = snap_plane_layouts.planes[i].offset_in_bytes;
      (*plane_info)[i].step = snap_plane_layouts.planes[i].sample_increment_bits / 8;
      (*plane_info)[i].stride = static_cast<int>(
          static_cast<float>(snap_plane_layouts.planes[i].horizontal_stride_in_bytes) /
          (static_cast<float>(snap_plane_layouts.planes[i].sample_increment_bits) / 8.0f));
      (*plane_info)[i].stride_bytes = snap_plane_layouts.planes[i].horizontal_stride_in_bytes;
      (*plane_info)[i].scanlines = snap_plane_layouts.planes[i].scanlines;
      (*plane_info)[i].size = snap_plane_layouts.planes[i].size_in_bytes;
    }

    return SnapError::NONE;
  } else {
    ALOGE("%s: Failed to get plane layouts from SnapAlloc. Error code: %d", __FUNCTION__, status);
  }

  return status;
}

int GrallocSnapHelper::ConvertSnapDataspaceToGrallocDataspace(SnapDataspace &snap_dataspace,
                                                              GrallocDataspace *gr_dataspace) {
  GrallocDataspace primaries, transfer, range = GrallocDataspace::UNKNOWN;
  switch (snap_dataspace.colorPrimaries) {
    case QtiColorPrimaries_BT709_5:
      primaries = GrallocDataspace::STANDARD_BT709;
      break;
    case QtiColorPrimaries_BT470_6M:
      primaries = GrallocDataspace::STANDARD_BT470M;
      break;
    case QtiColorPrimaries_BT601_6_625:
      primaries = GrallocDataspace::STANDARD_BT601_625;
      break;
    case QtiColorPrimaries_BT601_6_525:
      primaries = GrallocDataspace::STANDARD_BT601_525;
      break;
    case QtiColorPrimaries_GenericFilm:
      primaries = GrallocDataspace::STANDARD_FILM;
      break;
    case QtiColorPrimaries_BT2020:
      primaries = GrallocDataspace::STANDARD_BT2020;
      break;
    case QtiColorPrimaries_AdobeRGB:
      primaries = GrallocDataspace::STANDARD_ADOBE_RGB;
      break;
    case QtiColorPrimaries_DCIP3:
      primaries = GrallocDataspace::STANDARD_DCI_P3;
      break;
    default:
      ALOGV("%s: Failed to convert primaries %d", __FUNCTION__, snap_dataspace.colorPrimaries);
      return SnapError::BAD_VALUE;
  }

  switch (snap_dataspace.transfer) {
    case QtiTransfer_sRGB:
      transfer = GrallocDataspace::TRANSFER_SRGB;
      break;
    case QtiTransfer_Gamma2_2:
      transfer = GrallocDataspace::TRANSFER_GAMMA2_2;
      break;
    case QtiTransfer_Gamma2_8:
      transfer = GrallocDataspace::TRANSFER_GAMMA2_8;
      break;
    case QtiTransfer_SMPTE_170M:
      transfer = GrallocDataspace::TRANSFER_SMPTE_170M;
      break;
    case QtiTransfer_Linear:
      transfer = GrallocDataspace::TRANSFER_LINEAR;
      break;
    case QtiTransfer_HLG:
      transfer = GrallocDataspace::TRANSFER_HLG;
      break;
    case QtiTransfer_SMPTE_ST2084:
      transfer = GrallocDataspace::TRANSFER_ST2084;
      break;
    default:
      ALOGV("%s: Failed to convert transfer %d", __FUNCTION__, snap_dataspace.transfer);
      return SnapError::BAD_VALUE;
  }

  switch (snap_dataspace.range) {
    case QtiRange_Full:
      range = GrallocDataspace::RANGE_FULL;
      break;
    case QtiRange_Limited:
      range = GrallocDataspace::RANGE_LIMITED;
      break;
    case QtiRange_Extended:
      range = GrallocDataspace::RANGE_EXTENDED;
      break;
    default:
      ALOGV("%s: Failed to convert range %d", __FUNCTION__, snap_dataspace.range);
      return SnapError::BAD_VALUE;
  }

  *gr_dataspace = (GrallocDataspace)((uint32_t)primaries | (uint32_t)transfer | (uint32_t)range);
  return SnapError::NONE;
}

// Converts Dataspace to Colorspace
int GrallocSnapHelper::GetColorSpaceFromDataspaceMetadata(SnapDataspace snap_dataspace,
                                                          uint32_t *color_space) {
  int err = 0;
  switch (snap_dataspace.colorPrimaries) {
    case QtiColorPrimaries_BT709_5: {
      *color_space = HAL_CSC_ITU_R_709;
      break;
    }
    case QtiColorPrimaries_BT601_6_625:
    case QtiColorPrimaries_BT601_6_525: {
      *color_space =
          ((static_cast<bool>(snap_dataspace.range)) ? HAL_CSC_ITU_R_601_FR : HAL_CSC_ITU_R_601);
      break;
    }
    case QtiColorPrimaries_BT2020: {
      *color_space =
          ((static_cast<bool>(snap_dataspace.range)) ? HAL_CSC_ITU_R_2020_FR : HAL_CSC_ITU_R_2020);
      break;
    }
    default: {
      err = -1;
      *color_space = 0;
      ALOGW("Unknown Color primary = %d", snap_dataspace.colorPrimaries);
      break;
    }
  }
  return err;
}

int GrallocSnapHelper::GetSnapDataspaceMetadataFromColorSpace(uint32_t color_space,
                                                              SnapDataspace *snap_dataspace) {
  snap_dataspace->transfer = QtiTransfer_sRGB;
  switch (color_space) {
    case HAL_CSC_ITU_R_601: {
      snap_dataspace->colorPrimaries = QtiColorPrimaries_BT601_6_525;
      snap_dataspace->range = QtiRange_Limited;
      break;
    }
    case HAL_CSC_ITU_R_601_FR: {
      snap_dataspace->colorPrimaries = QtiColorPrimaries_BT601_6_525;
      snap_dataspace->range = QtiRange_Full;
      break;
    }
    case HAL_CSC_ITU_R_709: {
      snap_dataspace->colorPrimaries = QtiColorPrimaries_BT709_5;
      snap_dataspace->range = QtiRange_Limited;
      break;
    }
    case HAL_CSC_ITU_R_709_FR: {
      snap_dataspace->colorPrimaries = QtiColorPrimaries_BT709_5;
      snap_dataspace->range = QtiRange_Full;
      break;
    }
    case HAL_CSC_ITU_R_2020: {
      snap_dataspace->colorPrimaries = QtiColorPrimaries_BT2020;
      snap_dataspace->range = QtiRange_Limited;
      break;
    }
    case HAL_CSC_ITU_R_2020_FR: {
      snap_dataspace->colorPrimaries = QtiColorPrimaries_BT2020;
      snap_dataspace->range = QtiRange_Full;
      break;
    }
    default:
      ALOGE("Cannot convert ColorSpace_t %d to SnapDataspace", color_space);
      return SnapError::BAD_VALUE;
  }
  return SnapError::NONE;
}

void GrallocSnapHelper::ConvertSnapToGrallocPlaneComponentType(
    SnapPlaneLayoutComponentType snap_component_type, GrallocExtendableType *gr_component_type) {
  if (snap_to_gralloc_extendable_plane_layout_component_type_.find(snap_component_type) !=
      snap_to_gralloc_extendable_plane_layout_component_type_.end()) {
    *gr_component_type =
        snap_to_gralloc_extendable_plane_layout_component_type_.at(snap_component_type);
  }
}

int GrallocSnapHelper::ConvertGrallocDataspaceToSnapDataspace(GrallocDataspace gr_dataspace,
                                                              SnapDataspace *snap_dataspace) {
  SnapDataspace dataspace;
  uint32_t primaries = (uint32_t)gr_dataspace & (uint32_t)GrallocDataspace::STANDARD_MASK;
  uint32_t transfer = (uint32_t)gr_dataspace & (uint32_t)GrallocDataspace::TRANSFER_MASK;
  uint32_t range = (uint32_t)gr_dataspace & (uint32_t)GrallocDataspace::RANGE_MASK;
  switch (primaries) {
    case (uint32_t)GrallocDataspace::STANDARD_BT709:
      dataspace.colorPrimaries = QtiColorPrimaries_BT709_5;
      break;
    case (uint32_t)GrallocDataspace::STANDARD_BT470M:
      dataspace.colorPrimaries = QtiColorPrimaries_BT470_6M;
      break;
    case (uint32_t)GrallocDataspace::STANDARD_BT601_625:
    case (uint32_t)GrallocDataspace::STANDARD_BT601_625_UNADJUSTED:
      dataspace.colorPrimaries = QtiColorPrimaries_BT601_6_625;
      break;
    case (uint32_t)GrallocDataspace::STANDARD_BT601_525:
    case (uint32_t)GrallocDataspace::STANDARD_BT601_525_UNADJUSTED:
      dataspace.colorPrimaries = QtiColorPrimaries_BT601_6_525;
      break;
    case (uint32_t)GrallocDataspace::STANDARD_FILM:
      dataspace.colorPrimaries = QtiColorPrimaries_GenericFilm;
      break;
    case (uint32_t)GrallocDataspace::STANDARD_BT2020:
      dataspace.colorPrimaries = QtiColorPrimaries_BT2020;
      break;
    case (uint32_t)GrallocDataspace::STANDARD_ADOBE_RGB:
      dataspace.colorPrimaries = QtiColorPrimaries_AdobeRGB;
      break;
    case (uint32_t)GrallocDataspace::STANDARD_DCI_P3:
      dataspace.colorPrimaries = QtiColorPrimaries_DCIP3;
      break;
    default:
      ALOGV("%s: Failed to convert primaries %d", __FUNCTION__, primaries);
      return SnapError::BAD_VALUE;
  }

  switch (transfer) {
    case (uint32_t)GrallocDataspace::TRANSFER_SRGB:
      dataspace.transfer = QtiTransfer_sRGB;
      break;
    case (uint32_t)GrallocDataspace::TRANSFER_GAMMA2_2:
      dataspace.transfer = QtiTransfer_Gamma2_2;
      break;
    case (uint32_t)GrallocDataspace::TRANSFER_GAMMA2_8:
      dataspace.transfer = QtiTransfer_Gamma2_8;
      break;
    case (uint32_t)GrallocDataspace::TRANSFER_SMPTE_170M:
      dataspace.transfer = QtiTransfer_SMPTE_170M;
      break;
    case (uint32_t)GrallocDataspace::TRANSFER_LINEAR:
      dataspace.transfer = QtiTransfer_Linear;
      break;
    case (uint32_t)GrallocDataspace::TRANSFER_HLG:
      dataspace.transfer = QtiTransfer_HLG;
      break;
    case (uint32_t)GrallocDataspace::TRANSFER_ST2084:
      dataspace.transfer = QtiTransfer_SMPTE_ST2084;
      break;
    default:
      ALOGV("%s: Failed to convert transfer %d", __FUNCTION__, transfer);
      return SnapError::BAD_VALUE;
  }

  switch (range) {
    case (uint32_t)GrallocDataspace::RANGE_FULL:
      dataspace.range = QtiRange_Full;
      break;
    case (uint32_t)GrallocDataspace::RANGE_LIMITED:
      dataspace.range = QtiRange_Limited;
      break;
    case (uint32_t)GrallocDataspace::RANGE_EXTENDED:
      dataspace.range = QtiRange_Extended;
      break;
    default:
      ALOGV("%s: Failed to convert range %d", __FUNCTION__, range);
      return SnapError::BAD_VALUE;
  }
  snap_dataspace->colorPrimaries = dataspace.colorPrimaries;
  snap_dataspace->transfer = dataspace.transfer;
  snap_dataspace->range = dataspace.range;
  return SnapError::NONE;
}

// Gralloc <-> Snapalloc conversion helper functions
SnapError GrallocSnapHelper::GetSnapFormat(int hal_format, uint64_t usage,
                                           SnapFormatDescriptor *snap_fmt_desc) {
  if (gralloc_ubwc_to_snap_format_.find(hal_format) != gralloc_ubwc_to_snap_format_.end()) {
    ALOGW(
        "%s: Explicit UBWC formats such as %d are no longer supported, please switch to using base "
        "/ linear formats + UBWC usage bits",
        __FUNCTION__, static_cast<int>(hal_format));
    *snap_fmt_desc = gralloc_ubwc_to_snap_format_.at(hal_format);
  } else if (gralloc_to_snap_format_.find(hal_format) != gralloc_to_snap_format_.end()) {
    *snap_fmt_desc = gralloc_to_snap_format_.at(hal_format);
  } else if (std::find(unsupported_formats.begin(), unsupported_formats.end(), hal_format) !=
             unsupported_formats.end()) {
    ALOGW("%s:: Unsupported format - %d", __FUNCTION__, hal_format);
    return SnapError::UNSUPPORTED;
  } else {
    ALOGW("%s: No map for gralloc format %d to snap format", __FUNCTION__, hal_format);
    return SnapError::BAD_VALUE;
  }

  ALOGD_IF(enable_logs_, "%s gralloc format %d snap format %d modifier %d", __FUNCTION__,
           hal_format, snap_fmt_desc->format, snap_fmt_desc->modifier);
  return SnapError::NONE;
}

SnapUsage GrallocSnapHelper::GetSnapUsage(uint64_t usage, int hal_format) {
  SnapUsage snap_usage = CPU_READ_NEVER;

  if (gralloc_ubwc_to_snap_format_.find(hal_format) != gralloc_ubwc_to_snap_format_.end()) {
    snap_usage |= SnapUsage::QTI_ALLOC_UBWC;
    // If explicit UBWC format, do not set CPU flags
    // TODO: revisit this once explicit UBWC formats deprecated
  } else {
    uint64_t cpu_read_usage = static_cast<uint64_t>(usage & SnapUsage::CPU_READ_MASK);
    uint64_t cpu_write_usage = static_cast<uint64_t>(usage & SnapUsage::CPU_WRITE_MASK);

    if (cpu_read_usage != 0 || cpu_write_usage != 0) {
      for (auto entry : cpu_gralloc_to_snap_usage_) {
        if (cpu_read_usage == entry.first || cpu_write_usage == entry.first) {
          snap_usage |= entry.second;
        }
      }
    }
  }
  for (auto entry : gralloc_to_snap_usage_) {
    // If bit is set in gralloc usage, set equivalent bit in snap usage
    if (usage & entry.first) {
      snap_usage |= entry.second;
    }
  }

  return snap_usage;
}

SnapUBWCVersion GrallocSnapHelper::GetSnapUBWCVersion(UBWC_Version version) {
  if (gralloc_to_snap_ubwc_version_.find(version) != gralloc_to_snap_ubwc_version_.end()) {
    return gralloc_to_snap_ubwc_version_.at(version);
  }
  return UBWC_VERSION_MAX;
}

UBWC_Version GrallocSnapHelper::GetGrallocUBWCVersion(SnapUBWCVersion version) {
  if (snap_to_gralloc_ubwc_version_.find(version) != snap_to_gralloc_ubwc_version_.end()) {
    return snap_to_gralloc_ubwc_version_.at(version);
  }

  return UBWC_MAX_VERSION;
}

SnapError GrallocSnapHelper::ValidateGrallocUsage(uint64_t gralloc_usage) {
  // Bits 33-47 must be zero and are reserved for future versions
  uint64_t future_usage_bit_mask = static_cast<uint64_t>(0xFFFE00000000);
  if ((gralloc_usage & future_usage_bit_mask) != 0) {
    return SnapError::BAD_VALUE;
  }
  return SnapError::NONE;
}

SnapError GrallocSnapHelper::GetSnapDescriptor(gralloc::BufferDescriptor gr_desc,
                                               SnapDescriptor &snap_desc) {
  SnapFormatDescriptor snap_fmt_desc;
  auto err = GetSnapFormat(gr_desc.GetFormat(), gr_desc.GetUsage(), &snap_fmt_desc);
  if (err) {
    ALOGW("%s: Error while getting snap descriptor - gr_format - %d", __FUNCTION__,
          gr_desc.GetFormat());
    return err;
  } else {
    auto name_length = std::min(gr_desc.GetName().size(), static_cast<size_t>(MAX_NAME_LEN - 1));
    memcpy(snap_desc.name, gr_desc.GetName().data(), name_length);
    snap_desc.format = snap_fmt_desc.format;
    err = ValidateGrallocUsage(gr_desc.GetUsage());
    if (err) {
      ALOGW("%s: Error while getting snap descriptor - Unknown Usage bit set - %lu", __FUNCTION__,
            gr_desc.GetUsage());
      return err;
    }
    snap_desc.usage = GetSnapUsage(gr_desc.GetUsage(), gr_desc.GetFormat());
    snap_desc.width = gr_desc.GetWidth();
    snap_desc.height = gr_desc.GetHeight();
    snap_desc.layerCount = gr_desc.GetLayerCount();
    snap_desc.reservedSize = gr_desc.GetReservedSize();
    SnapKeyValuePair modifier = {.key = "pixel_format_modifier",
                                 .value = static_cast<uint64_t>(snap_fmt_desc.modifier)};
    snap_desc.additionalOptions.emplace_back(modifier);
    ALOGD_IF(enable_logs_,
             "%s gr format %d gr usage %lu snap format %d snap modifier %d snap "
             "usage %lu name from gralloc descriptor %s snap_desc %s",
             __FUNCTION__, gr_desc.GetFormat(), gr_desc.GetUsage(), snap_fmt_desc.format,
             snap_fmt_desc.modifier, snap_desc.usage, gr_desc.GetName().c_str(), snap_desc.name);
  }
  return SnapError::NONE;
}

SnapError GrallocSnapHelper::GetSnapDescriptor(gralloc::BufferInfo gr_desc,
                                               SnapDescriptor &snap_desc) {
  SnapFormatDescriptor snap_fmt_desc;
  auto err = GetSnapFormat(gr_desc.format, gr_desc.usage, &snap_fmt_desc);
  if (err) {
    ALOGW("%s: Error while getting SnapDescriptor - gr_format %d", __FUNCTION__, gr_desc.format);
    return err;
  } else {
    snap_desc.format = snap_fmt_desc.format;
    err = ValidateGrallocUsage(gr_desc.usage);
    if (err) {
      ALOGW("%s: Error while getting snap descriptor - Unknown Usage bit set - %lu", __FUNCTION__,
            gr_desc.usage);
      return err;
    }
    snap_desc.usage = GetSnapUsage(gr_desc.usage, gr_desc.format);
    snap_desc.width = gr_desc.width;
    snap_desc.height = gr_desc.height;
    snap_desc.layerCount = gr_desc.layer_count;
    SnapKeyValuePair modifier = {.key = "pixel_format_modifier",
                                 .value = static_cast<uint64_t>(snap_fmt_desc.modifier)};
    snap_desc.additionalOptions.emplace_back(modifier);

    ALOGD_IF(enable_logs_,
             "%s gr format %d gr usage %lu snap format %d snap modifier %d snap "
             "usage %lu",
             __FUNCTION__, gr_desc.format, gr_desc.usage, snap_fmt_desc.format,
             snap_fmt_desc.modifier, snap_desc.usage);
  }
  return SnapError::NONE;
}

int GrallocSnapHelper::GetGrallocFormat(SnapFormatDescriptor snap_fmt_desc, SnapUsage usage,
                                        int *gr_format) {
  if (snap_to_gralloc_format_.find(snap_fmt_desc) != snap_to_gralloc_format_.end()) {
    *gr_format = snap_to_gralloc_format_.at(snap_fmt_desc);
  } else {
    ALOGE("%s: No map for format: 0x%x, modifier %d", __FUNCTION__, snap_fmt_desc.format,
          snap_fmt_desc.modifier);
    return SnapError::BAD_VALUE;
  }

  ALOGD_IF(enable_logs_, "%s snap format %d modifier %d gralloc format %d", __FUNCTION__,
           snap_fmt_desc.format, snap_fmt_desc.modifier, *gr_format);
  return SnapError::NONE;
}

int GrallocSnapHelper::GetSnapFlatFormat(SnapFormatDescriptor snap_fmt_desc, SnapUsage usage,
                                         SnapPixelFormat *snap_format) {
  if (snap_to_flat_format_.find(snap_fmt_desc) != snap_to_flat_format_.end()) {
    *snap_format = snap_to_flat_format_.at(snap_fmt_desc);
  } else if ((usage & SnapUsage::QTI_ALLOC_UBWC) &&
             (snap_to_flat_ubwc_format_.find(snap_fmt_desc) != snap_to_flat_ubwc_format_.end())) {
    *snap_format = snap_to_flat_ubwc_format_.at(snap_fmt_desc);
  } else {
    ALOGW("%s: No map for format: 0x%x", __FUNCTION__, snap_fmt_desc.format);
    return SnapError::BAD_VALUE;
  }

  ALOGD_IF(enable_logs_, "%s snap format %d modifier %d flat format %d", __FUNCTION__,
           snap_fmt_desc.format, snap_fmt_desc.modifier, *snap_format);
  return SnapError::NONE;
}

uint64_t GrallocSnapHelper::GetGrallocUsage(SnapUsage snap_usage) {
  uint64_t gralloc_usage = 0;

  uint64_t cpu_read_usage =
      static_cast<uint64_t>(snap_usage) & static_cast<uint64_t>(SnapUsage::CPU_READ_MASK);
  uint64_t cpu_write_usage =
      static_cast<uint64_t>(snap_usage) & static_cast<uint64_t>(SnapUsage::CPU_WRITE_MASK);

  if (cpu_read_usage != 0 || cpu_write_usage != 0) {
    for (auto entry : cpu_snap_to_gralloc_usage_) {
      if (cpu_read_usage == static_cast<uint64_t>(entry.first) ||
          cpu_write_usage == static_cast<uint64_t>(entry.first)) {
        gralloc_usage |= entry.second;
      }
    }
  }

  for (auto entry : snap_to_gralloc_usage_) {
    // If bit is set in gralloc usage, set equivalent bit in snap usage
    if (snap_usage & entry.first) {
      gralloc_usage |= entry.second;
    }
  }

  return gralloc_usage;
}

// LEGACY (MAPPER4 COMPATIBLE) IMPLEMENTATION
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
GrallocSnapHelperLegacy *GrallocSnapHelperLegacy::s_instance = nullptr;

GrallocSnapHelperLegacy *GrallocSnapHelperLegacy::GetInstance() {
  static mutex s_lock;
  lock_guard<mutex> obj(s_lock);
  if (!s_instance) {
    s_instance = new GrallocSnapHelperLegacy();
  }

  return s_instance;
}

GrallocSnapHelperLegacy::GrallocSnapHelperLegacy() {
  snap_alloc_enable_ = false;

  char property[PROPERTY_VALUE_MAX];
  property_get(ENABLE_SNAPALLOC_PROP, property, "0");
  if (!(strncmp(property, "1", PROPERTY_VALUE_MAX)) ||
      !(strncmp(property, "true", PROPERTY_VALUE_MAX))) {
    snap_alloc_enable_ = true;
  }

  if (!snap_alloc_enable_) {
    ALOGD("%s: SnapAlloc is disabled", __FUNCTION__);
    return;
  }

  const std::string snapalloc_lib_name = "vendor.qti.hardware.display.snapalloc-impl.so";
  snap_impl_lib_ = ::dlopen(snapalloc_lib_name.c_str(), RTLD_NOW);
  if (!snap_impl_lib_) {
    ALOGE("%s: Dlopen error for snapalloc impl: %s", __FUNCTION__, dlerror());
    snap_alloc_enable_ = false;
    return;
  }

  *reinterpret_cast<void **>(&LINK_FETCH_ISnapAlloc) = ::dlsym(snap_impl_lib_, "FETCH_ISnapAlloc");
  if (LINK_FETCH_ISnapAlloc) {
    snapallocator_ = LINK_FETCH_ISnapAlloc(&debugger_impl_);
  }

  if (!LINK_FETCH_ISnapAlloc || snapallocator_ == nullptr) {
    ALOGE("%s: Failed to link FETCH_ISnapAlloc - %s", __FUNCTION__, strerror(errno));
    snap_alloc_enable_ = false;
    return;
  }

  *reinterpret_cast<void **>(&LINK_FETCH_ISnapMapper) =
      ::dlsym(snap_impl_lib_, "FETCH_ISnapMapper");
  if (LINK_FETCH_ISnapMapper) {
    snapmapper_ = LINK_FETCH_ISnapMapper(&debugger_impl_);
  }

  if (!LINK_FETCH_ISnapMapper || snapmapper_ == nullptr) {
    ALOGE("%s: Failed to link FETCH_ISnapAlloc - %s", __FUNCTION__, strerror(errno));
    snap_alloc_enable_ = false;
    return;
  }

  // Inverse of the hard coded maps to translate in the other direction
  for (auto entry : snap_to_gralloc_format_) {
    gralloc_to_snap_format_.emplace(std::make_pair(entry.second, entry.first));
  }
  for (auto entry : snap_to_gralloc_ubwc_format_) {
    gralloc_ubwc_to_snap_format_.emplace(std::make_pair(entry.second, entry.first));
  }
  for (auto entry : gralloc_to_snap_usage_) {
    snap_to_gralloc_usage_.emplace(std::make_pair(entry.second, entry.first));
  }
  for (auto entry : cpu_gralloc_to_snap_usage_) {
    cpu_snap_to_gralloc_usage_.emplace(std::make_pair(entry.second, entry.first));
  }
  for (auto entry : gralloc_to_snap_ubwc_version_) {
    snap_to_gralloc_ubwc_version_.emplace(std::make_pair(entry.second, entry.first));
  }
}

GrallocSnapHelperLegacy::~GrallocSnapHelperLegacy() {
  std::lock_guard<std::mutex> lock(map_lock_);

  for (auto &entry : handles_map_) {
    native_handle_delete(entry.first);
  }
  handles_map_.clear();

  if (snap_impl_lib_)
    dlclose(snap_impl_lib_);
}

int GrallocSnapHelperLegacy::Allocate(
    gralloc::BufferDescriptor gr_desc, int buffer_count,
    aidl::android::hardware::graphics::allocator::AllocationResult *result) {
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  if (result == nullptr) {
    ALOGE("%s: Invalid AllocationResult pointer passed in", __FUNCTION__);
    return SnapError::BAD_VALUE;
  }

  SnapDescriptor snap_desc = {};
  auto err = GetSnapDescriptor(gr_desc, snap_desc);
  if (err) {
    ALOGE("%s: Failed to get Snap Descriptor", __FUNCTION__);
    return err;
  }
  SnapAllocationResult snap_result;

  auto status = snapallocator_->Allocate(snap_desc, buffer_count, &snap_result);

  if (status != SnapError::NONE) {
    ALOGE("%s: Failed to allocate via SnapAlloc. Error code: %d", __FUNCTION__, status);
    return status;
  }

  result->stride = snap_result.stride;
  result->buffers.resize(snap_result.handles.size());
  for (int i = 0; i < snap_result.handles.size(); i++) {
    result->buffers[i] = AIDLNativeHandleFromSnapHandle(snap_result.handles[i], false);
    snapmapper_->Release(*snap_result.handles[i]);
  }

  return status;
}

int GrallocSnapHelperLegacy::Import(native_handle_t *gr_hnd) {
  if (gr_hnd == nullptr) {
    ALOGE("%s: Invalid gralloc handle", __FUNCTION__);
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }
  std::lock_guard<std::mutex> lock(map_lock_);

  SnapError status = SnapError::BAD_BUFFER;
  if (handles_map_.find(gr_hnd) == handles_map_.end()) {
    SnapHandle *handle = SnapHandleFromCNativeHandle(const_cast<native_handle_t *>(gr_hnd), false);
    if (handle != nullptr) {
      auto status = snapmapper_->Retain(*handle);
      if (status == SnapError::NONE) {
        // Maintain map so that native_handle_t doesn't need to be duped during calls after import
        handles_map_.emplace(std::make_pair(gr_hnd, handle));
        ALOGD_IF(enable_logs_, "%s - handles_map_.size() %d after emplace into map", __FUNCTION__,
                 handles_map_.size());
        return SnapError::NONE;
      } else {
        ALOGE("%s: Failed to import via SnapAlloc. Error code: %d", __FUNCTION__, status);
        return status;
      }
    } else {
      ALOGE("%s: Failed to create snap handle from native_handle_t %p", __FUNCTION__, gr_hnd);
      return status;
    }
  }

  // Handle already in map
  return SnapError::NONE;
}

int GrallocSnapHelperLegacy::Free(native_handle_t *gr_hnd) {
  if (gr_hnd == nullptr) {
    ALOGE("%s: Invalid gralloc handle", __FUNCTION__);
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }
  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    auto status = snapmapper_->Release(*hnd);
    if (status == SnapError::NONE || status == SnapError::BUF_NOT_FREED) {
      // Only free fds if SnapHandle has been freed
      if (status == SnapError::NONE) {
        handles_map_.erase(gr_hnd);
        native_handle_close(gr_hnd);
        native_handle_delete(gr_hnd);
      }
      return SnapError::NONE;
    } else {
      ALOGE("%s: Failed to free via SnapAlloc. Error code: %d", __FUNCTION__, status);
    }
  }
  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelperLegacy::Lock(native_handle_t *gr_hnd, uint64_t gr_usage,
                                  CropRectangle_t gr_access_region, int fence_fd,
                                  uint64_t *base_addr) {
  if (gr_hnd == nullptr) {
    ALOGE("%s: Invalid gralloc handle", __FUNCTION__);
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    // TODO: Get pixel format requested here to determine if it's explicit UBWC format
    SnapUsage usage = GetSnapUsage(gr_usage, 0);
    SnapRect access_region = {.left = gr_access_region.left,
                              .top = gr_access_region.top,
                              .right = gr_access_region.right,
                              .bottom = gr_access_region.bottom};

    SnapFence acquire_fence;
    acquire_fence.fence_fd = dup(fence_fd);
    SnapAddress ret_addr;

    auto status = snapmapper_->Lock(*hnd, static_cast<SnapUsage>(usage), access_region,
                                    acquire_fence, &ret_addr);
    if (status == SnapError::NONE) {
      *base_addr = ret_addr.addressPointer;
      return SnapError::NONE;
    } else {
      ALOGE("%s: Failed to lock via SnapAlloc. Error code: %d", __FUNCTION__, status);
      return status;
    }
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }

  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelperLegacy::Unlock(native_handle_t *gr_hnd, void *in_fence) {
  if (gr_hnd == nullptr) {
    ALOGE("%s: Invalid gralloc handle", __FUNCTION__);
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    SnapFence release_fence;
    auto status = snapmapper_->Unlock(*hnd, &release_fence);
    if (status == SnapError::NONE) {
      in_fence = nullptr;
    } else {
      ALOGE("%s: Failed to unlock via SnapAlloc. Error code: %d", __FUNCTION__, status);
    }
    return status;
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }

  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelperLegacy::ValidateBufferSize(native_handle_t *gr_hnd,
                                                gralloc::BufferInfo gr_desc) {
  if (gr_hnd == nullptr) {
    ALOGE("%s: Invalid gralloc handle", __FUNCTION__);
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    SnapDescriptor snap_desc = {};
    auto err = GetSnapDescriptor(gr_desc, snap_desc);
    if (err) {
      return err;
    }
    auto status = snapmapper_->ValidateBufferSize(*hnd, snap_desc);
    if (status != SnapError::NONE) {
      ALOGE("%s: Failed to validate buffer size via SnapAlloc. Error code: %d", __FUNCTION__,
            status);
    }
    return status;
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }

  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelperLegacy::FlushLockedBuffer(native_handle_t *gr_hnd) {
  if (gr_hnd == nullptr) {
    ALOGE("%s: Invalid gralloc handle", __FUNCTION__);
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    auto status = snapmapper_->FlushLockedBuffer(*hnd);
    if (status != SnapError::NONE) {
      ALOGE("%s: Failed to flush locked buffer via SnapAlloc. Error code: %d", __FUNCTION__,
            status);
    }
    return status;
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }

  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelperLegacy::RereadLockedBuffer(native_handle_t *gr_hnd) {
  if (gr_hnd == nullptr) {
    ALOGE("%s: Invalid gralloc handle", __FUNCTION__);
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    auto status = snapmapper_->RereadLockedBuffer(*hnd);
    if (status != SnapError::NONE) {
      ALOGE("%s: Failed to reread locked buffer via SnapAlloc. Error code: %d", __FUNCTION__,
            status);
    }
    return status;
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }

  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelperLegacy::GetReservedRegion(native_handle_t *gr_hnd, void **reserved_region,
                                               uint64_t *reserved_region_size) {
  if (gr_hnd == nullptr) {
    ALOGE("%s: Invalid gralloc handle", __FUNCTION__);
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    SnapReservedRegion snap_reserved_region;
    auto status =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::RESERVED_REGION, &snap_reserved_region);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGE("%s: Unable to get reserved region from snap", __FUNCTION__);
      return status;
    }
    *reserved_region =
        reinterpret_cast<void *>(snap_reserved_region.reserved_region_addr.addressPointer);
    *reserved_region_size = static_cast<uint64_t>(snap_reserved_region.size);
    return SnapError::NONE;
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }

  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelperLegacy::IsSupported(gralloc::BufferDescriptor gr_desc, bool *is_supported) {
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  SnapDescriptor snap_desc = {};
  auto err = GetSnapDescriptor(gr_desc, snap_desc);
  if (err) {
    return err;
  }
  auto status = snapallocator_->IsSupported(snap_desc, is_supported);
  if (status != SnapError::NONE) {
    ALOGE("%s: Failed to check if descriptor is supported via SnapAlloc. Error code: %d",
          __FUNCTION__, status);
  }

  return status;
}

SnapError GrallocSnapHelperLegacy::CheckMetadataSet(SnapMetadataType type, SnapError status,
                                                    bool check_metadata_set) {
  if (status) {
    if ((status != SnapError::METADATA_NOT_SET) ||
        (status == SnapError::METADATA_NOT_SET && check_metadata_set)) {
      ALOGW("%s - Error %d while getting the metadata type %d from snapmapper", __FUNCTION__,
            static_cast<int>(status), static_cast<int>(type));
    } else {
      status = SnapError::NONE;
    }
  }
  return status;
}

int GrallocSnapHelperLegacy::GetAllHandles(std::vector<buffer_handle_t> *out_handle_list) {
  std::lock_guard<std::mutex> lock(map_lock_);
  if (handles_map_.empty()) {
    return SnapError::NO_RESOURCES;
  }
  out_handle_list->reserve(handles_map_.size());
  for (auto handle : handles_map_) {
    out_handle_list->push_back(static_cast<buffer_handle_t>(handle.first));
  }
  return SnapError::NONE;
}

SnapError GrallocSnapHelperLegacy::BufferIDHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                  uint32_t aidl_size, void *gralloc_in_set,
                                                  void *gralloc_out_get, SnapDescriptor *buf_des,
                                                  bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    if (aidl_size) {
      uint64_t snap_buffer_id = 0;
      error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BUFFER_ID, &snap_buffer_id);
      if (error) {
        ALOGW("%s - Error while getting the metadata type %d from snapmapper", __FUNCTION__,
              static_cast<int>(SnapMetadataType::BUFFER_ID));
        return error;
      }
      *mapper_return = Mapper5Encode<StandardMetadataType::BUFFER_ID>(
          snap_buffer_id, gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    } else if (hidl_bytestream) {
      uint64_t snap_buffer_id = 0;
      if (buf_des != nullptr) {
        error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::BUFFER_ID,
                                                     &snap_buffer_id);
      } else {
        error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BUFFER_ID, &snap_buffer_id);
      }
      error = CheckMetadataSet(SnapMetadataType::BUFFER_ID, error, check_metadata_set);
      if (android::gralloc4::encodeBufferId(snap_buffer_id,
                                            static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      // Both snap and gralloc use uint64_t for buffer ID - write directly to gralloc output to reduce # copies
      error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BUFFER_ID, gralloc_out_get);
      if (error) {
        ALOGW("%s - Error while getting the metadata type %d from snapmapper", __FUNCTION__,
              static_cast<int>(SnapMetadataType::BUFFER_ID));
        return error;
      }
    }
  } else if (gralloc_in_set != nullptr) {
    error = SnapError::BAD_VALUE;
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::UsageHelper(SnapHandle *hnd, bool hidl_bytestream,
                                               uint32_t aidl_size, void *gralloc_in_set,
                                               void *gralloc_out_get, SnapDescriptor *buf_des,
                                               bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapUsage snap_usage = static_cast<SnapUsage>(0);
    if (buf_des != nullptr) {
      error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::USAGE, &snap_usage);
    } else {
      error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::USAGE, &snap_usage);
    }
    error = CheckMetadataSet(SnapMetadataType::USAGE, error, check_metadata_set);
    uint64_t gr_usage = GetGrallocUsage(snap_usage);
    if (aidl_size) {
      *mapper_return = Mapper5Encode<StandardMetadataType::USAGE>(
          static_cast<GrallocBufferUsage>(gr_usage), gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    } else if (hidl_bytestream) {
      if (android::gralloc4::encodeUsage(gr_usage,
                                         static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<uint64_t *>(gralloc_out_get) = gr_usage;
    }
  } else if (gralloc_in_set != nullptr) {
    error = SnapError::BAD_VALUE;
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::DataspaceHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                   uint32_t aidl_size, void *gralloc_in_set,
                                                   void *gralloc_out_get, SnapDescriptor *buf_des,
                                                   bool check_metadata_set,
                                                   int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  int conversion_err = SnapError::NONE;
  if (gralloc_out_get != nullptr) {
    SnapDataspace snap_dataspace = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::DATASPACE, &snap_dataspace);
    error = CheckMetadataSet(SnapMetadataType::DATASPACE, error, check_metadata_set);
    GrallocDataspace gr_dataspace = {};
    ConvertSnapDataspaceToGrallocDataspace(snap_dataspace, &gr_dataspace);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeDataspace(gr_dataspace,
                                             static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<GrallocDataspace *>(gralloc_out_get) = gr_dataspace;
    }
  } else if (gralloc_in_set != nullptr) {
    SnapDataspace snap_dataspace = {};
    GrallocDataspace gr_dataspace = {};
    if (hidl_bytestream) {
      if (android::gralloc4::decodeDataspace(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                             &gr_dataspace)) {
        return SnapError::UNSUPPORTED;
      }
      conversion_err = ConvertGrallocDataspaceToSnapDataspace(gr_dataspace, &snap_dataspace);
    } else {
      gr_dataspace = *static_cast<GrallocDataspace *>(gralloc_in_set);
      conversion_err = ConvertGrallocDataspaceToSnapDataspace(
          *static_cast<GrallocDataspace *>(gralloc_in_set), &snap_dataspace);
    }
    if (conversion_err != SnapError::NONE) {
      ALOGW("%s: Attempting to set invalid gralloc dataspace - %d", __FUNCTION__, gr_dataspace);
      return SnapError::UNSUPPORTED;
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::DATASPACE, &snap_dataspace);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::ColorspaceHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                    uint32_t aidl_size, void *gralloc_in_set,
                                                    void *gralloc_out_get, SnapDescriptor *buf_des,
                                                    bool check_metadata_set,
                                                    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapDataspace snap_dataspace = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::DATASPACE, &snap_dataspace);
    error = CheckMetadataSet(SnapMetadataType::DATASPACE, error, check_metadata_set);
    uint32_t color_space = 0;
    GetColorSpaceFromDataspaceMetadata(snap_dataspace, &color_space);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeUint32(qtigralloc::MetadataType_ColorSpace, color_space,
                                          static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<uint32_t *>(gralloc_out_get) = color_space;
    }
  } else if (gralloc_in_set != nullptr) {
    SnapDataspace snap_dataspace = {};
    if (hidl_bytestream) {
      uint32_t color_space = 0;
      if (android::gralloc4::decodeUint32(qtigralloc::MetadataType_ColorSpace,
                                          *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                          &color_space)) {
        return SnapError::UNSUPPORTED;
      }
      GetSnapDataspaceMetadataFromColorSpace(color_space, &snap_dataspace);
    } else {
      GetSnapDataspaceMetadataFromColorSpace(*static_cast<uint32_t *>(gralloc_in_set),
                                             &snap_dataspace);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::DATASPACE, &snap_dataspace);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::NameHelper(SnapHandle *hnd, bool hidl_bytestream,
                                              uint32_t aidl_size, void *gralloc_in_set,
                                              void *gralloc_out_get, SnapDescriptor *buf_des,
                                              bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  std::string name = "";
  if (gralloc_in_set != nullptr) {
    return error;
  }
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::NAME, &name);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::NAME, &name);
  }
  error = CheckMetadataSet(SnapMetadataType::NAME, error, check_metadata_set);

  if (aidl_size) {
    *mapper_return =
        Mapper5Encode<StandardMetadataType::NAME>(name, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  } else if (hidl_bytestream) {
    if (android::gralloc4::encodeName(name, static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
      return SnapError::BAD_VALUE;
    }
  } else {
    *static_cast<std::string *>(gralloc_out_get) = static_cast<std::string>(name.c_str());
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::WidthHelper(SnapHandle *hnd, bool hidl_bytestream,
                                               uint32_t aidl_size, void *gralloc_in_set,
                                               void *gralloc_out_get, SnapDescriptor *buf_des,
                                               bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint64_t snap_width = 0;
  if (gralloc_in_set != nullptr) {
    return error;
  }
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::WIDTH, &snap_width);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::WIDTH, &snap_width);
  }
  error = CheckMetadataSet(SnapMetadataType::WIDTH, error, check_metadata_set);

  if (aidl_size) {
    *mapper_return =
        Mapper5Encode<StandardMetadataType::WIDTH>(snap_width, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  } else if (hidl_bytestream) {
    if (android::gralloc4::encodeWidth(snap_width,
                                       static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
      return SnapError::BAD_VALUE;
    }
  } else {
    *static_cast<uint64_t *>(gralloc_out_get) = snap_width;
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::HeightHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                uint32_t aidl_size, void *gralloc_in_set,
                                                void *gralloc_out_get, SnapDescriptor *buf_des,
                                                bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint64_t snap_height = 0;
  if (gralloc_in_set != nullptr) {
    return error;
  }
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::HEIGHT, &snap_height);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::HEIGHT, &snap_height);
  }
  error = CheckMetadataSet(SnapMetadataType::HEIGHT, error, check_metadata_set);

  if (aidl_size) {
    *mapper_return =
        Mapper5Encode<StandardMetadataType::HEIGHT>(snap_height, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  } else if (hidl_bytestream) {
    if (android::gralloc4::encodeHeight(snap_height,
                                        static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
      return SnapError::BAD_VALUE;
    }
  } else {
    *static_cast<uint64_t *>(gralloc_out_get) = snap_height;
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::LayerCountHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                    uint32_t aidl_size, void *gralloc_in_set,
                                                    void *gralloc_out_get, SnapDescriptor *buf_des,
                                                    bool check_metadata_set,
                                                    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint64_t layer_count = 0;
  if (gralloc_in_set != nullptr) {
    return SnapError::BAD_VALUE;
  }
  if (buf_des != nullptr) {
    error =
        snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::LAYER_COUNT, &layer_count);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::LAYER_COUNT, &layer_count);
  }
  error = CheckMetadataSet(SnapMetadataType::LAYER_COUNT, error, check_metadata_set);

  if (aidl_size) {
    *mapper_return = Mapper5Encode<StandardMetadataType::LAYER_COUNT>(layer_count, gralloc_out_get,
                                                                      *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  } else if (hidl_bytestream) {
    if (android::gralloc4::encodeLayerCount(layer_count,
                                            static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
      return SnapError::BAD_VALUE;
    }
  } else {
    *static_cast<uint64_t *>(gralloc_out_get) = layer_count;
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::PixelFormatRequestedHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  SnapPixelFormat snap_pixel_format = SnapPixelFormat::PIXEL_FORMAT_UNSPECIFIED;
  SnapUsage snap_usage = static_cast<SnapUsage>(0);
  uint64_t modifier = 0;
  // Gralloc4 expects PIXEL_FORMAT_ALLOCATED vs Gralloc5 expecting PIXEL_FORMAT_REQUESTED
  SnapMetadataType metadata_type = SnapMetadataType::PIXEL_FORMAT_ALLOCATED;
  if (aidl_size) {
    metadata_type = SnapMetadataType::PIXEL_FORMAT_REQUESTED;
  }

  if (gralloc_in_set != nullptr) {
    return SnapError::BAD_VALUE;
  }
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, metadata_type, &snap_pixel_format);
    error = CheckMetadataSet(metadata_type, error, check_metadata_set);
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::USAGE, &snap_usage);
    error = CheckMetadataSet(SnapMetadataType::USAGE, error, check_metadata_set);
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::FORMAT_MODIFIER,
                                                 &modifier);
    error = CheckMetadataSet(SnapMetadataType::FORMAT_MODIFIER, error, check_metadata_set);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, metadata_type, &snap_pixel_format);
    error = CheckMetadataSet(metadata_type, error, check_metadata_set);
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::USAGE, &snap_usage);
    error = CheckMetadataSet(SnapMetadataType::USAGE, error, check_metadata_set);
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::FORMAT_MODIFIER, &modifier);
    error = CheckMetadataSet(SnapMetadataType::FORMAT_MODIFIER, error, check_metadata_set);
  }
  SnapFormatDescriptor snap_fmt_desc = {.format = snap_pixel_format,
                                        .modifier = static_cast<SnapPixelFormatModifier>(modifier)};
  int gr_format = 0;
  GetGrallocFormat(snap_fmt_desc, snap_usage, &gr_format);
  if (!gr_format) {
    gr_format = static_cast<int>(snap_pixel_format);
  }
  if (aidl_size) {
    *mapper_return = Mapper5Encode<StandardMetadataType::PIXEL_FORMAT_REQUESTED>(
        static_cast<GrallocPixelFormat>(gr_format), gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  } else if (hidl_bytestream) {
    if (android::gralloc4::encodePixelFormatRequested(
            static_cast<PixelFormat>(gr_format),
            static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
      return SnapError::BAD_VALUE;
    }
  } else {
    *static_cast<PixelFormat *>(gralloc_out_get) = static_cast<PixelFormat>(gr_format);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::PixelFormatAllocatedHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  SnapPixelFormat snap_pixel_format = SnapPixelFormat::PIXEL_FORMAT_UNSPECIFIED;
  SnapUsage snap_usage = static_cast<SnapUsage>(0);
  uint64_t modifier = 0;

  if (gralloc_in_set != nullptr) {
    return SnapError::BAD_VALUE;
  }
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::PIXEL_FORMAT_ALLOCATED,
                                                 &snap_pixel_format);
    error = CheckMetadataSet(SnapMetadataType::PIXEL_FORMAT_ALLOCATED, error, check_metadata_set);
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::USAGE, &snap_usage);
    error = CheckMetadataSet(SnapMetadataType::USAGE, error, check_metadata_set);
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::FORMAT_MODIFIER,
                                                 &modifier);
    error = CheckMetadataSet(SnapMetadataType::FORMAT_MODIFIER, error, check_metadata_set);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::PIXEL_FORMAT_ALLOCATED,
                                     &snap_pixel_format);
    error = CheckMetadataSet(SnapMetadataType::PIXEL_FORMAT_ALLOCATED, error, check_metadata_set);
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::USAGE, &snap_usage);
    error = CheckMetadataSet(SnapMetadataType::USAGE, error, check_metadata_set);
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::FORMAT_MODIFIER, &modifier);
    error = CheckMetadataSet(SnapMetadataType::FORMAT_MODIFIER, error, check_metadata_set);
  }
  SnapFormatDescriptor snap_fmt_desc = {.format = snap_pixel_format,
                                        .modifier = static_cast<SnapPixelFormatModifier>(modifier)};
  int gr_format = 0;
  GetGrallocFormat(snap_fmt_desc, snap_usage, &gr_format);
  if (!gr_format) {
    gr_format = static_cast<int>(snap_pixel_format);
  }

  // This type is only supported as a vendor metadata type in Gralloc5
  *static_cast<PixelFormat *>(gralloc_out_get) = static_cast<PixelFormat>(gr_format);

  return error;
}

SnapError GrallocSnapHelperLegacy::PixelFormatFourCCHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                           uint32_t aidl_size, void *gralloc_in_set,
                                                           void *gralloc_out_get,
                                                           SnapDescriptor *buf_des,
                                                           bool check_metadata_set,
                                                           int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint32_t pixel_format_fourcc = 0;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::PIXEL_FORMAT_FOURCC,
                                                 &pixel_format_fourcc);
  } else if (gralloc_out_get != nullptr) {
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::PIXEL_FORMAT_FOURCC, &pixel_format_fourcc);
  }
  error = CheckMetadataSet(SnapMetadataType::PIXEL_FORMAT_FOURCC, error, check_metadata_set);

  if (aidl_size) {
    *mapper_return = Mapper5Encode<StandardMetadataType::PIXEL_FORMAT_FOURCC>(
        pixel_format_fourcc, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  } else if (hidl_bytestream) {
    if (android::gralloc4::encodePixelFormatFourCC(
            pixel_format_fourcc, static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
      return SnapError::BAD_VALUE;
    }
  } else {
    *static_cast<uint32_t *>(gralloc_out_get) = pixel_format_fourcc;
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::DRMPixelFormatModifierHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint64_t pixel_format_modifier = 0;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(
        *buf_des, SnapMetadataType::DRM_PIXEL_FORMAT_MODIFIER, &pixel_format_modifier);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::DRM_PIXEL_FORMAT_MODIFIER,
                                     &pixel_format_modifier);
  }
  error = CheckMetadataSet(SnapMetadataType::DRM_PIXEL_FORMAT_MODIFIER, error, check_metadata_set);

  if (aidl_size) {
    *mapper_return = Mapper5Encode<StandardMetadataType::PIXEL_FORMAT_MODIFIER>(
        pixel_format_modifier, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  } else if (hidl_bytestream) {
    if (android::gralloc4::encodePixelFormatModifier(
            pixel_format_modifier, static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
      return SnapError::BAD_VALUE;
    }
  } else {
    *static_cast<uint64_t *>(gralloc_out_get) = pixel_format_modifier;
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::AllocationSizeHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                        uint32_t aidl_size, void *gralloc_in_set,
                                                        void *gralloc_out_get,
                                                        SnapDescriptor *buf_des,
                                                        bool check_metadata_set,
                                                        int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint32_t allocation_size = 0;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::ALLOCATION_SIZE,
                                                 &allocation_size);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::ALLOCATION_SIZE, &allocation_size);
  }
  error = CheckMetadataSet(SnapMetadataType::ALLOCATION_SIZE, error, check_metadata_set);

  if (aidl_size) {
    *mapper_return = Mapper5Encode<StandardMetadataType::ALLOCATION_SIZE>(
        static_cast<uint64_t>(allocation_size), gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  } else if (hidl_bytestream) {
    if (android::gralloc4::encodeAllocationSize(
            static_cast<uint64_t>(allocation_size),
            static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
      return SnapError::BAD_VALUE;
    }
  } else {
    *static_cast<uint32_t *>(gralloc_out_get) = allocation_size;
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::ProtectedContentHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                          uint32_t aidl_size, void *gralloc_in_set,
                                                          void *gralloc_out_get,
                                                          SnapDescriptor *buf_des,
                                                          bool check_metadata_set,
                                                          int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint64_t protect_content = 0;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::PROTECTED_CONTENT,
                                                 &protect_content);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::PROTECTED_CONTENT, &protect_content);
  }
  error = CheckMetadataSet(SnapMetadataType::PROTECTED_CONTENT, error, check_metadata_set);

  if (aidl_size) {
    *mapper_return = Mapper5Encode<StandardMetadataType::PROTECTED_CONTENT>(
        protect_content, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  } else if (hidl_bytestream) {
    if (android::gralloc4::encodeProtectedContent(
            protect_content, static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
      return SnapError::BAD_VALUE;
    }
  } else {
    *static_cast<uint64_t *>(gralloc_out_get) = protect_content;
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::CompressionHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                     uint32_t aidl_size, void *gralloc_in_set,
                                                     void *gralloc_out_get, SnapDescriptor *buf_des,
                                                     bool check_metadata_set,
                                                     int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  int64_t snap_compression = 0;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::COMPRESSION,
                                                 &snap_compression);
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::COMPRESSION, &snap_compression);
  }
  error = CheckMetadataSet(SnapMetadataType::COMPRESSION, error, check_metadata_set);
  GrallocExtendableType gr_compression = {};

  if (snap_compression == vendor_qti_hardware_display_common_Compression::COMPRESSION_NONE) {
    gr_compression = android::gralloc4::Compression_None;
  } else {
    if (aidl_size) {
      gr_compression = {"QTI", snap_compression};
    } else {
      gr_compression = qtigralloc::Compression_QtiUBWC;
    }
  }

  if (aidl_size) {
    *mapper_return = Mapper5Encode<StandardMetadataType::COMPRESSION>(
        gr_compression, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  } else if (hidl_bytestream) {
    if (android::gralloc4::encodeCompression(gr_compression,
                                             static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
      return SnapError::BAD_VALUE;
    }
  } else {
    *static_cast<GrallocExtendableType *>(gralloc_out_get) = gr_compression;
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::InterlacedHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                    uint32_t aidl_size, void *gralloc_in_set,
                                                    void *gralloc_out_get, SnapDescriptor *buf_des,
                                                    bool check_metadata_set,
                                                    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (gralloc_out_get != nullptr) {
    int64_t snap_interlaced = 0;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::INTERLACED, &snap_interlaced);
    error = CheckMetadataSet(SnapMetadataType::INTERLACED, error, check_metadata_set);
    GrallocExtendableType gr_interlaced = {};
    if (snap_interlaced == vendor_qti_hardware_display_common_Interlaced::INTERLACED_NONE) {
      gr_interlaced = android::gralloc4::Interlaced_None;
    } else {
      gr_interlaced = qtigralloc::Interlaced_Qti;
    }
    if (aidl_size) {
      *mapper_return = Mapper5Encode<StandardMetadataType::INTERLACED>(
          gr_interlaced, gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    } else if (hidl_bytestream) {
      if (android::gralloc4::encodeInterlaced(gr_interlaced,
                                              static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<GrallocExtendableType *>(gralloc_out_get) = gr_interlaced;
    }
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::ChromaSitingHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                      uint32_t aidl_size, void *gralloc_in_set,
                                                      void *gralloc_out_get,
                                                      SnapDescriptor *buf_des,
                                                      bool check_metadata_set,
                                                      int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (gralloc_out_get != nullptr) {
    int64_t snap_chroma_siting = 0;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CHROMA_SITING, &snap_chroma_siting);
    error = CheckMetadataSet(SnapMetadataType::CHROMA_SITING, error, check_metadata_set);
    GrallocExtendableType gr_chroma_siting = {};
    if (snap_chroma_siting == vendor_qti_hardware_display_common_ChromaSiting::CHROMA_SITING_NONE) {
      gr_chroma_siting = android::gralloc4::ChromaSiting_None;
    }
    if (aidl_size) {
      *mapper_return = Mapper5Encode<StandardMetadataType::CHROMA_SITING>(
          gr_chroma_siting, gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    } else if (hidl_bytestream) {
      if (android::gralloc4::encodeChromaSiting(
              gr_chroma_siting, static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<GrallocExtendableType *>(gralloc_out_get) = gr_chroma_siting;
    }
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::PlaneLayoutsHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                      uint32_t aidl_size, void *gralloc_in_set,
                                                      void *gralloc_out_get,
                                                      SnapDescriptor *buf_des,
                                                      bool check_metadata_set,
                                                      int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  SnapBufferLayout snap_buffer_layout = {};
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (buf_des != nullptr) {
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::PLANE_LAYOUTS,
                                                 &snap_buffer_layout);
    if (!error) {
      int64_t ubwc_enabled_in_snap;
      error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::IS_UBWC,
                                                   &ubwc_enabled_in_snap);
      // Added to keep parity with getFormatLayout since sdm, composer and gralloc don't expect
      // meta planes for this usecase.
      if ((IsUncompressedRGBFormat(static_cast<int>(buf_des->format)) ||
           IsCompressedRGBFormat(static_cast<int>(buf_des->format))) &&
          ubwc_enabled_in_snap) {
        snap_buffer_layout.plane_count /= 2;
      }
    }
  } else if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::PLANE_LAYOUTS, &snap_buffer_layout);
  }
  error = CheckMetadataSet(SnapMetadataType::PLANE_LAYOUTS, error, check_metadata_set);
  std::vector<GrallocPlaneLayout> gr_plane_layouts;
  ConvertSnapBufferlayoutToGrallocPlaneLayout(hnd, buf_des, snap_buffer_layout, &gr_plane_layouts);
  if (aidl_size) {
    *mapper_return = Mapper5Encode<StandardMetadataType::PLANE_LAYOUTS>(
        gr_plane_layouts, gralloc_out_get, *mapper_return);
    if (*mapper_return < 0) {
      return SnapError::BAD_VALUE;
    }
  } else if (hidl_bytestream) {
    if (android::gralloc4::encodePlaneLayouts(gr_plane_layouts,
                                              static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
      return SnapError::BAD_VALUE;
    }
  } else {
    *static_cast<std::vector<GrallocPlaneLayout> *>(gralloc_out_get) = gr_plane_layouts;
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::YuvPlaneInfoHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                      uint32_t aidl_size, void *gralloc_in_set,
                                                      void *gralloc_out_get,
                                                      SnapDescriptor *buf_des,
                                                      bool check_metadata_set,
                                                      int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  SnapBufferLayout snap_buffer_layout = {};
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::PLANE_LAYOUTS, &snap_buffer_layout);
  }
  error = CheckMetadataSet(SnapMetadataType::PLANE_LAYOUTS, error, check_metadata_set);
  std::vector<GrallocPlaneLayout> gr_plane_layouts;
  ConvertSnapBufferlayoutToGrallocPlaneLayout(hnd, buf_des, snap_buffer_layout, &gr_plane_layouts);
  android_ycbcr outYCbCr[2];
  uint64_t base_addr = 0;
  if (hnd != nullptr) {
    auto status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BASE_ADDRESS, &base_addr);
  }
  ConvertGrallocPlaneLayoutToAndroidYCbCr(base_addr, gr_plane_layouts, outYCbCr);
  qti_ycbcr layout[2];
  for (int i = 0; i < 2; i++) {
    layout[i].y = outYCbCr[i].y;
    layout[i].cr = outYCbCr[i].cr;
    layout[i].cb = outYCbCr[i].cb;
    layout[i].yStride = static_cast<uint32_t>(outYCbCr[i].ystride);
    layout[i].cStride = static_cast<uint32_t>(outYCbCr[i].cstride);
    layout[i].chromaStep = static_cast<uint32_t>(outYCbCr[i].chroma_step);
  }
  uint64_t yOffset = (reinterpret_cast<uint64_t>(layout[0].y) - base_addr);
  uint64_t crOffset = (reinterpret_cast<uint64_t>(layout[0].cr) - base_addr);
  uint64_t cbOffset = (reinterpret_cast<uint64_t>(layout[0].cb) - base_addr);
  ALOGD_IF(enable_logs_,
           " %s: layout: y: %" PRIu64 " , cr: %" PRIu64 " , cb: %" PRIu64
           " , yStride: %d, cStride: %d, chromaStep: %d ",
           __FUNCTION__, yOffset, crOffset, cbOffset, layout[0].yStride, layout[0].cStride,
           layout[0].chromaStep);
  if (hidl_bytestream) {
    if (qtigralloc::encodeYUVPlaneInfoMetadata(
            layout, static_cast<hidl_vec<uint8_t> *>(gralloc_out_get)) != GrallocError::NONE) {
      return SnapError::BAD_VALUE;
    }
  } else {
    memcpy(gralloc_out_get, layout, YCBCR_LAYOUT_ARRAY_SIZE * sizeof(qti_ycbcr));
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::CropHelper(SnapHandle *hnd, bool hidl_bytestream,
                                              uint32_t aidl_size, void *gralloc_in_set,
                                              void *gralloc_out_get, SnapDescriptor *buf_des,
                                              bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapRect snap_rect = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CROP, &snap_rect);
    error = CheckMetadataSet(SnapMetadataType::CROP, error, check_metadata_set);
    std::vector<Rect> out_crop = {
        {snap_rect.left, snap_rect.top, snap_rect.right, snap_rect.bottom}};
    if (aidl_size) {
      *mapper_return =
          Mapper5Encode<StandardMetadataType::CROP>(out_crop, gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    } else if (hidl_bytestream) {
      if (android::gralloc4::encodeCrop(out_crop,
                                        static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<Rect *>(gralloc_out_get) = out_crop.at(0);
    }
  } else if (gralloc_in_set != nullptr) {
    SnapRect snap_rect = {};
    if (aidl_size) {
      auto decoded_result = Mapper5Decode<StandardMetadataType::CROP>(gralloc_in_set, aidl_size);
      if (!decoded_result.has_value() || decoded_result->size() != 1) {
        return SnapError::UNSUPPORTED;
      }
      snap_rect = {.left = decoded_result->at(0).left,
                   .top = decoded_result->at(0).top,
                   .right = decoded_result->at(0).right,
                   .bottom = decoded_result->at(0).bottom};
    } else if (hidl_bytestream) {
      std::vector<Rect> gr_crop;
      auto status = android::gralloc4::decodeCrop(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                                  &gr_crop);
      if (!status && gr_crop.size() == 1) {
        snap_rect = {.left = gr_crop[0].left,
                     .top = gr_crop[0].top,
                     .right = gr_crop[0].right,
                     .bottom = gr_crop[0].bottom};
      } else {
        return SnapError::UNSUPPORTED;
      }
    } else {
      Rect gr_crop = *static_cast<Rect *>(gralloc_in_set);
      snap_rect = {.left = gr_crop.left,
                   .top = gr_crop.top,
                   .right = gr_crop.right,
                   .bottom = gr_crop.bottom};
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::CROP, &snap_rect);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::BlendModeHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                   uint32_t aidl_size, void *gralloc_in_set,
                                                   void *gralloc_out_get, SnapDescriptor *buf_des,
                                                   bool check_metadata_set,
                                                   int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapBlendMode snap_blendmode = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BLEND_MODE, &snap_blendmode);
    error = CheckMetadataSet(SnapMetadataType::BLEND_MODE, error, check_metadata_set);

    if (aidl_size) {
      *mapper_return = Mapper5Encode<StandardMetadataType::BLEND_MODE>(
          static_cast<BlendMode>(snap_blendmode), gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    } else if (hidl_bytestream) {
      if (android::gralloc4::encodeBlendMode(static_cast<BlendMode>(snap_blendmode),
                                             static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<BlendMode *>(gralloc_out_get) = static_cast<BlendMode>(snap_blendmode);
    }
  } else if (gralloc_in_set != nullptr) {
    SnapBlendMode snap_blendmode = {};
    if (aidl_size) {
      auto decoded_result =
          Mapper5Decode<StandardMetadataType::BLEND_MODE>(gralloc_in_set, aidl_size);
      if (!decoded_result.has_value()) {
        return SnapError::UNSUPPORTED;
      }
      snap_blendmode = static_cast<SnapBlendMode>(*decoded_result);
    } else if (hidl_bytestream) {
      aidl::android::hardware::graphics::common::BlendMode blend_mode;
      if (android::gralloc4::decodeBlendMode(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                             &blend_mode)) {
        return SnapError::UNSUPPORTED;
      }
      snap_blendmode = static_cast<SnapBlendMode>(blend_mode);
    } else {
      aidl::android::hardware::graphics::common::BlendMode gr_blend_mode;
      gr_blend_mode =
          *static_cast<aidl::android::hardware::graphics::common::BlendMode *>(gralloc_in_set);
      snap_blendmode = static_cast<SnapBlendMode>(gr_blend_mode);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::BLEND_MODE, &snap_blendmode);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::VTTimestampHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                     uint32_t aidl_size, void *gralloc_in_set,
                                                     void *gralloc_out_get, SnapDescriptor *buf_des,
                                                     bool check_metadata_set,
                                                     int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    uint64_t vt_timestamp = 0;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::VT_TIMESTAMP, &vt_timestamp);
    error = CheckMetadataSet(SnapMetadataType::VT_TIMESTAMP, error, check_metadata_set);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeUint64(qtigralloc::MetadataType_VTTimestamp, vt_timestamp,
                                          static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<uint64_t *>(gralloc_out_get) = static_cast<uint64_t>(vt_timestamp);
    }
  } else if (gralloc_in_set != nullptr) {
    uint64_t vt_timestamp = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeUint64(qtigralloc::MetadataType_VTTimestamp,
                                          *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                          &vt_timestamp)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      vt_timestamp = *static_cast<uint64_t *>(gralloc_in_set);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::VT_TIMESTAMP, &vt_timestamp);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::BufferDequeueDurationHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    int64_t dequeue_duration = 0;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BUFFER_DEQUEUE_DURATION,
                                     &dequeue_duration);
    error = CheckMetadataSet(SnapMetadataType::BUFFER_DEQUEUE_DURATION, error, check_metadata_set);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeInt64(qtigralloc::MetadataType_BufferDequeueDuration,
                                         dequeue_duration,
                                         static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<int64_t *>(gralloc_out_get) = static_cast<int64_t>(dequeue_duration);
    }
  } else if (gralloc_in_set != nullptr) {
    int64_t dequeue_duration = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeInt64(qtigralloc::MetadataType_BufferDequeueDuration,
                                         *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                         &dequeue_duration)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      dequeue_duration = *static_cast<int64_t *>(gralloc_in_set);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::BUFFER_DEQUEUE_DURATION,
                                     &dequeue_duration);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::CompressionMetadataHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapAnamorphicMetadata anamorphic_metadata;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::ANAMORPHIC_COMPRESSION_METADATA,
                                     &anamorphic_metadata);
    error = CheckMetadataSet(SnapMetadataType::ANAMORPHIC_COMPRESSION_METADATA, error,
                             check_metadata_set);
    if (hidl_bytestream) {
      hidl_vec<uint8_t> *in = static_cast<hidl_vec<uint8_t> *>(gralloc_out_get);
      in->resize(sizeof(SnapAnamorphicMetadata));
      memcpy(in->data(), &anamorphic_metadata, sizeof(anamorphic_metadata));
    } else {
      *static_cast<SnapAnamorphicMetadata *>(gralloc_out_get) = anamorphic_metadata;
    }
  } else if (gralloc_in_set != nullptr) {
    SnapAnamorphicMetadata anamorphic_metadata;
    if (hidl_bytestream) {
      hidl_vec<uint8_t> *in = static_cast<hidl_vec<uint8_t> *>(gralloc_in_set);
      memcpy(&anamorphic_metadata, in->data(), sizeof(anamorphic_metadata));
    } else {
      anamorphic_metadata = *static_cast<SnapAnamorphicMetadata *>(gralloc_in_set);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::ANAMORPHIC_COMPRESSION_METADATA,
                                     &anamorphic_metadata);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::PPParamInterlacedHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                           uint32_t aidl_size, void *gralloc_in_set,
                                                           void *gralloc_out_get,
                                                           SnapDescriptor *buf_des,
                                                           bool check_metadata_set,
                                                           int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    int32_t pp_param_interlaced = 0;
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::PP_PARAM_INTERLACED, &pp_param_interlaced);
    error = CheckMetadataSet(SnapMetadataType::PP_PARAM_INTERLACED, error, check_metadata_set);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeInt32(qtigralloc::MetadataType_PPParamInterlaced,
                                         pp_param_interlaced,
                                         static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<int32_t *>(gralloc_out_get) = static_cast<int32_t>(pp_param_interlaced);
    }
  } else if (gralloc_in_set != nullptr) {
    int32_t pp_param_interlaced = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeInt32(qtigralloc::MetadataType_PPParamInterlaced,
                                         *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                         &pp_param_interlaced)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      pp_param_interlaced = *static_cast<int32_t *>(gralloc_in_set);
    }
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::PP_PARAM_INTERLACED, &pp_param_interlaced);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::VideoPerfModeHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                       uint32_t aidl_size, void *gralloc_in_set,
                                                       void *gralloc_out_get,
                                                       SnapDescriptor *buf_des,
                                                       bool check_metadata_set,
                                                       int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    uint32_t video_perf_mode = 0;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::VIDEO_PERF_MODE, &video_perf_mode);
    error = CheckMetadataSet(SnapMetadataType::VIDEO_PERF_MODE, error, check_metadata_set);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeUint32(qtigralloc::MetadataType_VideoPerfMode, video_perf_mode,
                                          static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<uint32_t *>(gralloc_out_get) = static_cast<uint32_t>(video_perf_mode);
    }
  } else if (gralloc_in_set != nullptr) {
    uint32_t video_perf_mode = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeUint32(qtigralloc::MetadataType_VideoPerfMode,
                                          *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                          &video_perf_mode)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      video_perf_mode = *static_cast<uint32_t *>(gralloc_in_set);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::VIDEO_PERF_MODE, &video_perf_mode);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::GraphicsMetadataHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                          uint32_t aidl_size, void *gralloc_in_set,
                                                          void *gralloc_out_get,
                                                          SnapDescriptor *buf_des,
                                                          bool check_metadata_set,
                                                          int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapGraphicsMetadata snap_graphics_metadata = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::GRAPHICS_METADATA,
                                     &snap_graphics_metadata);
    error = CheckMetadataSet(SnapMetadataType::GRAPHICS_METADATA, error, check_metadata_set);
    GraphicsMetadata gr_graphics_metadata = {};
    gr_graphics_metadata.size = sizeof(snap_graphics_metadata.data);
    std::memcpy(gr_graphics_metadata.data, snap_graphics_metadata.data,
                sizeof(snap_graphics_metadata.data));
    if (hidl_bytestream) {
      if (qtigralloc::encodeGraphicsMetadata(gr_graphics_metadata,
                                             static_cast<hidl_vec<uint8_t> *>(gralloc_out_get)) !=
          GrallocError::NONE) {
        return SnapError::BAD_VALUE;
      }
    } else {
      memcpy(gralloc_out_get, &gr_graphics_metadata.data, sizeof(gr_graphics_metadata.data));
    }
  } else if (gralloc_in_set != nullptr) {
    SnapGraphicsMetadata snap_graphics_metadata = {};
    if (hidl_bytestream) {
      GraphicsMetadata gr_graphics_metadata = {};
      if (qtigralloc::decodeGraphicsMetadata(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                             &gr_graphics_metadata) != GrallocError::NONE) {
        return SnapError::UNSUPPORTED;
      }
      snap_graphics_metadata.size = sizeof(gr_graphics_metadata.data);
      std::memcpy(snap_graphics_metadata.data, gr_graphics_metadata.data,
                  sizeof(gr_graphics_metadata.data));
    } else {
      GraphicsMetadata gr_graphics_metadata = *static_cast<GraphicsMetadata *>(gralloc_in_set);
      snap_graphics_metadata.size = sizeof(gr_graphics_metadata.data);
      std::memcpy(snap_graphics_metadata.data, gr_graphics_metadata.data,
                  sizeof(gr_graphics_metadata.data));
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::GRAPHICS_METADATA,
                                     &snap_graphics_metadata);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::RefreshRateHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                     uint32_t aidl_size, void *gralloc_in_set,
                                                     void *gralloc_out_get, SnapDescriptor *buf_des,
                                                     bool check_metadata_set,
                                                     int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    float refresh_rate = 0.0;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::REFRESH_RATE, &refresh_rate);
    error = CheckMetadataSet(SnapMetadataType::REFRESH_RATE, error, check_metadata_set);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeFloat(qtigralloc::MetadataType_RefreshRate, refresh_rate,
                                         static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<float *>(gralloc_out_get) = static_cast<float>(refresh_rate);
    }
  } else if (gralloc_in_set != nullptr) {
    float refresh_rate = 0.0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeFloat(qtigralloc::MetadataType_RefreshRate,
                                         *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                         &refresh_rate)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      refresh_rate = *static_cast<float *>(gralloc_in_set);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::REFRESH_RATE, &refresh_rate);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::MapSecureBufferHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                         uint32_t aidl_size, void *gralloc_in_set,
                                                         void *gralloc_out_get,
                                                         SnapDescriptor *buf_des,
                                                         bool check_metadata_set,
                                                         int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    int32_t map_secure_buffer = 0;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::MAP_SECURE_BUFFER, &map_secure_buffer);
    error = CheckMetadataSet(SnapMetadataType::MAP_SECURE_BUFFER, error, check_metadata_set);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeInt32(qtigralloc::MetadataType_MapSecureBuffer,
                                         map_secure_buffer,
                                         static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<int32_t *>(gralloc_out_get) = static_cast<int32_t>(map_secure_buffer);
    }
  } else if (gralloc_in_set != nullptr) {
    int32_t map_secure_buffer = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeInt32(qtigralloc::MetadataType_MapSecureBuffer,
                                         *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                         &map_secure_buffer)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      map_secure_buffer = *static_cast<int32_t *>(gralloc_in_set);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::MAP_SECURE_BUFFER, &map_secure_buffer);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::LinearFormatHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                      uint32_t aidl_size, void *gralloc_in_set,
                                                      void *gralloc_out_get,
                                                      SnapDescriptor *buf_des,
                                                      bool check_metadata_set,
                                                      int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    uint32_t linear_format = 0;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::LINEAR_FORMAT, &linear_format);
    error = CheckMetadataSet(SnapMetadataType::LINEAR_FORMAT, error, check_metadata_set);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeUint32(qtigralloc::MetadataType_LinearFormat, linear_format,
                                          static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<uint32_t *>(gralloc_out_get) = static_cast<uint32_t>(linear_format);
    }
  } else if (gralloc_in_set != nullptr) {
    uint32_t linear_format = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeUint32(qtigralloc::MetadataType_LinearFormat,
                                          *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                          &linear_format)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      linear_format = *static_cast<uint32_t *>(gralloc_in_set);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::LINEAR_FORMAT, &linear_format);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::SingleBufferModeHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                          uint32_t aidl_size, void *gralloc_in_set,
                                                          void *gralloc_out_get,
                                                          SnapDescriptor *buf_des,
                                                          bool check_metadata_set,
                                                          int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    uint32_t single_buffer_mode = 0;
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::SINGLE_BUFFER_MODE, &single_buffer_mode);
    error = CheckMetadataSet(SnapMetadataType::SINGLE_BUFFER_MODE, error, check_metadata_set);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeUint32(qtigralloc::MetadataType_SingleBufferMode,
                                          single_buffer_mode,
                                          static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<uint32_t *>(gralloc_out_get) = static_cast<uint32_t>(single_buffer_mode);
    }
  } else if (gralloc_in_set != nullptr) {
    uint32_t single_buffer_mode = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeUint32(qtigralloc::MetadataType_SingleBufferMode,
                                          *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                          &single_buffer_mode)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      single_buffer_mode = *static_cast<uint32_t *>(gralloc_in_set);
    }
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::SINGLE_BUFFER_MODE, &single_buffer_mode);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::FDHelper(SnapHandle *hnd, bool hidl_bytestream,
                                            uint32_t aidl_size, void *gralloc_in_set,
                                            void *gralloc_out_get, SnapDescriptor *buf_des,
                                            bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    int32_t fd = -1;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::FD, &fd);
    error = CheckMetadataSet(SnapMetadataType::FD, error, check_metadata_set);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeInt32(qtigralloc::MetadataType_FD, fd,
                                         static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<int32_t *>(gralloc_out_get) = static_cast<int32_t>(fd);
    }
  } else if (gralloc_in_set != nullptr) {
    int32_t fd = -1;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeInt32(qtigralloc::MetadataType_FD,
                                         *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set), &fd)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      fd = *static_cast<int32_t *>(gralloc_in_set);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::FD, &fd);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::AlignedWidthInPixelsHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint32_t aligned_width = 0;
  if (gralloc_out_get != nullptr) {
    if (buf_des != nullptr) {
      error = snapmapper_->GetFromBufferDescriptor(
          *buf_des, SnapMetadataType::ALIGNED_WIDTH_IN_PIXELS, &aligned_width);
    } else {
      error =
          snapmapper_->GetMetadata(*hnd, SnapMetadataType::ALIGNED_WIDTH_IN_PIXELS, &aligned_width);
    }
    error = CheckMetadataSet(SnapMetadataType::ALIGNED_WIDTH_IN_PIXELS, error, check_metadata_set);
    if (aidl_size) {
      *mapper_return = Mapper5Encode<StandardMetadataType::STRIDE>(aligned_width, gralloc_out_get,
                                                                   *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    } else if (hidl_bytestream) {
      if (android::gralloc4::encodeUint32(qtigralloc::MetadataType_AlignedWidthInPixels,
                                          aligned_width,
                                          static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<uint32_t *>(gralloc_out_get) = static_cast<uint32_t>(aligned_width);
    }
  } else if (gralloc_in_set != nullptr) {
    uint32_t aligned_width = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeUint32(qtigralloc::MetadataType_AlignedWidthInPixels,
                                          *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                          &aligned_width)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      aligned_width = *static_cast<uint32_t *>(gralloc_in_set);
    }
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::ALIGNED_WIDTH_IN_PIXELS, &aligned_width);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::AlignedHeightInPixelsHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  uint32_t aligned_height = 0;
  if (gralloc_out_get != nullptr) {
    if (buf_des != nullptr) {
      error = snapmapper_->GetFromBufferDescriptor(
          *buf_des, SnapMetadataType::ALIGNED_HEIGHT_IN_PIXELS, &aligned_height);
    } else {
      error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::ALIGNED_HEIGHT_IN_PIXELS,
                                       &aligned_height);
    }
    error = CheckMetadataSet(SnapMetadataType::ALIGNED_HEIGHT_IN_PIXELS, error, check_metadata_set);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeUint32(qtigralloc::MetadataType_AlignedHeightInPixels,
                                          aligned_height,
                                          static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<uint32_t *>(gralloc_out_get) = static_cast<uint32_t>(aligned_height);
    }
  } else if (gralloc_in_set != nullptr) {
    uint32_t aligned_height = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeUint32(qtigralloc::MetadataType_AlignedHeightInPixels,
                                          *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                          &aligned_height)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      aligned_height = *static_cast<uint32_t *>(gralloc_in_set);
    }
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::ALIGNED_HEIGHT_IN_PIXELS, &aligned_height);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::VendorMetadataStatusHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    bool vendor_metadata_state[METADATA_SET_SIZE];
    bool vendor_metadata_state_legacy[METADATA_SET_SIZE] = {};

    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::VENDOR_METADATA_STATUS,
                                     &vendor_metadata_state);
    error = CheckMetadataSet(SnapMetadataType::VENDOR_METADATA_STATUS, error, check_metadata_set);

    for (int type = 0; type < METADATA_SET_SIZE; type++) {
      int legacy_type = QTI_VT_TIMESTAMP + type;
      if (metadata_type_map.find(legacy_type) != metadata_type_map.end()) {
        int snap_type = metadata_type_map[legacy_type];
        vendor_metadata_state_legacy[GET_VENDOR_METADATA_STATUS_INDEX(legacy_type)] =
            vendor_metadata_state[GET_VENDOR_METADATA_STATUS_INDEX(snap_type)];
      } else if (deprecated_metadata_type_map.find(legacy_type) !=
            deprecated_metadata_type_map.end()){
        for (auto snap_type : deprecated_metadata_type_map[legacy_type]) {
          vendor_metadata_state_legacy[GET_VENDOR_METADATA_STATUS_INDEX(legacy_type)] |=
            vendor_metadata_state[GET_VENDOR_METADATA_STATUS_INDEX(snap_type)];
        }
      }
    }

    if (hidl_bytestream) {
      if (qtigralloc::encodeMetadataState(vendor_metadata_state_legacy,
                                          static_cast<hidl_vec<uint8_t> *>(gralloc_out_get)) !=
          GrallocError::NONE) {
        return SnapError::BAD_VALUE;
      }
    } else {
      std::memcpy(gralloc_out_get, vendor_metadata_state_legacy, sizeof(bool) * METADATA_SET_SIZE);
    }
  } else if (gralloc_in_set != nullptr) {
    if (hidl_bytestream) {
      bool vendor_metadata_state[METADATA_SET_SIZE];
      if (qtigralloc::decodeMetadataState(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                          vendor_metadata_state) != GrallocError::NONE) {
        return SnapError::UNSUPPORTED;
      }
      error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::VENDOR_METADATA_STATUS,
                                       &vendor_metadata_state);
    } else {
      error =
          snapmapper_->SetMetadata(*hnd, SnapMetadataType::VENDOR_METADATA_STATUS, gralloc_in_set);
    }
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::StandardMetadataStatusHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    bool standard_metadata_state[METADATA_SET_SIZE];
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::STANDARD_METADATA_STATUS,
                                     &standard_metadata_state);
    error = CheckMetadataSet(SnapMetadataType::STANDARD_METADATA_STATUS, error, check_metadata_set);
    if (hidl_bytestream) {
      if (qtigralloc::encodeMetadataState(standard_metadata_state,
                                          static_cast<hidl_vec<uint8_t> *>(gralloc_out_get)) !=
          GrallocError::NONE) {
        return SnapError::BAD_VALUE;
      }
    } else {
      std::memcpy(gralloc_out_get, standard_metadata_state, sizeof(bool) * METADATA_SET_SIZE);
    }
  } else if (gralloc_in_set != nullptr) {
    bool standard_metadata_state[METADATA_SET_SIZE];
    if (hidl_bytestream) {
      if (qtigralloc::decodeMetadataState(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                          standard_metadata_state) != GrallocError::NONE) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      std::memcpy(standard_metadata_state, gralloc_in_set, sizeof(bool) * METADATA_SET_SIZE);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::STANDARD_METADATA_STATUS,
                                     &standard_metadata_state);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::BufferTypeHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                    uint32_t aidl_size, void *gralloc_in_set,
                                                    void *gralloc_out_get, SnapDescriptor *buf_des,
                                                    bool check_metadata_set,
                                                    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    uint32_t buffer_type = 0;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BUFFER_TYPE, &buffer_type);
    error = CheckMetadataSet(SnapMetadataType::BUFFER_TYPE, error, check_metadata_set);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeUint32(qtigralloc::MetadataType_BufferType, buffer_type,
                                          static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<uint32_t *>(gralloc_out_get) = static_cast<uint32_t>(buffer_type);
    }
  } else if (gralloc_in_set != nullptr) {
    uint32_t buffer_type = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeUint32(qtigralloc::MetadataType_BufferType,
                                          *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                          &buffer_type)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      buffer_type = *static_cast<uint32_t *>(gralloc_in_set);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::BUFFER_TYPE, &buffer_type);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::CustomDimensionsStrideHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    if (hidl_bytestream) {
      uint32_t stride = 0;
      error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CUSTOM_DIMENSIONS_STRIDE, &stride);
      error =
          CheckMetadataSet(SnapMetadataType::CUSTOM_DIMENSIONS_STRIDE, error, check_metadata_set);
      if (android::gralloc4::encodeUint32(qtigralloc::MetadataType_CustomDimensionsStride, stride,
                                          static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CUSTOM_DIMENSIONS_STRIDE,
                                       gralloc_out_get);
      error =
          CheckMetadataSet(SnapMetadataType::CUSTOM_DIMENSIONS_STRIDE, error, check_metadata_set);
    }
  } else if (gralloc_in_set != nullptr) {
    uint32_t stride = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeUint32(qtigralloc::MetadataType_CustomDimensionsStride,
                                          *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                          &stride)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      stride = *static_cast<uint32_t *>(gralloc_in_set);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::CUSTOM_DIMENSIONS_STRIDE, &stride);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::CustomDimensionsHeightHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    if (hidl_bytestream) {
      uint32_t height = 0;
      error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CUSTOM_DIMENSIONS_HEIGHT, &height);
      error =
          CheckMetadataSet(SnapMetadataType::CUSTOM_DIMENSIONS_HEIGHT, error, check_metadata_set);
      if (android::gralloc4::encodeUint32(qtigralloc::MetadataType_CustomDimensionsHeight, height,
                                          static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CUSTOM_DIMENSIONS_HEIGHT,
                                       gralloc_out_get);
      if (error) {
        ALOGW("%s - Error while getting the metadata type %d from snapmapper", __FUNCTION__,
              static_cast<int>(SnapMetadataType::CUSTOM_DIMENSIONS_HEIGHT));
        return error;
      }
    }
  } else if (gralloc_in_set != nullptr) {
    uint32_t height = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeUint32(qtigralloc::MetadataType_CustomDimensionsHeight,
                                          *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                          &height)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      height = *static_cast<uint32_t *>(gralloc_in_set);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::CUSTOM_DIMENSIONS_HEIGHT, &height);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::RGBDataAddressHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                        uint32_t aidl_size, void *gralloc_in_set,
                                                        void *gralloc_out_get,
                                                        SnapDescriptor *buf_des,
                                                        bool check_metadata_set,
                                                        int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    uint64_t rgb_data = 0;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::RGB_DATA_ADDRESS, &rgb_data);
    error = CheckMetadataSet(SnapMetadataType::RGB_DATA_ADDRESS, error, check_metadata_set);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeUint64(qtigralloc::MetadataType_RgbDataAddress, rgb_data,
                                          static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<uint64_t *>(gralloc_out_get) = static_cast<uint64_t>(rgb_data);
    }
  } else if (gralloc_in_set != nullptr) {
    uint64_t rgb_data = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeUint64(qtigralloc::MetadataType_RgbDataAddress,
                                          *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                          &rgb_data)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      rgb_data = *static_cast<uint64_t *>(gralloc_in_set);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::RGB_DATA_ADDRESS, &rgb_data);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::BufferPermissionHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                          uint32_t aidl_size, void *gralloc_in_set,
                                                          void *gralloc_out_get,
                                                          SnapDescriptor *buf_des,
                                                          bool check_metadata_set,
                                                          int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapBufferPermission snap_buf_perm[BUFFERCLIENT_MAX] = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BUFFER_PERMISSION, &snap_buf_perm);
    error = CheckMetadataSet(SnapMetadataType::BUFFER_PERMISSION, error, check_metadata_set);
    BufferPermission gr_buf_perm[BUFFERCLIENT_MAX] = {};
    if (hidl_bytestream) {
      std::memcpy(&gr_buf_perm, &snap_buf_perm, sizeof(snap_buf_perm));
      if (qtigralloc::encodeBufferPermission(gr_buf_perm,
                                             static_cast<hidl_vec<uint8_t> *>(gralloc_out_get)) !=
          GrallocError::NONE) {
        return SnapError::BAD_VALUE;
      }
    } else {
      std::memcpy(gralloc_out_get, &snap_buf_perm, sizeof(snap_buf_perm));
    }
  } else if (gralloc_in_set != nullptr) {
    SnapBufferPermission snap_buf_perm[BUFFERCLIENT_MAX] = {};
    BufferPermission gr_buf_perm[BUFFERCLIENT_MAX] = {};
    if (hidl_bytestream) {
      if (qtigralloc::decodeBufferPermission(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                             gr_buf_perm) != GrallocError::NONE) {
        return SnapError::UNSUPPORTED;
      }
      std::memcpy(snap_buf_perm, gr_buf_perm, sizeof(gr_buf_perm));
    } else {
      std::memcpy(snap_buf_perm, gralloc_in_set, sizeof(SnapBufferPermission));
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::BUFFER_PERMISSION, &snap_buf_perm);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::MemHandleHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                   uint32_t aidl_size, void *gralloc_in_set,
                                                   void *gralloc_out_get, SnapDescriptor *buf_des,
                                                   bool check_metadata_set,
                                                   int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    int64_t mem_handle = 0;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::MEM_HANDLE, &mem_handle);
    error = CheckMetadataSet(SnapMetadataType::MEM_HANDLE, error, check_metadata_set);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeInt64(qtigralloc::MetadataType_MemHandle, mem_handle,
                                         static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<int64_t *>(gralloc_out_get) = static_cast<int64_t>(mem_handle);
    }
  } else if (gralloc_in_set != nullptr) {
    int64_t mem_handle = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeInt64(qtigralloc::MetadataType_MemHandle,
                                         *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                         &mem_handle)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      mem_handle = *static_cast<int64_t *>(gralloc_in_set);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::MEM_HANDLE, &mem_handle);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::TimedRenderingHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                        uint32_t aidl_size, void *gralloc_in_set,
                                                        void *gralloc_out_get,
                                                        SnapDescriptor *buf_des,
                                                        bool check_metadata_set,
                                                        int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    uint32_t timed_rendering = 0;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::TIMED_RENDERING, &timed_rendering);
    error = CheckMetadataSet(SnapMetadataType::TIMED_RENDERING, error, check_metadata_set);
    if (hidl_bytestream) {
      if (android::gralloc4::encodeUint32(qtigralloc::MetadataType_TimedRendering, timed_rendering,
                                          static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<uint32_t *>(gralloc_out_get) = static_cast<uint32_t>(timed_rendering);
    }
  } else if (gralloc_in_set != nullptr) {
    uint32_t timed_rendering = 0;
    if (hidl_bytestream) {
      if (android::gralloc4::decodeUint32(qtigralloc::MetadataType_TimedRendering,
                                          *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                          &timed_rendering)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      timed_rendering = *static_cast<uint32_t *>(gralloc_in_set);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::TIMED_RENDERING, &timed_rendering);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::UBWCCRStatsInfoHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                         uint32_t aidl_size, void *gralloc_in_set,
                                                         void *gralloc_out_get,
                                                         SnapDescriptor *buf_des,
                                                         bool check_metadata_set,
                                                         int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapUBWCStats snap_ubwc_stats[QTI_UBWC_STATS_ARRAY_SIZE] = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::UBWC_CR_STATS_INFO, snap_ubwc_stats);
    error = CheckMetadataSet(SnapMetadataType::UBWC_CR_STATS_INFO, error, check_metadata_set);
    UBWCStats gr_ubwc_stats[QTI_UBWC_STATS_ARRAY_SIZE] = {};
    std::memcpy(&gr_ubwc_stats, &snap_ubwc_stats, sizeof(snap_ubwc_stats));
    if (hidl_bytestream) {
      if (qtigralloc::encodeUBWCStats(gr_ubwc_stats, static_cast<hidl_vec<uint8_t> *>(
                                                         gralloc_out_get)) != GrallocError::NONE) {
        return SnapError::BAD_VALUE;
      }
    } else {
      memcpy(gralloc_out_get, &gr_ubwc_stats, sizeof(gr_ubwc_stats));
    }
  } else if (gralloc_in_set != nullptr) {
    SnapUBWCStats snap_ubwc_stats[QTI_UBWC_STATS_ARRAY_SIZE] = {};
    if (hidl_bytestream) {
      UBWCStats gr_ubwc_stats[QTI_UBWC_STATS_ARRAY_SIZE] = {};
      if (qtigralloc::decodeUBWCStats(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                      gr_ubwc_stats) != GrallocError::NONE) {
        return SnapError::UNSUPPORTED;
      }
      memcpy(&snap_ubwc_stats, &gr_ubwc_stats, sizeof(gr_ubwc_stats));
    } else {
      UBWCStats *gr_ubwc_stats = reinterpret_cast<UBWCStats *>(gralloc_in_set);
      memcpy(snap_ubwc_stats, gr_ubwc_stats, sizeof(snap_ubwc_stats));
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::UBWC_CR_STATS_INFO, &snap_ubwc_stats);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::CVPMetadataHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                     uint32_t aidl_size, void *gralloc_in_set,
                                                     void *gralloc_out_get, SnapDescriptor *buf_des,
                                                     bool check_metadata_set,
                                                     int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapCVPMetadata snap_cvp_metadata = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CVP_METADATA, &snap_cvp_metadata);
    error = CheckMetadataSet(SnapMetadataType::CVP_METADATA, error, check_metadata_set);
    CVPMetadata gr_cvp_metadata = {};
    memcpy(&gr_cvp_metadata, &snap_cvp_metadata, sizeof(snap_cvp_metadata));
    if (hidl_bytestream) {
      if (qtigralloc::encodeCVPMetadata(gr_cvp_metadata,
                                        static_cast<hidl_vec<uint8_t> *>(gralloc_out_get)) !=
          GrallocError::NONE) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<CVPMetadata *>(gralloc_out_get) = gr_cvp_metadata;
    }
  } else if (gralloc_in_set != nullptr) {
    SnapCVPMetadata snap_cvp_metadata = {};
    CVPMetadata gr_cvp_metadata = {};
    if (hidl_bytestream) {
      if (qtigralloc::decodeCVPMetadata(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                        &gr_cvp_metadata) != GrallocError::NONE) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      gr_cvp_metadata = *static_cast<CVPMetadata *>(gralloc_in_set);
    }
    memcpy(&snap_cvp_metadata, &gr_cvp_metadata, sizeof(gr_cvp_metadata));
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::CVP_METADATA, &snap_cvp_metadata);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::VideoTranscodeStatsHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapVideoTranscodeStatsMetadata snap_videotranscode_stats = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::VIDEO_TRANSCODE_STATS,
                                     &snap_videotranscode_stats);
    error = CheckMetadataSet(SnapMetadataType::VIDEO_TRANSCODE_STATS, error, check_metadata_set);
    VideoTranscodeStatsMetadata gr_videotranscode_stats = {};
    memcpy(&gr_videotranscode_stats, &snap_videotranscode_stats, sizeof(snap_videotranscode_stats));
    if (hidl_bytestream) {
      if (qtigralloc::encodeVideoTranscodeStatsMetadata(
              gr_videotranscode_stats, static_cast<hidl_vec<uint8_t> *>(gralloc_out_get)) !=
          GrallocError::NONE) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<VideoTranscodeStatsMetadata *>(gralloc_out_get) = gr_videotranscode_stats;
    }
  } else if (gralloc_in_set != nullptr) {
    SnapVideoTranscodeStatsMetadata snap_videotranscode_stats = {};
    VideoTranscodeStatsMetadata gr_videotranscode_stats = {};
    if (hidl_bytestream) {
      if (qtigralloc::decodeVideoTranscodeStatsMetadata(
              *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set), &gr_videotranscode_stats) !=
          GrallocError::NONE) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      gr_videotranscode_stats = *static_cast<VideoTranscodeStatsMetadata *>(gralloc_in_set);
    }
    memcpy(&snap_videotranscode_stats, &gr_videotranscode_stats, sizeof(gr_videotranscode_stats));
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::VIDEO_TRANSCODE_STATS,
                                     &snap_videotranscode_stats);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::VideoTSInfoHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                     uint32_t aidl_size, void *gralloc_in_set,
                                                     void *gralloc_out_get, SnapDescriptor *buf_des,
                                                     bool check_metadata_set,
                                                     int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapVideoTimestampInfo snap_video_timestamp_info = {};
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::VIDEO_TS_INFO, &snap_video_timestamp_info);
    error = CheckMetadataSet(SnapMetadataType::VIDEO_TS_INFO, error, check_metadata_set);
    VideoTimestampInfo gr_video_timestamp_info = {};
    memcpy(&gr_video_timestamp_info, &snap_video_timestamp_info, sizeof(snap_video_timestamp_info));
    if (hidl_bytestream) {
      if (qtigralloc::encodeVideoTimestampInfo(gr_video_timestamp_info,
                                               static_cast<hidl_vec<uint8_t> *>(gralloc_out_get)) !=
          GrallocError::NONE) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<VideoTimestampInfo *>(gralloc_out_get) = gr_video_timestamp_info;
    }
  } else if (gralloc_in_set != nullptr) {
    SnapVideoTimestampInfo snap_video_timestamp_info = {};
    VideoTimestampInfo gr_video_timestamp_info = {};
    if (hidl_bytestream) {
      if (qtigralloc::decodeVideoTimestampInfo(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                               &gr_video_timestamp_info) != GrallocError::NONE) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      gr_video_timestamp_info = *static_cast<VideoTimestampInfo *>(gralloc_in_set);
    }
    memcpy(&snap_video_timestamp_info, &gr_video_timestamp_info, sizeof(gr_video_timestamp_info));
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::VIDEO_TS_INFO, &snap_video_timestamp_info);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::VideoHistogramStatsHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapVideoHistogramMetadata snap_video_histogram_metadata = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::VIDEO_HISTOGRAM_STATS,
                                     &snap_video_histogram_metadata);
    error = CheckMetadataSet(SnapMetadataType::VIDEO_HISTOGRAM_STATS, error, check_metadata_set);
    VideoHistogramMetadata gr_video_histogram_metadata = {};
    memcpy(&gr_video_histogram_metadata, &snap_video_histogram_metadata,
           sizeof(snap_video_histogram_metadata));
    if (hidl_bytestream) {
      if (qtigralloc::encodeVideoHistogramMetadata(
              gr_video_histogram_metadata, static_cast<hidl_vec<uint8_t> *>(gralloc_out_get)) !=
          GrallocError::NONE) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<VideoHistogramMetadata *>(gralloc_out_get) = gr_video_histogram_metadata;
    }
  } else if (gralloc_in_set != nullptr) {
    SnapVideoHistogramMetadata snap_video_histogram_metadata = {};
    VideoHistogramMetadata gr_video_histogram_metadata = {};
    if (hidl_bytestream) {
      if (qtigralloc::decodeVideoHistogramMetadata(
              *static_cast<hidl_vec<uint8_t> *>(gralloc_in_set), &gr_video_histogram_metadata) !=
          GrallocError::NONE) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      gr_video_histogram_metadata = *static_cast<VideoHistogramMetadata *>(gralloc_in_set);
    }
    memcpy(&snap_video_histogram_metadata, &gr_video_histogram_metadata,
           sizeof(gr_video_histogram_metadata));
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::VIDEO_HISTOGRAM_STATS,
                                     &snap_video_histogram_metadata);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::CustomContentMetadataHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapCustomContentMetadata snap_customcontent_metadata = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CUSTOM_CONTENT_METADATA,
                                     &snap_customcontent_metadata);
    error = CheckMetadataSet(SnapMetadataType::CUSTOM_CONTENT_METADATA, error, check_metadata_set);
    CustomContentMetadata custom_content_metadata = {};
    memcpy(&custom_content_metadata, &snap_customcontent_metadata,
           sizeof(snap_customcontent_metadata));
    if (hidl_bytestream) {
      if (qtigralloc::encodeCustomContentMetadata(
              &custom_content_metadata, static_cast<hidl_vec<uint8_t> *>(gralloc_out_get)) !=
          GrallocError::NONE) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<CustomContentMetadata *>(gralloc_out_get) =
          static_cast<CustomContentMetadata>(custom_content_metadata);
    }
  } else if (gralloc_in_set != nullptr) {
    CustomContentMetadata custom_content_metadata = {};
    SnapCustomContentMetadata snap_customcontent_metadata = {};
    if (hidl_bytestream) {
      if (qtigralloc::decodeCustomContentMetadata(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                                  &custom_content_metadata) != GrallocError::NONE) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      memcpy(&custom_content_metadata, gralloc_in_set, sizeof(SnapCustomContentMetadata));
    }
    memcpy(&snap_customcontent_metadata, &custom_content_metadata, sizeof(custom_content_metadata));
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::CUSTOM_CONTENT_METADATA,
                                     &snap_customcontent_metadata);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::SMPTE2094_10Helper(SnapHandle *hnd, bool hidl_bytestream,
                                                      uint32_t aidl_size, void *gralloc_in_set,
                                                      void *gralloc_out_get,
                                                      SnapDescriptor *buf_des,
                                                      bool check_metadata_set,
                                                      int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapCustomContentMetadata snap_custom_metadata = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::SMPTE2094_10, &snap_custom_metadata);
    error = CheckMetadataSet(SnapMetadataType::SMPTE2094_10, error, check_metadata_set);
    std::vector<uint8_t> custom_metadata_payload = {};
    custom_metadata_payload.resize(sizeof(snap_custom_metadata.metadataPayload));
    memcpy(custom_metadata_payload.data(), &snap_custom_metadata.metadataPayload,
           sizeof(snap_custom_metadata.metadataPayload));
    if (hidl_bytestream) {
      if (snap_custom_metadata.size <= CUSTOM_METADATA_SIZE_BYTES) {
        if (android::gralloc4::encodeSmpte2094_10(
                custom_metadata_payload, static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
          return SnapError::BAD_VALUE;
        }
      } else {
        if (android::gralloc4::encodeSmpte2094_10(
                std::nullopt, static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
          return SnapError::BAD_VALUE;
        }
      }
    } else {
      *static_cast<std::vector<uint8_t> *>(gralloc_out_get) = custom_metadata_payload;
    }
  } else if (gralloc_in_set != nullptr) {
    SnapCustomContentMetadata snap_custom_metadata = {};
    std::optional<std::vector<uint8_t>> custom_metadata_payload = {};
    if (hidl_bytestream) {
      if (android::gralloc4::decodeSmpte2094_10(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                                &custom_metadata_payload)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      custom_metadata_payload = *static_cast<std::optional<std::vector<uint8_t>> *>(gralloc_in_set);
    }
    snap_custom_metadata.size = static_cast<int>(custom_metadata_payload->size());
    memcpy(&snap_custom_metadata.metadataPayload, custom_metadata_payload->data(),
           custom_metadata_payload->size());
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::SMPTE2094_10, &snap_custom_metadata);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::MasteringDisplayHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                          uint32_t aidl_size, void *gralloc_in_set,
                                                          void *gralloc_out_get,
                                                          SnapDescriptor *buf_des,
                                                          bool check_metadata_set,
                                                          int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  // Conversion factors for Snap <=> AIDL/AOSP conversion
  // AIDL equivalent struct uses 1:1 units where as Snap uses 1/50k for primaries and whitepoint,
  // and 1/10k for minDisplayLuminance
  constexpr float snap_units[2] = {50000.0f, 10000.0f};
  if (gralloc_out_get != nullptr) {
    SnapMasteringDisplay snap_mastering_display_values = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::MASTERING_DISPLAY,
                                     &snap_mastering_display_values);
    error = CheckMetadataSet(SnapMetadataType::MASTERING_DISPLAY, error, check_metadata_set);
    std::optional<GrallocSmpte2086> mastering_display_values = {};
    GrallocSmpte2086 smpte2086;
    if (snap_mastering_display_values.colorVolumeSEIEnabled) {
      smpte2086.primaryRed = {
          static_cast<float>(snap_mastering_display_values.primaryRed.x) / snap_units[0],
          static_cast<float>(snap_mastering_display_values.primaryRed.y) / snap_units[0]};
      smpte2086.primaryGreen = {
          static_cast<float>(snap_mastering_display_values.primaryGreen.x) / snap_units[0],
          static_cast<float>(snap_mastering_display_values.primaryGreen.y) / snap_units[0]};
      smpte2086.primaryBlue = {
          static_cast<float>(snap_mastering_display_values.primaryBlue.x) / snap_units[0],
          static_cast<float>(snap_mastering_display_values.primaryBlue.y) / snap_units[0]};
      smpte2086.whitePoint = {
          static_cast<float>(snap_mastering_display_values.whitePoint.x) / snap_units[0],
          static_cast<float>(snap_mastering_display_values.whitePoint.y) / snap_units[0]};
      smpte2086.maxLuminance =
          static_cast<float>(snap_mastering_display_values.maxDisplayLuminance);
      smpte2086.minLuminance =
          static_cast<float>(snap_mastering_display_values.minDisplayLuminance) / snap_units[1];
      mastering_display_values = std::move(smpte2086);
    }
    if (aidl_size) {
      *mapper_return = Mapper5Encode<StandardMetadataType::SMPTE2086>(
          mastering_display_values, gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    } else if (hidl_bytestream) {
      if (android::gralloc4::encodeSmpte2086(mastering_display_values,
                                             static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<std::optional<GrallocSmpte2086> *>(gralloc_out_get) = mastering_display_values;
    }
  } else if (gralloc_in_set != nullptr) {
    SnapMasteringDisplay snap_mastering_display_values = {};
    std::optional<GrallocSmpte2086> mastering_display_values = {};
    if (aidl_size) {
      auto decoded_result =
          Mapper5Decode<StandardMetadataType::SMPTE2086>(gralloc_in_set, aidl_size);
      if (!decoded_result.has_value()) {
        return SnapError::UNSUPPORTED;
      }
      mastering_display_values = *decoded_result;
    } else if (hidl_bytestream) {
      if (android::gralloc4::decodeSmpte2086(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                             &mastering_display_values)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      mastering_display_values = *static_cast<std::optional<GrallocSmpte2086> *>(gralloc_in_set);
    }
    if (mastering_display_values != std::nullopt) {
      snap_mastering_display_values.colorVolumeSEIEnabled = true;
      snap_mastering_display_values.primaryRed = {
          static_cast<uint32_t>(mastering_display_values->primaryRed.x * snap_units[0]),
          static_cast<uint32_t>(mastering_display_values->primaryRed.y * snap_units[0])};
      snap_mastering_display_values.primaryGreen = {
          static_cast<uint32_t>(mastering_display_values->primaryGreen.x * snap_units[0]),
          static_cast<uint32_t>(mastering_display_values->primaryGreen.y * snap_units[0])};
      snap_mastering_display_values.primaryBlue = {
          static_cast<uint32_t>(mastering_display_values->primaryBlue.x * snap_units[0]),
          static_cast<uint32_t>(mastering_display_values->primaryBlue.y * snap_units[0])};
      snap_mastering_display_values.whitePoint = {
          static_cast<uint32_t>(mastering_display_values->whitePoint.x * snap_units[0]),
          static_cast<uint32_t>(mastering_display_values->whitePoint.y * snap_units[0])};
      snap_mastering_display_values.maxDisplayLuminance =
          static_cast<uint32_t>(mastering_display_values->maxLuminance);
      snap_mastering_display_values.minDisplayLuminance =
          static_cast<uint32_t>(mastering_display_values->minLuminance * snap_units[1]);
    } else {
      snap_mastering_display_values.colorVolumeSEIEnabled = false;
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::MASTERING_DISPLAY,
                                     &snap_mastering_display_values);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::ContentLightLevelHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                           uint32_t aidl_size, void *gralloc_in_set,
                                                           void *gralloc_out_get,
                                                           SnapDescriptor *buf_des,
                                                           bool check_metadata_set,
                                                           int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapContentLightLevel snap_content_light_level = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CONTENT_LIGHT_LEVEL,
                                     &snap_content_light_level);
    error = CheckMetadataSet(SnapMetadataType::CONTENT_LIGHT_LEVEL, error, check_metadata_set);
    std::optional<GrallocCta861_3> content_light_level = {};
    GrallocCta861_3 cta861_3;
    if (snap_content_light_level.lightLevelSEIEnabled) {
      cta861_3.maxContentLightLevel =
          static_cast<float>(snap_content_light_level.maxContentLightLevel);
      cta861_3.maxFrameAverageLightLevel =
          static_cast<float>(snap_content_light_level.maxFrameAverageLightLevel);
      content_light_level = std::move(cta861_3);
    }
    if (aidl_size) {
      *mapper_return = Mapper5Encode<StandardMetadataType::CTA861_3>(
          content_light_level, gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    } else if (hidl_bytestream) {
      if (android::gralloc4::encodeCta861_3(content_light_level,
                                            static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
        return SnapError::BAD_VALUE;
      }
    } else {
      *static_cast<std::optional<GrallocCta861_3> *>(gralloc_out_get) = content_light_level;
    }
  } else if (gralloc_in_set != nullptr) {
    SnapContentLightLevel snap_content_light_level = {};
    std::optional<GrallocCta861_3> content_light_level = {};
    if (aidl_size) {
      auto decoded_result =
          Mapper5Decode<StandardMetadataType::CTA861_3>(gralloc_in_set, aidl_size);
      if (!decoded_result.has_value()) {
        return SnapError::UNSUPPORTED;
      }
      content_light_level = *decoded_result;
    } else if (hidl_bytestream) {
      if (android::gralloc4::decodeCta861_3(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                            &content_light_level)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      content_light_level = *static_cast<std::optional<GrallocCta861_3> *>(gralloc_in_set);
    }
    if (content_light_level != std::nullopt) {
      snap_content_light_level.lightLevelSEIEnabled = true;
      snap_content_light_level.maxContentLightLevel =
          static_cast<uint32_t>(content_light_level->maxContentLightLevel);
      snap_content_light_level.maxFrameAverageLightLevel =
          static_cast<uint32_t>(content_light_level->maxFrameAverageLightLevel);
    } else {
      snap_content_light_level.lightLevelSEIEnabled = false;
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::CONTENT_LIGHT_LEVEL,
                                     &snap_content_light_level);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::DynamicMetadataHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                         uint32_t aidl_size, void *gralloc_in_set,
                                                         void *gralloc_out_get,
                                                         SnapDescriptor *buf_des,
                                                         bool check_metadata_set,
                                                         int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapDynamicMetadata snap_dynamic_metadata = {};
    error =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::DYNAMIC_METADATA, &snap_dynamic_metadata);
    error = CheckMetadataSet(SnapMetadataType::DYNAMIC_METADATA, error, check_metadata_set);
    std::vector<uint8_t> dynamic_metadata_payload = {};
    dynamic_metadata_payload.resize(sizeof(snap_dynamic_metadata.dynamicMetaDataPayload));
    memcpy(dynamic_metadata_payload.data(), &snap_dynamic_metadata.dynamicMetaDataPayload,
           sizeof(snap_dynamic_metadata.dynamicMetaDataPayload));
    if (aidl_size) {
      *mapper_return = Mapper5Encode<StandardMetadataType::SMPTE2094_40>(
          dynamic_metadata_payload, gralloc_out_get, *mapper_return);
      if (*mapper_return < 0) {
        return SnapError::BAD_VALUE;
      }
    } else if (hidl_bytestream) {
      if (snap_dynamic_metadata.dynamicMetaDataValid &&
          snap_dynamic_metadata.dynamicMetaDataLen <= HDR_DYNAMIC_META_DATA_SZ) {
        if (android::gralloc4::encodeSmpte2094_40(
                dynamic_metadata_payload, static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
          return SnapError::BAD_VALUE;
        }
      } else {
        if (android::gralloc4::encodeSmpte2094_40(
                std::nullopt, static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
          return SnapError::BAD_VALUE;
        }
      }
    } else {
      *static_cast<std::vector<uint8_t> *>(gralloc_out_get) = dynamic_metadata_payload;
    }
  } else if (gralloc_in_set != nullptr) {
    SnapDynamicMetadata snap_dynamic_metadata = {};
    std::optional<std::vector<uint8_t>> dynamic_metadata_payload = {};
    if (aidl_size) {
      auto decoded_result =
          Mapper5Decode<StandardMetadataType::SMPTE2094_40>(gralloc_in_set, aidl_size);
      if (!decoded_result.has_value()) {
        return SnapError::UNSUPPORTED;
      }
      dynamic_metadata_payload = *decoded_result;
    } else if (hidl_bytestream) {
      if (android::gralloc4::decodeSmpte2094_40(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                                &dynamic_metadata_payload)) {
        return SnapError::UNSUPPORTED;
      }
    } else {
      dynamic_metadata_payload =
          *static_cast<std::optional<std::vector<uint8_t>> *>(gralloc_in_set);
    }
    if (dynamic_metadata_payload != std::nullopt) {
      snap_dynamic_metadata.dynamicMetaDataLen = static_cast<int>(dynamic_metadata_payload->size());
      memcpy(&snap_dynamic_metadata.dynamicMetaDataPayload, dynamic_metadata_payload->data(),
             dynamic_metadata_payload->size());
      snap_dynamic_metadata.dynamicMetaDataValid = true;
    } else {
      snap_dynamic_metadata.dynamicMetaDataValid = false;
    }
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::DYNAMIC_METADATA, &snap_dynamic_metadata);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::ColorRemappingInfoHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapColorRemappingInfo snap_color_remapping_info = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::COLOR_REMAPPING_INFO,
                                     &snap_color_remapping_info);
    error = CheckMetadataSet(SnapMetadataType::COLOR_REMAPPING_INFO, error, check_metadata_set);
    ColorRemappingInfo gr_crI = {};
    memcpy(&gr_crI, &snap_color_remapping_info, sizeof(snap_color_remapping_info));
    if (hidl_bytestream) {
      hidl_vec<uint8_t> *in = static_cast<hidl_vec<uint8_t> *>(gralloc_out_get);
      memcpy(in->data(), &gr_crI, sizeof(gr_crI));
    } else {
      *static_cast<ColorRemappingInfo *>(gralloc_out_get) = gr_crI;
    }
  } else if (gralloc_in_set != nullptr) {
    SnapColorRemappingInfo snap_color_remapping_info = {};
    ColorRemappingInfo gr_crI = {};
    if (hidl_bytestream) {
      hidl_vec<uint8_t> *in = static_cast<hidl_vec<uint8_t> *>(gralloc_in_set);
      memcpy(&gr_crI, in->data(), sizeof(gr_crI));
    } else {
      gr_crI = *static_cast<ColorRemappingInfo *>(gralloc_in_set);
    }
    memcpy(&snap_color_remapping_info, &gr_crI, sizeof(gr_crI));
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::COLOR_REMAPPING_INFO,
                                     &snap_color_remapping_info);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::HeapNameHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                  uint32_t aidl_size, void *gralloc_in_set,
                                                  void *gralloc_out_get, SnapDescriptor *buf_des,
                                                  bool check_metadata_set, int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  std::string heap_name = "";
  if (gralloc_in_set != nullptr) {
    return SnapError::UNSUPPORTED;
  }
  if (gralloc_out_get != nullptr) {
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::HEAP_NAME, &heap_name);
  }
  error = CheckMetadataSet(SnapMetadataType::HEAP_NAME, error, check_metadata_set);
  if (hidl_bytestream) {
    if (android::gralloc4::encodeString(qtigralloc::MetadataType_HeapName, heap_name,
                                        static_cast<hidl_vec<uint8_t> *>(gralloc_out_get))) {
      return SnapError::BAD_VALUE;
    }
  } else {
    *static_cast<std::string *>(gralloc_out_get) = static_cast<std::string>(heap_name.c_str());
  }
  return error;
}

int GrallocSnapHelperLegacy::GetMetadata(native_handle_t *gr_hnd, uint64_t gr_metadata_type,
                                         void *out, bool convert_bytestream,
                                         bool check_metadata_set, uint32_t aidl_size,
                                         int32_t *mapper_return) {
  if (gr_hnd == nullptr) {
    ALOGE("%s: Invalid gralloc handle", __FUNCTION__);
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    auto snap_metadata_type = GetSnapMetadataType(gr_metadata_type);
    if (snap_metadata_type == SnapMetadataType::METADATA_TYPE_INVALID) {
      if (deprecated_metadata_conversion_helper_function_map_.find(gr_metadata_type) !=
          deprecated_metadata_conversion_helper_function_map_.end()) {
        MetadataHelper metadata_helper_func =
            deprecated_metadata_conversion_helper_function_map_[gr_metadata_type];
        return ((this->*metadata_helper_func)(hnd, convert_bytestream, 0, nullptr, out, nullptr,
                                              check_metadata_set, mapper_return));
      } else {
        ALOGW("%s: No map for metadata_type: %lu", __FUNCTION__, gr_metadata_type);
        return SnapError::UNSUPPORTED;
      }
    }

    if (metadata_conversion_helper_function_map.find(snap_metadata_type) !=
        metadata_conversion_helper_function_map.end()) {
      MetadataHelper metadata_helper_func =
          metadata_conversion_helper_function_map[snap_metadata_type];
      auto error = ((this->*metadata_helper_func)(hnd, convert_bytestream, aidl_size, nullptr, out,
                                                  nullptr, check_metadata_set, mapper_return));
      if (error == SnapError::METADATA_NOT_SET && !check_metadata_set) {
        ALOGI(
            "%s: Metadata type %d is not set.Returning default values as check_metadata_set is %d",
            __FUNCTION__, gr_metadata_type, check_metadata_set);
        return SnapError::NONE;
      }
      return error;
    } else {
      return SnapError::UNSUPPORTED;
    }
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
    return SnapError::BAD_BUFFER;
  }

  return SnapError::UNSUPPORTED;
}

int GrallocSnapHelperLegacy::GetMetadataState(native_handle_t *gr_hnd,
                                              SnapMetadataType metadata_type, bool *out) {
  if (gr_hnd == nullptr) {
    ALOGE("%s: Invalid gralloc handle", __FUNCTION__);
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    auto status = snapmapper_->GetMetadataState(*hnd, metadata_type, out);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGW("%s: Failed to get metadata state for metadata type %d via SnapAlloc. Error code: %d",
            __FUNCTION__, metadata_type, status);
    }
    return status;
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }

  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelperLegacy::SetMetadata(native_handle_t *gr_hnd, uint64_t gr_metadata_type,
                                         hidl_vec<uint8_t> in) {
  if (gr_hnd == nullptr) {
    ALOGE("%s: Invalid gralloc handle", __FUNCTION__);
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    auto snap_metadata_type = GetSnapMetadataType(gr_metadata_type);
    if (snap_metadata_type == SnapMetadataType::METADATA_TYPE_INVALID) {
      if (deprecated_metadata_conversion_helper_function_map_.find(gr_metadata_type) !=
          deprecated_metadata_conversion_helper_function_map_.end()) {
        MetadataHelper metadata_helper_func =
            deprecated_metadata_conversion_helper_function_map_[gr_metadata_type];
        return ((this->*metadata_helper_func)(hnd, true, 0, &in, nullptr, nullptr, false, nullptr));
      } else {
        ALOGW("%s: No map for metadata_type: %lu", __FUNCTION__, gr_metadata_type);
        return SnapError::UNSUPPORTED;
      }
    }

    if (metadata_conversion_helper_function_map.find(snap_metadata_type) !=
        metadata_conversion_helper_function_map.end()) {
      MetadataHelper metadata_helper_func =
          metadata_conversion_helper_function_map[snap_metadata_type];
      auto error =
          ((this->*metadata_helper_func)(hnd, true, 0, &in, nullptr, nullptr, false, nullptr));
      if (error == SnapError::BAD_VALUE || error == SnapError::UNSUPPORTED) {
        ALOGW("%s: Unable to set metadata - metadata type %d error %d", __FUNCTION__,
              snap_metadata_type, static_cast<int>(error));
      }
      return error;
    } else {
      return SnapError::UNSUPPORTED;
    }
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }
  return SnapError::BAD_BUFFER;
}

int GrallocSnapHelperLegacy::SetMetadata(native_handle_t *gr_hnd, uint64_t gr_metadata_type,
                                         void *in, uint32_t aidl_size) {
  if (gr_hnd == nullptr) {
    ALOGE("%s: Invalid gralloc handle", __FUNCTION__);
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  SnapHandle *hnd = nullptr;
  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    hnd = handles_map_.at(gr_hnd);
  }

  if (hnd != nullptr) {
    auto snap_metadata_type = GetSnapMetadataType(gr_metadata_type);
    if (snap_metadata_type == SnapMetadataType::METADATA_TYPE_INVALID) {
      if (deprecated_metadata_conversion_helper_function_map_.find(gr_metadata_type) !=
          deprecated_metadata_conversion_helper_function_map_.end()) {
        MetadataHelper metadata_helper_func =
            deprecated_metadata_conversion_helper_function_map_[gr_metadata_type];
        return ((this->*metadata_helper_func)(hnd, false, aidl_size, in, nullptr, nullptr, false,
                                              nullptr));
      } else {
        ALOGW("%s: No map for metadata_type: %lu", __FUNCTION__, gr_metadata_type);
        return SnapError::UNSUPPORTED;
      }
    }

    if (metadata_conversion_helper_function_map.find(snap_metadata_type) !=
        metadata_conversion_helper_function_map.end()) {
      MetadataHelper metadata_helper_func =
          metadata_conversion_helper_function_map[snap_metadata_type];
      auto error = ((this->*metadata_helper_func)(hnd, false, aidl_size, in, nullptr, nullptr,
                                                  false, nullptr));
      if (error == SnapError::BAD_VALUE || error == SnapError::UNSUPPORTED) {
        ALOGW("%s: Unable to set metadata - metadata type %d error %d", __FUNCTION__,
              snap_metadata_type, error);
      }
      return error;
    } else {
      return SnapError::UNSUPPORTED;
    }
  } else {
    ALOGE("%s: Failed to get SnapHandle for gralloc handle %p", __FUNCTION__, gr_hnd);
  }
  return SnapError::BAD_BUFFER;
}

SnapError GrallocSnapHelperLegacy::ColorMetadataHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                       uint32_t aidl_size, void *gralloc_in_set,
                                                       void *gralloc_out_get,
                                                       SnapDescriptor *buf_des,
                                                       bool check_metadata_set,
                                                       int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    ColorMetaData color_metadata;
    SnapDataspace dataspace;
    auto status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::DATASPACE, &dataspace);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGW("%s Unable to get DATASPACE from snap", __FUNCTION__);
    } else {
      color_metadata.colorPrimaries =
          static_cast<ColorPrimaries>(static_cast<uint32_t>(dataspace.colorPrimaries));
      color_metadata.range = static_cast<ColorRange>(static_cast<uint32_t>(dataspace.range));
      color_metadata.transfer =
          static_cast<GammaTransfer>(static_cast<uint32_t>(dataspace.transfer));
    }

    SnapMasteringDisplay snap_mastering_display_values;
    status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::MASTERING_DISPLAY,
                                      &snap_mastering_display_values);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGW("%s Unable to get MASTERING_DISPLAY from snap", __FUNCTION__);
    } else {
      color_metadata.masteringDisplayInfo.colorVolumeSEIEnabled = true;
      color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[0][0] =
          static_cast<uint32_t>(snap_mastering_display_values.primaryRed.x);
      color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[0][1] =
          static_cast<uint32_t>(snap_mastering_display_values.primaryRed.y);

      color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[1][0] =
          static_cast<uint32_t>(snap_mastering_display_values.primaryGreen.x);
      color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[1][1] =
          static_cast<uint32_t>(snap_mastering_display_values.primaryGreen.y);

      color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[2][0] =
          static_cast<uint32_t>(snap_mastering_display_values.primaryBlue.x);
      color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[2][1] =
          static_cast<uint32_t>(snap_mastering_display_values.primaryBlue.y);

      color_metadata.masteringDisplayInfo.primaries.whitePoint[0] =
          static_cast<uint32_t>(snap_mastering_display_values.whitePoint.x);
      color_metadata.masteringDisplayInfo.primaries.whitePoint[1] =
          static_cast<uint32_t>(snap_mastering_display_values.whitePoint.y);

      color_metadata.masteringDisplayInfo.maxDisplayLuminance =
          static_cast<uint32_t>(snap_mastering_display_values.maxDisplayLuminance);
      color_metadata.masteringDisplayInfo.minDisplayLuminance =
          static_cast<uint32_t>(snap_mastering_display_values.minDisplayLuminance);
    }

    SnapContentLightLevel snap_content_light_level;
    status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::CONTENT_LIGHT_LEVEL,
                                      &snap_content_light_level);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGW("%s Unable to get CONTENT_LIGHT_LEVEL from snap", __FUNCTION__);
    } else {
      color_metadata.contentLightLevel.lightLevelSEIEnabled = true;
      color_metadata.contentLightLevel.maxContentLightLevel =
          static_cast<uint32_t>(snap_content_light_level.maxContentLightLevel);
      color_metadata.contentLightLevel.minPicAverageLightLevel =
          static_cast<uint32_t>(snap_content_light_level.maxFrameAverageLightLevel);
    }

    SnapDynamicMetadata snap_dynamic_metadata;
    status =
        snapmapper_->GetMetadata(*hnd, SnapMetadataType::DYNAMIC_METADATA, &snap_dynamic_metadata);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGW("%s Unable to get DYNAMIC_METADATA from snap", __FUNCTION__);
    } else {
      color_metadata.dynamicMetaDataLen = snap_dynamic_metadata.dynamicMetaDataLen;
      color_metadata.dynamicMetaDataValid = true;
      memcpy(&color_metadata.dynamicMetaDataPayload, &snap_dynamic_metadata.dynamicMetaDataPayload,
             snap_dynamic_metadata.dynamicMetaDataLen);
    }

    SnapColorRemappingInfo snap_color_remapping_info;
    status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::COLOR_REMAPPING_INFO,
                                      &snap_color_remapping_info);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGW("%s: Unable to get COLOR_REMAPPING_INFO from snap", __FUNCTION__);
    } else {
      ColorRemappingInfo gr_crI;
      gr_crI.criEnabled = static_cast<bool>(snap_color_remapping_info.criEnabled);
      gr_crI.crId = static_cast<uint32_t>(snap_color_remapping_info.crId);
      gr_crI.crCancelFlag = static_cast<uint32_t>(snap_color_remapping_info.crCancelFlag);
      gr_crI.crPersistenceFlag = static_cast<uint32_t>(snap_color_remapping_info.crPersistenceFlag);
      gr_crI.crVideoSignalInfoPresentFlag =
          static_cast<uint32_t>(snap_color_remapping_info.crVideoSignalInfoPresentFlag);
      gr_crI.crRange = static_cast<uint32_t>(snap_color_remapping_info.crRange);
      gr_crI.crInputBitDepth = static_cast<uint32_t>(snap_color_remapping_info.crInputBitDepth);
      gr_crI.crOutputBitDepth = static_cast<uint32_t>(snap_color_remapping_info.crOutputBitDepth);
      gr_crI.crMatrixPresentFlag =
          static_cast<uint32_t>(snap_color_remapping_info.crMatrixPresentFlag);
      gr_crI.crLog2MatrixDenom = static_cast<uint32_t>(snap_color_remapping_info.crLog2MatrixDenom);
      std::copy(std::begin(snap_color_remapping_info.crPreLutNumValMinusOne),
                std::end(snap_color_remapping_info.crPreLutNumValMinusOne),
                std::begin(gr_crI.crPreLutNumValMinusOne));
      std::copy(std::begin(snap_color_remapping_info.crPreLutCodedValue),
                std::end(snap_color_remapping_info.crPreLutCodedValue),
                std::begin(gr_crI.crPreLutCodedValue));
      std::copy(std::begin(snap_color_remapping_info.crPreLutTargetValue),
                std::end(snap_color_remapping_info.crPreLutTargetValue),
                std::begin(gr_crI.crPreLutTargetValue));
      std::copy(std::begin(snap_color_remapping_info.crCoefficients),
                std::end(snap_color_remapping_info.crCoefficients),
                std::begin(gr_crI.crCoefficients));
      std::copy(std::begin(snap_color_remapping_info.crPostLutNumValMinusOne),
                std::end(snap_color_remapping_info.crPostLutNumValMinusOne),
                std::begin(gr_crI.crPostLutNumValMinusOne));
      std::copy(std::begin(snap_color_remapping_info.crPostLutCodedValue),
                std::end(snap_color_remapping_info.crPostLutCodedValue),
                std::begin(gr_crI.crPostLutCodedValue));
      gr_crI.crMatrixCoefficients = static_cast<MatrixCoEfficients>(
          static_cast<uint32_t>(snap_color_remapping_info.crMatrixCoefficients));
      gr_crI.crTransferFunction = static_cast<GammaTransfer>(
          static_cast<uint32_t>(snap_color_remapping_info.crTransferFunction));
      gr_crI.crPrimaries =
          static_cast<ColorPrimaries>(static_cast<uint32_t>(snap_color_remapping_info.crPrimaries));
      color_metadata.cRI = gr_crI;
    }

    SnapMatrixCoEfficients snap_matrix_coefficients;
    status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::MATRIX_COEFFICIENTS,
                                      &snap_matrix_coefficients);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGW("%s: Unable to get MATRIX_COEFFICIENTS from snap", __FUNCTION__);
    } else {
      color_metadata.matrixCoefficients =
          static_cast<MatrixCoEfficients>(static_cast<int32_t>(snap_matrix_coefficients));
    }
    if (!hidl_bytestream) {
      *static_cast<ColorMetaData *>(gralloc_out_get) = color_metadata;
    } else {
      qtigralloc::encodeColorMetadata(*reinterpret_cast<ColorMetaData *>(&color_metadata),
                                      static_cast<hidl_vec<uint8_t> *>(gralloc_out_get));
    }
    return SnapError::NONE;
  } else if (gralloc_in_set != nullptr) {
    ColorMetaData color_metadata;
    if (hidl_bytestream) {
      qtigralloc::decodeColorMetadata(*static_cast<hidl_vec<uint8_t> *>(gralloc_in_set),
                                      &color_metadata);
    } else {
      color_metadata = *static_cast<ColorMetaData *>(gralloc_in_set);
    }

    SnapDataspace snap_dataspace;
    snap_dataspace.colorPrimaries = static_cast<SnapColorPrimaries>(color_metadata.colorPrimaries);
    snap_dataspace.range = static_cast<SnapColorRange>(color_metadata.range);
    snap_dataspace.transfer = static_cast<SnapGammaTransfer>(color_metadata.transfer);
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::DATASPACE, &snap_dataspace);
    if (error != SnapError::NONE) {
      ALOGE(
          "%s: Failed to set snap metadata type - DATASPACE via SnapAlloc. Error code: "
          "%d",
          __FUNCTION__, error);
      return error;
    }

    SnapMasteringDisplay snap_mastering_display_values;
    // Only convert values if enabled - otherwise send empty struct
    if (color_metadata.masteringDisplayInfo.colorVolumeSEIEnabled) {
      snap_mastering_display_values.primaryRed = {
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[0][0]),
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[0][1])};
      snap_mastering_display_values.primaryGreen = {
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[1][0]),
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[1][1])};
      snap_mastering_display_values.primaryBlue = {
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[2][0]),
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[2][1])};
      snap_mastering_display_values.whitePoint = {
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.whitePoint[0]),
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.primaries.whitePoint[1])};
      snap_mastering_display_values.maxDisplayLuminance =
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.maxDisplayLuminance);
      snap_mastering_display_values.minDisplayLuminance =
          static_cast<uint32_t>(color_metadata.masteringDisplayInfo.minDisplayLuminance);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::MASTERING_DISPLAY,
                                     &snap_mastering_display_values);
    if (error != SnapError::NONE) {
      ALOGE(
          "%s: Failed to set snap metadata type - MASTERING_DISPLAY via SnapAlloc. Error code: "
          "%d",
          __FUNCTION__, error);
      return error;
    }

    SnapContentLightLevel snap_content_light_level;
    // Only convert values if enabled - otherwise send empty struct
    if (color_metadata.contentLightLevel.lightLevelSEIEnabled) {
      snap_content_light_level.maxContentLightLevel =
          static_cast<float>(color_metadata.contentLightLevel.maxContentLightLevel);
      snap_content_light_level.maxFrameAverageLightLevel =
          static_cast<float>(color_metadata.contentLightLevel.minPicAverageLightLevel);
    }
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::CONTENT_LIGHT_LEVEL,
                                     &snap_content_light_level);
    if (error != SnapError::NONE) {
      ALOGE(
          "%s: Failed to set snap metadata type - CONTENT_LIGHT_LEVEL via SnapAlloc. Error "
          "code: %d",
          __FUNCTION__, error);
      return error;
    }
    SnapDynamicMetadata snap_dynamic_metadata;
    if (color_metadata.dynamicMetaDataValid &&
        color_metadata.dynamicMetaDataLen <= HDR_DYNAMIC_META_DATA_SZ) {
      snap_dynamic_metadata.dynamicMetaDataLen = color_metadata.dynamicMetaDataLen;
      snap_dynamic_metadata.dynamicMetaDataValid = color_metadata.dynamicMetaDataValid;
      memcpy(&snap_dynamic_metadata.dynamicMetaDataPayload, &color_metadata.dynamicMetaDataPayload,
             color_metadata.dynamicMetaDataLen);
    }
    error =
        snapmapper_->SetMetadata(*hnd, SnapMetadataType::DYNAMIC_METADATA, &snap_dynamic_metadata);
    if (error != SnapError::NONE) {
      ALOGE(
          "%s: Failed to set snap metadata type - DYNAMIC_METADATA via SnapAlloc. Error code: "
          "%d",
          __FUNCTION__, error);
    }

    SnapColorRemappingInfo snap_color_remapping_info;
    if (static_cast<bool>(color_metadata.cRI.criEnabled)) {
      ColorRemappingInfo gr_crI = color_metadata.cRI;
      snap_color_remapping_info.criEnabled = static_cast<bool>(gr_crI.criEnabled);
      snap_color_remapping_info.crId = static_cast<int>(gr_crI.crId);
      snap_color_remapping_info.crCancelFlag = static_cast<int>(gr_crI.crCancelFlag);
      snap_color_remapping_info.crPersistenceFlag = static_cast<int>(gr_crI.crPersistenceFlag);
      snap_color_remapping_info.crVideoSignalInfoPresentFlag =
          static_cast<int>(gr_crI.crVideoSignalInfoPresentFlag);
      snap_color_remapping_info.crRange = static_cast<int>(gr_crI.crRange);
      snap_color_remapping_info.crInputBitDepth = static_cast<int>(gr_crI.crInputBitDepth);
      snap_color_remapping_info.crOutputBitDepth = static_cast<int>(gr_crI.crOutputBitDepth);
      snap_color_remapping_info.crMatrixPresentFlag = static_cast<int>(gr_crI.crMatrixPresentFlag);
      snap_color_remapping_info.crLog2MatrixDenom = static_cast<int>(gr_crI.crLog2MatrixDenom);
      std::copy(std::begin(gr_crI.crPreLutNumValMinusOne), std::end(gr_crI.crPreLutNumValMinusOne),
                std::begin(snap_color_remapping_info.crPreLutNumValMinusOne));
      std::copy(std::begin(gr_crI.crPreLutCodedValue), std::end(gr_crI.crPreLutCodedValue),
                std::begin(snap_color_remapping_info.crPreLutCodedValue));
      std::copy(std::begin(gr_crI.crPreLutTargetValue), std::end(gr_crI.crPreLutTargetValue),
                std::begin(snap_color_remapping_info.crPreLutTargetValue));
      std::copy(std::begin(gr_crI.crCoefficients), std::end(gr_crI.crCoefficients),
                std::begin(snap_color_remapping_info.crCoefficients));
      std::copy(std::begin(gr_crI.crPostLutNumValMinusOne),
                std::end(gr_crI.crPostLutNumValMinusOne),
                std::begin(snap_color_remapping_info.crPostLutNumValMinusOne));
      std::copy(std::begin(gr_crI.crPostLutCodedValue), std::end(gr_crI.crPostLutCodedValue),
                std::begin(snap_color_remapping_info.crPostLutCodedValue));
      snap_color_remapping_info.crMatrixCoefficients =
          static_cast<SnapMatrixCoEfficients>(static_cast<int>(gr_crI.crMatrixCoefficients));
      snap_color_remapping_info.crTransferFunction =
          static_cast<SnapGammaTransfer>(static_cast<int>(gr_crI.crTransferFunction));
      snap_color_remapping_info.crPrimaries =
          static_cast<SnapColorPrimaries>(static_cast<int>(gr_crI.crPrimaries));
      error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::COLOR_REMAPPING_INFO,
                                       &snap_color_remapping_info);
      if (error != SnapError::NONE) {
        ALOGE(
            "%s: Failed to set snap metadata type - COLOR_REMAPPING_INFO via SnapAlloc. Error "
            "code: %d",
            __FUNCTION__, error);
      }
    }

    SnapMatrixCoEfficients snap_matrix_coefficients;
    if (color_metadata.matrixCoefficients > 0) {
      int64_t snap_matrix_coefficients = static_cast<int64_t>(color_metadata.matrixCoefficients);
      error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::MATRIX_COEFFICIENTS,
                                       &snap_matrix_coefficients);
      if (error != SnapError::NONE) {
        ALOGE(
            "%s: Failed to set snap metadata type - MATRIX_COEFFICIENTS via SnapAlloc. Error "
            "code: %d",
            __FUNCTION__, error);
      }
    }
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::PrivateFlagsHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                      uint32_t aidl_size, void *gralloc_in_set,
                                                      void *gralloc_out_get,
                                                      SnapDescriptor *buf_des,
                                                      bool check_metadata_set,
                                                      int32_t *mapper_return) {
  int64_t snap_private_flags = 0;
  int64_t is_ubwc = 0, is_tile_rendered = 0, is_cached = 0;
  // Get usage flags
  SnapUsage snap_usage;
  auto status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::USAGE, &snap_usage);
  if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
    ALOGE("%s Unable to get USAGE from snap", __FUNCTION__);
    return status;
  }
  status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::IS_UBWC, &is_ubwc);
  if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
    ALOGE("%s Unable to get IS_UBWC from snap", __FUNCTION__);
    return status;
  }

  status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::IS_TILE_RENDERED, &is_tile_rendered);
  if (status != SnapError::NONE) {
    ALOGE("%s Unable to get IS_TILE_RENDERED from snap", __FUNCTION__);
    return status;
  }
  status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::IS_CACHED, &is_cached);
  if (status != SnapError::NONE) {
    ALOGE("%s Unable to get IS_CACHED from snap", __FUNCTION__);
    return status;
  }

  int gr_priv_flags = GetGrallocPrivateFlags(snap_usage, is_ubwc, is_tile_rendered, is_cached);
  if (!hidl_bytestream) {
    *static_cast<int32_t *>(gralloc_out_get) = gr_priv_flags;
  } else {
    android::gralloc4::encodeInt32(qtigralloc::MetadataType_PrivateFlags, gr_priv_flags,
                                   static_cast<hidl_vec<uint8_t> *>(gralloc_out_get));
  }
  return SnapError::NONE;
}

SnapError GrallocSnapHelperLegacy::IsUBWCHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                uint32_t aidl_size, void *gralloc_in_set,
                                                void *gralloc_out_get, SnapDescriptor *buf_des,
                                                bool check_metadata_set, int32_t *mapper_return) {
  int64_t is_ubwc = 0;

  auto status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::IS_UBWC, &is_ubwc);
  if (status != SnapError::NONE) {
    ALOGE("%s Unable to get IS_UBWC from snap", __FUNCTION__);
    return status;
  }

  *static_cast<int32_t *>(gralloc_out_get) = is_ubwc;
  return SnapError::NONE;
}

SnapError GrallocSnapHelperLegacy::IsTileRenderedHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                        uint32_t aidl_size, void *gralloc_in_set,
                                                        void *gralloc_out_get,
                                                        SnapDescriptor *buf_des,
                                                        bool check_metadata_set,
                                                        int32_t *mapper_return) {
  int64_t is_tile_rendered = 0;

  auto status =
      snapmapper_->GetMetadata(*hnd, SnapMetadataType::IS_TILE_RENDERED, &is_tile_rendered);
  if (status != SnapError::NONE) {
    ALOGE("%s Unable to get IS_TILE_RENDERED from snap", __FUNCTION__);
    return status;
  }

  *static_cast<int32_t *>(gralloc_out_get) = is_tile_rendered;
  return SnapError::NONE;
}

SnapError GrallocSnapHelperLegacy::IsCachedHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                  uint32_t aidl_size, void *gralloc_in_set,
                                                  void *gralloc_out_get, SnapDescriptor *buf_des,
                                                  bool check_metadata_set, int32_t *mapper_return) {
  int64_t is_cached = 0;

  auto status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::IS_CACHED, &is_cached);
  if (status != SnapError::NONE) {
    ALOGE("%s Unable to get IS_CACHED from snap", __FUNCTION__);
    return status;
  }

  *static_cast<int32_t *>(gralloc_out_get) = is_cached;
  return SnapError::NONE;
}

SnapError GrallocSnapHelperLegacy::BaseAddressHelper(SnapHandle *hnd, bool hidl_bytestream,
                                                     uint32_t aidl_size, void *gralloc_in_set,
                                                     void *gralloc_out_get, SnapDescriptor *buf_des,
                                                     bool check_metadata_set,
                                                     int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    uint64_t base_address = 0;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::BASE_ADDRESS, &base_address);
    error = CheckMetadataSet(SnapMetadataType::BASE_ADDRESS, error, check_metadata_set);
    // This type is only supported as a vendor metadata type in Gralloc5
    *static_cast<uint64_t *>(gralloc_out_get) = static_cast<uint64_t>(base_address);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::MatrixCoefficientsHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    SnapMatrixCoEfficients snap_matrix_coefficients = {};
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::MATRIX_COEFFICIENTS,
                                     &snap_matrix_coefficients);
    error = CheckMetadataSet(SnapMetadataType::MATRIX_COEFFICIENTS, error, check_metadata_set);
    // This type is only supported as a vendor metadata type in Gralloc5
    *static_cast<SnapMatrixCoEfficients *>(gralloc_out_get) = snap_matrix_coefficients;
  } else if (gralloc_in_set != nullptr) {
    SnapMatrixCoEfficients snap_matrix_coefficients = {};
    // This type is only supported as a vendor metadata type in Gralloc5
    snap_matrix_coefficients = *static_cast<SnapMatrixCoEfficients *>(gralloc_in_set);
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::MATRIX_COEFFICIENTS,
                                     &snap_matrix_coefficients);
  }
  return error;
}

SnapError GrallocSnapHelperLegacy::EarlyNotifyLineCountHelper(
    SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
    void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
    int32_t *mapper_return) {
  auto error = SnapError::BAD_VALUE;
  if (gralloc_out_get != nullptr) {
    int32_t early_notify_line_count = 0;
    error = snapmapper_->GetMetadata(*hnd, SnapMetadataType::EARLYNOTIFY_LINECOUNT,
                                     &early_notify_line_count);
    error = CheckMetadataSet(SnapMetadataType::EARLYNOTIFY_LINECOUNT, error, check_metadata_set);
    *static_cast<int32_t *>(gralloc_out_get) = static_cast<int32_t>(early_notify_line_count);
  } else if (gralloc_in_set != nullptr) {
    int32_t early_notify_line_count = 0;
    early_notify_line_count = *static_cast<int32_t *>(gralloc_in_set);
    error = snapmapper_->SetMetadata(*hnd, SnapMetadataType::EARLYNOTIFY_LINECOUNT,
                                     &early_notify_line_count);
  }
  return error;
}

int GrallocSnapHelperLegacy::GetFromBufferDescriptor(gralloc::BufferDescriptor gr_desc,
                                                     uint64_t gr_metadata_type, void *out,
                                                     bool convert_to_hidl_bytestream) {
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  SnapDescriptor snap_desc = {};
  auto err = GetSnapDescriptor(gr_desc, snap_desc);
  if (err) {
    return err;
  }
  auto snap_metadata_type = GetSnapMetadataType(gr_metadata_type);
  if (bufferdescription_conversion_helper_function_map.find(snap_metadata_type) !=
      bufferdescription_conversion_helper_function_map.end()) {
    MetadataHelper metadata_helper_func =
        bufferdescription_conversion_helper_function_map[snap_metadata_type];
    return ((this->*metadata_helper_func)(nullptr, convert_to_hidl_bytestream, 0, nullptr, out,
                                          &snap_desc, false, nullptr));
  } else {
    return SnapError::UNSUPPORTED;
  }

  return SnapError::UNSUPPORTED;
}

int GrallocSnapHelperLegacy::ConvertSnapBufferlayoutToGrallocPlaneLayout(
    SnapHandle *hnd, SnapDescriptor *buf_des, const SnapBufferLayout snap_buffer_layout,
    std::vector<GrallocPlaneLayout> *gr_plane_layouts) {
  uint64_t width, height;
  SnapPixelFormat snap_pixel_format = SnapPixelFormat::PIXEL_FORMAT_UNSPECIFIED;
  bool is_raw = false;
  bool individually_packed = true;

  if (hnd != nullptr) {
    // Get unaligned width
    auto status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::WIDTH, &width);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGE("%s: Unable to get unaligned width", __FUNCTION__);
      return status;
    }
    //Get unaligned height
    status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::HEIGHT, &height);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGE("%s: Unable to get unaligned height", __FUNCTION__);
      return status;
    }
    // Get pixel format
    status = snapmapper_->GetMetadata(*hnd, SnapMetadataType::PIXEL_FORMAT_ALLOCATED,
                                      &snap_pixel_format);
    if (status != SnapError::NONE && status != SnapError::METADATA_NOT_SET) {
      ALOGE("%s: Unable to get pixel format", __FUNCTION__);
      return status;
    }
  } else if (buf_des != nullptr) {
    auto error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::WIDTH, &width);
    if (error != SnapError::NONE) {
      ALOGE("%s: Unable to get unaligned width", __FUNCTION__);
      return error;
    }
    error = snapmapper_->GetFromBufferDescriptor(*buf_des, SnapMetadataType::HEIGHT, &height);
    if (error != SnapError::NONE) {
      ALOGE("%s: Unable to get unaligned height", __FUNCTION__);
      return error;
    }
    snap_pixel_format = buf_des->format;
  }
  auto snap_plane_layout = snap_buffer_layout.planes;
  int plane_count = snap_buffer_layout.plane_count;
  gr_plane_layouts->resize(plane_count);
  int bpp = snap_buffer_layout.bpp;

  // For RAW formats update information to meet IMapper5 VTS / Gralloc4 specs
  // Sets sampleIncrementInBits to 0 and component sizeInBits to -1 if sampleIncrementInBits is not
  // divisible by 8. These values aren't valid for formats such as RAW10 and RAW12 which aren't
  // individually packed due to their size not being in multiples of 8-bits
  switch (snap_pixel_format) {
    case SnapPixelFormat::RAW10:
    case SnapPixelFormat::RAW12:
      individually_packed = false;
      [[fallthrough]];
    case SnapPixelFormat::RAW8:
    case SnapPixelFormat::RAW16:
      is_raw = true;
      break;
    default:
      is_raw = false;
  }

  for (int i = 0; i < plane_count; i++) {
    (*gr_plane_layouts)[i].sampleIncrementInBits =
        is_raw && !individually_packed
            ? 0
            : static_cast<int64_t>(snap_plane_layout[i].sample_increment_bits);
    (*gr_plane_layouts)[i].strideInBytes =
        static_cast<int64_t>(snap_plane_layout[i].horizontal_stride_in_bytes);
    (*gr_plane_layouts)[i].totalSizeInBytes =
        static_cast<int64_t>(snap_plane_layout[i].size_in_bytes);
    (*gr_plane_layouts)[i].horizontalSubsampling =
        static_cast<int64_t>(snap_plane_layout[i].horizontal_subsampling);
    (*gr_plane_layouts)[i].verticalSubsampling =
        static_cast<int64_t>(snap_plane_layout[i].vertical_subsampling);
    // Convert horizontal and vertical subsampling into factor (e.g., 1 >> 2 = 0)
    (*gr_plane_layouts)[i].widthInSamples =
        width >> (snap_plane_layout[i].horizontal_subsampling >> 1ull);
    (*gr_plane_layouts)[i].heightInSamples =
        height >> (snap_plane_layout[i].vertical_subsampling >> 1ull);
    (*gr_plane_layouts)[i].offsetInBytes =
        static_cast<int64_t>(snap_plane_layout[i].offset_in_bytes);

    ALOGD_IF(
        enable_logs_,
        "%s: Plane No: %d, sampleIncrementInBits %d, strideInBytes %d, totalSizeInBytes %d, "
        "horizontalSubsampling %d, verticalSubsampling %d, widthInSamples %d,  heightInSamples %d, "
        "offsetInBytes %d",
        __FUNCTION__, i, (*gr_plane_layouts)[i].sampleIncrementInBits,
        (*gr_plane_layouts)[i].strideInBytes, (*gr_plane_layouts)[i].totalSizeInBytes,
        (*gr_plane_layouts)[i].horizontalSubsampling, (*gr_plane_layouts)[i].verticalSubsampling,
        (*gr_plane_layouts)[i].widthInSamples, (*gr_plane_layouts)[i].heightInSamples,
        (*gr_plane_layouts)[i].offsetInBytes);

    std::vector<GrallocPlaneLayoutComponent> gr_plane_layout_components;
    int snap_component_count = snap_plane_layout[i].component_count;
    auto snap_plane_layout_components = snap_plane_layout[i].components;
    for (int j = 0; j < snap_component_count; j++) {
      GrallocPlaneLayoutComponent gr_plane_layout_component;
      ConvertSnapToGrallocPlaneComponentType(snap_plane_layout_components[j].type,
                                             &gr_plane_layout_component.type);
      gr_plane_layout_component.offsetInBits = snap_plane_layout_components[j].offset_in_bits;
      gr_plane_layout_component.sizeInBits =
          (is_raw && !individually_packed) ? -1 : snap_plane_layout_components[j].size_in_bits;
      (*gr_plane_layouts)[i].components.push_back(gr_plane_layout_component);
    }
  }
  return SnapError::NONE;
}

int GrallocSnapHelperLegacy::ConvertGrallocPlaneLayoutToAndroidYCbCr(
    uint64_t base_addr, const std::vector<GrallocPlaneLayout> gr_plane_layouts,
    struct android_ycbcr *outYCbCr) {
  outYCbCr->y = nullptr;
  outYCbCr->cb = nullptr;
  outYCbCr->cr = nullptr;
  outYCbCr->ystride = 0;
  outYCbCr->cstride = 0;
  outYCbCr->chroma_step = 0;
  int next_plane = 0;
  for (const auto &planeLayout : gr_plane_layouts) {
    bool contains_meta = false;
    for (const auto &planeLayoutComponent : planeLayout.components) {
      if (planeLayoutComponent.type.value == qtigralloc::PlaneLayoutComponentType_Meta.value) {
        contains_meta = true;
      }
    }
    if (!contains_meta) {
      for (const auto &planeLayoutComponent : planeLayout.components) {
        auto tmpData =
            base_addr + planeLayout.offsetInBytes + (planeLayoutComponent.offsetInBits / 8);
        uint64_t sampleIncrementInBytes;
        auto type = static_cast<GrallocPlaneLayoutComponentType>(planeLayoutComponent.type.value);
        switch (type) {
          case GrallocPlaneLayoutComponentType::Y: {
            outYCbCr->y = reinterpret_cast<void *>(tmpData);
            outYCbCr->ystride = planeLayout.strideInBytes;
            break;
          }
          case GrallocPlaneLayoutComponentType::CB:
          case GrallocPlaneLayoutComponentType::CR: {
            sampleIncrementInBytes = planeLayout.sampleIncrementInBits / 8;
            outYCbCr->cstride = planeLayout.strideInBytes;
            outYCbCr->chroma_step = sampleIncrementInBytes;
            if (type == GrallocPlaneLayoutComponentType::CB) {
              outYCbCr->cb = reinterpret_cast<void *>(tmpData);
            } else {
              outYCbCr->cr = reinterpret_cast<void *>(tmpData);
            }
            break;
          }
          default:
            break;
        }
      }
    }
    // Interlaced UBWC formats have 8 Planes
    if (gr_plane_layouts.size() == 8) {
      // Planes 0-3 fills top field in android_ycbcr & planes 4-7 fills bottom field
      if (next_plane == 3) {
        outYCbCr = outYCbCr + 1;
      }
    }
    next_plane++;
  }

  ALOGD_IF(
      enable_logs_,
      "%s: base_addr %d, outYCbCr->y %d, outYCbCr->cb %d, outYCbCr->cr %d, outYCbCr->ystride %d, "
      "outYCbCr->cstride %d, outYCbCr->chroma_step %d",
      __FUNCTION__, base_addr, outYCbCr->y, outYCbCr->cb, outYCbCr->cr, outYCbCr->ystride,
      outYCbCr->cstride, outYCbCr->chroma_step);
  return SnapError::NONE;
}

bool GrallocSnapHelperLegacy::IsBufferImported(native_handle_t *gr_hnd) {
  if (gr_hnd == nullptr) {
    ALOGE("%s: Invalid gralloc handle", __FUNCTION__);
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  std::lock_guard<std::mutex> lock(map_lock_);

  if (handles_map_.find(gr_hnd) != handles_map_.end()) {
    SnapHandle *hnd = handles_map_.at(gr_hnd);
    if (hnd != nullptr) {
      ALOGI("%s: Gralloc handle %p has been imported", __FUNCTION__, gr_hnd);
      return true;
    }
  }
  ALOGE("%s: Gralloc handle %p has not been imported", __FUNCTION__, gr_hnd);
  return false;
}

int GrallocSnapHelperLegacy::GetCustomDimensions(native_handle_t *gr_hnd, int *stride,
                                                 int *height) {
  if (gr_hnd == nullptr) {
    ALOGE("%s: Invalid gralloc handle", __FUNCTION__);
    return SnapError::BAD_BUFFER;
  }
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  ALOGE("%s: GetCustomDimensions not implemented for Snapalloc", __FUNCTION__);
  return SnapError::UNSUPPORTED;
}

int GrallocSnapHelperLegacy::ConvertSnapPlaneLayoutComponentToGralloc(SnapPlaneLayout *layout) {
  int gralloc_component = 0;
  for (int i = 0; i < layout->component_count; i++) {
    if (snap_to_gralloc_plane_layout_component_.find(layout->components[i].type) !=
        snap_to_gralloc_plane_layout_component_.end()) {
      gralloc_component |= snap_to_gralloc_plane_layout_component_.at(layout->components[i].type);
    }
  }
  return gralloc_component;
}

int GrallocSnapHelperLegacy::GetFormatLayout(gralloc::BufferInfo gr_desc, void *out, uint32_t *size,
                                             int interlaced) {
  if (!IsSnapAllocEnabled()) {
    ALOGW("%s: SnapAlloc is disabled", __FUNCTION__);
    return SnapError::UNSUPPORTED;
  }

  SnapDescriptor snap_desc = {};
  auto err = GetSnapDescriptor(gr_desc, snap_desc);
  if (err) {
    return err;
  }
  if (interlaced) {
    static SnapKeyValuePair modifier = {.key = "interlaced", .value = static_cast<uint64_t>(1)};
    snap_desc.additionalOptions.emplace_back(modifier);
  }
  SnapBufferLayout snap_plane_layouts;
  auto status = snapmapper_->GetFromBufferDescriptor(snap_desc, SnapMetadataType::PLANE_LAYOUTS,
                                                     &snap_plane_layouts);

  if (status == SnapError::NONE) {
    unsigned int alloc_size;
    status = snapmapper_->GetFromBufferDescriptor(snap_desc, SnapMetadataType::ALLOCATION_SIZE,
                                                  &alloc_size);
    *size = static_cast<uint32_t>(alloc_size);

    int64_t ubwc_enabled_in_snap;
    status = snapmapper_->GetFromBufferDescriptor(snap_desc, SnapMetadataType::IS_UBWC,
                                                  &ubwc_enabled_in_snap);

    std::vector<gralloc::PlaneLayoutInfo> *plane_info =
        reinterpret_cast<std::vector<gralloc::PlaneLayoutInfo> *>(out);
    // TODO: need to properly filter out meta plane for UBWC RGBA case - hacking for now to unblock
    if ((IsUncompressedRGBFormat(static_cast<int>(snap_desc.format)) ||
         IsCompressedRGBFormat(static_cast<int>(snap_desc.format))) &&
        ubwc_enabled_in_snap) {
      plane_info->resize(snap_plane_layouts.plane_count / 2);
    } else {
      plane_info->resize(snap_plane_layouts.plane_count);
    }
    for (int i = 0; i < plane_info->size(); i++) {
      (*plane_info)[i].component = static_cast<gralloc::PlaneComponent>(
          ConvertSnapPlaneLayoutComponentToGralloc(&snap_plane_layouts.planes[i]));
      (*plane_info)[i].h_subsampling = snap_plane_layouts.planes[i].horizontal_subsampling >> 1ull;
      (*plane_info)[i].v_subsampling = snap_plane_layouts.planes[i].vertical_subsampling >> 1ull;
      (*plane_info)[i].offset = snap_plane_layouts.planes[i].offset_in_bytes;
      (*plane_info)[i].step = snap_plane_layouts.planes[i].sample_increment_bits / 8;
      (*plane_info)[i].stride = static_cast<int>(
          static_cast<float>(snap_plane_layouts.planes[i].horizontal_stride_in_bytes) /
          (static_cast<float>(snap_plane_layouts.planes[i].sample_increment_bits) / 8.0f));
      (*plane_info)[i].stride_bytes = snap_plane_layouts.planes[i].horizontal_stride_in_bytes;
      (*plane_info)[i].scanlines = snap_plane_layouts.planes[i].scanlines;
      (*plane_info)[i].size = snap_plane_layouts.planes[i].size_in_bytes;
    }

    return SnapError::NONE;
  } else {
    ALOGE("%s: Failed to get plane layouts from SnapAlloc. Error code: %d", __FUNCTION__, status);
  }

  return status;
}

int GrallocSnapHelperLegacy::ConvertSnapDataspaceToGrallocDataspace(
    SnapDataspace &snap_dataspace, GrallocDataspace *gr_dataspace) {
  GrallocDataspace primaries, transfer, range = GrallocDataspace::UNKNOWN;
  switch (snap_dataspace.colorPrimaries) {
    case QtiColorPrimaries_BT709_5:
      primaries = GrallocDataspace::STANDARD_BT709;
      break;
    case QtiColorPrimaries_BT470_6M:
      primaries = GrallocDataspace::STANDARD_BT470M;
      break;
    case QtiColorPrimaries_BT601_6_625:
      primaries = GrallocDataspace::STANDARD_BT601_625;
      break;
    case QtiColorPrimaries_BT601_6_525:
      primaries = GrallocDataspace::STANDARD_BT601_525;
      break;
    case QtiColorPrimaries_GenericFilm:
      primaries = GrallocDataspace::STANDARD_FILM;
      break;
    case QtiColorPrimaries_BT2020:
      primaries = GrallocDataspace::STANDARD_BT2020;
      break;
    case QtiColorPrimaries_AdobeRGB:
      primaries = GrallocDataspace::STANDARD_ADOBE_RGB;
      break;
    case QtiColorPrimaries_DCIP3:
      primaries = GrallocDataspace::STANDARD_DCI_P3;
      break;
    default:
      ALOGV("%s: Failed to convert primaries %d", __FUNCTION__, snap_dataspace.colorPrimaries);
      return SnapError::BAD_VALUE;
  }

  switch (snap_dataspace.transfer) {
    case QtiTransfer_sRGB:
      transfer = GrallocDataspace::TRANSFER_SRGB;
      break;
    case QtiTransfer_Gamma2_2:
      transfer = GrallocDataspace::TRANSFER_GAMMA2_2;
      break;
    case QtiTransfer_Gamma2_8:
      transfer = GrallocDataspace::TRANSFER_GAMMA2_8;
      break;
    case QtiTransfer_SMPTE_170M:
      transfer = GrallocDataspace::TRANSFER_SMPTE_170M;
      break;
    case QtiTransfer_Linear:
      transfer = GrallocDataspace::TRANSFER_LINEAR;
      break;
    case QtiTransfer_HLG:
      transfer = GrallocDataspace::TRANSFER_HLG;
      break;
    case QtiTransfer_SMPTE_ST2084:
      transfer = GrallocDataspace::TRANSFER_ST2084;
      break;
    default:
      ALOGV("%s: Failed to convert transfer %d", __FUNCTION__, snap_dataspace.transfer);
      return SnapError::BAD_VALUE;
  }

  switch (snap_dataspace.range) {
    case QtiRange_Full:
      range = GrallocDataspace::RANGE_FULL;
      break;
    case QtiRange_Limited:
      range = GrallocDataspace::RANGE_LIMITED;
      break;
    case QtiRange_Extended:
      range = GrallocDataspace::RANGE_EXTENDED;
      break;
    default:
      ALOGV("%s: Failed to convert range %d", __FUNCTION__, snap_dataspace.range);
      return SnapError::BAD_VALUE;
  }

  *gr_dataspace = (GrallocDataspace)((uint32_t)primaries | (uint32_t)transfer | (uint32_t)range);
  return SnapError::NONE;
}

// Converts Dataspace to Colorspace
int GrallocSnapHelperLegacy::GetColorSpaceFromDataspaceMetadata(SnapDataspace snap_dataspace,
                                                                uint32_t *color_space) {
  int err = 0;
  switch (snap_dataspace.colorPrimaries) {
    case QtiColorPrimaries_BT709_5: {
      *color_space = HAL_CSC_ITU_R_709;
      break;
    }
    case QtiColorPrimaries_BT601_6_625:
    case QtiColorPrimaries_BT601_6_525: {
      *color_space =
          ((static_cast<bool>(snap_dataspace.range)) ? HAL_CSC_ITU_R_601_FR : HAL_CSC_ITU_R_601);
      break;
    }
    case QtiColorPrimaries_BT2020: {
      *color_space =
          ((static_cast<bool>(snap_dataspace.range)) ? HAL_CSC_ITU_R_2020_FR : HAL_CSC_ITU_R_2020);
      break;
    }
    default: {
      err = -1;
      *color_space = 0;
      ALOGW("%s: Unknown Color primary = %d", __FUNCTION__, snap_dataspace.colorPrimaries);
      break;
    }
  }
  return err;
}

int GrallocSnapHelperLegacy::GetSnapDataspaceMetadataFromColorSpace(uint32_t color_space,
                                                                    SnapDataspace *snap_dataspace) {
  snap_dataspace->transfer = QtiTransfer_sRGB;
  switch (color_space) {
    case HAL_CSC_ITU_R_601: {
      snap_dataspace->colorPrimaries = QtiColorPrimaries_BT601_6_525;
      snap_dataspace->range = QtiRange_Limited;
      break;
    }
    case HAL_CSC_ITU_R_601_FR: {
      snap_dataspace->colorPrimaries = QtiColorPrimaries_BT601_6_525;
      snap_dataspace->range = QtiRange_Full;
      break;
    }
    case HAL_CSC_ITU_R_709: {
      snap_dataspace->colorPrimaries = QtiColorPrimaries_BT709_5;
      snap_dataspace->range = QtiRange_Limited;
      break;
    }
    case HAL_CSC_ITU_R_709_FR: {
      snap_dataspace->colorPrimaries = QtiColorPrimaries_BT709_5;
      snap_dataspace->range = QtiRange_Full;
      break;
    }
    case HAL_CSC_ITU_R_2020: {
      snap_dataspace->colorPrimaries = QtiColorPrimaries_BT2020;
      snap_dataspace->range = QtiRange_Limited;
      break;
    }
    case HAL_CSC_ITU_R_2020_FR: {
      snap_dataspace->colorPrimaries = QtiColorPrimaries_BT2020;
      snap_dataspace->range = QtiRange_Full;
      break;
    }
    default:
      ALOGE("%s: Cannot convert ColorSpace_t %d to SnapDataspace", __FUNCTION__, color_space);
      return SnapError::BAD_VALUE;
  }
  return SnapError::NONE;
}

void GrallocSnapHelperLegacy::ConvertSnapToGrallocPlaneComponentType(
    SnapPlaneLayoutComponentType snap_component_type, GrallocExtendableType *gr_component_type) {
  if (snap_to_gralloc_extendable_plane_layout_component_type_.find(snap_component_type) !=
      snap_to_gralloc_extendable_plane_layout_component_type_.end()) {
    *gr_component_type =
        snap_to_gralloc_extendable_plane_layout_component_type_.at(snap_component_type);
  }
}

int GrallocSnapHelperLegacy::ConvertGrallocDataspaceToSnapDataspace(GrallocDataspace gr_dataspace,
                                                                    SnapDataspace *snap_dataspace) {
  SnapDataspace dataspace;
  uint32_t primaries = (uint32_t)gr_dataspace & (uint32_t)GrallocDataspace::STANDARD_MASK;
  uint32_t transfer = (uint32_t)gr_dataspace & (uint32_t)GrallocDataspace::TRANSFER_MASK;
  uint32_t range = (uint32_t)gr_dataspace & (uint32_t)GrallocDataspace::RANGE_MASK;

  switch (primaries) {
    case (uint32_t)GrallocDataspace::STANDARD_BT709:
      dataspace.colorPrimaries = QtiColorPrimaries_BT709_5;
      break;
    case (uint32_t)GrallocDataspace::STANDARD_BT470M:
      dataspace.colorPrimaries = QtiColorPrimaries_BT470_6M;
      break;
    case (uint32_t)GrallocDataspace::STANDARD_BT601_625:
    case (uint32_t)GrallocDataspace::STANDARD_BT601_625_UNADJUSTED:
      dataspace.colorPrimaries = QtiColorPrimaries_BT601_6_625;
      break;
    case (uint32_t)GrallocDataspace::STANDARD_BT601_525:
    case (uint32_t)GrallocDataspace::STANDARD_BT601_525_UNADJUSTED:
      dataspace.colorPrimaries = QtiColorPrimaries_BT601_6_525;
      break;
    case (uint32_t)GrallocDataspace::STANDARD_FILM:
      dataspace.colorPrimaries = QtiColorPrimaries_GenericFilm;
      break;
    case (uint32_t)GrallocDataspace::STANDARD_BT2020:
      dataspace.colorPrimaries = QtiColorPrimaries_BT2020;
      break;
    case (uint32_t)GrallocDataspace::STANDARD_ADOBE_RGB:
      dataspace.colorPrimaries = QtiColorPrimaries_AdobeRGB;
      break;
    case (uint32_t)GrallocDataspace::STANDARD_DCI_P3:
      dataspace.colorPrimaries = QtiColorPrimaries_DCIP3;
      break;
    default:
      ALOGV("%s: Failed to convert primaries %d", __FUNCTION__, primaries);
      return SnapError::BAD_VALUE;
  }

  switch (transfer) {
    case (uint32_t)GrallocDataspace::TRANSFER_SRGB:
      dataspace.transfer = QtiTransfer_sRGB;
      break;
    case (uint32_t)GrallocDataspace::TRANSFER_GAMMA2_2:
      dataspace.transfer = QtiTransfer_Gamma2_2;
      break;
    case (uint32_t)GrallocDataspace::TRANSFER_GAMMA2_8:
      dataspace.transfer = QtiTransfer_Gamma2_8;
      break;
    case (uint32_t)GrallocDataspace::TRANSFER_SMPTE_170M:
      dataspace.transfer = QtiTransfer_SMPTE_170M;
      break;
    case (uint32_t)GrallocDataspace::TRANSFER_LINEAR:
      dataspace.transfer = QtiTransfer_Linear;
      break;
    case (uint32_t)GrallocDataspace::TRANSFER_HLG:
      dataspace.transfer = QtiTransfer_HLG;
      break;
    case (uint32_t)GrallocDataspace::TRANSFER_ST2084:
      dataspace.transfer = QtiTransfer_SMPTE_ST2084;
      break;
    default:
      ALOGV("%s: Failed to convert transfer %d", __FUNCTION__, transfer);
      return SnapError::BAD_VALUE;
  }

  switch (range) {
    case (uint32_t)GrallocDataspace::RANGE_FULL:
      dataspace.range = QtiRange_Full;
      break;
    case (uint32_t)GrallocDataspace::RANGE_LIMITED:
      dataspace.range = QtiRange_Limited;
      break;
    case (uint32_t)GrallocDataspace::RANGE_EXTENDED:
      dataspace.range = QtiRange_Extended;
      break;
    default:
      ALOGV("%s: Failed to convert range %d", __FUNCTION__, range);
      return SnapError::BAD_VALUE;
  }
  snap_dataspace->colorPrimaries = dataspace.colorPrimaries;
  snap_dataspace->transfer = dataspace.transfer;
  snap_dataspace->range = dataspace.range;
  return SnapError::NONE;
}

// Gralloc <-> Snapalloc conversion helper functions
SnapError GrallocSnapHelperLegacy::GetSnapFormat(int hal_format, uint64_t usage,
                                                 SnapFormatDescriptor *snap_fmt_desc) {
  if (gralloc_ubwc_to_snap_format_.find(hal_format) != gralloc_ubwc_to_snap_format_.end()) {
    *snap_fmt_desc = gralloc_ubwc_to_snap_format_.at(hal_format);
  } else if (gralloc_to_snap_format_.find(hal_format) != gralloc_to_snap_format_.end()) {
    *snap_fmt_desc = gralloc_to_snap_format_.at(hal_format);
  } else if (std::find(unsupported_formats.begin(), unsupported_formats.end(), hal_format) !=
             unsupported_formats.end()) {
    ALOGW("%s:: Unsupported format - %d", __FUNCTION__, hal_format);
    return SnapError::UNSUPPORTED;
  } else {
    ALOGW("%s:: No map for gralloc format %d to snap format", __FUNCTION__, hal_format);
    return SnapError::BAD_VALUE;
  }

  ALOGD_IF(enable_logs_, "%s: gralloc format %d snap format %d modifier %d", __FUNCTION__,
           hal_format, snap_fmt_desc->format, snap_fmt_desc->modifier);
  return SnapError::NONE;
}

SnapUsage GrallocSnapHelperLegacy::GetSnapUsage(uint64_t usage, int hal_format) {
  SnapUsage snap_usage = CPU_READ_NEVER;

  if (gralloc_ubwc_to_snap_format_.find(hal_format) != gralloc_ubwc_to_snap_format_.end()) {
    snap_usage |= SnapUsage::QTI_ALLOC_UBWC;
    // If explicit UBWC format, do not set CPU flags
    // TODO: revisit this once explicit UBWC formats deprecated
  } else {
    uint64_t cpu_read_usage = static_cast<uint64_t>(usage & SnapUsage::CPU_READ_MASK);
    uint64_t cpu_write_usage = static_cast<uint64_t>(usage & SnapUsage::CPU_WRITE_MASK);

    if (cpu_read_usage != 0 || cpu_write_usage != 0) {
      for (auto entry : cpu_gralloc_to_snap_usage_) {
        if (cpu_read_usage == entry.first || cpu_write_usage == entry.first) {
          snap_usage |= entry.second;
        }
      }
    }
  }
  for (auto entry : gralloc_to_snap_usage_) {
    // If bit is set in gralloc usage, set equivalent bit in snap usage
    if (usage & entry.first) {
      snap_usage |= entry.second;
    }
  }

  return snap_usage;
}

SnapUBWCVersion GrallocSnapHelperLegacy::GetSnapUBWCVersion(UBWC_Version version) {
  if (gralloc_to_snap_ubwc_version_.find(version) != gralloc_to_snap_ubwc_version_.end()) {
    return gralloc_to_snap_ubwc_version_.at(version);
  }
  return UBWC_VERSION_MAX;
}

UBWC_Version GrallocSnapHelperLegacy::GetGrallocUBWCVersion(SnapUBWCVersion version) {
  if (snap_to_gralloc_ubwc_version_.find(version) != snap_to_gralloc_ubwc_version_.end()) {
    return snap_to_gralloc_ubwc_version_.at(version);
  }

  return UBWC_MAX_VERSION;
}

SnapError GrallocSnapHelperLegacy::GetSnapDescriptor(gralloc::BufferDescriptor gr_desc,
                                                     SnapDescriptor &snap_desc) {
  SnapFormatDescriptor snap_fmt_desc;
  auto err = GetSnapFormat(gr_desc.GetFormat(), gr_desc.GetUsage(), &snap_fmt_desc);
  if (err) {
    ALOGW("%s: Error while getting snap descriptor - gr_format - %d", __FUNCTION__,
          gr_desc.GetFormat());
    return err;
  } else {
    auto name_length = std::min(gr_desc.GetName().size(), static_cast<size_t>(MAX_NAME_LEN - 1));
    memcpy(snap_desc.name, gr_desc.GetName().data(), name_length);
    snap_desc.format = snap_fmt_desc.format;
    err = ValidateGrallocUsage(gr_desc.GetUsage());
    if (err) {
      ALOGW("%s: Error while getting snap descriptor - Unknown Usage bit set - %lu", __FUNCTION__,
            gr_desc.GetUsage());
      return err;
    }
    snap_desc.usage = GetSnapUsage(gr_desc.GetUsage(), gr_desc.GetFormat());
    snap_desc.width = gr_desc.GetWidth();
    snap_desc.height = gr_desc.GetHeight();
    snap_desc.layerCount = gr_desc.GetLayerCount();
    snap_desc.reservedSize = gr_desc.GetReservedSize();
    SnapKeyValuePair modifier = {.key = "pixel_format_modifier",
                                 .value = static_cast<uint64_t>(snap_fmt_desc.modifier)};
    snap_desc.additionalOptions.emplace_back(modifier);
    ALOGD_IF(enable_logs_,
             "%s: gr format %d gr usage %lu snap format %d snap modifier %d snap "
             "usage %lu",
             __FUNCTION__, gr_desc.GetFormat(), gr_desc.GetUsage(), snap_fmt_desc.format,
             snap_fmt_desc.modifier, snap_desc.usage);
    ALOGD_IF(enable_logs_, "%s: name from gralloc descriptor %s snap_desc %s", __FUNCTION__,
             gr_desc.GetName().c_str(), snap_desc.name);
  }
  return SnapError::NONE;
}

SnapError GrallocSnapHelperLegacy::ValidateGrallocUsage(uint64_t gralloc_usage) {
  // Bits 33-47 must be zero and are reserved for future versions
  uint64_t future_usage_bit_mask = static_cast<uint64_t>(0xFFFE00000000);
  if ((gralloc_usage & future_usage_bit_mask) != 0) {
    return SnapError::BAD_VALUE;
  }
  return SnapError::NONE;
}

SnapError GrallocSnapHelperLegacy::GetSnapDescriptor(gralloc::BufferInfo gr_desc,
                                                     SnapDescriptor &snap_desc) {
  SnapFormatDescriptor snap_fmt_desc;
  auto err = GetSnapFormat(gr_desc.format, gr_desc.usage, &snap_fmt_desc);
  if (err) {
    ALOGW("%s: Error while getting snap descriptor - gr_format - %d", __FUNCTION__, gr_desc.format);
    return err;
  } else {
    snap_desc.format = snap_fmt_desc.format;
    err = ValidateGrallocUsage(gr_desc.usage);
    if (err) {
      ALOGW("%s: Error while getting snap descriptor - Unknown Usage bit set - %lu", __FUNCTION__,
            gr_desc.usage);
      return err;
    }
    snap_desc.usage = GetSnapUsage(gr_desc.usage, gr_desc.format);
    snap_desc.width = gr_desc.width;
    snap_desc.height = gr_desc.height;
    snap_desc.layerCount = gr_desc.layer_count;
    SnapKeyValuePair modifier = {.key = "pixel_format_modifier",
                                 .value = static_cast<uint64_t>(snap_fmt_desc.modifier)};
    snap_desc.additionalOptions.emplace_back(modifier);

    ALOGD_IF(enable_logs_,
             "%s: gr format %d gr usage %lu snap format %d snap modifier %d snap "
             "usage %lu",
             __FUNCTION__, gr_desc.format, gr_desc.usage, snap_fmt_desc.format,
             snap_fmt_desc.modifier, snap_desc.usage);
  }
  return SnapError::NONE;
}

SnapMetadataType GrallocSnapHelperLegacy::GetSnapMetadataType(uint64_t gr_metadata_type) {
  SnapMetadataType metadata_type = SnapMetadataType::METADATA_TYPE_INVALID;
  auto it = metadata_type_map.find(gr_metadata_type);
  if (it != metadata_type_map.end()) {
    metadata_type = it->second;
  }

  return metadata_type;
}

int GrallocSnapHelperLegacy::GetGrallocFormat(SnapFormatDescriptor snap_fmt_desc, SnapUsage usage,
                                              int *gr_format) {
  if ((usage & SnapUsage::QTI_ALLOC_UBWC) &&
      (snap_to_gralloc_ubwc_format_.find(snap_fmt_desc) != snap_to_gralloc_ubwc_format_.end())) {
    *gr_format = snap_to_gralloc_ubwc_format_.at(snap_fmt_desc);
  } else if (snap_to_gralloc_format_.find(snap_fmt_desc) != snap_to_gralloc_format_.end()) {
    *gr_format = snap_to_gralloc_format_.at(snap_fmt_desc);
  } else {
    ALOGE("%s: No map for format: 0x%x", __FUNCTION__, snap_fmt_desc.format);
    return SnapError::BAD_VALUE;
  }

  ALOGD_IF(enable_logs_, "%s:  snap format %d modifier %d gralloc format %d", __FUNCTION__,
           snap_fmt_desc.format, snap_fmt_desc.modifier, *gr_format);
  return SnapError::NONE;
}

uint64_t GrallocSnapHelperLegacy::GetGrallocUsage(SnapUsage snap_usage) {
  uint64_t gralloc_usage = 0;

  uint64_t cpu_read_usage =
      static_cast<uint64_t>(snap_usage) & static_cast<uint64_t>(SnapUsage::CPU_READ_MASK);
  uint64_t cpu_write_usage =
      static_cast<uint64_t>(snap_usage) & static_cast<uint64_t>(SnapUsage::CPU_WRITE_MASK);

  if (cpu_read_usage != 0 || cpu_write_usage != 0) {
    for (auto entry : cpu_snap_to_gralloc_usage_) {
      if (cpu_read_usage == static_cast<uint64_t>(entry.first) ||
          cpu_write_usage == static_cast<uint64_t>(entry.first)) {
        gralloc_usage |= entry.second;
      }
    }
  }

  for (auto entry : snap_to_gralloc_usage_) {
    // If bit is set in gralloc usage, set equivalent bit in snap usage
    if (snap_usage & entry.first) {
      gralloc_usage |= entry.second;
    }
  }

  return gralloc_usage;
}

// TODO (lower priority) : convert this into a map
int GrallocSnapHelperLegacy::GetGrallocPrivateFlags(SnapUsage snap_usage, int64_t is_ubwc,
                                                    int64_t is_tile_rendered, int64_t is_cached) {
  // Must be set by default for all SPF allocations - read by graphics
  int gr_priv_flags = qtigralloc::PRIV_FLAGS_USES_ION;

  if (is_cached) {
    gr_priv_flags |= qtigralloc::PRIV_FLAGS_CACHED;
  }

  if (is_tile_rendered) {
    gr_priv_flags |= qtigralloc::PRIV_FLAGS_TILE_RENDERED;
  }

  if (is_ubwc) {
    gr_priv_flags |= qtigralloc::PRIV_FLAGS_UBWC_ALIGNED;
    gr_priv_flags |= qtigralloc::PRIV_FLAGS_TILE_RENDERED;
    if (snap_usage & SnapUsage::QTI_PRIVATE_ALLOC_UBWC_PI) {
      gr_priv_flags |= qtigralloc::PRIV_FLAGS_UBWC_ALIGNED_PI;
    }
  }

  if (snap_usage & SnapUsage::GPU_TEXTURE) {
    gr_priv_flags |= qtigralloc::PRIV_FLAGS_HW_TEXTURE;
  }

  if (snap_usage & SnapUsage::VIDEO_ENCODER) {
    gr_priv_flags |= qtigralloc::PRIV_FLAGS_VIDEO_ENCODER;
  }

  if (snap_usage & SnapUsage::VIDEO_ENCODER || snap_usage & SnapUsage::VIDEO_DECODER ||
      snap_usage & SnapUsage::CAMERA_OUTPUT || snap_usage & SnapUsage::GPU_RENDER_TARGET) {
    gr_priv_flags |= qtigralloc::PRIV_FLAGS_NON_CPU_WRITER;
  }

  if (snap_usage & SnapUsage::VIDEO_ENCODER) {
    gr_priv_flags |= qtigralloc::PRIV_FLAGS_VIDEO_ENCODER;
  }

  if (snap_usage & SnapUsage::CAMERA_OUTPUT) {
    gr_priv_flags |= qtigralloc::PRIV_FLAGS_CAMERA_WRITE;
  }

  if (snap_usage & SnapUsage::CAMERA_INPUT) {
    gr_priv_flags |= qtigralloc::PRIV_FLAGS_CAMERA_READ;
  }

  if (snap_usage & SnapUsage::QTI_PRIVATE_SECURE_DISPLAY) {
    gr_priv_flags |= qtigralloc::PRIV_FLAGS_SECURE_DISPLAY;
  }

  if (snap_usage & SnapUsage::PROTECTED) {
    gr_priv_flags |= qtigralloc::PRIV_FLAGS_SECURE_BUFFER;
  }

  return gr_priv_flags;
}
}  // namespace gralloc
