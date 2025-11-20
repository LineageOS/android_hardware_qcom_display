/*
 * Copyright (c) 2023-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "QtiAllocatorAIDL.h"

#include <cutils/properties.h>
#include <log/log.h>
#include <aidlcommonsupport/NativeHandle.h>
#include <vendor/qti/hardware/display/mapper/4.0/IQtiMapper.h>
#include <android/binder_ibinder_platform.h>

#include <vector>
#include <iostream>
#include <fstream>

#include "QtiMapper4.h"
#include "QtiMapper5.h"
#include "gr_utils.h"
#include "mapper_utils.h"

#include <dlfcn.h>
#include <vndksupport/linker.h>

#define FIFO_BASED_DISPLAY_PRIORITY 2

using gralloc::Error;
using mapper::GetStandardMetadata;
using mapper::GetVendorMetadata;

namespace aidl {
namespace android {
namespace hardware {
namespace graphics {
namespace allocator {
namespace impl {

typedef AIMapper_Error (*AIMapper_loadIMapperFn)(AIMapper *_Nullable *_Nonnull outImplementation);
static AIMapper *mapper_ = nullptr;

static void GetProperties(gralloc::GrallocProperties *props) {
  props->use_system_heap_for_sensors = property_get_bool(USE_SYSTEM_HEAP_FOR_SENSORS_PROP, 1);

  props->ubwc_disable = property_get_bool(DISABLE_UBWC_PROP, 0);

  props->ahardware_buffer_disable = property_get_bool(DISABLE_AHARDWARE_BUFFER_PROP, 0);
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

void QtiAllocatorAIDL::LoadQtiMapper5() {
  if (!mapper_) {
    std::string suffix;
    if (!getIMapperLibrarySuffix(&suffix).isOk()) {
      suffix = "qti";
    }
    std::string lib_name = "mapper." + suffix + ".so";
    void *so = android_load_sphal_library(lib_name.c_str(), RTLD_LOCAL | RTLD_NOW);
    if (!so) {
      ALOGW("Failed to load %s", lib_name.c_str());
      return;
    }

    auto loadIMapper = (AIMapper_loadIMapperFn)dlsym(so, "AIMapper_loadIMapper");
    AIMapper_Error error = loadIMapper(&mapper_);
    if (error != AIMAPPER_ERROR_NONE) {
      ALOGW("AIMapper_loadIMapper failed %d", error);
    }
  }
}

QtiAllocatorAIDL::QtiAllocatorAIDL() {
  // Attempt to load IMapper5
  LoadQtiMapper5();

  gralloc::GrallocProperties properties;
  GetProperties(&properties);
  buf_mgr_ = BufferManager::GetInstance();
  buf_mgr_->SetGrallocDebugProperties(properties);
  enable_logs_ = property_get_bool(ENABLE_LOGS_PROP, 0);
  if (mapper_) {
    snap_helper_ = gralloc::GrallocSnapHelper::GetInstance();
  } else {
    snap_helper_ = gralloc::GrallocSnapHelperLegacy::GetInstance();
  }
  enable_allocation_data_dumping_ = property_get_bool(ENABLE_ALLOCATION_DATA_DUMPING, 0);
  if (enable_allocation_data_dumping_) {
    // check if the json file exists
    std::string filename_prefix = "/data/vendor/display/gralloc/";
    std::time_t time = std::time({});
    tm *gm_time = std::gmtime(&time);
    if (gm_time == nullptr) {
      ALOGE("Failed to get current time");
      return;
    }
    char time_string[std::size("yyyy-mm-dd_hh_mm_ss")];
    std::strftime(std::data(time_string), std::size(time_string), "%F_%H_%M_%S", gm_time);
    json_file_name_ += filename_prefix + std::string(time_string) + std::string(".json");

    std::ifstream ifile;
    ifile.open(json_file_name_);
    if (ifile) {
      ALOGD_IF(enable_logs_, "Json File exists");
    } else {
      // check if directory exists
      struct stat sb;
      if (!(stat(filename_prefix.c_str(), &sb) == 0)) {
        ALOGD_IF(enable_logs_, "Creating a new directory");
        if (mkdir("/data/vendor/display/gralloc/", 0777) == -1) {
          ALOGE("Failed to create directory to store the json file");
          enable_allocation_data_dumping_ = false;
          return;
        }
      }
      ALOGD_IF(enable_logs_, "Creating a new file");
      std::fstream json_file;
      json_file.open(json_file_name_, std::ios::out);
      json_file << "[]" << std::endl;
      json_file.close();
      chmod(json_file_name_.c_str(), 0777);
    }
  }
}

int QtiAllocatorAIDL::dumpAllocationData(std::vector<buffer_handle_t> buffers,
                                         AllocationResult *result, gralloc::BufferDescriptor desc,
                                         int32_t count) {
  // Get the mapper service if not already obtained
  LoadQtiMapper5();
  if (!mapper_) {
    return 0;
  }

  ALOGD_IF(enable_logs_, "Mapper service obtained successfully");
  uint64_t id, size_from_get, usage_from_get, reserved_region_size = 0;
  int64_t is_ubwc = 0, is_tile_rendered = 0, is_cached = 0;
  uint32_t width_from_get, height_from_get, fd;
  PixelFormat format_from_get;
  std::string heap_name;
  std::vector<AidlPlaneLayout> plane_layouts;
  Json::Value json_entry;
  Json::Value planes(Json::arrayValue);
  std::streamoff filesize;
  std::string json_string;
  std::fstream fs;
  void *reserved_region_ptr = nullptr;

  native_handle_t *hnd = ::android::makeFromAidl(result->buffers[0]);
  if (!hnd) {
    ALOGE("Failed retrieve hnd");
    return 0;
  }
  ALOGD_IF(enable_logs_, "Successfully retrieved the handle");

  buffer_handle_t buf_hnd = nullptr;

  auto mapper_err = STABLEMAPPER(mapper_).importBuffer(hnd, &buf_hnd);
  if (mapper_err != AIMAPPER_ERROR_NONE) {
    ALOGW("Failed to import buffer.");
    goto end;
  }

  // Get pixel format
  {
    auto result =
        GetStandardMetadata<StandardMetadataType::PIXEL_FORMAT_REQUESTED>(mapper_, buf_hnd);
    if (!result.has_value()) {
      ALOGW("Failed to get Metadata - PixelFormatRequested");
      goto end;
    }
    format_from_get = static_cast<PixelFormat>(*result);
  }

  // Get buffer id
  {
    auto result = GetStandardMetadata<StandardMetadataType::BUFFER_ID>(mapper_, buf_hnd);
    if (!result.has_value()) {
      ALOGW("Failed to get Metadata - BufferId");
      goto end;
    }
    id = static_cast<uint64_t>(*result);
  }

  // Get stride
  {
    auto result = GetStandardMetadata<StandardMetadataType::STRIDE>(mapper_, buf_hnd);
    if (!result.has_value()) {
      ALOGW("Failed to get Metadata - Stride/AlignedWidthInPixels");
      goto end;
    }
    width_from_get = static_cast<uint32_t>(*result);
  }

  // Get aligned height
  mapper_err = GetVendorMetadata(mapper_, buf_hnd, SnapMetadataType::ALIGNED_HEIGHT_IN_PIXELS,
                                 static_cast<void *>(&height_from_get), sizeof(height_from_get));
  if (mapper_err != AIMAPPER_ERROR_NONE) {
    goto end;
  }

  // Get allocation size
  {
    auto result = GetStandardMetadata<StandardMetadataType::ALLOCATION_SIZE>(mapper_, buf_hnd);
    if (!result.has_value()) {
      ALOGW("Failed to get Metadata - AllocationSize");
      goto end;
    }
    size_from_get = static_cast<uint64_t>(*result);
  }

  // Get FD
  mapper_err = GetVendorMetadata(mapper_, buf_hnd, SnapMetadataType::FD, static_cast<void *>(&fd),
                                 sizeof(fd));
  if (mapper_err != AIMAPPER_ERROR_NONE) {
    goto end;
  }

  // Get private flags
  mapper_err = GetVendorMetadata(mapper_, buf_hnd, SnapMetadataType::IS_UBWC,
                                 static_cast<void *>(&is_ubwc), sizeof(is_ubwc));
  if (mapper_err != AIMAPPER_ERROR_NONE) {
    goto end;
  }
  mapper_err = GetVendorMetadata(mapper_, buf_hnd, SnapMetadataType::IS_TILE_RENDERED,
                                 static_cast<void *>(&is_tile_rendered), sizeof(is_tile_rendered));
  if (mapper_err != AIMAPPER_ERROR_NONE) {
    goto end;
  }
  mapper_err = GetVendorMetadata(mapper_, buf_hnd, SnapMetadataType::IS_CACHED,
                                 static_cast<void *>(&is_cached), sizeof(is_cached));
  if (mapper_err != AIMAPPER_ERROR_NONE) {
    goto end;
  }

  // Get usage
  {
    auto result = GetStandardMetadata<StandardMetadataType::USAGE>(mapper_, buf_hnd);
    if (!result.has_value()) {
      ALOGW("Failed to get Metadata - AllocationSize");
      goto end;
    }
    usage_from_get = static_cast<uint64_t>(*result);
  }

  // Get reserved region size
  mapper_err =
      STABLEMAPPER(mapper_).getReservedRegion(buf_hnd, &reserved_region_ptr, &reserved_region_size);
  if (mapper_err != AIMAPPER_ERROR_NONE) {
    ALOGW("Failed to get Reserved Region");
    goto end;
  }

  // Get plane layouts
  {
    auto result = GetStandardMetadata<StandardMetadataType::PLANE_LAYOUTS>(mapper_, buf_hnd);
    if (!result.has_value()) {
      ALOGW("Failed to get Metadata - PlaneLayouts");
      goto end;
    }
    plane_layouts = static_cast<std::vector<AidlPlaneLayout>>(*result);
  }

#ifdef QTI_HEAP_NAME
  // Get heap name
  mapper_err = GetVendorMetadata(mapper_, buf_hnd, SnapMetadataType::HEAP_NAME,
                                 static_cast<void *>(&heap_name), sizeof(heap_name));
  if (mapper_err != AIMAPPER_ERROR_NONE) {
    goto end;
  }

  json_entry["heapName"] = heap_name.c_str();
#endif

  json_entry["handleId"] = id;
  json_entry["width"] = width_from_get;
  json_entry["height"] = height_from_get;
  json_entry["requestedWidth"] = desc.GetWidth();
  json_entry["requestedHeight"] = desc.GetHeight();
  json_entry["size"] = size_from_get;
  json_entry["fd"] = fd;
  json_entry["isUBWC"] = is_ubwc;
  json_entry["isTileRendered"] = is_tile_rendered;
  json_entry["isCached"] = is_cached;
  json_entry["usage"] = usage_from_get;
  json_entry["format"] = static_cast<int32_t>(format_from_get);
  json_entry["layerCount"] = desc.GetLayerCount();
  json_entry["reservedSize"] = reserved_region_size;
  json_entry["name"] = desc.GetName().c_str();
  json_entry["requestedUsage"] = desc.GetUsage();
  json_entry["requestedFormat"] = desc.GetFormat();
  for (auto i = 0; i < plane_layouts.size(); i++) {
    const auto plane_layout = plane_layouts[i];
    Json::Value json_plane_layout;
    json_plane_layout["verticalSubsampling"] = plane_layout.verticalSubsampling;
    json_plane_layout["offsetInBytes"] = plane_layout.offsetInBytes;
    json_plane_layout["sampleIncrementInBits"] = plane_layout.sampleIncrementInBits;
    json_plane_layout["strideInBytes"] = plane_layout.strideInBytes;
    json_plane_layout["widthInSamples"] = plane_layout.widthInSamples;
    json_plane_layout["heightInSamples"] = plane_layout.heightInSamples;
    json_plane_layout["horizontalSubsampling"] = plane_layout.horizontalSubsampling;
    Json::Value components(Json::arrayValue);
    for (auto j = 0; j < plane_layout.components.size(); j++) {
      Json::Value component;
      const auto &plane_layout_component = plane_layout.components[j];
      component["component"] = plane_layout_component.type.value;
      component["sizeInBits"] = plane_layout_component.sizeInBits;
      component["offsetInBits"] = plane_layout_component.offsetInBits;
      components.append(component);
    }
    json_plane_layout["components"] = components;
    planes.append(json_plane_layout);
  }
  json_entry["planes"] = planes;
  {
    // Lock exists only for the json file
    // append json object to file
    std::lock_guard<std::mutex> lock(json_dump_lock_);
    fs.open(json_file_name_, std::ios::in | std::ios::out);
    fs.seekg(0, std::ios::end);
    filesize = fs.tellg();
    fs.seekp(filesize - 2);
    if (!is_json_first_entry_) {
      json_string = ",\n";
    }
    json_string.append(json_entry.toStyledString());
    json_string.append("\n]");
    fs.write(json_string.c_str(), json_string.size());
    is_json_first_entry_ = false;
    fs.close();
  }

end:
  if (!snap_helper_->IsSnapAllocEnabled()) {
    buf_mgr_->ReleaseBuffer(QTI_HANDLE_CONST(buf_hnd));
  } else {
    auto status = snap_helper_->Free(const_cast<native_handle_t *>(buf_hnd));
  }
  native_handle_delete(hnd);
  hnd = nullptr;
  return 0;
}

ndk::ScopedAStatus QtiAllocatorAIDL::AllocateBuffer(gralloc::BufferDescriptor desc, int32_t count,
                                                    AllocationResult *result) {
  std::vector<buffer_handle_t> buffers;
  auto err = Error::UNSUPPORTED;

  if (snap_helper_->IsSnapAllocEnabled()) {
    auto status = snap_helper_->Allocate(desc, count, result);
    if (status) {
      return ToBinderStatus(Error::UNSUPPORTED);
    }
  } else {
    buffers.reserve(count);
    for (uint32_t i = 0; i < count; i++) {
      buffer_handle_t buffer;
      err = buf_mgr_->AllocateBuffer(desc, &buffer);
      if (err != Error::NONE) {
        return ToBinderStatus(err);
      }
      buffers.emplace_back(buffer);
    }

    if (buffers.size() > 0) {
      result->stride = static_cast<uint32_t>(QTI_HANDLE_CONST(buffers[0])->width);
    }

    result->buffers.resize(count);
    for (int32_t i = 0; i < count; i++) {
      auto buffer = buffers[i];
      result->buffers[i] = ::android::dupToAidl(buffer);
    }
  }

  if (enable_allocation_data_dumping_) {
    dumpAllocationData(buffers, result, desc, count);
  }

  if (!snap_helper_->IsSnapAllocEnabled()) {
    for (int32_t i = 0; i < count; i++) {
      buffer_handle_t buffer = buffers[i];
      buf_mgr_->ReleaseBuffer(QTI_HANDLE_CONST(buffer));
    }
  }
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus QtiAllocatorAIDL::allocate(const std::vector<uint8_t> &descriptor, int32_t count,
                                              AllocationResult *result) {
  ALOGD_IF(enable_logs_, "Allocating buffers count: %d", count);
  gralloc::BufferDescriptor desc;

  auto err = ::vendor::qti::hardware::display::mapper::V4_0::implementation::QtiMapper::Decode(
      descriptor, &desc);
  if (err != Error::NONE) {
    ALOGE("Failed to allocate. Can't decode buffer descriptor: %d", err);
    return ToBinderStatus(err);
  }

  return AllocateBuffer(desc, count, result);
}

static gralloc::BufferDescriptor convertAidlToGrallocDescriptor(const BufferDescriptorInfo &info) {
  gralloc::BufferDescriptor desc;

  desc.SetName(std::string(reinterpret_cast<const char *>(info.name.data())));
  desc.SetDimensions(static_cast<int>(info.width), static_cast<int>(info.height));
  desc.SetLayerCount(static_cast<uint32_t>(info.layerCount));
  desc.SetColorFormat(static_cast<int>(info.format));
  desc.SetUsage(static_cast<uint64_t>(info.usage));
  desc.SetReservedSize(static_cast<uint64_t>(info.reservedSize));

  return desc;
}

ndk::ScopedAStatus QtiAllocatorAIDL::allocate2(const BufferDescriptorInfo &in_descriptor,
                                               int32_t in_count, AllocationResult *_aidl_return) {
  ALOGD_IF(enable_logs_, "Allocating buffers count: %d", in_count);
  if (!in_descriptor.additionalOptions.empty()) {
    return ToBinderStatus(Error::UNSUPPORTED);
  }

  gralloc::BufferDescriptor desc = convertAidlToGrallocDescriptor(in_descriptor);

  return AllocateBuffer(desc, in_count, _aidl_return);
}

ndk::ScopedAStatus QtiAllocatorAIDL::getIMapperLibrarySuffix(std::string *_aidl_return) {
  *_aidl_return = "qti";
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus QtiAllocatorAIDL::isSupported(const BufferDescriptorInfo &in_descriptor,
                                                 bool *_aidl_return) {
  if (!in_descriptor.additionalOptions.empty()) {
    *_aidl_return = false;
    return ndk::ScopedAStatus::ok();
  }

  gralloc::BufferDescriptor desc = convertAidlToGrallocDescriptor(in_descriptor);
  buffer_handle_t buffer;
  if (snap_helper_->IsSnapAllocEnabled()) {
    snap_helper_->IsSupported(desc, _aidl_return);
  } else {
    if (buf_mgr_->AllocateBuffer(desc, &buffer, 0, true) != Error::NONE) {
      *_aidl_return = false;
    }
    *_aidl_return = true;
  }
  return ndk::ScopedAStatus::ok();
}

ndk::SpAIBinder QtiAllocatorAIDL::createBinder() {
  auto binder = BnAllocator::createBinder();
  const int policy = SCHED_FIFO;
  int priority = sched_get_priority_min(policy);
  // Display priority is 2. Allocator binder thread priority should be always less than display
  // priority to avoid framedrops and janks.
  if (priority < FIFO_BASED_DISPLAY_PRIORITY) {
    AIBinder_setMinSchedulerPolicy(binder.get(), policy, priority);
  }
  AIBinder_setInheritRt(binder.get(), true);
  return binder;
}

}  // namespace impl
}  // namespace allocator
}  // namespace graphics
}  // namespace hardware
}  // namespace android
}  // namespace aidl
