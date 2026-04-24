/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef __GR_SNAP_HELPER_H__
#define __GR_SNAP_HELPER_H__

#include <functional>
#include <list>

#include <Address.h>
#include <BlendMode.h>
#include <BufferClient.h>
#include <SnapHandle.h>
#include <BufferLayout.h>
#include <BufferPermission.h>
#include <BufferUsage.h>
#include <CVPMetadata.h>
#include <ChromaSiting.h>
#include <Compression.h>
#include <CustomContentMetadata.h>
#include <Dataspace.h>
#include <Error.h>
#include <Fence.h>
#include <GraphicsMetadata.h>
#include <Interlaced.h>
#include <MetadataStatus.h>
#include <MetadataType.h>
#include <PixelFormat.h>
#include <PlaneLayout.h>
#include <PlaneLayoutComponent.h>
#include <PlaneLayoutComponentType.h>
#include <QtiColorPrimaries.h>
#include <QtiColorRange.h>
#include <QtiColorRemappingInfo.h>
#include <QtiContentLightLevel.h>
#include <QtiDynamicMetadata.h>
#include <QtiGammaTransfer.h>
#include <QtiGrallocMetadata.h>
#include <QtiMasteringDisplay.h>
#include <QtiMatrixCoEfficients.h>
#include <QtiAnamorphicMetadata.h>
#include <Rect.h>
#include <UBWCStats.h>
#include <VideoHistogramMetadata.h>
#include <VideoTimestampInfo.h>
#include <VideoTranscodeStatsMetadata.h>
#include <AllocationResult.h>
#include <BufferDescriptor.h>
#include <PixelFormatModifier.h>
#include <ReservedRegion.h>
#include <ISnapMapper.h>
#include <ISnapAlloc.h>
#include <ThreeDimensionalRefInfo.h>

#include <aidl/android/hardware/common/NativeHandle.h>
#include <aidl/android/hardware/graphics/common/Dataspace.h>
#include <aidl/android/hardware/graphics/common/BufferUsage.h>
#include <aidl/android/hardware/graphics/common/PixelFormat.h>

#include "gr_buf_descriptor.h"
#include "gr_snap_debugger.h"

namespace aidl::android::hardware::graphics::allocator {
class AllocationResult;
}

using SnapError = vendor::qti::hardware::display::snapalloc::Error;
using vendor::qti::hardware::display::snapalloc::ISnapAlloc;
using vendor::qti::hardware::display::snapalloc::ISnapMapper;
using SnapAllocationResult = vendor::qti::hardware::display::snapalloc::AllocationResult;
using vendor::qti::hardware::display::snapalloc::SnapHandle;
using SnapDescriptor = vendor::qti::hardware::display::snapalloc::BufferDescriptor;
using SnapAddress = vendor_qti_hardware_display_common_Address;
using SnapUsage = vendor_qti_hardware_display_common_BufferUsage;
using SnapMetadataType = vendor_qti_hardware_display_common_MetadataType;
using SnapPixelFormat = vendor_qti_hardware_display_common_PixelFormat;
using SnapKeyValuePair = vendor_qti_hardware_display_common_KeyValuePair;
using SnapPlaneLayout = vendor_qti_hardware_display_common_PlaneLayout;
using SnapPlaneLayoutComponent = vendor_qti_hardware_display_common_PlaneLayoutComponent;
using SnapPlaneLayoutComponentType = vendor_qti_hardware_display_common_PlaneLayoutComponentType;
using aidl::android::hardware::common::NativeHandle;
using GrallocBufferUsage = ::aidl::android::hardware::graphics::common::BufferUsage;
using GrallocPixelFormat = aidl::android::hardware::graphics::common::PixelFormat;
using GrallocPlaneLayout = aidl::android::hardware::graphics::common::PlaneLayout;
using GrallocPlaneLayoutComponent = aidl::android::hardware::graphics::common::PlaneLayoutComponent;
using GrallocPlaneLayoutComponentType =
    aidl::android::hardware::graphics::common::PlaneLayoutComponentType;
using GrallocExtendableType = aidl::android::hardware::graphics::common::ExtendableType;
using SnapColorPrimaries = vendor_qti_hardware_display_common_QtiColorPrimaries;
using SnapColorRange = vendor_qti_hardware_display_common_QtiColorRange;
using SnapColorRemappingInfo = vendor_qti_hardware_display_common_QtiColorRemappingInfo;
using SnapContentLightLevel = vendor_qti_hardware_display_common_QtiContentLightLevel;
using SnapDynamicMetadata = vendor_qti_hardware_display_common_QtiDynamicMetadata;
using SnapGammaTransfer = vendor_qti_hardware_display_common_QtiGammaTransfer;
using SnapMasteringDisplay = vendor_qti_hardware_display_common_QtiMasteringDisplay;
using SnapMatrixCoEfficients = vendor_qti_hardware_display_common_QtiMatrixCoEfficients;
using SnapUBWCStats = vendor_qti_hardware_display_common_UBWCStats;
using SnapUBWCVersion = vendor_qti_hardware_display_common_UBWCVersion;
using SnapDataspace = vendor_qti_hardware_display_common_Dataspace;
using GrallocCta861_3 = aidl::android::hardware::graphics::common::Cta861_3;
using GrallocDataspace = aidl::android::hardware::graphics::common::Dataspace;
using GrallocSmpte2086 = aidl::android::hardware::graphics::common::Smpte2086;
using SnapBufferLayout = vendor_qti_hardware_display_common_BufferLayout;
using SnapPlaneLayout = vendor_qti_hardware_display_common_PlaneLayout;
using SnapPlaneLayoutComponent = vendor_qti_hardware_display_common_PlaneLayoutComponent;
using SnapPixelFormatModifier = vendor_qti_hardware_display_common_PixelFormatModifier;
using SnapReservedRegion = vendor_qti_hardware_display_common_ReservedRegion;
using SnapPlaneLayoutComponentType = vendor_qti_hardware_display_common_PlaneLayoutComponentType;
using SnapRect = vendor_qti_hardware_display_common_Rect;
using SnapBlendMode = vendor_qti_hardware_display_common_BlendMode;
using SnapGraphicsMetadata = vendor_qti_hardware_display_common_GraphicsMetadata;
using aidl::android::hardware::graphics::common::BlendMode;
using aidl::android::hardware::graphics::common::Rect;
using SnapBufferPermission = vendor_qti_hardware_display_common_BufferPermission;
using SnapCVPMetadata = vendor_qti_hardware_display_common_CVPMetadata;
using SnapVideoTranscodeStatsMetadata =
    vendor_qti_hardware_display_common_VideoTranscodeStatsMetadata;
using SnapVideoTimestampInfo = vendor_qti_hardware_display_common_VideoTimestampInfo;
using SnapVideoHistogramMetadata = vendor_qti_hardware_display_common_VideoHistogramMetadata;
using SnapCustomContentMetadata = vendor_qti_hardware_display_common_CustomContentMetadata;
using SnapAnamorphicMetadata = vendor_qti_hardware_display_common_QtiAnamorphicMetadata;
using SnapThreeDimensionalRefInfo = vendor_qti_hardware_display_common_ThreeDimensionalRefInfo;

using ::android::hardware::hidl_vec;
using GrallocError = android::hardware::graphics::mapper::V4_0::Error;

namespace gralloc {

struct SnapFormatDescriptor {
  SnapPixelFormat format;
  SnapPixelFormatModifier modifier;

  bool operator==(const SnapFormatDescriptor &snap_fmt_desc) const {
    if (format == snap_fmt_desc.format && modifier == snap_fmt_desc.modifier) {
      return true;
    }
    return false;
  }
};

class SnapFormatDescriptorHash {
 public:
  size_t operator()(const SnapFormatDescriptor &snap_fmt_desc) const {
    return (std::hash<int>{}(static_cast<uint64_t>(snap_fmt_desc.format)) ^
            std::hash<int>{}(static_cast<uint64_t>(snap_fmt_desc.modifier)));
  }
};

class GrallocSnapHelperIntf {
 public:
  virtual ~GrallocSnapHelperIntf(){};
  virtual int Allocate(gralloc::BufferDescriptor gr_desc, int buffer_count,
                       aidl::android::hardware::graphics::allocator::AllocationResult *result);
  virtual int Import(native_handle_t *gr_hnd);
  virtual int Free(native_handle_t *gr_hnd);
  virtual int Lock(native_handle_t *gr_hnd, uint64_t gr_usage, CropRectangle_t gr_access_region,
                   int fence_fd, uint64_t *base_addr);
  virtual int Unlock(native_handle_t *gr_hnd, void *fence);
  virtual int ValidateBufferSize(native_handle_t *gr_hnd, gralloc::BufferInfo gr_desc);
  virtual int FlushLockedBuffer(native_handle_t *gr_hnd);
  virtual int RereadLockedBuffer(native_handle_t *gr_hnd);
  virtual int IsSupported(gralloc::BufferDescriptor gr_desc, bool *is_supported);
  // aidl_size acts as an aidl_convert_bytestream flag encoding to std::vector<uint8_t> for standard
  // metadata. mapper_return is the required size for output buffer, if a smaller buffer is provided
  // GetMetadata needs to be called again with appropriate size to obtain the standard metadata.
  virtual int GetMetadata(native_handle_t *gr_hnd, uint64_t gr_metadata_type, void *out,
                          bool convert_bytestream, bool check_metadata_set = true,
                          uint32_t aidl_size = 0, int32_t *mapper_return = nullptr);
  virtual int GetMetadataState(native_handle_t *gr_hnd, SnapMetadataType gr_metadata_type,
                               bool *out);
  virtual int SetMetadata(native_handle_t *gr_hnd, uint64_t gr_metadata_type, hidl_vec<uint8_t> in);
  // aidl_size here is size of the aidl / std::vector<uint8_t> bytestream to decode for standard
  // metadata
  virtual int SetMetadata(native_handle_t *gr_hnd, uint64_t gr_metadata_type, void *in,
                          uint32_t aidl_size);
  virtual int GetFromBufferDescriptor(gralloc::BufferDescriptor gr_desc, uint64_t gr_metadata_type,
                                      void *out, bool convert_to_hidl_bytestream);
  virtual int GetAllHandles(std::vector<buffer_handle_t> *out_handle_list);

  virtual bool IsSnapAllocEnabled();

  virtual bool IsBufferImported(native_handle_t *gr_hnd);
  virtual int GetCustomDimensions(native_handle_t *gr_hnd, int *stride, int *height);
  virtual int GetFormatLayout(gralloc::BufferInfo gr_desc, void *out, uint32_t *size,
                              int interlaced);
  virtual int GetReservedRegion(native_handle_t *gr_hnd, void **reserved_region,
                                uint64_t *reserved_region_size);
};

class GrallocSnapHelper : public GrallocSnapHelperIntf {
 public:
  static GrallocSnapHelper *GetInstance();
  int Allocate(gralloc::BufferDescriptor gr_desc, int buffer_count,
               aidl::android::hardware::graphics::allocator::AllocationResult *result);
  int Import(native_handle_t *gr_hnd);
  int Free(native_handle_t *gr_hnd);
  int Lock(native_handle_t *gr_hnd, uint64_t gr_usage, CropRectangle_t gr_access_region,
           int fence_fd, uint64_t *base_addr);
  int Unlock(native_handle_t *gr_hnd, void *fence);
  int ValidateBufferSize(native_handle_t *gr_hnd, gralloc::BufferInfo gr_desc);
  int FlushLockedBuffer(native_handle_t *gr_hnd);
  int RereadLockedBuffer(native_handle_t *gr_hnd);
  int IsSupported(gralloc::BufferDescriptor gr_desc, bool *is_supported);
  // aidl_size acts as an aidl_convert_bytestream flag encoding to std::vector<uint8_t> for standard
  // metadata. mapper_return is the required size for output buffer, if a smaller buffer is provided
  // GetMetadata needs to be called again with appropriate size to obtain the standard metadata.
  int GetMetadata(native_handle_t *gr_hnd, uint64_t gr_metadata_type, void *out,
                  bool convert_bytestream, bool check_metadata_set = true, uint32_t aidl_size = 0,
                  int32_t *mapper_return = nullptr);
  int GetMetadataState(native_handle_t *gr_hnd, SnapMetadataType gr_metadata_type, bool *out);
  // TODO: Remove when gralloc4 is completely deprecated
  int SetMetadata(native_handle_t *gr_hnd, uint64_t gr_metadata_type, hidl_vec<uint8_t> in) {
    return -1;
  };
  // aidl_size here is size of the aidl / std::vector<uint8_t> bytestream to decode for standard
  // metadata
  int SetMetadata(native_handle_t *gr_hnd, uint64_t gr_metadata_type, void *in, uint32_t aidl_size);
  int GetFromBufferDescriptor(gralloc::BufferDescriptor gr_desc, uint64_t gr_metadata_type,
                              void *out, bool convert_to_hidl_bytestream);
  int GetAllHandles(std::vector<buffer_handle_t> *out_handle_list);

  bool IsSnapAllocEnabled() { return snap_alloc_enable_; };

  bool IsBufferImported(native_handle_t *gr_hnd);
  int GetCustomDimensions(native_handle_t *gr_hnd, int *stride, int *height);
  int GetFormatLayout(gralloc::BufferInfo gr_desc, void *out, uint32_t *size, int interlaced);
  int GetReservedRegion(native_handle_t *gr_hnd, void **reserved_region,
                        uint64_t *reserved_region_size);
  int ImportViewBuffer(native_handle_t *meta_handle, uint32_t view,
                       buffer_handle_t *out_buffer_handle);
  int GetBaseView(native_handle_t *gr_hnd, uint32_t *view);

 private:
  GrallocSnapHelper();
  ~GrallocSnapHelper();
  SnapError GetSnapFormat(int hal_format, uint64_t usage, SnapFormatDescriptor *snap_fmt_desc);
  SnapUsage GetSnapUsage(uint64_t usage, int hal_format);
  int GetGrallocFormat(SnapFormatDescriptor snap_fmt_desc, SnapUsage usage, int *gr_format);
  int GetSnapFlatFormat(SnapFormatDescriptor snap_fmt_desc, SnapUsage usage,
                        SnapPixelFormat *snap_format);
  uint64_t GetGrallocUsage(SnapUsage snap_usage);
  SnapError ValidateGrallocUsage(uint64_t gralloc_usage);
  SnapError GetSnapDescriptor(gralloc::BufferDescriptor gr_desc, SnapDescriptor &snap_desc);
  SnapError GetSnapDescriptor(gralloc::BufferInfo gr_desc, SnapDescriptor &snap_desc);

  int ConvertSnapDataspaceToGrallocDataspace(SnapDataspace &snap_dataspace,
                                             GrallocDataspace *gr_dataspace);
  int ConvertGrallocDataspaceToSnapDataspace(GrallocDataspace gr_dataspace,
                                             SnapDataspace *snap_dataspace);
  int ConvertSnapPlaneLayoutComponentToGralloc(SnapPlaneLayout *layout);
  void ConvertSnapToGrallocPlaneComponentType(SnapPlaneLayoutComponentType snap_component_type,
                                              GrallocExtendableType *gr_component_type);
  SnapUBWCVersion GetSnapUBWCVersion(UBWC_Version version);
  UBWC_Version GetGrallocUBWCVersion(SnapUBWCVersion version);
  int ConvertSnapBufferlayoutToGrallocPlaneLayout(
      SnapHandle *hnd, SnapDescriptor *buf_des, const SnapBufferLayout snap_buffer_layout,
      std::vector<GrallocPlaneLayout> *gr_plane_layouts);
  int ConvertGrallocPlaneLayoutToAndroidYCbCr(
      uint64_t base_addr, const std::vector<GrallocPlaneLayout> gr_plane_layouts,
      struct android_ycbcr *outYCbCr);
  int GetColorSpaceFromDataspaceMetadata(SnapDataspace snap_dataspace, uint32_t *color_space);
  int GetSnapDataspaceMetadataFromColorSpace(uint32_t color_space, SnapDataspace *snap_dataspace);
  std::shared_ptr<ISnapMapper> snapmapper_;
  std::shared_ptr<ISnapAlloc> snapallocator_;
  bool snap_alloc_enable_ = false;
  void *snap_impl_lib_ = nullptr;
  std::shared_ptr<ISnapAlloc> (*LINK_FETCH_ISnapAlloc)(DebugCallbackIntf *) = nullptr;
  std::shared_ptr<ISnapMapper> (*LINK_FETCH_ISnapMapper)(DebugCallbackIntf *) = nullptr;
  static GrallocSnapHelper *s_instance;
  GrallocSnapDebugger debugger_impl_{};
  bool enable_logs_ = false;

  std::unordered_map<SnapFormatDescriptor, uint64_t, SnapFormatDescriptorHash>
      snap_to_gralloc_format_ = {
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_YCbCr_420_SP},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_HEIF},
           HAL_PIXEL_FORMAT_NV12_HEIF},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_ENCODEABLE},
           HAL_PIXEL_FORMAT_NV12_ENCODEABLE},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_TILED},
           HAL_PIXEL_FORMAT_YCbCr_420_SP_TILED},
          {{.format = SnapPixelFormat::YCrCb_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::YCRCB_420_SP)},
          {{.format = SnapPixelFormat::YCrCb_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS},
          {{.format = SnapPixelFormat::YCrCb_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_ADRENO},
           HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO},
          {{.format = SnapPixelFormat::YCrCb_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_ENCODEABLE},
           HAL_PIXEL_FORMAT_NV21_ENCODEABLE},
          // Linear format enum doesn't exist for TP10 in gralloc
          {{.format = SnapPixelFormat::TP10, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC},
          {{.format = SnapPixelFormat::YCBCR_P010, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_YCbCr_420_P010},
          {{.format = SnapPixelFormat::YCBCR_P010, .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS},
          {{.format = SnapPixelFormat::RGBA_8888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RGBA_8888},
          {{.format = SnapPixelFormat::BGRA_8888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_BGRA_8888},
          {{.format = SnapPixelFormat::RGBX_8888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RGBX_8888},
          {{.format = SnapPixelFormat::BGR_565, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_BGR_565},
          {{.format = SnapPixelFormat::RGBA_FP16, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::RGBA_FP16)},
          {{.format = SnapPixelFormat::RGB_888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RGB_888},
          {{.format = SnapPixelFormat::RGB_565, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::RGB_565)},
          {{.format = SnapPixelFormat::YV12, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::YV12)},
          {{.format = SnapPixelFormat::R_8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(aidl::android::hardware::graphics::common::PixelFormat::R_8)},
          // TODO: using FLEX to distinguish from aidl R_8 -- need to add separate modifier for this case
          {{.format = SnapPixelFormat::R_8, .modifier = PIXEL_FORMAT_MODIFIER_FLEX},
           HAL_PIXEL_FORMAT_R_8},
          {{.format = SnapPixelFormat::RG_88, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(HAL_PIXEL_FORMAT_RG_88)},
          {{.format = SnapPixelFormat::RGBA_1010102, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::RGBA_1010102)},
          {{.format = SnapPixelFormat::NV21_ZSL, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_NV21_ZSL},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::IMPLEMENTATION_DEFINED)},
          {{.format = SnapPixelFormat::YCBCR_420_888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::YCBCR_420_888)},
          {{.format = SnapPixelFormat::RAW8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RAW8},
          {{.format = SnapPixelFormat::RAW10, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RAW10},
          {{.format = SnapPixelFormat::RAW12, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RAW12},
          {{.format = SnapPixelFormat::RAW14, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RAW14},
          {{.format = SnapPixelFormat::RAW16, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RAW16},
          {{.format = SnapPixelFormat::DEPTH_16, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::DEPTH_16)},
          {{.format = SnapPixelFormat::DEPTH_24, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::DEPTH_24)},
          {{.format = SnapPixelFormat::DEPTH_24_STENCIL_8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::DEPTH_24_STENCIL_8)},
          {{.format = SnapPixelFormat::DEPTH_32F, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::DEPTH_32F)},
          {{.format = SnapPixelFormat::DEPTH_32F_STENCIL_8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::DEPTH_32F_STENCIL_8)},
          {{.format = SnapPixelFormat::STENCIL_8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::STENCIL_8)},
          {{.format = SnapPixelFormat::BLOB, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::BLOB)},
          {{.format = SnapPixelFormat::YCBCR_422_SP, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::YCBCR_422_SP)},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_LINEAR_FLEX},
           HAL_PIXEL_FORMAT_NV12_LINEAR_FLEX},
          {{.format = SnapPixelFormat::RAW_OPAQUE, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RAW_OPAQUE},
          {{.format = SnapPixelFormat::YCBCR_422_I, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_YCBCR_422_I},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::IMPLEMENTATION_DEFINED)},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           static_cast<int>(PixelFormat::IMPLEMENTATION_DEFINED)},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_ENCODEABLE},
           static_cast<int>(PixelFormat::IMPLEMENTATION_DEFINED)},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_HEIF},
           static_cast<int>(PixelFormat::IMPLEMENTATION_DEFINED)},
          {{.format = SnapPixelFormat::Y8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::Y8)},
          {{.format = SnapPixelFormat::Y16, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::Y16)},
          {{.format = SnapPixelFormat::YCrCb_422_SP, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_YCrCb_422_SP},
          {{.format = SnapPixelFormat::BGRX_8888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_BGRX_8888},
          {{.format = SnapPixelFormat::YCrCb_422_I, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_YCrCb_422_I},
          {{.format = SnapPixelFormat::CbYCrY_422_I, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_CbYCrY_422_I},
          {{.format = SnapPixelFormat::ARGB_2101010, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_ARGB_2101010},
          {{.format = SnapPixelFormat::XRGB_2101010, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_XRGB_2101010},
          {{.format = SnapPixelFormat::ABGR_2101010, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_ABGR_2101010},
          {{.format = SnapPixelFormat::XBGR_2101010, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_XBGR_2101010},
          {{.format = SnapPixelFormat::BGRA_1010102, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_BGRA_1010102},
          {{.format = SnapPixelFormat::BGRX_1010102, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_BGRX_1010102},
          {{.format = SnapPixelFormat::BGR_888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_BGR_888},
          {{.format = SnapPixelFormat::COMPRESSED_RGBA_ASTC_4x4_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_4x4_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_RGBA_ASTC_5x4_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_5x4_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_RGBA_ASTC_5x5_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_5x5_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_RGBA_ASTC_6x5_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_6x5_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_RGBA_ASTC_6x6_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_6x6_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_RGBA_ASTC_8x5_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x5_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_RGBA_ASTC_8x6_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x6_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_RGBA_ASTC_8x8_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_8x8_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_RGBA_ASTC_10x5_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x5_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_RGBA_ASTC_10x6_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x6_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_RGBA_ASTC_10x8_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x8_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_RGBA_ASTC_10x10_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_10x10_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_RGBA_ASTC_12x10_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_12x10_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_RGBA_ASTC_12x12_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_RGBA_ASTC_12x12_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR},
          {{.format = SnapPixelFormat::COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR},
          {{.format = SnapPixelFormat::RGBA_5551, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RGBA_5551},
          {{.format = SnapPixelFormat::RGBA_4444, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RGBA_4444},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX},
           HAL_PIXEL_FORMAT_NV12_UBWC_FLEX},
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX_2_BATCH},
           HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH},
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX_4_BATCH},
           HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH},
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX_8_BATCH},
           HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH},
          // Remove entries with PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC when support for
          // explicit UBWC formats get deprecated with SNAP_SUPPORTS_EXPLICIT_UBWC_FORMATS
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC},
           HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC},
          {{.format = SnapPixelFormat::YCBCR_P010, .modifier = PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC},
           HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC},
          {{.format = SnapPixelFormat::YCBCR_420_888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::YCBCR_420_888)},
          {{.format = SnapPixelFormat::YCBCR_420_888, .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           static_cast<int>(PixelFormat::YCBCR_420_888)},
          {{.format = SnapPixelFormat::YCBCR_420_888, .modifier = PIXEL_FORMAT_MODIFIER_ENCODEABLE},
           static_cast<int>(PixelFormat::YCBCR_420_888)},
          {{.format = SnapPixelFormat::YCBCR_420_888, .modifier = PIXEL_FORMAT_MODIFIER_HEIF},
           static_cast<int>(PixelFormat::YCBCR_420_888)},
      };

  std::unordered_map<SnapFormatDescriptor, SnapPixelFormat, SnapFormatDescriptorHash>
      snap_to_flat_format_ = {
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_HEIF},
           SnapPixelFormat::NV12_HEIF},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           SnapPixelFormat::YCbCr_420_SP_VENUS},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_ENCODEABLE},
           SnapPixelFormat::NV12_ENCODEABLE},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_TILED},
           SnapPixelFormat::YCbCr_420_SP_TILED},
          {{.format = SnapPixelFormat::YCrCb_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::YCrCb_420_SP},
          {{.format = SnapPixelFormat::YCrCb_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           SnapPixelFormat::YCrCb_420_SP_VENUS},
          {{.format = SnapPixelFormat::YCrCb_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_ADRENO},
           SnapPixelFormat::YCrCb_420_SP_ADRENO},
          {{.format = SnapPixelFormat::YCrCb_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_ENCODEABLE},
           SnapPixelFormat::NV21_ENCODEABLE},
          {{.format = SnapPixelFormat::TP10, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::TP10},
          {{.format = SnapPixelFormat::YCBCR_P010, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::YCBCR_P010},
          {{.format = SnapPixelFormat::YCBCR_P010, .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           SnapPixelFormat::YCbCr_420_P010_VENUS},
          {{.format = SnapPixelFormat::RGBA_8888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RGBA_8888},
          {{.format = SnapPixelFormat::BGRA_8888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::BGRA_8888},
          {{.format = SnapPixelFormat::RGBX_8888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RGBX_8888},
          {{.format = SnapPixelFormat::BGR_565, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::BGR_565},
          {{.format = SnapPixelFormat::RGBA_FP16, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RGBA_FP16},
          {{.format = SnapPixelFormat::RGB_888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RGB_888},
          {{.format = SnapPixelFormat::RGB_565, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RGB_565},
          {{.format = SnapPixelFormat::YV12, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::YV12},
          {{.format = SnapPixelFormat::R_8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::R_8},
          {{.format = SnapPixelFormat::RG_88, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RG_88},
          {{.format = SnapPixelFormat::RGBA_1010102, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RGBA_1010102},
          {{.format = SnapPixelFormat::NV21_ZSL, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::NV21_ZSL},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::IMPLEMENTATION_DEFINED},
          {{.format = SnapPixelFormat::YCBCR_420_888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::YCBCR_420_888},
          {{.format = SnapPixelFormat::RAW8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RAW8},
          {{.format = SnapPixelFormat::RAW10, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RAW10},
          {{.format = SnapPixelFormat::RAW12, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RAW12},
          {{.format = SnapPixelFormat::RAW14, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RAW14},
          {{.format = SnapPixelFormat::RAW16, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RAW16},
          {{.format = SnapPixelFormat::DEPTH_16, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::DEPTH_16},
          {{.format = SnapPixelFormat::DEPTH_24, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::DEPTH_24},
          {{.format = SnapPixelFormat::DEPTH_24_STENCIL_8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::DEPTH_24_STENCIL_8},
          {{.format = SnapPixelFormat::DEPTH_32F, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::DEPTH_32F},
          {{.format = SnapPixelFormat::DEPTH_32F_STENCIL_8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::DEPTH_32F_STENCIL_8},
          {{.format = SnapPixelFormat::STENCIL_8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::STENCIL_8},
          {{.format = SnapPixelFormat::BLOB, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::BLOB},
          {{.format = SnapPixelFormat::YCBCR_422_SP, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::YCBCR_422_SP},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_LINEAR_FLEX},
           SnapPixelFormat::NV12_LINEAR_FLEX},
          {{.format = SnapPixelFormat::RAW_OPAQUE, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RAW_OPAQUE},
          {{.format = SnapPixelFormat::YCBCR_422_I, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::YCBCR_422_I},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::IMPLEMENTATION_DEFINED},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           SnapPixelFormat::IMPLEMENTATION_DEFINED},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_ENCODEABLE},
           SnapPixelFormat::IMPLEMENTATION_DEFINED},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_HEIF},
           SnapPixelFormat::IMPLEMENTATION_DEFINED},
          {{.format = SnapPixelFormat::Y8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::Y8},
          {{.format = SnapPixelFormat::Y16, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::Y16},
          {{.format = SnapPixelFormat::YCrCb_422_SP, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::YCrCb_422_SP},
          {{.format = SnapPixelFormat::BGRX_8888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::BGRX_8888},
          {{.format = SnapPixelFormat::YCrCb_422_I, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::YCrCb_422_I},
          {{.format = SnapPixelFormat::CbYCrY_422_I, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::CbYCrY_422_I},
          {{.format = SnapPixelFormat::ARGB_2101010, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::ARGB_2101010},
          {{.format = SnapPixelFormat::XRGB_2101010, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::XRGB_2101010},
          {{.format = SnapPixelFormat::ABGR_2101010, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::ABGR_2101010},
          {{.format = SnapPixelFormat::XBGR_2101010, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::XBGR_2101010},
          {{.format = SnapPixelFormat::BGRA_1010102, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::BGRA_1010102},
          {{.format = SnapPixelFormat::BGRX_1010102, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::BGRX_1010102},
          {{.format = SnapPixelFormat::BGR_888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::BGR_888},
          {{.format = SnapPixelFormat::RGBA_5551, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RGBA_5551},
          {{.format = SnapPixelFormat::RGBA_4444, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RGBA_4444},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::YCbCr_420_SP},
      };

  std::unordered_map<SnapFormatDescriptor, SnapPixelFormat, SnapFormatDescriptorHash>
      snap_to_flat_ubwc_format_ = {
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX},
           SnapPixelFormat::NV12_UBWC_FLEX},
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX_2_BATCH},
           SnapPixelFormat::NV12_UBWC_FLEX_2_BATCH},
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX_4_BATCH},
           SnapPixelFormat::NV12_UBWC_FLEX_4_BATCH},
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX_8_BATCH},
           SnapPixelFormat::NV12_UBWC_FLEX_8_BATCH},
          // Remove entries with PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC when support for
          // explicit UBWC formats get deprecated with SNAP_SUPPORTS_EXPLICIT_UBWC_FORMATS
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC},
           SnapPixelFormat::YCbCr_420_SP_VENUS},
          {{.format = SnapPixelFormat::YCBCR_P010, .modifier = PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC},
           SnapPixelFormat::YCBCR_P010},
      };

  std::unordered_map<uint64_t, SnapFormatDescriptor> gralloc_to_snap_format_;

  // TODO: Remove after SDM/HWC has fully moved to Snap defs
  // UBWC format enums are deprecated, base types + UBWC usage flag needs to be used instead
  std::unordered_map<SnapFormatDescriptor, uint64_t, SnapFormatDescriptorHash>
      snap_to_gralloc_ubwc_format_ = {
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_4R},
           HAL_PIXEL_FORMAT_YCbCr_420_SP_4R_UBWC},
          {{.format = SnapPixelFormat::TP10, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC},
          {{.format = SnapPixelFormat::YCBCR_P010, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX},
           HAL_PIXEL_FORMAT_NV12_UBWC_FLEX},
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX_2_BATCH},
           HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH},
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX_4_BATCH},
           HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH},
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX_8_BATCH},
           HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH},
          // Remove entries with PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC when support for
          // explicit UBWC formats get deprecated with SNAP_SUPPORTS_EXPLICIT_UBWC_FORMATS
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC},
           HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC},
          {{.format = SnapPixelFormat::YCBCR_P010, .modifier = PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC},
           HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC},
      };

  std::unordered_map<uint64_t, SnapFormatDescriptor> gralloc_ubwc_to_snap_format_;

  std::unordered_map<SnapPlaneLayoutComponentType, int> snap_to_gralloc_plane_layout_component_ = {
      {PLANE_LAYOUT_COMPONENT_TYPE_Y, PLANE_COMPONENT_Y},
      {PLANE_LAYOUT_COMPONENT_TYPE_CB, PLANE_COMPONENT_Cb},
      {PLANE_LAYOUT_COMPONENT_TYPE_CR, PLANE_COMPONENT_Cr},
      {PLANE_LAYOUT_COMPONENT_TYPE_R, PLANE_COMPONENT_R},
      {PLANE_LAYOUT_COMPONENT_TYPE_G, PLANE_COMPONENT_G},
      {PLANE_LAYOUT_COMPONENT_TYPE_B, PLANE_COMPONENT_B},
      {PLANE_LAYOUT_COMPONENT_TYPE_A, PLANE_COMPONENT_A},
      {PLANE_LAYOUT_COMPONENT_TYPE_RAW, PLANE_COMPONENT_RAW},
      // META def is QTI defined, should only use the latest definition from SnapAlloc for Mapper5
      // Path
      {PLANE_LAYOUT_COMPONENT_TYPE_META, static_cast<int>(PLANE_LAYOUT_COMPONENT_TYPE_META)},
  };

  std::unordered_map<SnapPlaneLayoutComponentType, GrallocExtendableType>
      snap_to_gralloc_extendable_plane_layout_component_type_ = {
          {PLANE_LAYOUT_COMPONENT_TYPE_Y, android::gralloc4::PlaneLayoutComponentType_Y},
          {PLANE_LAYOUT_COMPONENT_TYPE_CB, android::gralloc4::PlaneLayoutComponentType_CB},
          {PLANE_LAYOUT_COMPONENT_TYPE_CR, android::gralloc4::PlaneLayoutComponentType_CR},
          {PLANE_LAYOUT_COMPONENT_TYPE_R, android::gralloc4::PlaneLayoutComponentType_R},
          {PLANE_LAYOUT_COMPONENT_TYPE_G, android::gralloc4::PlaneLayoutComponentType_G},
          {PLANE_LAYOUT_COMPONENT_TYPE_B, android::gralloc4::PlaneLayoutComponentType_B},
          {PLANE_LAYOUT_COMPONENT_TYPE_A, android::gralloc4::PlaneLayoutComponentType_A},
          {PLANE_LAYOUT_COMPONENT_TYPE_RAW, android::gralloc4::PlaneLayoutComponentType_RAW},
          {PLANE_LAYOUT_COMPONENT_TYPE_META, {"QTI", PLANE_LAYOUT_COMPONENT_TYPE_META}},
      };

  std::unordered_map<uint64_t, SnapUsage> gralloc_to_snap_usage_ = {
      {(uint64_t)GrallocBufferUsage::GPU_TEXTURE, SnapUsage::GPU_TEXTURE},
      {(uint64_t)GrallocBufferUsage::GPU_RENDER_TARGET, SnapUsage::GPU_RENDER_TARGET},
      {(uint64_t)GrallocBufferUsage::COMPOSER_OVERLAY, SnapUsage::COMPOSER_OVERLAY},
      {(uint64_t)GrallocBufferUsage::COMPOSER_CLIENT_TARGET, SnapUsage::COMPOSER_CLIENT_TARGET},
      {(uint64_t)GrallocBufferUsage::PROTECTED, SnapUsage::PROTECTED},
      {(uint64_t)GrallocBufferUsage::COMPOSER_CURSOR, SnapUsage::COMPOSER_CURSOR},
      {(uint64_t)GrallocBufferUsage::VIDEO_ENCODER, SnapUsage::VIDEO_ENCODER},
      {(uint64_t)GrallocBufferUsage::CAMERA_OUTPUT, SnapUsage::CAMERA_OUTPUT},
      {(uint64_t)GrallocBufferUsage::CAMERA_INPUT, SnapUsage::CAMERA_INPUT},
      {(uint64_t)GrallocBufferUsage::RENDERSCRIPT, SnapUsage::RENDERSCRIPT},
      {(uint64_t)GrallocBufferUsage::VIDEO_DECODER, SnapUsage::VIDEO_DECODER},
      {(uint64_t)GrallocBufferUsage::SENSOR_DIRECT_DATA, SnapUsage::SENSOR_DIRECT_DATA},
      {(uint64_t)GrallocBufferUsage::GPU_DATA_BUFFER, SnapUsage::GPU_DATA_BUFFER},
      {(uint64_t)GrallocBufferUsage::GPU_CUBE_MAP, SnapUsage::GPU_CUBE_MAP},
      {(uint64_t)GrallocBufferUsage::GPU_MIPMAP_COMPLETE, SnapUsage::GPU_MIPMAP_COMPLETE},
      {(uint64_t)GrallocBufferUsage::HW_IMAGE_ENCODER, SnapUsage::HW_IMAGE_ENCODER},
      {(uint64_t)GrallocBufferUsage::FRONT_BUFFER, SnapUsage::FRONT_BUFFER},
      {GRALLOC_USAGE_PRIVATE_VIDEO_HW, SnapUsage::QTI_PRIVATE_VIDEO_HW},
      {GRALLOC_USAGE_PRIVATE_ALLOC_UBWC, SnapUsage::QTI_ALLOC_UBWC},
      {GRALLOC_USAGE_PRIVATE_ALLOC_UBWC_PI, SnapUsage::QTI_PRIVATE_ALLOC_UBWC_PI},
      {GRALLOC_USAGE_PRIVATE_UNCACHED, SnapUsage::QTI_PRIVATE_UNCACHED},
      {GRALLOC_USAGE_PRIVATE_10BIT, SnapUsage::QTI_PRIVATE_10BIT},
      {GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY, SnapUsage::QTI_PRIVATE_SECURE_DISPLAY},
      {GRALLOC_USAGE_PRIVATE_VIDEO_NV21_ENCODER, SnapUsage::QTI_PRIVATE_VIDEO_NV21_ENCODER},
      {GRALLOC_USAGE_PRIVATE_CDSP, SnapUsage::QTI_PRIVATE_CDSP},
      {GRALLOC_USAGE_PRIVATE_WFD, SnapUsage::QTI_PRIVATE_WFD},
      {GRALLOC_USAGE_PRIVATE_VIDEO_HW, SnapUsage::QTI_PRIVATE_VIDEO_HW},
      {GRALLOC_USAGE_PRIVATE_TRUSTED_VM, SnapUsage::QTI_PRIVATE_TRUSTED_VM},
      {GRALLOC_USAGE_PRIVATE_ALLOC_UBWC_4R, SnapUsage::QTI_ALLOC_UBWC_4R},
      {GRALLOC_USAGE_PRIVATE_UBWC_L_8_TO_5, SnapUsage::QTI_ALLOC_UBWC_L_8_TO_5},
      {GRALLOC_USAGE_PRIVATE_UBWC_L_2_TO_1, SnapUsage::QTI_ALLOC_UBWC_L_2_TO_1},
      {(uint64_t)SnapUsage::QTI_PRIVATE_MULTI_VIEW_INFO, SnapUsage::QTI_PRIVATE_MULTI_VIEW_INFO},
  };

  std::unordered_map<SnapUsage, uint64_t> snap_to_gralloc_usage_;

  std::unordered_map<uint64_t, SnapUsage> cpu_gralloc_to_snap_usage_ = {
      {(uint64_t)GrallocBufferUsage::CPU_READ_RARELY, SnapUsage::CPU_READ_RARELY},
      {(uint64_t)GrallocBufferUsage::CPU_READ_OFTEN, SnapUsage::CPU_READ_OFTEN},
      {(uint64_t)GrallocBufferUsage::CPU_WRITE_RARELY, SnapUsage::CPU_WRITE_RARELY},
      {(uint64_t)GrallocBufferUsage::CPU_WRITE_OFTEN, SnapUsage::CPU_WRITE_OFTEN},
  };

  std::unordered_map<SnapUsage, uint64_t> cpu_snap_to_gralloc_usage_;

  std::unordered_map<UBWC_Version, SnapUBWCVersion> gralloc_to_snap_ubwc_version_{
      {UBWC_Version::UBWC_1_0, SnapUBWCVersion::UBWC_VERSION_1_0},
      {UBWC_Version::UBWC_2_0, SnapUBWCVersion::UBWC_VERSION_2_0},
      {UBWC_Version::UBWC_3_0, SnapUBWCVersion::UBWC_VERSION_3_0},
      {UBWC_Version::UBWC_4_0, SnapUBWCVersion::UBWC_VERSION_4_0},
      {UBWC_Version::UBWC_UNUSED, SnapUBWCVersion::UBWC_VERSION_UNUSED},
      {UBWC_Version::UBWC_MAX_VERSION, SnapUBWCVersion::UBWC_VERSION_MAX},
  };

  std::unordered_map<SnapUBWCVersion, UBWC_Version> snap_to_gralloc_ubwc_version_;

  std::unordered_map<uint64_t, SnapMetadataType> metadata_type_map = {
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::BUFFER_ID),
       SnapMetadataType::BUFFER_ID},
      {static_cast<uint64_t>(aidl::android::hardware::graphics::common::StandardMetadataType::NAME),
       SnapMetadataType::NAME},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::WIDTH),
       SnapMetadataType::WIDTH},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::HEIGHT),
       SnapMetadataType::HEIGHT},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::LAYER_COUNT),
       SnapMetadataType::LAYER_COUNT},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::PIXEL_FORMAT_REQUESTED),
       SnapMetadataType::PIXEL_FORMAT_REQUESTED},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::PIXEL_FORMAT_FOURCC),
       SnapMetadataType::PIXEL_FORMAT_FOURCC},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::PIXEL_FORMAT_MODIFIER),
       SnapMetadataType::DRM_PIXEL_FORMAT_MODIFIER},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::USAGE),
       SnapMetadataType::USAGE},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::PLANE_LAYOUTS),
       SnapMetadataType::PLANE_LAYOUTS},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::PROTECTED_CONTENT),
       SnapMetadataType::PROTECTED_CONTENT},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::ALLOCATION_SIZE),
       SnapMetadataType::ALLOCATION_SIZE},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::BLEND_MODE),
       SnapMetadataType::BLEND_MODE},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::DATASPACE),
       SnapMetadataType::DATASPACE},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::SMPTE2086),
       SnapMetadataType::MASTERING_DISPLAY},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::CTA861_3),
       SnapMetadataType::CONTENT_LIGHT_LEVEL},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::SMPTE2094_40),
       SnapMetadataType::DYNAMIC_METADATA},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::SMPTE2094_10),
       SnapMetadataType::SMPTE2094_10},
      {static_cast<uint64_t>(aidl::android::hardware::graphics::common::StandardMetadataType::CROP),
       SnapMetadataType::CROP},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::CHROMA_SITING),
       SnapMetadataType::CHROMA_SITING},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::INTERLACED),
       SnapMetadataType::INTERLACED},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::COMPRESSION),
       SnapMetadataType::COMPRESSION},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::STRIDE),
       SnapMetadataType::STRIDE},
      {QTI_GRAPHICS_METADATA, SnapMetadataType::GRAPHICS_METADATA},
      {QTI_STANDARD_METADATA_STATUS, SnapMetadataType::STANDARD_METADATA_STATUS},
      {QTI_VENDOR_METADATA_STATUS, SnapMetadataType::VENDOR_METADATA_STATUS},
      {QTI_CUSTOM_DIMENSIONS_STRIDE, SnapMetadataType::CUSTOM_DIMENSIONS_STRIDE},
      {QTI_CUSTOM_DIMENSIONS_HEIGHT, SnapMetadataType::CUSTOM_DIMENSIONS_HEIGHT},
      {QTI_RGB_DATA_ADDRESS, SnapMetadataType::RGB_DATA_ADDRESS},
      {QTI_ALIGNED_WIDTH_IN_PIXELS, SnapMetadataType::ALIGNED_WIDTH_IN_PIXELS},
      {QTI_ALIGNED_HEIGHT_IN_PIXELS, SnapMetadataType::ALIGNED_HEIGHT_IN_PIXELS},
      {QTI_BUFFER_TYPE, SnapMetadataType::BUFFER_TYPE},
      {QTI_MEM_HANDLE, SnapMetadataType::MEM_HANDLE},
      {QTI_FD, SnapMetadataType::FD},
      {QTI_PP_PARAM_INTERLACED, SnapMetadataType::PP_PARAM_INTERLACED},
      {QTI_REFRESH_RATE, SnapMetadataType::REFRESH_RATE},
      {QTI_MAP_SECURE_BUFFER, SnapMetadataType::MAP_SECURE_BUFFER},
      {QTI_LINEAR_FORMAT, SnapMetadataType::LINEAR_FORMAT},
      {QTI_SINGLE_BUFFER_MODE, SnapMetadataType::SINGLE_BUFFER_MODE},
      {QTI_VT_TIMESTAMP, SnapMetadataType::VT_TIMESTAMP},
      {QTI_UBWC_CR_STATS_INFO, SnapMetadataType::UBWC_CR_STATS_INFO},
      {QTI_VIDEO_PERF_MODE, SnapMetadataType::VIDEO_PERF_MODE},
      {QTI_CVP_METADATA, SnapMetadataType::CVP_METADATA},
      {QTI_VIDEO_HISTOGRAM_STATS, SnapMetadataType::VIDEO_HISTOGRAM_STATS},
      {QTI_VIDEO_TRANSCODE_STATS, SnapMetadataType::VIDEO_TRANSCODE_STATS},
      {QTI_VIDEO_TS_INFO, SnapMetadataType::VIDEO_TS_INFO},
      {QTI_TIMED_RENDERING, SnapMetadataType::TIMED_RENDERING},
      {QTI_BUFFER_PERMISSION, SnapMetadataType::BUFFER_PERMISSION},
      {QTI_CUSTOM_CONTENT_METADATA, SnapMetadataType::CUSTOM_CONTENT_METADATA},
      {QTI_HEAP_NAME, SnapMetadataType::HEAP_NAME},
      {QTI_EARLYNOTIFY_LINECOUNT, SnapMetadataType::EARLYNOTIFY_LINECOUNT},
      // New enum entries
      {SnapMetadataType::HEAP_NAME, SnapMetadataType::HEAP_NAME},
      {SnapMetadataType::IS_UBWC, SnapMetadataType::IS_UBWC},
      {SnapMetadataType::IS_TILE_RENDERED, SnapMetadataType::IS_TILE_RENDERED},
      {SnapMetadataType::IS_CACHED, SnapMetadataType::IS_CACHED},
      {SnapMetadataType::MASTERING_DISPLAY, SnapMetadataType::MASTERING_DISPLAY},
      {SnapMetadataType::CONTENT_LIGHT_LEVEL, SnapMetadataType::CONTENT_LIGHT_LEVEL},
      {SnapMetadataType::DYNAMIC_METADATA, SnapMetadataType::DYNAMIC_METADATA},
      {SnapMetadataType::MATRIX_COEFFICIENTS, SnapMetadataType::MATRIX_COEFFICIENTS},
      {SnapMetadataType::COLOR_REMAPPING_INFO, SnapMetadataType::COLOR_REMAPPING_INFO},
      {SnapMetadataType::BASE_ADDRESS, SnapMetadataType::BASE_ADDRESS},
      {SnapMetadataType::PIXEL_FORMAT_ALLOCATED, SnapMetadataType::PIXEL_FORMAT_ALLOCATED},
      {SnapMetadataType::ANAMORPHIC_COMPRESSION_METADATA,
       SnapMetadataType::ANAMORPHIC_COMPRESSION_METADATA},
      {SnapMetadataType::THREE_DIMENSIONAL_REF_INFO, SnapMetadataType::THREE_DIMENSIONAL_REF_INFO},
  };

  std::list<int> unsupported_formats = {
      static_cast<int>(aidl::android::hardware::graphics::common::PixelFormat::R_16_UINT),
      static_cast<int>(aidl::android::hardware::graphics::common::PixelFormat::RG_1616_UINT),
      static_cast<int>(aidl::android::hardware::graphics::common::PixelFormat::RGBA_10101010),
  };

  typedef SnapError (GrallocSnapHelper::*MetadataHelper)(
      SnapHandle *, uint32_t aidl_size, void *gralloc_in_set, void *gralloc_out_get,
      SnapDescriptor *buf_des, bool check_metadata_set, int32_t *mapper_return);

  SnapError BufferIDHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                           void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                           bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError UsageHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                        void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                        bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError DataspaceHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                            void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                            bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError NameHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                       void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                       bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError WidthHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                        void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                        bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError HeightHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                         void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                         bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError LayerCountHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                             void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                             bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError PixelFormatRequestedHelper(SnapHandle *, uint32_t aidl_size,
                                       void *gralloc_in_set = nullptr,
                                       void *gralloc_out_get = nullptr,
                                       SnapDescriptor *buf_des = nullptr,
                                       bool check_metadata_set = true,
                                       int32_t *mapper_return = nullptr);
  SnapError PixelFormatFourCCHelper(SnapHandle *, uint32_t aidl_size,
                                    void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                    SnapDescriptor *buf_des = nullptr,
                                    bool check_metadata_set = true,
                                    int32_t *mapper_return = nullptr);
  SnapError DRMPixelFormatModifierHelper(SnapHandle *, uint32_t aidl_size,
                                         void *gralloc_in_set = nullptr,
                                         void *gralloc_out_get = nullptr,
                                         SnapDescriptor *buf_des = nullptr,
                                         bool check_metadata_set = true,
                                         int32_t *mapper_return = nullptr);
  SnapError AllocationSizeHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                                 void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                                 bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError ProtectedContentHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                                   void *gralloc_out_get = nullptr,
                                   SnapDescriptor *buf_des = nullptr,
                                   bool check_metadata_set = true,
                                   int32_t *mapper_return = nullptr);
  SnapError CompressionHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                              void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                              bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError InterlacedHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                             void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                             bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError ChromaSitingHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                               void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                               bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError PlaneLayoutsHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                               void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                               bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError CropHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                       void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                       bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError BlendModeHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                            void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                            bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError VTTimestampHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                              void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                              bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError PPParamInterlacedHelper(SnapHandle *, uint32_t aidl_size,
                                    void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                    SnapDescriptor *buf_des = nullptr,
                                    bool check_metadata_set = true,
                                    int32_t *mapper_return = nullptr);
  SnapError VideoPerfModeHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                                void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                                bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError GraphicsMetadataHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                                   void *gralloc_out_get = nullptr,
                                   SnapDescriptor *buf_des = nullptr,
                                   bool check_metadata_set = true,
                                   int32_t *mapper_return = nullptr);
  SnapError UBWCCRStatsInfoHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                                  void *gralloc_out_get = nullptr,
                                  SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                                  int32_t *mapper_return = nullptr);
  SnapError RefreshRateHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                              void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                              bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError MapSecureBufferHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                                  void *gralloc_out_get = nullptr,
                                  SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                                  int32_t *mapper_return = nullptr);
  SnapError LinearFormatHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                               void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                               bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError SingleBufferModeHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                                   void *gralloc_out_get = nullptr,
                                   SnapDescriptor *buf_des = nullptr,
                                   bool check_metadata_set = true,
                                   int32_t *mapper_return = nullptr);
  SnapError CVPMetadataHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                              void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                              bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError VideoHistogramStatsHelper(SnapHandle *, uint32_t aidl_size,
                                      void *gralloc_in_set = nullptr,
                                      void *gralloc_out_get = nullptr,
                                      SnapDescriptor *buf_des = nullptr,
                                      bool check_metadata_set = true,
                                      int32_t *mapper_return = nullptr);
  SnapError FDHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                     void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                     bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError AlignedWidthInPixelsHelper(SnapHandle *, uint32_t aidl_size,
                                       void *gralloc_in_set = nullptr,
                                       void *gralloc_out_get = nullptr,
                                       SnapDescriptor *buf_des = nullptr,
                                       bool check_metadata_set = true,
                                       int32_t *mapper_return = nullptr);
  SnapError AlignedHeightInPixelsHelper(SnapHandle *, uint32_t aidl_size,
                                        void *gralloc_in_set = nullptr,
                                        void *gralloc_out_get = nullptr,
                                        SnapDescriptor *buf_des = nullptr,
                                        bool check_metadata_set = true,
                                        int32_t *mapper_return = nullptr);
  SnapError StandardMetadataStatusHelper(SnapHandle *, uint32_t aidl_size,
                                         void *gralloc_in_set = nullptr,
                                         void *gralloc_out_get = nullptr,
                                         SnapDescriptor *buf_des = nullptr,
                                         bool check_metadata_set = true,
                                         int32_t *mapper_return = nullptr);
  SnapError VendorMetadataStatusHelper(SnapHandle *, uint32_t aidl_size,
                                       void *gralloc_in_set = nullptr,
                                       void *gralloc_out_get = nullptr,
                                       SnapDescriptor *buf_des = nullptr,
                                       bool check_metadata_set = true,
                                       int32_t *mapper_return = nullptr);
  SnapError BufferTypeHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                             void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                             bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError VideoTSInfoHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                              void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                              bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError CustomDimensionsStrideHelper(SnapHandle *, uint32_t aidl_size,
                                         void *gralloc_in_set = nullptr,
                                         void *gralloc_out_get = nullptr,
                                         SnapDescriptor *buf_des = nullptr,
                                         bool check_metadata_set = true,
                                         int32_t *mapper_return = nullptr);
  SnapError CustomDimensionsHeightHelper(SnapHandle *, uint32_t aidl_size,
                                         void *gralloc_in_set = nullptr,
                                         void *gralloc_out_get = nullptr,
                                         SnapDescriptor *buf_des = nullptr,
                                         bool check_metadata_set = true,
                                         int32_t *mapper_return = nullptr);
  SnapError RGBDataAddressHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                                 void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                                 bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError BufferPermissionHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                                   void *gralloc_out_get = nullptr,
                                   SnapDescriptor *buf_des = nullptr,
                                   bool check_metadata_set = true,
                                   int32_t *mapper_return = nullptr);
  SnapError MemHandleHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                            void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                            bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError TimedRenderingHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                                 void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                                 bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError CustomContentMetadataHelper(SnapHandle *, uint32_t aidl_size,
                                        void *gralloc_in_set = nullptr,
                                        void *gralloc_out_get = nullptr,
                                        SnapDescriptor *buf_des = nullptr,
                                        bool check_metadata_set = true,
                                        int32_t *mapper_return = nullptr);
  SnapError VideoTranscodeStatsHelper(SnapHandle *, uint32_t aidl_size,
                                      void *gralloc_in_set = nullptr,
                                      void *gralloc_out_get = nullptr,
                                      SnapDescriptor *buf_des = nullptr,
                                      bool check_metadata_set = true,
                                      int32_t *mapper_return = nullptr);
  SnapError MasteringDisplayHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                                   void *gralloc_out_get = nullptr,
                                   SnapDescriptor *buf_des = nullptr,
                                   bool check_metadata_set = true,
                                   int32_t *mapper_return = nullptr);
  SnapError ContentLightLevelHelper(SnapHandle *, uint32_t aidl_size,
                                    void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                    SnapDescriptor *buf_des = nullptr,
                                    bool check_metadata_set = true,
                                    int32_t *mapper_return = nullptr);
  SnapError DynamicMetadataHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                                  void *gralloc_out_get = nullptr,
                                  SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                                  int32_t *mapper_return = nullptr);
  SnapError SMPTE2094_10Helper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                               void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                               bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError MatrixCoefficientsHelper(SnapHandle *, uint32_t aidl_size,
                                     void *gralloc_in_set = nullptr,
                                     void *gralloc_out_get = nullptr,
                                     SnapDescriptor *buf_des = nullptr,
                                     bool check_metadata_set = true,
                                     int32_t *mapper_return = nullptr);
  SnapError ColorRemappingInfoHelper(SnapHandle *, uint32_t aidl_size,
                                     void *gralloc_in_set = nullptr,
                                     void *gralloc_out_get = nullptr,
                                     SnapDescriptor *buf_des = nullptr,
                                     bool check_metadata_set = true,
                                     int32_t *mapper_return = nullptr);
  SnapError HeapNameHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                           void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                           bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError IsUBWCHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                         void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                         bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError IsTileRenderedHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                                 void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                                 bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError IsCachedHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                           void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                           bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError PixelFormatAllocatedHelper(SnapHandle *, uint32_t aidl_size,
                                       void *gralloc_in_set = nullptr,
                                       void *gralloc_out_get = nullptr,
                                       SnapDescriptor *buf_des = nullptr,
                                       bool check_metadata_set = true,
                                       int32_t *mapper_return = nullptr);
  SnapError BaseAddressHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                              void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                              bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError EarlyNotifyLineCountHelper(SnapHandle *, uint32_t aidl_size,
                                       void *gralloc_in_set = nullptr,
                                       void *gralloc_out_get = nullptr,
                                       SnapDescriptor *buf_des = nullptr,
                                       bool check_metadata_set = true,
                                       int32_t *mapper_return = nullptr);
  SnapError BufferDequeueDurationHelper(SnapHandle *, uint32_t aidl_size,
                                        void *gralloc_in_set = nullptr,
                                        void *gralloc_out_get = nullptr,
                                        SnapDescriptor *buf_des = nullptr,
                                        bool check_metadata_set = true,
                                        int32_t *mapper_return = nullptr);
  SnapError CompressionMetadataHelper(SnapHandle *hnd, uint32_t aidl_size,
                                      void *gralloc_in_set = nullptr,
                                      void *gralloc_out_get = nullptr,
                                      SnapDescriptor *buf_des = nullptr,
                                      bool check_metadata_set = true,
                                      int32_t *mapper_return = nullptr);
  SnapError BaseViewHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                           void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                           bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError MultiViewHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                            void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                            bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError ThreeDimensionalRefInfoHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                                void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                                bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError ViewIdHelper(SnapHandle *, uint32_t aidl_size, void *gralloc_in_set = nullptr,
                         void *gralloc_out_get = nullptr, SnapDescriptor *buf_des = nullptr,
                         bool check_metadata_set = true, int32_t *mapper_return = nullptr);

  std::unordered_map<vendor_qti_hardware_display_common_MetadataType, MetadataHelper>
      metadata_conversion_helper_function_map = {
          {BUFFER_ID, &GrallocSnapHelper::BufferIDHelper},
          {NAME, &GrallocSnapHelper::NameHelper},
          {WIDTH, &GrallocSnapHelper::WidthHelper},
          {HEIGHT, &GrallocSnapHelper::HeightHelper},
          {LAYER_COUNT, &GrallocSnapHelper::LayerCountHelper},
          {PIXEL_FORMAT_REQUESTED, &GrallocSnapHelper::PixelFormatRequestedHelper},
          {PIXEL_FORMAT_FOURCC, &GrallocSnapHelper::PixelFormatFourCCHelper},
          {DRM_PIXEL_FORMAT_MODIFIER, &GrallocSnapHelper::DRMPixelFormatModifierHelper},
          {USAGE, &GrallocSnapHelper::UsageHelper},
          {ALLOCATION_SIZE, &GrallocSnapHelper::AllocationSizeHelper},
          {PROTECTED_CONTENT, &GrallocSnapHelper::ProtectedContentHelper},
          {COMPRESSION, &GrallocSnapHelper::CompressionHelper},
          {INTERLACED, &GrallocSnapHelper::InterlacedHelper},
          {CHROMA_SITING, &GrallocSnapHelper::ChromaSitingHelper},
          {PLANE_LAYOUTS, &GrallocSnapHelper::PlaneLayoutsHelper},
          {CROP, &GrallocSnapHelper::CropHelper},
          {DATASPACE, &GrallocSnapHelper::DataspaceHelper},
          {BLEND_MODE, &GrallocSnapHelper::BlendModeHelper},
          {VT_TIMESTAMP, &GrallocSnapHelper::VTTimestampHelper},
          {PP_PARAM_INTERLACED, &GrallocSnapHelper::PPParamInterlacedHelper},
          {VIDEO_PERF_MODE, &GrallocSnapHelper::VideoPerfModeHelper},
          {GRAPHICS_METADATA, &GrallocSnapHelper::GraphicsMetadataHelper},
          {UBWC_CR_STATS_INFO, &GrallocSnapHelper::UBWCCRStatsInfoHelper},
          {REFRESH_RATE, &GrallocSnapHelper::RefreshRateHelper},
          {MAP_SECURE_BUFFER, &GrallocSnapHelper::MapSecureBufferHelper},
          {LINEAR_FORMAT, &GrallocSnapHelper::LinearFormatHelper},
          {SINGLE_BUFFER_MODE, &GrallocSnapHelper::SingleBufferModeHelper},
          {CVP_METADATA, &GrallocSnapHelper::CVPMetadataHelper},
          {VIDEO_HISTOGRAM_STATS, &GrallocSnapHelper::VideoHistogramStatsHelper},
          {FD, &GrallocSnapHelper::FDHelper},
          {ALIGNED_WIDTH_IN_PIXELS, &GrallocSnapHelper::AlignedWidthInPixelsHelper},
          {STRIDE, &GrallocSnapHelper::AlignedWidthInPixelsHelper},
          {ALIGNED_HEIGHT_IN_PIXELS, &GrallocSnapHelper::AlignedHeightInPixelsHelper},
          {STANDARD_METADATA_STATUS, &GrallocSnapHelper::StandardMetadataStatusHelper},
          {VENDOR_METADATA_STATUS, &GrallocSnapHelper::VendorMetadataStatusHelper},
          {BUFFER_TYPE, &GrallocSnapHelper::BufferTypeHelper},
          {VIDEO_TS_INFO, &GrallocSnapHelper::VideoTSInfoHelper},
          {CUSTOM_DIMENSIONS_STRIDE, &GrallocSnapHelper::CustomDimensionsStrideHelper},
          {CUSTOM_DIMENSIONS_HEIGHT, &GrallocSnapHelper::CustomDimensionsHeightHelper},
          {RGB_DATA_ADDRESS, &GrallocSnapHelper::RGBDataAddressHelper},
          {BUFFER_PERMISSION, &GrallocSnapHelper::BufferPermissionHelper},
          {MEM_HANDLE, &GrallocSnapHelper::MemHandleHelper},
          {TIMED_RENDERING, &GrallocSnapHelper::TimedRenderingHelper},
          {CUSTOM_CONTENT_METADATA, &GrallocSnapHelper::CustomContentMetadataHelper},
          {VIDEO_TRANSCODE_STATS, &GrallocSnapHelper::VideoTranscodeStatsHelper},
          {MASTERING_DISPLAY, &GrallocSnapHelper::MasteringDisplayHelper},
          {static_cast<vendor_qti_hardware_display_common_MetadataType>(
               StandardMetadataType::SMPTE2086),
           &GrallocSnapHelper::MasteringDisplayHelper},
          {CONTENT_LIGHT_LEVEL, &GrallocSnapHelper::ContentLightLevelHelper},
          {static_cast<vendor_qti_hardware_display_common_MetadataType>(
               StandardMetadataType::CTA861_3),
           &GrallocSnapHelper::ContentLightLevelHelper},
          {DYNAMIC_METADATA, &GrallocSnapHelper::DynamicMetadataHelper},
          {static_cast<vendor_qti_hardware_display_common_MetadataType>(
               StandardMetadataType::SMPTE2094_40),
           &GrallocSnapHelper::DynamicMetadataHelper},
          {static_cast<vendor_qti_hardware_display_common_MetadataType>(
               StandardMetadataType::SMPTE2094_10),
           &GrallocSnapHelper::SMPTE2094_10Helper},
          {COLOR_REMAPPING_INFO, &GrallocSnapHelper::ColorRemappingInfoHelper},
          {HEAP_NAME, &GrallocSnapHelper::HeapNameHelper},
          {IS_UBWC, &GrallocSnapHelper::IsUBWCHelper},
          {IS_TILE_RENDERED, &GrallocSnapHelper::IsTileRenderedHelper},
          {IS_CACHED, &GrallocSnapHelper::IsCachedHelper},
          {PIXEL_FORMAT_ALLOCATED, &GrallocSnapHelper::PixelFormatAllocatedHelper},
          {BASE_ADDRESS, &GrallocSnapHelper::BaseAddressHelper},
          {MATRIX_COEFFICIENTS, &GrallocSnapHelper::MatrixCoefficientsHelper},
          {EARLYNOTIFY_LINECOUNT, &GrallocSnapHelper::EarlyNotifyLineCountHelper},
          {BUFFER_DEQUEUE_DURATION, &GrallocSnapHelper::BufferDequeueDurationHelper},
          {ANAMORPHIC_COMPRESSION_METADATA, &GrallocSnapHelper::CompressionMetadataHelper},
          {BASE_VIEW, &GrallocSnapHelper::BaseViewHelper},
          {MULTI_VIEW_INFO, &GrallocSnapHelper::MultiViewHelper},
          {THREE_DIMENSIONAL_REF_INFO, &GrallocSnapHelper::ThreeDimensionalRefInfoHelper},
          {VIEW_ID, &GrallocSnapHelper::ViewIdHelper},
      };

  std::unordered_map<vendor_qti_hardware_display_common_MetadataType, MetadataHelper>
      bufferdescription_conversion_helper_function_map = {
          {NAME, &GrallocSnapHelper::NameHelper},
          {WIDTH, &GrallocSnapHelper::WidthHelper},
          {HEIGHT, &GrallocSnapHelper::HeightHelper},
          {LAYER_COUNT, &GrallocSnapHelper::LayerCountHelper},
          {PIXEL_FORMAT_REQUESTED, &GrallocSnapHelper::PixelFormatRequestedHelper},
          {PIXEL_FORMAT_FOURCC, &GrallocSnapHelper::PixelFormatFourCCHelper},
          {USAGE, &GrallocSnapHelper::UsageHelper},
          {ALLOCATION_SIZE, &GrallocSnapHelper::AllocationSizeHelper},
          {PROTECTED_CONTENT, &GrallocSnapHelper::ProtectedContentHelper},
          {COMPRESSION, &GrallocSnapHelper::CompressionHelper},
          {PLANE_LAYOUTS, &GrallocSnapHelper::PlaneLayoutsHelper},
          {ALIGNED_WIDTH_IN_PIXELS, &GrallocSnapHelper::AlignedWidthInPixelsHelper},
          {ALIGNED_HEIGHT_IN_PIXELS, &GrallocSnapHelper::AlignedHeightInPixelsHelper},
      };

  SnapError ColorMetadataHelper(SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set,
                                void *gralloc_out_get, SnapDescriptor *buf_des = nullptr,
                                bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError ColorspaceHelper(SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set,
                             void *gralloc_out_get, SnapDescriptor *buf_des = nullptr,
                             bool check_metadata_set = true, int32_t *mapper_return = nullptr);
  SnapError YuvPlaneInfoHelper(SnapHandle *hnd, uint32_t aidl_size, void *gralloc_in_set,
                               void *gralloc_out_get, SnapDescriptor *buf_des = nullptr,
                               bool check_metadata_set = true, int32_t *mapper_return = nullptr);
};

class GrallocSnapHelperLegacy : public GrallocSnapHelperIntf {
 public:
  static GrallocSnapHelperLegacy *GetInstance();
  int Allocate(gralloc::BufferDescriptor gr_desc, int buffer_count,
               aidl::android::hardware::graphics::allocator::AllocationResult *result);
  int Import(native_handle_t *gr_hnd);
  int Free(native_handle_t *gr_hnd);
  int Lock(native_handle_t *gr_hnd, uint64_t gr_usage, CropRectangle_t gr_access_region,
           int fence_fd, uint64_t *base_addr);
  int Unlock(native_handle_t *gr_hnd, void *fence);
  int ValidateBufferSize(native_handle_t *gr_hnd, gralloc::BufferInfo gr_desc);
  int FlushLockedBuffer(native_handle_t *gr_hnd);
  int RereadLockedBuffer(native_handle_t *gr_hnd);
  int IsSupported(gralloc::BufferDescriptor gr_desc, bool *is_supported);
  // aidl_size acts as an aidl_convert_bytestream flag encoding to std::vector<uint8_t> for standard
  // metadata. mapper_return is the required size for output buffer, if a smaller buffer is provided
  // GetMetadata needs to be called again with appropriate size to obtain the standard metadata.
  int GetMetadata(native_handle_t *gr_hnd, uint64_t gr_metadata_type, void *out,
                  bool convert_bytestream, bool check_metadata_set = true, uint32_t aidl_size = 0,
                  int32_t *mapper_return = nullptr);
  int GetMetadataState(native_handle_t *gr_hnd, SnapMetadataType gr_metadata_type, bool *out);
  int SetMetadata(native_handle_t *gr_hnd, uint64_t gr_metadata_type, hidl_vec<uint8_t> in);
  // aidl_size here is size of the aidl / std::vector<uint8_t> bytestream to decode for standard
  // metadata
  int SetMetadata(native_handle_t *gr_hnd, uint64_t gr_metadata_type, void *in, uint32_t aidl_size);
  int GetFromBufferDescriptor(gralloc::BufferDescriptor gr_desc, uint64_t gr_metadata_type,
                              void *out, bool convert_to_hidl_bytestream);
  int GetAllHandles(std::vector<buffer_handle_t> *out_handle_list);

  bool IsSnapAllocEnabled() { return snap_alloc_enable_; };

  bool IsBufferImported(native_handle_t *gr_hnd);
  int GetCustomDimensions(native_handle_t *gr_hnd, int *stride, int *height);
  int GetFormatLayout(gralloc::BufferInfo gr_desc, void *out, uint32_t *size, int interlaced);
  int GetReservedRegion(native_handle_t *gr_hnd, void **reserved_region,
                        uint64_t *reserved_region_size);

 private:
  GrallocSnapHelperLegacy();
  ~GrallocSnapHelperLegacy();
  SnapError GetSnapFormat(int hal_format, uint64_t usage, SnapFormatDescriptor *snap_fmt_desc);
  SnapUsage GetSnapUsage(uint64_t usage, int hal_format);
  int GetGrallocFormat(SnapFormatDescriptor snap_fmt_desc, SnapUsage usage, int *gr_format);
  uint64_t GetGrallocUsage(SnapUsage snap_usage);
  int GetGrallocPrivateFlags(SnapUsage snap_usage, int64_t is_ubwc, int64_t is_tile_rendered,
                             int64_t is_cached);
  SnapError ValidateGrallocUsage(uint64_t gralloc_usage);
  SnapError GetSnapDescriptor(gralloc::BufferDescriptor gr_desc, SnapDescriptor &snap_desc);
  SnapError GetSnapDescriptor(gralloc::BufferInfo gr_desc, SnapDescriptor &snap_desc);
  SnapMetadataType GetSnapMetadataType(uint64_t gr_metadata_type);

  int ConvertSnapDataspaceToGrallocDataspace(SnapDataspace &snap_dataspace,
                                             GrallocDataspace *gr_dataspace);
  int ConvertGrallocDataspaceToSnapDataspace(GrallocDataspace gr_dataspace,
                                             SnapDataspace *snap_dataspace);
  int ConvertSnapPlaneLayoutComponentToGralloc(SnapPlaneLayout *layout);
  void ConvertSnapToGrallocPlaneComponentType(SnapPlaneLayoutComponentType snap_component_type,
                                              GrallocExtendableType *gr_component_type);
  SnapUBWCVersion GetSnapUBWCVersion(UBWC_Version version);
  UBWC_Version GetGrallocUBWCVersion(SnapUBWCVersion version);
  int ConvertSnapBufferlayoutToGrallocPlaneLayout(
      SnapHandle *hnd, SnapDescriptor *buf_des, const SnapBufferLayout snap_buffer_layout,
      std::vector<GrallocPlaneLayout> *gr_plane_layouts);
  int ConvertGrallocPlaneLayoutToAndroidYCbCr(
      uint64_t base_addr, const std::vector<GrallocPlaneLayout> gr_plane_layouts,
      struct android_ycbcr *outYCbCr);
  int GetColorSpaceFromDataspaceMetadata(SnapDataspace snap_dataspace, uint32_t *color_space);
  int GetSnapDataspaceMetadataFromColorSpace(uint32_t color_space, SnapDataspace *snap_dataspace);
  SnapError CheckMetadataSet(SnapMetadataType type, SnapError status, bool check_metadata_set);
  std::shared_ptr<ISnapMapper> snapmapper_;
  std::shared_ptr<ISnapAlloc> snapallocator_;
  bool snap_alloc_enable_ = false;
  void *snap_impl_lib_ = nullptr;
  std::shared_ptr<ISnapAlloc> (*LINK_FETCH_ISnapAlloc)(DebugCallbackIntf *) = nullptr;
  std::shared_ptr<ISnapMapper> (*LINK_FETCH_ISnapMapper)(DebugCallbackIntf *) = nullptr;
  static GrallocSnapHelperLegacy *s_instance;
  GrallocSnapDebugger debugger_impl_{};
  bool enable_logs_ = false;

  std::unordered_map<SnapFormatDescriptor, uint64_t, SnapFormatDescriptorHash>
      snap_to_gralloc_format_ = {
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_YCbCr_420_SP},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_HEIF},
           HAL_PIXEL_FORMAT_NV12_HEIF},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_ENCODEABLE},
           HAL_PIXEL_FORMAT_NV12_ENCODEABLE},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_TILED},
           HAL_PIXEL_FORMAT_YCbCr_420_SP_TILED},
          {{.format = SnapPixelFormat::YCrCb_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::YCRCB_420_SP)},
          {{.format = SnapPixelFormat::YCrCb_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS},
          {{.format = SnapPixelFormat::YCrCb_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_ADRENO},
           HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO},
          {{.format = SnapPixelFormat::YCrCb_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_ENCODEABLE},
           HAL_PIXEL_FORMAT_NV21_ENCODEABLE},
          {{.format = SnapPixelFormat::YCBCR_P010, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_YCbCr_420_P010},
          {{.format = SnapPixelFormat::YCBCR_P010, .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS},
          {{.format = SnapPixelFormat::RGBA_8888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RGBA_8888},
          {{.format = SnapPixelFormat::BGRA_8888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_BGRA_8888},
          {{.format = SnapPixelFormat::RGBX_8888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RGBX_8888},
          {{.format = SnapPixelFormat::BGR_565, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_BGR_565},
          {{.format = SnapPixelFormat::RGBA_FP16, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::RGBA_FP16)},
          {{.format = SnapPixelFormat::RGB_888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RGB_888},
          {{.format = SnapPixelFormat::RGB_565, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::RGB_565)},
          {{.format = SnapPixelFormat::YV12, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::YV12)},
          {{.format = SnapPixelFormat::R_8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(aidl::android::hardware::graphics::common::PixelFormat::R_8)},
          // TODO: using FLEX to distinguish from aidl R_8 -- need to add separate modifier for this case
          {{.format = SnapPixelFormat::R_8, .modifier = PIXEL_FORMAT_MODIFIER_FLEX},
           HAL_PIXEL_FORMAT_R_8},
          {{.format = SnapPixelFormat::RG_88, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(HAL_PIXEL_FORMAT_RG_88)},
          {{.format = SnapPixelFormat::RGBA_1010102, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::RGBA_1010102)},
          {{.format = SnapPixelFormat::NV21_ZSL, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_NV21_ZSL},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::IMPLEMENTATION_DEFINED)},
          {{.format = SnapPixelFormat::YCBCR_420_888, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::YCBCR_420_888)},
          {{.format = SnapPixelFormat::RAW8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RAW8},
          {{.format = SnapPixelFormat::RAW10, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RAW10},
          {{.format = SnapPixelFormat::RAW12, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RAW12},
          {{.format = SnapPixelFormat::RAW14, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           SnapPixelFormat::RAW14},
          {{.format = SnapPixelFormat::RAW16, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RAW16},
          {{.format = SnapPixelFormat::DEPTH_16, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::DEPTH_16)},
          {{.format = SnapPixelFormat::DEPTH_24, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::DEPTH_24)},
          {{.format = SnapPixelFormat::DEPTH_24_STENCIL_8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::DEPTH_24_STENCIL_8)},
          {{.format = SnapPixelFormat::DEPTH_32F, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::DEPTH_32F)},
          {{.format = SnapPixelFormat::DEPTH_32F_STENCIL_8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::DEPTH_32F_STENCIL_8)},
          {{.format = SnapPixelFormat::STENCIL_8, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::STENCIL_8)},
          {{.format = SnapPixelFormat::BLOB, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::BLOB)},
          {{.format = SnapPixelFormat::YCBCR_422_SP, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::YCBCR_422_SP)},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_LINEAR_FLEX},
           HAL_PIXEL_FORMAT_NV12_LINEAR_FLEX},
          {{.format = SnapPixelFormat::RAW_OPAQUE, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RAW_OPAQUE},
          {{.format = SnapPixelFormat::YCBCR_422_I, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_YCBCR_422_I},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::IMPLEMENTATION_DEFINED)},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           static_cast<int>(PixelFormat::IMPLEMENTATION_DEFINED)},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_ENCODEABLE},
           static_cast<int>(PixelFormat::IMPLEMENTATION_DEFINED)},
          {{.format = SnapPixelFormat::IMPLEMENTATION_DEFINED,
            .modifier = PIXEL_FORMAT_MODIFIER_HEIF},
           static_cast<int>(PixelFormat::IMPLEMENTATION_DEFINED)},
          {{.format = SnapPixelFormat::RGBA_5551, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RGBA_5551},
          {{.format = SnapPixelFormat::RGBA_4444, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_RGBA_4444},
          {{.format = SnapPixelFormat::Y16, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           static_cast<int>(PixelFormat::Y16)},
          // Remove entries with PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC when support for
          // explicit UBWC formats get deprecated with SNAP_SUPPORTS_EXPLICIT_UBWC_FORMATS
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC},
           HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC},
          {{.format = SnapPixelFormat::YCBCR_P010, .modifier = PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC},
           HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC},
      };

  std::unordered_map<uint64_t, SnapFormatDescriptor> gralloc_to_snap_format_;

  std::unordered_map<SnapFormatDescriptor, uint64_t, SnapFormatDescriptorHash>
      snap_to_gralloc_ubwc_format_ = {
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_VENUS},
           HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_4R},
           HAL_PIXEL_FORMAT_YCbCr_420_SP_4R_UBWC},
          {{.format = SnapPixelFormat::TP10, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC},
          {{.format = SnapPixelFormat::YCBCR_P010, .modifier = PIXEL_FORMAT_MODIFIER_NONE},
           HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC},
          {{.format = SnapPixelFormat::YCbCr_420_SP, .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX},
           HAL_PIXEL_FORMAT_NV12_UBWC_FLEX},
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX_2_BATCH},
           HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH},
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX_4_BATCH},
           HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH},
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_UBWC_FLEX_8_BATCH},
           HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH},
          // Remove entries with PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC when support for
          // explicit UBWC formats get deprecated with SNAP_SUPPORTS_EXPLICIT_UBWC_FORMATS
          {{.format = SnapPixelFormat::YCbCr_420_SP,
            .modifier = PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC},
           HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC},
          {{.format = SnapPixelFormat::YCBCR_P010, .modifier = PIXEL_FORMAT_MODIFIER_EXPLICIT_UBWC},
           HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC},
      };

  std::unordered_map<uint64_t, SnapFormatDescriptor> gralloc_ubwc_to_snap_format_;

  std::unordered_map<SnapPlaneLayoutComponentType, int> snap_to_gralloc_plane_layout_component_ = {
      {PLANE_LAYOUT_COMPONENT_TYPE_Y, PLANE_COMPONENT_Y},
      {PLANE_LAYOUT_COMPONENT_TYPE_CB, PLANE_COMPONENT_Cb},
      {PLANE_LAYOUT_COMPONENT_TYPE_CR, PLANE_COMPONENT_Cr},
      {PLANE_LAYOUT_COMPONENT_TYPE_R, PLANE_COMPONENT_R},
      {PLANE_LAYOUT_COMPONENT_TYPE_G, PLANE_COMPONENT_G},
      {PLANE_LAYOUT_COMPONENT_TYPE_B, PLANE_COMPONENT_B},
      {PLANE_LAYOUT_COMPONENT_TYPE_A, PLANE_COMPONENT_A},
      {PLANE_LAYOUT_COMPONENT_TYPE_RAW, PLANE_COMPONENT_RAW},
      {PLANE_LAYOUT_COMPONENT_TYPE_META, PLANE_COMPONENT_META},
  };

  std::unordered_map<SnapPlaneLayoutComponentType, GrallocExtendableType>
      snap_to_gralloc_extendable_plane_layout_component_type_ = {
          {PLANE_LAYOUT_COMPONENT_TYPE_Y, android::gralloc4::PlaneLayoutComponentType_Y},
          {PLANE_LAYOUT_COMPONENT_TYPE_CB, android::gralloc4::PlaneLayoutComponentType_CB},
          {PLANE_LAYOUT_COMPONENT_TYPE_CR, android::gralloc4::PlaneLayoutComponentType_CR},
          {PLANE_LAYOUT_COMPONENT_TYPE_R, android::gralloc4::PlaneLayoutComponentType_R},
          {PLANE_LAYOUT_COMPONENT_TYPE_G, android::gralloc4::PlaneLayoutComponentType_G},
          {PLANE_LAYOUT_COMPONENT_TYPE_B, android::gralloc4::PlaneLayoutComponentType_B},
          {PLANE_LAYOUT_COMPONENT_TYPE_A, android::gralloc4::PlaneLayoutComponentType_A},
          {PLANE_LAYOUT_COMPONENT_TYPE_RAW, android::gralloc4::PlaneLayoutComponentType_RAW},
          {PLANE_LAYOUT_COMPONENT_TYPE_META, qtigralloc::PlaneLayoutComponentType_Meta},
      };

  std::unordered_map<uint64_t, SnapUsage> gralloc_to_snap_usage_ = {
      {(uint64_t)GrallocBufferUsage::GPU_TEXTURE, SnapUsage::GPU_TEXTURE},
      {(uint64_t)GrallocBufferUsage::GPU_RENDER_TARGET, SnapUsage::GPU_RENDER_TARGET},
      {(uint64_t)GrallocBufferUsage::COMPOSER_OVERLAY, SnapUsage::COMPOSER_OVERLAY},
      {(uint64_t)GrallocBufferUsage::COMPOSER_CLIENT_TARGET, SnapUsage::COMPOSER_CLIENT_TARGET},
      {(uint64_t)GrallocBufferUsage::PROTECTED, SnapUsage::PROTECTED},
      {(uint64_t)GrallocBufferUsage::COMPOSER_CURSOR, SnapUsage::COMPOSER_CURSOR},
      {(uint64_t)GrallocBufferUsage::VIDEO_ENCODER, SnapUsage::VIDEO_ENCODER},
      {(uint64_t)GrallocBufferUsage::CAMERA_OUTPUT, SnapUsage::CAMERA_OUTPUT},
      {(uint64_t)GrallocBufferUsage::CAMERA_INPUT, SnapUsage::CAMERA_INPUT},
      {(uint64_t)GrallocBufferUsage::RENDERSCRIPT, SnapUsage::RENDERSCRIPT},
      {(uint64_t)GrallocBufferUsage::VIDEO_DECODER, SnapUsage::VIDEO_DECODER},
      {(uint64_t)GrallocBufferUsage::SENSOR_DIRECT_DATA, SnapUsage::SENSOR_DIRECT_DATA},
      {(uint64_t)GrallocBufferUsage::GPU_DATA_BUFFER, SnapUsage::GPU_DATA_BUFFER},
      {(uint64_t)GrallocBufferUsage::GPU_CUBE_MAP, SnapUsage::GPU_CUBE_MAP},
      {(uint64_t)GrallocBufferUsage::GPU_MIPMAP_COMPLETE, SnapUsage::GPU_MIPMAP_COMPLETE},
      {(uint64_t)GrallocBufferUsage::HW_IMAGE_ENCODER, SnapUsage::HW_IMAGE_ENCODER},
      {(uint64_t)GrallocBufferUsage::FRONT_BUFFER, SnapUsage::FRONT_BUFFER},
      {GRALLOC_USAGE_PRIVATE_VIDEO_HW, SnapUsage::QTI_PRIVATE_VIDEO_HW},
      {GRALLOC_USAGE_PRIVATE_ALLOC_UBWC, SnapUsage::QTI_ALLOC_UBWC},
      {GRALLOC_USAGE_PRIVATE_ALLOC_UBWC_PI, SnapUsage::QTI_PRIVATE_ALLOC_UBWC_PI},
      {GRALLOC_USAGE_PRIVATE_UNCACHED, SnapUsage::QTI_PRIVATE_UNCACHED},
      {GRALLOC_USAGE_PRIVATE_10BIT, SnapUsage::QTI_PRIVATE_10BIT},
      {GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY, SnapUsage::QTI_PRIVATE_SECURE_DISPLAY},
      {GRALLOC_USAGE_PRIVATE_VIDEO_NV21_ENCODER, SnapUsage::QTI_PRIVATE_VIDEO_NV21_ENCODER},
      {GRALLOC_USAGE_PRIVATE_CDSP, SnapUsage::QTI_PRIVATE_CDSP},
      {GRALLOC_USAGE_PRIVATE_WFD, SnapUsage::QTI_PRIVATE_WFD},
      {GRALLOC_USAGE_PRIVATE_VIDEO_HW, SnapUsage::QTI_PRIVATE_VIDEO_HW},
      {GRALLOC_USAGE_PRIVATE_TRUSTED_VM, SnapUsage::QTI_PRIVATE_TRUSTED_VM},
      {GRALLOC_USAGE_PRIVATE_ALLOC_UBWC_4R, SnapUsage::QTI_ALLOC_UBWC_4R},
      {GRALLOC_USAGE_PRIVATE_UBWC_L_8_TO_5, SnapUsage::QTI_ALLOC_UBWC_L_8_TO_5},
      {GRALLOC_USAGE_PRIVATE_UBWC_L_2_TO_1, SnapUsage::QTI_ALLOC_UBWC_L_2_TO_1},
  };

  std::unordered_map<SnapUsage, uint64_t> snap_to_gralloc_usage_;

  std::unordered_map<uint64_t, SnapUsage> cpu_gralloc_to_snap_usage_ = {
      {(uint64_t)GrallocBufferUsage::CPU_READ_RARELY, SnapUsage::CPU_READ_RARELY},
      {(uint64_t)GrallocBufferUsage::CPU_READ_OFTEN, SnapUsage::CPU_READ_OFTEN},
      {(uint64_t)GrallocBufferUsage::CPU_WRITE_RARELY, SnapUsage::CPU_WRITE_RARELY},
      {(uint64_t)GrallocBufferUsage::CPU_WRITE_OFTEN, SnapUsage::CPU_WRITE_OFTEN},
  };

  std::unordered_map<SnapUsage, uint64_t> cpu_snap_to_gralloc_usage_;

  std::unordered_map<UBWC_Version, SnapUBWCVersion> gralloc_to_snap_ubwc_version_{
      {UBWC_Version::UBWC_1_0, SnapUBWCVersion::UBWC_VERSION_1_0},
      {UBWC_Version::UBWC_2_0, SnapUBWCVersion::UBWC_VERSION_2_0},
      {UBWC_Version::UBWC_3_0, SnapUBWCVersion::UBWC_VERSION_3_0},
      {UBWC_Version::UBWC_4_0, SnapUBWCVersion::UBWC_VERSION_4_0},
      {UBWC_Version::UBWC_UNUSED, SnapUBWCVersion::UBWC_VERSION_UNUSED},
      {UBWC_Version::UBWC_MAX_VERSION, SnapUBWCVersion::UBWC_VERSION_MAX},
  };

  std::unordered_map<SnapUBWCVersion, UBWC_Version> snap_to_gralloc_ubwc_version_;

  std::unordered_map<uint64_t, SnapMetadataType> metadata_type_map = {
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::BUFFER_ID),
       SnapMetadataType::BUFFER_ID},
      {static_cast<uint64_t>(aidl::android::hardware::graphics::common::StandardMetadataType::NAME),
       SnapMetadataType::NAME},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::WIDTH),
       SnapMetadataType::WIDTH},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::HEIGHT),
       SnapMetadataType::HEIGHT},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::LAYER_COUNT),
       SnapMetadataType::LAYER_COUNT},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::PIXEL_FORMAT_REQUESTED),
       SnapMetadataType::PIXEL_FORMAT_REQUESTED},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::PIXEL_FORMAT_FOURCC),
       SnapMetadataType::PIXEL_FORMAT_FOURCC},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::PIXEL_FORMAT_MODIFIER),
       SnapMetadataType::DRM_PIXEL_FORMAT_MODIFIER},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::USAGE),
       SnapMetadataType::USAGE},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::PLANE_LAYOUTS),
       SnapMetadataType::PLANE_LAYOUTS},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::PROTECTED_CONTENT),
       SnapMetadataType::PROTECTED_CONTENT},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::ALLOCATION_SIZE),
       SnapMetadataType::ALLOCATION_SIZE},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::BLEND_MODE),
       SnapMetadataType::BLEND_MODE},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::DATASPACE),
       SnapMetadataType::DATASPACE},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::SMPTE2086),
       SnapMetadataType::MASTERING_DISPLAY},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::CTA861_3),
       SnapMetadataType::CONTENT_LIGHT_LEVEL},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::SMPTE2094_40),
       SnapMetadataType::DYNAMIC_METADATA},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::SMPTE2094_10),
       SnapMetadataType::SMPTE2094_10},
      {static_cast<uint64_t>(aidl::android::hardware::graphics::common::StandardMetadataType::CROP),
       SnapMetadataType::CROP},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::CHROMA_SITING),
       SnapMetadataType::CHROMA_SITING},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::INTERLACED),
       SnapMetadataType::INTERLACED},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::COMPRESSION),
       SnapMetadataType::COMPRESSION},
      {static_cast<uint64_t>(
           aidl::android::hardware::graphics::common::StandardMetadataType::STRIDE),
       SnapMetadataType::STRIDE},
      {QTI_GRAPHICS_METADATA, SnapMetadataType::GRAPHICS_METADATA},
      {QTI_STANDARD_METADATA_STATUS, SnapMetadataType::STANDARD_METADATA_STATUS},
      {QTI_VENDOR_METADATA_STATUS, SnapMetadataType::VENDOR_METADATA_STATUS},
      {QTI_CUSTOM_DIMENSIONS_STRIDE, SnapMetadataType::CUSTOM_DIMENSIONS_STRIDE},
      {QTI_CUSTOM_DIMENSIONS_HEIGHT, SnapMetadataType::CUSTOM_DIMENSIONS_HEIGHT},
      {QTI_RGB_DATA_ADDRESS, SnapMetadataType::RGB_DATA_ADDRESS},
      {QTI_ALIGNED_WIDTH_IN_PIXELS, SnapMetadataType::ALIGNED_WIDTH_IN_PIXELS},
      {QTI_ALIGNED_HEIGHT_IN_PIXELS, SnapMetadataType::ALIGNED_HEIGHT_IN_PIXELS},
      {QTI_BUFFER_TYPE, SnapMetadataType::BUFFER_TYPE},
      {QTI_MEM_HANDLE, SnapMetadataType::MEM_HANDLE},
      {QTI_FD, SnapMetadataType::FD},
      {QTI_PP_PARAM_INTERLACED, SnapMetadataType::PP_PARAM_INTERLACED},
      {QTI_REFRESH_RATE, SnapMetadataType::REFRESH_RATE},
      {QTI_MAP_SECURE_BUFFER, SnapMetadataType::MAP_SECURE_BUFFER},
      {QTI_LINEAR_FORMAT, SnapMetadataType::LINEAR_FORMAT},
      {QTI_SINGLE_BUFFER_MODE, SnapMetadataType::SINGLE_BUFFER_MODE},
      {QTI_VT_TIMESTAMP, SnapMetadataType::VT_TIMESTAMP},
      {QTI_UBWC_CR_STATS_INFO, SnapMetadataType::UBWC_CR_STATS_INFO},
      {QTI_VIDEO_PERF_MODE, SnapMetadataType::VIDEO_PERF_MODE},
      {QTI_CVP_METADATA, SnapMetadataType::CVP_METADATA},
      {QTI_VIDEO_HISTOGRAM_STATS, SnapMetadataType::VIDEO_HISTOGRAM_STATS},
      {QTI_VIDEO_TRANSCODE_STATS, SnapMetadataType::VIDEO_TRANSCODE_STATS},
      {QTI_VIDEO_TS_INFO, SnapMetadataType::VIDEO_TS_INFO},
      {QTI_TIMED_RENDERING, SnapMetadataType::TIMED_RENDERING},
      {QTI_BUFFER_PERMISSION, SnapMetadataType::BUFFER_PERMISSION},
      {QTI_CUSTOM_CONTENT_METADATA, SnapMetadataType::CUSTOM_CONTENT_METADATA},
      {QTI_HEAP_NAME, SnapMetadataType::HEAP_NAME},
      {QTI_EARLYNOTIFY_LINECOUNT, SnapMetadataType::EARLYNOTIFY_LINECOUNT},
      {QTI_BUFFER_DEQUEUE_DURATION, SnapMetadataType::BUFFER_DEQUEUE_DURATION},
      // New enum entries
      {SnapMetadataType::HEAP_NAME, SnapMetadataType::HEAP_NAME},
      {SnapMetadataType::IS_UBWC, SnapMetadataType::IS_UBWC},
      {SnapMetadataType::IS_TILE_RENDERED, SnapMetadataType::IS_TILE_RENDERED},
      {SnapMetadataType::IS_CACHED, SnapMetadataType::IS_CACHED},
      {SnapMetadataType::MASTERING_DISPLAY, SnapMetadataType::MASTERING_DISPLAY},
      {SnapMetadataType::CONTENT_LIGHT_LEVEL, SnapMetadataType::CONTENT_LIGHT_LEVEL},
      {SnapMetadataType::DYNAMIC_METADATA, SnapMetadataType::DYNAMIC_METADATA},
      {SnapMetadataType::MATRIX_COEFFICIENTS, SnapMetadataType::MATRIX_COEFFICIENTS},
      {SnapMetadataType::COLOR_REMAPPING_INFO, SnapMetadataType::COLOR_REMAPPING_INFO},
      {SnapMetadataType::BASE_ADDRESS, SnapMetadataType::BASE_ADDRESS},
      {SnapMetadataType::PIXEL_FORMAT_ALLOCATED, SnapMetadataType::PIXEL_FORMAT_ALLOCATED},
      {SnapMetadataType::ANAMORPHIC_COMPRESSION_METADATA,
       SnapMetadataType::ANAMORPHIC_COMPRESSION_METADATA},
  };

  std::list<int> unsupported_formats = {
      static_cast<int>(aidl::android::hardware::graphics::common::PixelFormat::R_16_UINT),
      static_cast<int>(aidl::android::hardware::graphics::common::PixelFormat::RG_1616_UINT),
      static_cast<int>(aidl::android::hardware::graphics::common::PixelFormat::RGBA_10101010),
  };

  typedef SnapError (GrallocSnapHelperLegacy::*MetadataHelper)(
      SnapHandle *, bool hidl_bytestream, uint32_t aidl_size, void *gralloc_in_set,
      void *gralloc_out_get, SnapDescriptor *buf_des, bool check_metadata_set,
      int32_t *mapper_return);

  SnapError BufferIDHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                           void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                           SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                           int32_t *mapper_return = nullptr);
  SnapError UsageHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                        void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                        SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                        int32_t *mapper_return = nullptr);
  SnapError DataspaceHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                            void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                            SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                            int32_t *mapper_return = nullptr);
  SnapError NameHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                       void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                       SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                       int32_t *mapper_return = nullptr);
  SnapError WidthHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                        void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                        SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                        int32_t *mapper_return = nullptr);
  SnapError HeightHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                         void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                         SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                         int32_t *mapper_return = nullptr);
  SnapError LayerCountHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                             void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                             SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                             int32_t *mapper_return = nullptr);
  SnapError PixelFormatRequestedHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                       void *gralloc_in_set = nullptr,
                                       void *gralloc_out_get = nullptr,
                                       SnapDescriptor *buf_des = nullptr,
                                       bool check_metadata_set = true,
                                       int32_t *mapper_return = nullptr);
  SnapError PixelFormatFourCCHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                    void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                    SnapDescriptor *buf_des = nullptr,
                                    bool check_metadata_set = true,
                                    int32_t *mapper_return = nullptr);
  SnapError DRMPixelFormatModifierHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                         void *gralloc_in_set = nullptr,
                                         void *gralloc_out_get = nullptr,
                                         SnapDescriptor *buf_des = nullptr,
                                         bool check_metadata_set = true,
                                         int32_t *mapper_return = nullptr);
  SnapError AllocationSizeHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                 void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                 SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                                 int32_t *mapper_return = nullptr);
  SnapError ProtectedContentHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                   void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                   SnapDescriptor *buf_des = nullptr,
                                   bool check_metadata_set = true,
                                   int32_t *mapper_return = nullptr);
  SnapError CompressionHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                              void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                              SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                              int32_t *mapper_return = nullptr);
  SnapError InterlacedHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                             void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                             SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                             int32_t *mapper_return = nullptr);
  SnapError ChromaSitingHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                               void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                               SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                               int32_t *mapper_return = nullptr);
  SnapError PlaneLayoutsHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                               void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                               SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                               int32_t *mapper_return = nullptr);
  SnapError CropHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                       void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                       SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                       int32_t *mapper_return = nullptr);
  SnapError BlendModeHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                            void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                            SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                            int32_t *mapper_return = nullptr);
  SnapError VTTimestampHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                              void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                              SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                              int32_t *mapper_return = nullptr);
  SnapError PPParamInterlacedHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                    void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                    SnapDescriptor *buf_des = nullptr,
                                    bool check_metadata_set = true,
                                    int32_t *mapper_return = nullptr);
  SnapError VideoPerfModeHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                                int32_t *mapper_return = nullptr);
  SnapError GraphicsMetadataHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                   void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                   SnapDescriptor *buf_des = nullptr,
                                   bool check_metadata_set = true,
                                   int32_t *mapper_return = nullptr);
  SnapError UBWCCRStatsInfoHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                  void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                  SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                                  int32_t *mapper_return = nullptr);
  SnapError RefreshRateHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                              void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                              SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                              int32_t *mapper_return = nullptr);
  SnapError MapSecureBufferHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                  void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                  SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                                  int32_t *mapper_return = nullptr);
  SnapError LinearFormatHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                               void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                               SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                               int32_t *mapper_return = nullptr);
  SnapError SingleBufferModeHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                   void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                   SnapDescriptor *buf_des = nullptr,
                                   bool check_metadata_set = true,
                                   int32_t *mapper_return = nullptr);
  SnapError CVPMetadataHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                              void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                              SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                              int32_t *mapper_return = nullptr);
  SnapError VideoHistogramStatsHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                      void *gralloc_in_set = nullptr,
                                      void *gralloc_out_get = nullptr,
                                      SnapDescriptor *buf_des = nullptr,
                                      bool check_metadata_set = true,
                                      int32_t *mapper_return = nullptr);
  SnapError FDHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                     void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                     SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                     int32_t *mapper_return = nullptr);
  SnapError AlignedWidthInPixelsHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                       void *gralloc_in_set = nullptr,
                                       void *gralloc_out_get = nullptr,
                                       SnapDescriptor *buf_des = nullptr,
                                       bool check_metadata_set = true,
                                       int32_t *mapper_return = nullptr);
  SnapError AlignedHeightInPixelsHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                        void *gralloc_in_set = nullptr,
                                        void *gralloc_out_get = nullptr,
                                        SnapDescriptor *buf_des = nullptr,
                                        bool check_metadata_set = true,
                                        int32_t *mapper_return = nullptr);
  SnapError StandardMetadataStatusHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                         void *gralloc_in_set = nullptr,
                                         void *gralloc_out_get = nullptr,
                                         SnapDescriptor *buf_des = nullptr,
                                         bool check_metadata_set = true,
                                         int32_t *mapper_return = nullptr);
  SnapError VendorMetadataStatusHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                       void *gralloc_in_set = nullptr,
                                       void *gralloc_out_get = nullptr,
                                       SnapDescriptor *buf_des = nullptr,
                                       bool check_metadata_set = true,
                                       int32_t *mapper_return = nullptr);
  SnapError BufferTypeHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                             void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                             SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                             int32_t *mapper_return = nullptr);
  SnapError VideoTSInfoHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                              void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                              SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                              int32_t *mapper_return = nullptr);
  SnapError CustomDimensionsStrideHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                         void *gralloc_in_set = nullptr,
                                         void *gralloc_out_get = nullptr,
                                         SnapDescriptor *buf_des = nullptr,
                                         bool check_metadata_set = true,
                                         int32_t *mapper_return = nullptr);
  SnapError CustomDimensionsHeightHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                         void *gralloc_in_set = nullptr,
                                         void *gralloc_out_get = nullptr,
                                         SnapDescriptor *buf_des = nullptr,
                                         bool check_metadata_set = true,
                                         int32_t *mapper_return = nullptr);
  SnapError RGBDataAddressHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                 void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                 SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                                 int32_t *mapper_return = nullptr);
  SnapError BufferPermissionHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                   void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                   SnapDescriptor *buf_des = nullptr,
                                   bool check_metadata_set = true,
                                   int32_t *mapper_return = nullptr);
  SnapError MemHandleHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                            void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                            SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                            int32_t *mapper_return = nullptr);
  SnapError TimedRenderingHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                 void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                 SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                                 int32_t *mapper_return = nullptr);
  SnapError CustomContentMetadataHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                        void *gralloc_in_set = nullptr,
                                        void *gralloc_out_get = nullptr,
                                        SnapDescriptor *buf_des = nullptr,
                                        bool check_metadata_set = true,
                                        int32_t *mapper_return = nullptr);
  SnapError SMPTE2094_10Helper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                               void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                               SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                               int32_t *mapper_return = nullptr);
  SnapError VideoTranscodeStatsHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                      void *gralloc_in_set = nullptr,
                                      void *gralloc_out_get = nullptr,
                                      SnapDescriptor *buf_des = nullptr,
                                      bool check_metadata_set = true,
                                      int32_t *mapper_return = nullptr);
  SnapError MasteringDisplayHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                   void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                   SnapDescriptor *buf_des = nullptr,
                                   bool check_metadata_set = true,
                                   int32_t *mapper_return = nullptr);
  SnapError ContentLightLevelHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                    void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                    SnapDescriptor *buf_des = nullptr,
                                    bool check_metadata_set = true,
                                    int32_t *mapper_return = nullptr);
  SnapError DynamicMetadataHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                  void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                  SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                                  int32_t *mapper_return = nullptr);
  SnapError MatrixCoefficientsHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                     void *gralloc_in_set = nullptr,
                                     void *gralloc_out_get = nullptr,
                                     SnapDescriptor *buf_des = nullptr,
                                     bool check_metadata_set = true,
                                     int32_t *mapper_return = nullptr);
  SnapError ColorRemappingInfoHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                     void *gralloc_in_set = nullptr,
                                     void *gralloc_out_get = nullptr,
                                     SnapDescriptor *buf_des = nullptr,
                                     bool check_metadata_set = true,
                                     int32_t *mapper_return = nullptr);
  SnapError HeapNameHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                           void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                           SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                           int32_t *mapper_return = nullptr);
  SnapError IsUBWCHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                         void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                         SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                         int32_t *mapper_return = nullptr);
  SnapError IsTileRenderedHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                 void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                                 SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                                 int32_t *mapper_return = nullptr);
  SnapError IsCachedHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                           void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                           SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                           int32_t *mapper_return = nullptr);
  SnapError PixelFormatAllocatedHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                       void *gralloc_in_set = nullptr,
                                       void *gralloc_out_get = nullptr,
                                       SnapDescriptor *buf_des = nullptr,
                                       bool check_metadata_set = true,
                                       int32_t *mapper_return = nullptr);
  SnapError BaseAddressHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                              void *gralloc_in_set = nullptr, void *gralloc_out_get = nullptr,
                              SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                              int32_t *mapper_return = nullptr);
  SnapError EarlyNotifyLineCountHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                       void *gralloc_in_set = nullptr,
                                       void *gralloc_out_get = nullptr,
                                       SnapDescriptor *buf_des = nullptr,
                                       bool check_metadata_set = true,
                                       int32_t *mapper_return = nullptr);
  SnapError BufferDequeueDurationHelper(SnapHandle *, bool hidl_bytestream, uint32_t aidl_size,
                                        void *gralloc_in_set = nullptr,
                                        void *gralloc_out_get = nullptr,
                                        SnapDescriptor *buf_des = nullptr,
                                        bool check_metadata_set = true,
                                        int32_t *mapper_return = nullptr);
  SnapError CompressionMetadataHelper(SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size,
                                      void *gralloc_in_set = nullptr,
                                      void *gralloc_out_get = nullptr,
                                      SnapDescriptor *buf_des = nullptr,
                                      bool check_metadata_set = true,
                                      int32_t *mapper_return = nullptr);

  std::unordered_map<vendor_qti_hardware_display_common_MetadataType, MetadataHelper>
      metadata_conversion_helper_function_map = {
          {BUFFER_ID, &GrallocSnapHelperLegacy::BufferIDHelper},
          {NAME, &GrallocSnapHelperLegacy::NameHelper},
          {WIDTH, &GrallocSnapHelperLegacy::WidthHelper},
          {HEIGHT, &GrallocSnapHelperLegacy::HeightHelper},
          {LAYER_COUNT, &GrallocSnapHelperLegacy::LayerCountHelper},
          {PIXEL_FORMAT_REQUESTED, &GrallocSnapHelperLegacy::PixelFormatRequestedHelper},
          {PIXEL_FORMAT_FOURCC, &GrallocSnapHelperLegacy::PixelFormatFourCCHelper},
          {DRM_PIXEL_FORMAT_MODIFIER, &GrallocSnapHelperLegacy::DRMPixelFormatModifierHelper},
          {USAGE, &GrallocSnapHelperLegacy::UsageHelper},
          {ALLOCATION_SIZE, &GrallocSnapHelperLegacy::AllocationSizeHelper},
          {PROTECTED_CONTENT, &GrallocSnapHelperLegacy::ProtectedContentHelper},
          {COMPRESSION, &GrallocSnapHelperLegacy::CompressionHelper},
          {INTERLACED, &GrallocSnapHelperLegacy::InterlacedHelper},
          {CHROMA_SITING, &GrallocSnapHelperLegacy::ChromaSitingHelper},
          {PLANE_LAYOUTS, &GrallocSnapHelperLegacy::PlaneLayoutsHelper},
          {CROP, &GrallocSnapHelperLegacy::CropHelper},
          {DATASPACE, &GrallocSnapHelperLegacy::DataspaceHelper},
          {BLEND_MODE, &GrallocSnapHelperLegacy::BlendModeHelper},
          {VT_TIMESTAMP, &GrallocSnapHelperLegacy::VTTimestampHelper},
          {PP_PARAM_INTERLACED, &GrallocSnapHelperLegacy::PPParamInterlacedHelper},
          {VIDEO_PERF_MODE, &GrallocSnapHelperLegacy::VideoPerfModeHelper},
          {GRAPHICS_METADATA, &GrallocSnapHelperLegacy::GraphicsMetadataHelper},
          {UBWC_CR_STATS_INFO, &GrallocSnapHelperLegacy::UBWCCRStatsInfoHelper},
          {REFRESH_RATE, &GrallocSnapHelperLegacy::RefreshRateHelper},
          {MAP_SECURE_BUFFER, &GrallocSnapHelperLegacy::MapSecureBufferHelper},
          {LINEAR_FORMAT, &GrallocSnapHelperLegacy::LinearFormatHelper},
          {SINGLE_BUFFER_MODE, &GrallocSnapHelperLegacy::SingleBufferModeHelper},
          {CVP_METADATA, &GrallocSnapHelperLegacy::CVPMetadataHelper},
          {VIDEO_HISTOGRAM_STATS, &GrallocSnapHelperLegacy::VideoHistogramStatsHelper},
          {FD, &GrallocSnapHelperLegacy::FDHelper},
          {ALIGNED_WIDTH_IN_PIXELS, &GrallocSnapHelperLegacy::AlignedWidthInPixelsHelper},
          {STRIDE, &GrallocSnapHelperLegacy::AlignedWidthInPixelsHelper},
          {ALIGNED_HEIGHT_IN_PIXELS, &GrallocSnapHelperLegacy::AlignedHeightInPixelsHelper},
          {STANDARD_METADATA_STATUS, &GrallocSnapHelperLegacy::StandardMetadataStatusHelper},
          {VENDOR_METADATA_STATUS, &GrallocSnapHelperLegacy::VendorMetadataStatusHelper},
          {BUFFER_TYPE, &GrallocSnapHelperLegacy::BufferTypeHelper},
          {VIDEO_TS_INFO, &GrallocSnapHelperLegacy::VideoTSInfoHelper},
          {CUSTOM_DIMENSIONS_STRIDE, &GrallocSnapHelperLegacy::CustomDimensionsStrideHelper},
          {CUSTOM_DIMENSIONS_HEIGHT, &GrallocSnapHelperLegacy::CustomDimensionsHeightHelper},
          {RGB_DATA_ADDRESS, &GrallocSnapHelperLegacy::RGBDataAddressHelper},
          {BUFFER_PERMISSION, &GrallocSnapHelperLegacy::BufferPermissionHelper},
          {MEM_HANDLE, &GrallocSnapHelperLegacy::MemHandleHelper},
          {TIMED_RENDERING, &GrallocSnapHelperLegacy::TimedRenderingHelper},
          {CUSTOM_CONTENT_METADATA, &GrallocSnapHelperLegacy::CustomContentMetadataHelper},
          {VIDEO_TRANSCODE_STATS, &GrallocSnapHelperLegacy::VideoTranscodeStatsHelper},
          {MASTERING_DISPLAY, &GrallocSnapHelperLegacy::MasteringDisplayHelper},
          {static_cast<vendor_qti_hardware_display_common_MetadataType>(
               StandardMetadataType::SMPTE2086),
           &GrallocSnapHelperLegacy::MasteringDisplayHelper},
          {CONTENT_LIGHT_LEVEL, &GrallocSnapHelperLegacy::ContentLightLevelHelper},
          {static_cast<vendor_qti_hardware_display_common_MetadataType>(
               StandardMetadataType::CTA861_3),
           &GrallocSnapHelperLegacy::ContentLightLevelHelper},
          {DYNAMIC_METADATA, &GrallocSnapHelperLegacy::DynamicMetadataHelper},
          {static_cast<vendor_qti_hardware_display_common_MetadataType>(
               StandardMetadataType::SMPTE2094_40),
           &GrallocSnapHelperLegacy::DynamicMetadataHelper},
          {static_cast<vendor_qti_hardware_display_common_MetadataType>(
               StandardMetadataType::SMPTE2094_10),
           &GrallocSnapHelperLegacy::SMPTE2094_10Helper},
          {COLOR_REMAPPING_INFO, &GrallocSnapHelperLegacy::ColorRemappingInfoHelper},
          {HEAP_NAME, &GrallocSnapHelperLegacy::HeapNameHelper},
          {IS_UBWC, &GrallocSnapHelperLegacy::IsUBWCHelper},
          {IS_TILE_RENDERED, &GrallocSnapHelperLegacy::IsTileRenderedHelper},
          {IS_CACHED, &GrallocSnapHelperLegacy::IsCachedHelper},
          {PIXEL_FORMAT_ALLOCATED, &GrallocSnapHelperLegacy::PixelFormatAllocatedHelper},
          {BASE_ADDRESS, &GrallocSnapHelperLegacy::BaseAddressHelper},
          {MATRIX_COEFFICIENTS, &GrallocSnapHelperLegacy::MatrixCoefficientsHelper},
          {EARLYNOTIFY_LINECOUNT, &GrallocSnapHelperLegacy::EarlyNotifyLineCountHelper},
          {BUFFER_DEQUEUE_DURATION, &GrallocSnapHelperLegacy::BufferDequeueDurationHelper},
          {ANAMORPHIC_COMPRESSION_METADATA, &GrallocSnapHelperLegacy::CompressionMetadataHelper},
      };

  std::unordered_map<vendor_qti_hardware_display_common_MetadataType, MetadataHelper>
      bufferdescription_conversion_helper_function_map = {
          {NAME, &GrallocSnapHelperLegacy::NameHelper},
          {WIDTH, &GrallocSnapHelperLegacy::WidthHelper},
          {HEIGHT, &GrallocSnapHelperLegacy::HeightHelper},
          {LAYER_COUNT, &GrallocSnapHelperLegacy::LayerCountHelper},
          {PIXEL_FORMAT_REQUESTED, &GrallocSnapHelperLegacy::PixelFormatRequestedHelper},
          {PIXEL_FORMAT_FOURCC, &GrallocSnapHelperLegacy::PixelFormatFourCCHelper},
          {USAGE, &GrallocSnapHelperLegacy::UsageHelper},
          {ALLOCATION_SIZE, &GrallocSnapHelperLegacy::AllocationSizeHelper},
          {PROTECTED_CONTENT, &GrallocSnapHelperLegacy::ProtectedContentHelper},
          {COMPRESSION, &GrallocSnapHelperLegacy::CompressionHelper},
          {PLANE_LAYOUTS, &GrallocSnapHelperLegacy::PlaneLayoutsHelper},
          {ALIGNED_WIDTH_IN_PIXELS, &GrallocSnapHelperLegacy::AlignedWidthInPixelsHelper},
          {ALIGNED_HEIGHT_IN_PIXELS, &GrallocSnapHelperLegacy::AlignedHeightInPixelsHelper},
      };

  SnapError ColorMetadataHelper(SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size,
                                void *gralloc_in_set, void *gralloc_out_get,
                                SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                                int32_t *mapper_return = nullptr);
  SnapError PrivateFlagsHelper(SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size,
                               void *gralloc_in_set, void *gralloc_out_get,
                               SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                               int32_t *mapper_return = nullptr);
  SnapError ColorspaceHelper(SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size,
                             void *gralloc_in_set, void *gralloc_out_get,
                             SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                             int32_t *mapper_return = nullptr);
  SnapError YuvPlaneInfoHelper(SnapHandle *hnd, bool hidl_bytestream, uint32_t aidl_size,
                               void *gralloc_in_set, void *gralloc_out_get,
                               SnapDescriptor *buf_des = nullptr, bool check_metadata_set = true,
                               int32_t *mapper_return = nullptr);

  std::unordered_map<uint64_t, MetadataHelper> deprecated_metadata_conversion_helper_function_map_ =
      {
          {QTI_COLOR_METADATA, &GrallocSnapHelperLegacy::ColorMetadataHelper},
          {QTI_PRIVATE_FLAGS, &GrallocSnapHelperLegacy::PrivateFlagsHelper},
          {QTI_COLORSPACE, &GrallocSnapHelperLegacy::ColorspaceHelper},
          {QTI_YUV_PLANE_INFO, &GrallocSnapHelperLegacy::YuvPlaneInfoHelper},
      };
  std::unordered_map<uint64_t, std::vector<SnapMetadataType>> deprecated_metadata_type_map = {
      {QTI_COLOR_METADATA, {SnapMetadataType::MASTERING_DISPLAY,
      SnapMetadataType::CONTENT_LIGHT_LEVEL, SnapMetadataType::DYNAMIC_METADATA,
      SnapMetadataType::COLOR_REMAPPING_INFO, SnapMetadataType::MATRIX_COEFFICIENTS}},
      {QTI_PRIVATE_FLAGS, {SnapMetadataType::IS_UBWC, SnapMetadataType::IS_TILE_RENDERED,
      SnapMetadataType::IS_CACHED}},
  };
};

}  // namespace gralloc

#endif  // __GR_SNAP_HELPER_H__
