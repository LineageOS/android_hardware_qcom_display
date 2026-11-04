/*
 * Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *
 * Copyright 2015 The Android Open Source Project
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

/*
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef __HWC_LAYERS_H__
#define __HWC_LAYERS_H__

/* This class translates HWC3 Layer functions to the SDM LayerStack
 */

#include <QtiGralloc.h>
#include <QtiGrallocDefs.h>
#include <core/layer_stack.h>
#include <core/layer_buffer.h>
#include <utils/utils.h>

#include <map>
#include <set>

#include "core/buffer_allocator.h"
#include "hwc_buffer_allocator.h"
#include "hwc_common.h"

using aidl::android::hardware::graphics::composer3::PerFrameMetadataKey;
using PixelFormat_V3 = aidl::android::hardware::graphics::common::PixelFormat;

#ifdef PXLW_IRIS7
namespace pxlw {
class PxlwIris7AidlWrapper;
}  // namespace pxlw
#endif

namespace sdm {

DisplayError SetCSC(const native_handle_t *pvt_handle, ColorMetaData *color_metadata);
bool GetColorPrimary(const int32_t &dataspace, ColorPrimaries *color_primary);
bool GetTransfer(const int32_t &dataspace, GammaTransfer *gamma_transfer);
bool GetRange(const int32_t &dataspace, ColorRange *color_range);
bool GetSDMColorSpace(const int32_t &dataspace, ColorMetaData *color_metadata);
bool IsBT2020(const ColorPrimaries &color_primary);
int32_t TranslateFromLegacyDataspace(const int32_t &legacy_ds);
DisplayError ColorMetadataToDataspace(ColorMetaData color_metadata, Dataspace *dataspace);

enum LayerTypes {
  kLayerUnknown = 0,
  kLayerApp = 1,
  kLayerGame = 2,
  kLayerBrowser = 3,
};

class HWCLayer {
 public:
  explicit HWCLayer(Display display_id, HWCBufferAllocator *buf_allocator);
  ~HWCLayer();
  uint32_t GetZ() const { return z_; }
  LayerId GetId() const { return id_; }
  std::string GetName() const { return name_; }
  LayerTypes GetType() const { return type_; }
  Layer *GetSDMLayer() { return layer_; }
  void ResetPerFrameData();

  HWC3::Error SetLayerBlendMode(BlendMode mode);
  HWC3::Error SetLayerBuffer(buffer_handle_t buffer, shared_ptr<Fence> acquire_fence);
  HWC3::Error SetLayerColor(Color color);
  HWC3::Error SetLayerCompositionType(Composition type);
  HWC3::Error SetLayerDataspace(int32_t dataspace);
  HWC3::Error SetLayerDisplayFrame(Rect frame);
  HWC3::Error SetCursorPosition(int32_t x, int32_t y);
  HWC3::Error SetLayerPlaneAlpha(float alpha);
  HWC3::Error SetLayerSourceCrop(FRect crop);
  HWC3::Error SetLayerSurfaceDamage(Region damage);
  HWC3::Error SetLayerTransform(Transform transform);
  HWC3::Error SetLayerVisibleRegion(Region visible);
  HWC3::Error SetLayerPerFrameMetadata(uint32_t num_elements, const PerFrameMetadataKey *keys,
                                       const float *metadata);
  HWC3::Error SetLayerPerFrameMetadataBlobs(uint32_t num_elements, const PerFrameMetadataKey *keys,
                                            const uint32_t *sizes, const uint8_t *metadata);
  HWC3::Error SetLayerZOrder(uint32_t z);
  HWC3::Error SetLayerType(LayerType type);
  HWC3::Error SetLayerFlag(LayerFlag flag);
  HWC3::Error SetLayerColorTransform(const float *matrix);
  HWC3::Error SetLayerBrightness(float brightness);
  void SetComposition(const LayerComposition &sdm_composition);
  Composition GetClientRequestedCompositionType() { return client_requested_; }
  Composition GetOrigClientRequestedCompositionType() { return client_requested_orig_; }
  void UpdateClientCompositionType(Composition type) { client_requested_ = type; }
  Composition GetDeviceSelectedCompositionType() { return device_selected_; }
  int32_t GetLayerDataspace() { return dataspace_; }
  uint32_t GetGeometryChanges() { return geometry_changes_; }
  void ResetGeometryChanges();
  void ResetValidation() { layer_->update_mask.reset(); }
  bool NeedsValidation() { return (geometry_changes_ || layer_->update_mask.any()); }
  bool IsSingleBuffered() { return single_buffer_; }
  bool IsScalingPresent();
  bool IsRotationPresent();
  bool IsDataSpaceSupported();
  bool IsProtected() { return secure_; }
  static LayerBufferFormat GetSDMFormat(const int32_t &source, const int flags);
  bool IsSurfaceUpdated() { return surface_updated_; }
  bool IsNonIntegralSourceCrop() { return non_integral_source_crop_; }
  bool HasMetaDataRefreshRate() { return has_metadata_refresh_rate_; }
  bool IsColorTransformSet() { return color_transform_matrix_set_; }
  void SetLayerAsMask();
  bool BufferLatched() { return buffer_flipped_; }
  void ResetBufferFlip() { buffer_flipped_ = false; }
  shared_ptr<Fence> GetReleaseFence();
  void SetReleaseFence(const shared_ptr<Fence> &release_fence);
  bool IsLayerCompatible() { return compatible_; }
  void IgnoreSdrHistogramMetadata(bool disable) { ignore_sdr_histogram_md_ = disable; }
#ifdef UDFPS_ZPOS
  bool IsFodPressed() { return fod_pressed_; }
#endif
#ifdef PXLW_IRIS7
  void CreateIrisLayer();
  bool IsIrisLandscapeVideo() const;
  void SetIrisOrientation(bool landscape);
#endif

 private:
#ifdef PXLW_IRIS7
  pxlw::PxlwIris7AidlWrapper *iris_ = nullptr;
  int32_t iris_orientation_ = 0;
#endif
  Layer *layer_ = nullptr;
  LayerTypes type_ = kLayerUnknown;
  uint32_t z_ = 0;
  const LayerId id_;
  std::string name_;
  const Display display_id_;
  static std::atomic<LayerId> next_id_;
  shared_ptr<Fence> release_fence_;
  HWCBufferAllocator *buffer_allocator_ = NULL;
  int32_t dataspace_ = INT32(Dataspace::UNKNOWN);
  LayerTransform layer_transform_ = {};
  LayerRect dst_rect_ = {};
  bool single_buffer_ = false;
  int buffer_fd_ = -1;
  bool dataspace_supported_ = false;
  bool surface_updated_ = true;
  bool non_integral_source_crop_ = false;
  bool has_metadata_refresh_rate_ = false;
  bool color_transform_matrix_set_ = false;
  bool buffer_flipped_ = false;
  bool secure_ = false;
  bool compatible_ = false;
  bool ignore_sdr_histogram_md_ = false;
#ifdef UDFPS_ZPOS
  bool fod_pressed_ = false;
#endif

  // Composition requested by client(SF) Original
  Composition client_requested_orig_ = Composition::DEVICE;
  // Composition requested by client(SF) Modified for internel use
  Composition client_requested_ = Composition::DEVICE;
  // Composition selected by SDM
  Composition device_selected_ = Composition::DEVICE;
  uint32_t geometry_changes_ = GeometryChanges::kNone;

  void SetRect(const Rect &source, LayerRect *target);
  void SetRect(const FRect &source, LayerRect *target);
  uint32_t GetUint32Color(const Color &source);
  void GetUBWCStatsFromMetaData(UBWCStats *cr_stats, UbwcCrStatsVector *cr_vec);
  DisplayError SetMetaData(const native_handle_t *pvt_handle, Layer *layer);
  uint32_t RoundToStandardFPS(float fps);
  void ValidateAndSetCSC(const native_handle_t *handle);
  void SetDirtyRegions(Region surface_damage);
};

struct SortLayersByZ {
  bool operator()(const HWCLayer *lhs, const HWCLayer *rhs) const {
    return lhs->GetZ() < rhs->GetZ();
  }
};

}  // namespace sdm
#endif  // __HWC_LAYERS_H__
