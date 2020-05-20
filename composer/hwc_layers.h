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
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef __HWC_LAYERS_H__
#define __HWC_LAYERS_H__

/* This class translates HWC2 Layer functions to the SDM LayerStack
 */

#include "gr_utils.h"
#include <QtiGralloc.h>
#include <core/layer_stack.h>
#include <core/layer_buffer.h>
#include <utils/utils.h>
#define HWC2_INCLUDE_STRINGIFICATION
#define HWC2_USE_CPP11
#include <hardware/hwcomposer2.h>
#undef HWC2_INCLUDE_STRINGIFICATION
#undef HWC2_USE_CPP11
#include <android/hardware/graphics/composer/2.3/IComposerClient.h>
#include <vendor/qti/hardware/display/composer/3.1/IQtiComposerClient.h>

#include <map>
#include <set>

#include "core/buffer_allocator.h"
#include "hwc_buffer_allocator.h"

using PerFrameMetadataKey =
    android::hardware::graphics::composer::V2_3::IComposerClient::PerFrameMetadataKey;
using vendor::qti::hardware::display::composer::V3_1::IQtiComposerClient;
using android::hardware::graphics::common::V1_2::Dataspace;

namespace sdm {

DisplayError SetCSC(const private_handle_t *pvt_handle, ColorMetaData *color_metadata);
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
  explicit HWCLayer(hwc2_display_t display_id, HWCBufferAllocator *buf_allocator);
  ~HWCLayer();
  uint32_t GetZ() const { return z_; }
  hwc2_layer_t GetId() const { return id_; }
  std::string GetName() const { return name_; }
  LayerTypes GetType() const { return type_; }
  Layer *GetSDMLayer() { return layer_; }
  void ResetPerFrameData();

  HWC2::Error SetLayerBlendMode(HWC2::BlendMode mode);
  HWC2::Error SetLayerBuffer(buffer_handle_t buffer, shared_ptr<Fence> acquire_fence);
  HWC2::Error SetLayerColor(hwc_color_t color);
  HWC2::Error SetLayerCompositionType(HWC2::Composition type);
  HWC2::Error SetLayerDataspace(int32_t dataspace);
  HWC2::Error SetLayerDisplayFrame(hwc_rect_t frame);
  HWC2::Error SetCursorPosition(int32_t x, int32_t y);
  HWC2::Error SetLayerPlaneAlpha(float alpha);
  HWC2::Error SetLayerSourceCrop(hwc_frect_t crop);
  HWC2::Error SetLayerSurfaceDamage(hwc_region_t damage);
  HWC2::Error SetLayerTransform(HWC2::Transform transform);
  HWC2::Error SetLayerVisibleRegion(hwc_region_t visible);
  HWC2::Error SetLayerPerFrameMetadata(uint32_t num_elements, const PerFrameMetadataKey *keys,
                                       const float *metadata);
  HWC2::Error SetLayerPerFrameMetadataBlobs(uint32_t num_elements, const PerFrameMetadataKey *keys,
                                            const uint32_t *sizes, const uint8_t* metadata);
  HWC2::Error SetLayerZOrder(uint32_t z);
  HWC2::Error SetLayerType(IQtiComposerClient::LayerType type);
  HWC2::Error SetLayerFlag(IQtiComposerClient::LayerFlag flag);
  HWC2::Error SetLayerColorTransform(const float *matrix);
  void SetComposition(const LayerComposition &sdm_composition);
  HWC2::Composition GetClientRequestedCompositionType() { return client_requested_; }
  HWC2::Composition GetOrigClientRequestedCompositionType() { return client_requested_orig_; }
  void UpdateClientCompositionType(HWC2::Composition type) { client_requested_ = type; }
  HWC2::Composition GetDeviceSelectedCompositionType() { return device_selected_; }
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
  void IgnoreSdrContentMetadata(bool disable) {
    ignore_sdr_content_md_ = disable;
  }
#ifdef UDFPS_ZPOS
  bool IsFodPressed() { return fod_pressed_; }
#endif

 private:
  Layer *layer_ = nullptr;
  LayerTypes type_ = kLayerUnknown;
  uint32_t z_ = 0;
  const hwc2_layer_t id_;
  std::string name_;
  const hwc2_display_t display_id_;
  static std::atomic<hwc2_layer_t> next_id_;
  shared_ptr<Fence> release_fence_;
  HWCBufferAllocator *buffer_allocator_ = NULL;
  int32_t dataspace_ =  HAL_DATASPACE_UNKNOWN;
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
  bool ignore_sdr_content_md_ = false;
#ifdef UDFPS_ZPOS
  bool fod_pressed_ = false;
#endif

  // Composition requested by client(SF) Original
  HWC2::Composition client_requested_orig_ = HWC2::Composition::Device;
  // Composition requested by client(SF) Modified for internel use
  HWC2::Composition client_requested_ = HWC2::Composition::Device;
  // Composition selected by SDM
  HWC2::Composition device_selected_ = HWC2::Composition::Device;
  uint32_t geometry_changes_ = GeometryChanges::kNone;

  void SetRect(const hwc_rect_t &source, LayerRect *target);
  void SetRect(const hwc_frect_t &source, LayerRect *target);
  uint32_t GetUint32Color(const hwc_color_t &source);
  void GetUBWCStatsFromMetaData(UBWCStats *cr_stats, UbwcCrStatsVector *cr_vec);
  DisplayError SetMetaData(const private_handle_t *pvt_handle, Layer *layer);
  uint32_t RoundToStandardFPS(float fps);
  void ValidateAndSetCSC(const private_handle_t *handle);
  void SetDirtyRegions(hwc_region_t surface_damage);
};

struct SortLayersByZ {
  bool operator()(const HWCLayer *lhs, const HWCLayer *rhs) const {
    return lhs->GetZ() < rhs->GetZ();
  }
};

}  // namespace sdm
#endif  // __HWC_LAYERS_H__
