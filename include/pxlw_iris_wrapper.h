/*
 * SPDX-FileCopyrightText: 2025 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>

#include <aidl/android/hardware/graphics/common/FRect.h>
#include <aidl/android/hardware/graphics/common/Rect.h>
#include <hidl/HidlSupport.h>

struct native_handle;

namespace sdm {
struct DisplayConfigVariableInfo;
struct LayerStack;
}  // namespace sdm

namespace pxlw {

// vendor.pixelworks.hardware.display@1.1::IrisFixedConfig
struct IrisFixedConfig {
  int32_t hdrFormal;
  android::hardware::hidl_vec<int32_t> hdrLut;
  int32_t memcEnable;
  int32_t memcLevel;
  int32_t dualChannel;
  int32_t movingLayer;
  int32_t memcVideoLayer;
  int32_t inMemcState;
  int32_t videoFps;
  int32_t videoInMemory;
  int32_t gameMode;
  int32_t captureDisable;
  int32_t dualPrepare;
  int32_t inOsdSwitch;
  int32_t dualPreload;
  int32_t metadataDone;
  android::hardware::hidl_vec<int32_t> reserved;
};

// Layer property metadata key and its orientation field.
constexpr int32_t kIrisLayerProperty = 0x400;
constexpr int32_t kIrisLayerOrientationShift = 17;

class PxlwIrisWrapper {
 public:
  static PxlwIrisWrapper *GetInstance();
  void SetRefreshCB(void (*callback)(void *), void *context);

 private:
  PxlwIrisWrapper() = default;
};

class PxlwSoftirisWrapper {
 public:
  PxlwSoftirisWrapper();
  virtual ~PxlwSoftirisWrapper();

  bool HasSoftIris();

  int InitPrimaryDisplay(int32_t vsync_period_ns, uint32_t width, uint32_t height);
  void SetColorModeWithRenderIntent(int32_t display_id, int32_t, int32_t colorMode,
                                    int32_t renderIntent);

 private:
  friend class PxlwIrisWrapper;
};

// HW Iris 7 (AIDL-based) wrapper entry points exported by vendor libpwirishalwrapper.so.
// Only used when PXLW_IRIS7 is defined.
class PxlwIris7AidlWrapper {
 public:
  bool HasIrisDual();
  void InitPrimaryDisplay(int32_t vsync_period_ns, uint32_t width, uint32_t height);
  void SetColorModeWithRenderIntent(int32_t display_id, int32_t display_type, int32_t color_mode,
                                    int32_t render_intent);

  void CreateLayer(uint64_t display_id, int64_t layer_id);
  void DestroyLayer(uint64_t display_id, int64_t layer_id);
  // The acquire fence is unused.
  void SetLayerBuffer(uint64_t display_id, int64_t layer_id, const native_handle *buffer,
                      int32_t acquire_fence);
  void SetLayerCompositionType(uint64_t display_id, int64_t layer_id, int32_t type);
  void SetLayerDisplayFrame(uint64_t display_id, int64_t layer_id,
                            aidl::android::hardware::graphics::common::Rect frame);
  void SetLayerPerFrameMetadata(uint64_t display_id, int64_t layer_id, uint32_t num_elements,
                                const int32_t *keys, const float *metadata);
  void SetLayerSourceCrop(uint64_t display_id, int64_t layer_id,
                          aidl::android::hardware::graphics::common::FRect crop);
  void SetLayerTransform(uint64_t display_id, int64_t layer_id, int32_t transform);
  void SetLayerZOrder(uint64_t display_id, int64_t layer_id, uint32_t z);

  void BuildLayerStackInternal(int32_t display_id, int32_t display_type, sdm::LayerStack *stack);
  int32_t PresentDisplay(uint64_t display_id);
  IrisFixedConfig GetCurrentIrisConfig();
  void SetActiveConfig(int32_t display_id, int32_t display_type, uint32_t config,
                       sdm::DisplayConfigVariableInfo *info);
  void BeforeSetPowerMode(uint64_t display_id, int32_t mode, bool from_idle);
  void AfterSetPowerMode(uint64_t display_id, int32_t mode, bool from_idle);

 private:
  PxlwIris7AidlWrapper() = default;
  friend class PxlwIrisWrapper;
};

}  // namespace pxlw
