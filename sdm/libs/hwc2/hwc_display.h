/*
 * Copyright (c) 2014-2019, The Linux Foundation. All rights reserved.
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

#ifndef __HWC_DISPLAY_H__
#define __HWC_DISPLAY_H__

#include <QService.h>
#include <android/hardware/graphics/common/1.1/types.h>
#include <core/core_interface.h>
#include <hardware/hwcomposer.h>
#include <private/color_params.h>
#include <qdMetaData.h>
#include <sys/stat.h>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <bitset>
#include <algorithm>
#include "hwc_buffer_allocator.h"
#include "hwc_callbacks.h"
#include "hwc_layers.h"
#include "display_null.h"
#include "hwc_display_event_handler.h"

using android::hardware::graphics::common::V1_1::ColorMode;
using android::hardware::graphics::common::V1_1::Dataspace;
using android::hardware::graphics::common::V1_1::RenderIntent;

namespace sdm {

class BlitEngine;
class HWCToneMapper;

// Subclasses set this to their type. This has to be different from DisplayType.
// This is to avoid RTTI and dynamic_cast
enum DisplayClass {
  DISPLAY_CLASS_BUILTIN,
  DISPLAY_CLASS_PLUGGABLE,
  DISPLAY_CLASS_VIRTUAL,
  DISPLAY_CLASS_NULL
};

enum SecureSessionType {
  kSecureDisplay,
  kSecureCamera,
  kSecureMax,
};

class HWCColorMode {
 public:
  explicit HWCColorMode(DisplayInterface *display_intf);
  ~HWCColorMode() {}
  HWC2::Error Init();
  HWC2::Error DeInit();
  void Dump(std::ostringstream* os);
  uint32_t GetColorModeCount();
  uint32_t GetRenderIntentCount(ColorMode mode);
  HWC2::Error GetColorModes(uint32_t *out_num_modes, ColorMode *out_modes);
  HWC2::Error GetRenderIntents(ColorMode mode, uint32_t *out_num_intents, RenderIntent *out_modes);
  HWC2::Error SetColorModeWithRenderIntent(ColorMode mode, RenderIntent intent);
  HWC2::Error SetColorModeById(int32_t color_mode_id);
  HWC2::Error SetColorModeFromClientApi(std::string mode_string);
  HWC2::Error SetColorTransform(const float *matrix, android_color_transform_t hint);
  HWC2::Error RestoreColorTransform();
  ColorMode GetCurrentColorMode() { return current_color_mode_; }
  HWC2::Error ApplyCurrentColorModeWithRenderIntent(bool hdr_present);
  HWC2::Error CacheColorModeWithRenderIntent(ColorMode mode, RenderIntent intent);
  RenderIntent GetCurrentRenderIntent() { return current_render_intent_; }

 private:
  static const uint32_t kColorTransformMatrixCount = 16;
  void PopulateColorModes();
  template <class T>
  void CopyColorTransformMatrix(const T *input_matrix, double *output_matrix) {
    for (uint32_t i = 0; i < kColorTransformMatrixCount; i++) {
      output_matrix[i] = static_cast<double>(input_matrix[i]);
    }
  }
  HWC2::Error ValidateColorModeWithRenderIntent(ColorMode mode, RenderIntent intent);
  HWC2::Error SetPreferredColorModeInternal(const std::string &mode_string, bool from_client,
    ColorMode *color_mode, DynamicRangeType *dynamic_range);

  DisplayInterface *display_intf_ = NULL;
  bool apply_mode_ = false;
  ColorMode current_color_mode_ = ColorMode::NATIVE;
  RenderIntent current_render_intent_ = RenderIntent::COLORIMETRIC;
  DynamicRangeType curr_dynamic_range_ = kSdrType;
  typedef std::map<DynamicRangeType, std::string> DynamicRangeMap;
  typedef std::map<RenderIntent, DynamicRangeMap> RenderIntentMap;
  // Initialize supported mode/render intent/dynamic range combination
  std::map<ColorMode, RenderIntentMap> color_mode_map_ = {};
  double color_matrix_[kColorTransformMatrixCount] = { 1.0, 0.0, 0.0, 0.0, \
                                                       0.0, 1.0, 0.0, 0.0, \
                                                       0.0, 0.0, 1.0, 0.0, \
                                                       0.0, 0.0, 0.0, 1.0 };
  std::map<ColorMode, DynamicRangeMap> preferred_mode_ = {};
};

class HWCDisplay : public DisplayEventHandler {
 public:
  enum DisplayStatus {
    kDisplayStatusInvalid = -1,
    kDisplayStatusOffline,
    kDisplayStatusOnline,
    kDisplayStatusPause,
    kDisplayStatusResume,
  };

  enum DisplayValidateState {
    kNormalValidate,
    kInternalValidate,
    kSkipValidate,
  };

  struct HWCLayerStack {
    HWCLayer *client_target = nullptr;                   // Also known as framebuffer target
    std::map<hwc2_layer_t, HWCLayer *> layer_map;        // Look up by Id - TODO
    std::multiset<HWCLayer *, SortLayersByZ> layer_set;  // Maintain a set sorted by Z
  };

  virtual ~HWCDisplay() {}
  virtual int Init();
  virtual int Deinit();

  // Framebuffer configurations
  virtual void SetIdleTimeoutMs(uint32_t timeout_ms);
  virtual HWC2::Error SetFrameDumpConfig(uint32_t count, uint32_t bit_mask_layer_type,
                                         int32_t format, bool post_processed);
  virtual DisplayError SetMaxMixerStages(uint32_t max_mixer_stages);
  virtual DisplayError ControlPartialUpdate(bool enable, uint32_t *pending) {
    return kErrorNotSupported;
  }
  virtual HWC2::PowerMode GetCurrentPowerMode();
  virtual int SetFrameBufferResolution(uint32_t x_pixels, uint32_t y_pixels);
  virtual void GetFrameBufferResolution(uint32_t *x_pixels, uint32_t *y_pixels);
  virtual int SetDisplayStatus(DisplayStatus display_status);
  virtual int OnMinHdcpEncryptionLevelChange(uint32_t min_enc_level);
  virtual int Perform(uint32_t operation, ...);
  virtual int HandleSecureSession(const std::bitset<kSecureMax> &secure_sessions,
                                  bool *power_on_pending);
  virtual int GetActiveSecureSession(std::bitset<kSecureMax> *secure_sessions);
  virtual DisplayError SetMixerResolution(uint32_t width, uint32_t height);
  virtual DisplayError GetMixerResolution(uint32_t *width, uint32_t *height);
  virtual void GetPanelResolution(uint32_t *width, uint32_t *height);
  virtual std::string Dump();
  virtual DisplayError TeardownConcurrentWriteback(void) {
    return kErrorNotSupported;
  }

  // Captures frame output in the buffer specified by output_buffer_info. The API is
  // non-blocking and the client is expected to check operation status later on.
  // Returns -1 if the input is invalid.
  virtual int FrameCaptureAsync(const BufferInfo &output_buffer_info, bool post_processed) {
    return -1;
  }
  // Returns the status of frame capture operation requested with FrameCaptureAsync().
  // -EAGAIN : No status obtain yet, call API again after another frame.
  // < 0 : Operation happened but failed.
  // 0 : Success.
  virtual int GetFrameCaptureStatus() { return -EAGAIN; }

  virtual DisplayError SetDetailEnhancerConfig(const DisplayDetailEnhancerData &de_data) {
    return kErrorNotSupported;
  }
  virtual HWC2::Error SetReadbackBuffer(const native_handle_t *buffer, int32_t acquire_fence,
                                        bool post_processed_output) {
    return HWC2::Error::Unsupported;
  }
  virtual HWC2::Error GetReadbackBufferFence(int32_t *release_fence) {
    return HWC2::Error::Unsupported;
  }

  virtual HWC2::Error SetDisplayDppsAdROI(uint32_t h_start, uint32_t h_end,
                                          uint32_t v_start, uint32_t v_end,
                                          uint32_t factor_in, uint32_t factor_out) {
    return HWC2::Error::Unsupported;
  }

  // Display Configurations
  static uint32_t GetThrottlingRefreshRate() { return HWCDisplay::throttling_refresh_rate_; }
  static void SetThrottlingRefreshRate(uint32_t newRefreshRate)
              { HWCDisplay::throttling_refresh_rate_ = newRefreshRate; }
  virtual int SetActiveDisplayConfig(uint32_t config);
  virtual int GetActiveDisplayConfig(uint32_t *config);
  virtual int GetDisplayConfigCount(uint32_t *count);
  virtual int GetDisplayAttributesForConfig(int config,
                                            DisplayConfigVariableInfo *display_attributes);
  virtual int SetState(bool connected) {
    return kErrorNotSupported;
  }
  virtual DisplayError Flush() {
    return kErrorNotSupported;
  }

  uint32_t GetMaxRefreshRate() { return max_refresh_rate_; }
  int SetPanelBrightness(int level);
  int GetPanelBrightness(int *level);
  int ToggleScreenUpdates(bool enable);
  int ColorSVCRequestRoute(const PPDisplayAPIPayload &in_payload, PPDisplayAPIPayload *out_payload,
                           PPPendingParams *pending_action);
  void SolidFillPrepare();
  void SolidFillCommit();
  DisplayClass GetDisplayClass();
  int GetVisibleDisplayRect(hwc_rect_t *rect);
  void BuildLayerStack(void);
  void BuildSolidFillStack(void);
  HWCLayer *GetHWCLayer(hwc2_layer_t layer_id);
  void ResetValidation() { validated_ = false; }
  uint32_t GetGeometryChanges() { return geometry_changes_; }
  bool CanSkipValidate();
  bool IsSkipValidateState() { return (validate_state_ == kSkipValidate); }
  bool IsInternalValidateState() { return (validated_ && (validate_state_ == kInternalValidate)); }
  void SetValidationState(DisplayValidateState state) { validate_state_ = state; }
  ColorMode GetCurrentColorMode() {
    return (color_mode_ ? color_mode_->GetCurrentColorMode() : ColorMode::SRGB);
  }
  RenderIntent GetCurrentRenderIntent() {
    return (color_mode_ ? color_mode_->GetCurrentRenderIntent() : RenderIntent::COLORIMETRIC);
  }
  bool HasClientComposition() { return has_client_composition_; }
  bool HWCClientNeedsValidate() {
    return (has_client_composition_ || layer_stack_.flags.single_buffered_layer_present);
  }
  virtual void SetFastPathComposition(bool enable) { fast_path_composition_ = enable; }
  virtual HWC2::Error SetColorModeFromClientApi(int32_t color_mode_id) {
    return HWC2::Error::Unsupported;
  }
  bool IsFirstCommitDone() { return !first_cycle_; }

  // HWC2 APIs
  virtual HWC2::Error AcceptDisplayChanges(void);
  virtual HWC2::Error GetActiveConfig(hwc2_config_t *out_config);
  virtual HWC2::Error SetActiveConfig(hwc2_config_t config);
  virtual HWC2::Error SetPanelLuminanceAttributes(float min_lum, float max_lum) {
    return HWC2::Error::Unsupported;
  }
  virtual HWC2::Error SetClientTarget(buffer_handle_t target, int32_t acquire_fence,
                                      int32_t dataspace, hwc_region_t damage);
  virtual HWC2::Error SetColorMode(ColorMode mode) { return HWC2::Error::Unsupported; }
  virtual HWC2::Error SetColorModeWithRenderIntent(ColorMode mode, RenderIntent intent) {
    return HWC2::Error::Unsupported;
  }
  virtual HWC2::Error SetColorModeById(int32_t color_mode_id) {
    return HWC2::Error::Unsupported;
  }
  virtual HWC2::Error RestoreColorTransform() {
    return HWC2::Error::Unsupported;
  }
  virtual HWC2::Error SetColorTransform(const float *matrix, android_color_transform_t hint) {
    return HWC2::Error::Unsupported;
  }
  virtual HWC2::Error HandleColorModeTransform(android_color_mode_t mode,
                                               android_color_transform_t hint,
                                               const double *matrix) {
    return HWC2::Error::Unsupported;
  }
  virtual DisplayError SetDynamicDSIClock(uint64_t bitclk) {
    return kErrorNotSupported;
  }
  virtual DisplayError GetDynamicDSIClock(uint64_t *bitclk) {
    return kErrorNotSupported;
  }
  virtual DisplayError GetSupportedDSIClock(std::vector<uint64_t> *bitclk) {
    return kErrorNotSupported;
  }
  virtual HWC2::Error UpdateDisplayId(hwc2_display_t id) {
    return HWC2::Error::Unsupported;
  }
  virtual HWC2::Error SetPendingRefresh() {
    return HWC2::Error::Unsupported;
  }
  virtual HWC2::Error GetDisplayConfigs(uint32_t *out_num_configs, hwc2_config_t *out_configs);
  virtual HWC2::Error GetDisplayAttribute(hwc2_config_t config, HWC2::Attribute attribute,
                                          int32_t *out_value);
  virtual HWC2::Error GetClientTargetSupport(uint32_t width, uint32_t height, int32_t format,
                                             int32_t dataspace);
  virtual HWC2::Error GetColorModes(uint32_t *outNumModes, ColorMode *outModes);
  virtual HWC2::Error GetRenderIntents(ColorMode mode, uint32_t *out_num_intents,
                                       RenderIntent *out_intents);
  virtual HWC2::Error GetChangedCompositionTypes(uint32_t *out_num_elements,
                                                 hwc2_layer_t *out_layers, int32_t *out_types);
  virtual HWC2::Error GetDisplayRequests(int32_t *out_display_requests, uint32_t *out_num_elements,
                                         hwc2_layer_t *out_layers, int32_t *out_layer_requests);
  virtual HWC2::Error GetDisplayName(uint32_t *out_size, char *out_name);
  virtual HWC2::Error GetDisplayType(int32_t *out_type);
  virtual HWC2::Error SetCursorPosition(hwc2_layer_t layer, int x, int y);
  virtual HWC2::Error SetVsyncEnabled(HWC2::Vsync enabled);
  virtual HWC2::Error SetPowerMode(HWC2::PowerMode mode, bool teardown);
  virtual HWC2::Error UpdatePowerMode(HWC2::PowerMode mode) {
    return HWC2::Error::None;
  }
  virtual HWC2::Error CreateLayer(hwc2_layer_t *out_layer_id);
  virtual HWC2::Error DestroyLayer(hwc2_layer_t layer_id);
  virtual HWC2::Error SetLayerZOrder(hwc2_layer_t layer_id, uint32_t z);
  virtual HWC2::Error Validate(uint32_t *out_num_types, uint32_t *out_num_requests) = 0;
  virtual HWC2::Error GetReleaseFences(uint32_t *out_num_elements, hwc2_layer_t *out_layers,
                                       int32_t *out_fences);
  virtual HWC2::Error Present(int32_t *out_retire_fence) = 0;
  virtual HWC2::Error GetHdrCapabilities(uint32_t *out_num_types, int32_t* out_types,
                                         float* out_max_luminance,
                                         float* out_max_average_luminance,
                                         float* out_min_luminance);
  virtual HWC2::Error GetPerFrameMetadataKeys(uint32_t *out_num_keys,
                                              PerFrameMetadataKey *out_keys);
  virtual HWC2::Error SetDisplayAnimating(bool animating) {
    animating_ = animating;
    validated_ = false;
    return HWC2::Error::None;
  }
  virtual HWC2::Error GetValidateDisplayOutput(uint32_t *out_num_types, uint32_t *out_num_requests);
  virtual bool IsDisplayCommandMode();
  virtual HWC2::Error SetQSyncMode(QSyncMode qsync_mode) {
    return HWC2::Error::Unsupported;
  }
  virtual DisplayError ControlIdlePowerCollapse(bool enable, bool synchronous) {
    return kErrorNone;
  }
  virtual HWC2::Error GetDisplayIdentificationData(uint8_t *out_port, uint32_t *out_data_size,
                                                   uint8_t *out_data);
  virtual void GetLayerStack(HWCLayerStack *stack);
  virtual void SetLayerStack(HWCLayerStack *stack);
  virtual void PostPowerMode();
  virtual void NotifyClientStatus(bool connected) { client_connected_ = connected; }

 protected:
  static uint32_t throttling_refresh_rate_;
  // Maximum number of layers supported by display manager.
  static const uint32_t kMaxLayerCount = 32;
  HWCDisplay(CoreInterface *core_intf, BufferAllocator *buffer_allocator, HWCCallbacks *callbacks,
             HWCDisplayEventHandler *event_handler, qService::QService *qservice, DisplayType type,
             hwc2_display_t id, int32_t sdm_id, bool needs_blit, DisplayClass display_class);

  // DisplayEventHandler methods
  virtual DisplayError VSync(const DisplayEventVSync &vsync);
  virtual DisplayError Refresh();
  virtual DisplayError CECMessage(char *message);
  virtual DisplayError HandleEvent(DisplayEvent event);
  virtual void DumpOutputBuffer(const BufferInfo &buffer_info, void *base, int fence);
  virtual HWC2::Error PrepareLayerStack(uint32_t *out_num_types, uint32_t *out_num_requests);
  virtual HWC2::Error CommitLayerStack(void);
  virtual HWC2::Error PostCommitLayerStack(int32_t *out_retire_fence);
  virtual DisplayError DisablePartialUpdateOneFrame() {
    return kErrorNotSupported;
  }
  const char *GetDisplayString();
  void MarkLayersForGPUBypass(void);
  void MarkLayersForClientComposition(void);
  void UpdateConfigs();
  virtual void ApplyScanAdjustment(hwc_rect_t *display_frame);
  uint32_t GetUpdatingLayersCount(void);
  bool IsLayerUpdating(HWCLayer *layer);
  uint32_t SanitizeRefreshRate(uint32_t req_refresh_rate);
  virtual void GetUnderScanConfig() { }
  int32_t SetClientTargetDataSpace(int32_t dataspace);

  enum {
    INPUT_LAYER_DUMP,
    OUTPUT_LAYER_DUMP,
  };

  bool validated_ = false;
  bool layer_stack_invalid_ = true;
  CoreInterface *core_intf_ = nullptr;
  HWCBufferAllocator *buffer_allocator_ = NULL;
  HWCCallbacks *callbacks_  = nullptr;
  HWCDisplayEventHandler *event_handler_ = nullptr;
  DisplayType type_ = kDisplayTypeMax;
  hwc2_display_t id_ = UINT64_MAX;
  int32_t sdm_id_ = -1;
  bool needs_blit_ = false;
  DisplayInterface *display_intf_ = NULL;
  LayerStack layer_stack_;
  HWCLayer *client_target_ = nullptr;                   // Also known as framebuffer target
  std::map<hwc2_layer_t, HWCLayer *> layer_map_;        // Look up by Id - TODO
  std::multiset<HWCLayer *, SortLayersByZ> layer_set_;  // Maintain a set sorted by Z
  std::map<hwc2_layer_t, HWC2::Composition> layer_changes_;
  std::map<hwc2_layer_t, HWC2::LayerRequest> layer_requests_;
  bool flush_on_error_ = false;
  bool flush_ = false;
  uint32_t dump_frame_count_ = 0;
  uint32_t dump_frame_index_ = 0;
  bool dump_input_layers_ = false;
  HWC2::PowerMode current_power_mode_ = HWC2::PowerMode::Off;
  bool swap_interval_zero_ = false;
  bool display_paused_ = false;
  uint32_t min_refresh_rate_ = 0;
  uint32_t max_refresh_rate_ = 0;
  uint32_t current_refresh_rate_ = 0;
  bool use_metadata_refresh_rate_ = false;
  uint32_t metadata_refresh_rate_ = 0;
  uint32_t force_refresh_rate_ = 0;
  bool boot_animation_completed_ = false;
  bool shutdown_pending_ = false;
  bool use_blit_comp_ = false;
  std::bitset<kSecureMax> active_secure_sessions_ = 0;
  bool solid_fill_enable_ = false;
  Layer *solid_fill_layer_ = NULL;
  LayerRect solid_fill_rect_ = {};
  LayerSolidFill solid_fill_color_ = {};
  LayerRect display_rect_;
  bool color_tranform_failed_ = false;
  HWCColorMode *color_mode_ = NULL;
  HWCToneMapper *tone_mapper_ = nullptr;
  uint32_t num_configs_ = 0;
  int disable_hdr_handling_ = 0;  // disables HDR handling.
  bool pending_commit_ = false;
  bool is_cmd_mode_ = false;
  bool partial_update_enabled_ = false;
  bool skip_commit_ = false;
  std::map<uint32_t, DisplayConfigVariableInfo> variable_config_map_;
  std::vector<uint32_t> hwc_config_map_;
  bool fast_path_composition_ = false;
  bool client_connected_ = true;
  bool pending_config_ = false;

 private:
  void DumpInputBuffers(void);
  bool CanSkipSdmPrepare(uint32_t *num_types, uint32_t *num_requests);
  void UpdateRefreshRate();
  void WaitOnPreviousFence();
  void UpdateActiveConfig();
  qService::QService *qservice_ = NULL;
  DisplayClass display_class_;
  uint32_t geometry_changes_ = GeometryChanges::kNone;
  bool animating_ = false;
  int null_display_mode_ = 0;
  bool has_client_composition_ = false;
  DisplayValidateState validate_state_ = kNormalValidate;
  bool fast_path_enabled_ = true;
  bool first_cycle_ = true;  // false if a display commit has succeeded on the device.
  int fbt_release_fence_ = -1;
  int release_fence_ = -1;
  hwc2_config_t pending_config_index_ = 0;
  int async_power_mode_ = 0;
};

inline int HWCDisplay::Perform(uint32_t operation, ...) {
  return 0;
}

}  // namespace sdm

#endif  // __HWC_DISPLAY_H__
