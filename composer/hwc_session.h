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

#ifndef __HWC_SESSION_H__
#define __HWC_SESSION_H__

#ifndef DISPLAY_CONFIG_VERSION_OPTIMAL
#include <vendor/display/config/1.15/IDisplayConfig.h>
#else
#include <vendor/display/config/1.0/IDisplayConfig.h>
#endif
#include <vendor/qti/hardware/display/composer/2.0/IQtiComposerClient.h>

#include <core/core_interface.h>
#include <utils/locker.h>
#include <qd_utils.h>
#include <display_config.h>
#include <vector>
#include <queue>
#include <utility>
#include <future>   // NOLINT
#include <map>

#include "hwc_callbacks.h"
#include "hwc_layers.h"
#include "hwc_display.h"
#include "hwc_display_builtin.h"
#include "hwc_display_pluggable.h"
#include "hwc_display_dummy.h"
#include "hwc_display_virtual.h"
#include "hwc_display_pluggable_test.h"
#include "hwc_color_manager.h"
#include "hwc_socket_handler.h"
#include "hwc_display_event_handler.h"
#include "hwc_buffer_sync_handler.h"
#include "hwc_display_virtual_factory.h"

namespace sdm {

#ifndef DISPLAY_CONFIG_VERSION_OPTIMAL
using vendor::display::config::V1_15::IDisplayConfig;
using vendor::display::config::V1_10::IDisplayCWBCallback;
using vendor::display::config::V1_15::IDisplayQsyncCallback;
#else
using vendor::display::config::V1_0::IDisplayConfig;
#endif

using ::android::hardware::Return;
using ::android::hardware::hidl_string;
using android::hardware::hidl_handle;
using ::android::hardware::hidl_vec;
using ::android::sp;

using vendor::qti::hardware::display::composer::V2_0::IQtiComposerClient;

int32_t GetDataspaceFromColorMode(ColorMode mode);

// Create a singleton uevent listener thread valid for life of hardware composer process.
// This thread blocks on uevents poll inside uevent library implementation. This poll exits
// only when there is a valid uevent, it can not be interrupted otherwise. Tieing life cycle
// of this thread with HWC session cause HWC deinitialization to wait infinitely for the
// thread to exit.
class HWCUEventListener {
 public:
  virtual ~HWCUEventListener() {}
  virtual void UEventHandler(const char *uevent_data, int length) = 0;
};

class HWCUEvent {
 public:
  HWCUEvent();
  static void UEventThread(HWCUEvent *hwc_event);
  void Register(HWCUEventListener *uevent_listener);
  inline bool InitDone() { return init_done_; }

 private:
  std::mutex mutex_;
  std::condition_variable caller_cv_;
  HWCUEventListener *uevent_listener_ = nullptr;
  bool init_done_ = false;
};

constexpr int32_t kDataspaceSaturationMatrixCount = 16;
constexpr int32_t kDataspaceSaturationPropertyElements = 9;
constexpr int32_t kPropertyMax = 256;

class HWCSession : hwc2_device_t, HWCUEventListener, IDisplayConfig, public qClient::BnQClient,
                   public HWCDisplayEventHandler {
 public:
  enum HotPlugEvent {
    kHotPlugNone,
    kHotPlugEvent,
  };

  HWCSession();
  int Init();
  int Deinit();
  HWC2::Error CreateVirtualDisplayObj(uint32_t width, uint32_t height, int32_t *format,
                                      hwc2_display_t *out_display_id);

  template <typename... Args>
  int32_t CallDisplayFunction(hwc2_display_t display, HWC2::Error (HWCDisplay::*member)(Args...),
                              Args... args) {
    if (display >= HWCCallbacks::kNumDisplays) {
      return HWC2_ERROR_BAD_DISPLAY;
    }

    {
      // Power state transition start.
      SCOPE_LOCK(power_state_[display]);
      if (power_state_transition_[display]) {
        display = map_hwc_display_.find(display)->second;
      }
    }

    SCOPE_LOCK(locker_[display]);
    auto status = HWC2::Error::BadDisplay;
    if (hwc_display_[display]) {
      auto hwc_display = hwc_display_[display];
      status = (hwc_display->*member)(std::forward<Args>(args)...);
    }
    return INT32(status);
  }

  template <typename... Args>
  int32_t CallLayerFunction(hwc2_display_t display, hwc2_layer_t layer,
                            HWC2::Error (HWCLayer::*member)(Args...), Args... args) {
    if (display >= HWCCallbacks::kNumDisplays) {
      return HWC2_ERROR_BAD_DISPLAY;
    }

    {
      // Power state transition start.
      SCOPE_LOCK(power_state_[display]);
      if (power_state_transition_[display]) {
        display = map_hwc_display_.find(display)->second;
      }
    }

    SCOPE_LOCK(locker_[display]);
    auto status = HWC2::Error::BadDisplay;
    if (hwc_display_[display]) {
      status = HWC2::Error::BadLayer;
      auto hwc_layer = hwc_display_[display]->GetHWCLayer(layer);
      if (hwc_layer != nullptr) {
        status = (hwc_layer->*member)(std::forward<Args>(args)...);
        if (hwc_display_[display]->GetGeometryChanges()) {
          hwc_display_[display]->ResetValidation();
        }
      }
    }
    return INT32(status);
  }

  // HWC2 Functions that require a concrete implementation in hwc session
  // and hence need to be member functions
  static HWCSession *GetInstance();
  void GetCapabilities(uint32_t *outCount, int32_t *outCapabilities);
  void Dump(uint32_t *out_size, char *out_buffer);

  int32_t AcceptDisplayChanges(hwc2_display_t display);
  int32_t CreateLayer(hwc2_display_t display, hwc2_layer_t *out_layer_id);
  int32_t CreateVirtualDisplay(uint32_t width, uint32_t height, int32_t *format,
                               hwc2_display_t *out_display_id);
  int32_t DestroyLayer(hwc2_display_t display, hwc2_layer_t layer);
  int32_t DestroyVirtualDisplay(hwc2_display_t display);
  int32_t PresentDisplay(hwc2_display_t display, int32_t *out_retire_fence);
  void RegisterCallback(int32_t descriptor, hwc2_callback_data_t callback_data,
                        hwc2_function_pointer_t pointer);
  int32_t SetOutputBuffer(hwc2_display_t display, buffer_handle_t buffer, int32_t releaseFence);
  int32_t SetPowerMode(hwc2_display_t display, int32_t int_mode);
  int32_t ValidateDisplay(hwc2_display_t display, uint32_t *out_num_types,
                          uint32_t *out_num_requests);
  int32_t SetColorMode(hwc2_display_t display, int32_t /*ColorMode*/ int_mode);
  int32_t SetColorModeWithRenderIntent(hwc2_display_t display, int32_t /*ColorMode*/ int_mode,
                                       int32_t /*RenderIntent*/ int_render_intent);
  int32_t SetColorTransform(hwc2_display_t display, const float *matrix,
                            int32_t /*android_color_transform_t*/ hint);
  int32_t GetReadbackBufferAttributes(hwc2_display_t display,
                                      int32_t *format, int32_t *dataspace);
  int32_t SetReadbackBuffer(hwc2_display_t display, const native_handle_t *buffer,
                            int32_t acquire_fence);
  int32_t GetReadbackBufferFence(hwc2_display_t display, int32_t *release_fence);
  uint32_t GetMaxVirtualDisplayCount();
  int32_t GetDisplayIdentificationData(hwc2_display_t display, uint8_t *outPort,
                                       uint32_t *outDataSize, uint8_t *outData);
  int32_t GetDisplayCapabilities(hwc2_display_t display, uint32_t *outNumCapabilities,
                                 uint32_t *outCapabilities);
  int32_t GetDisplayBrightnessSupport(hwc2_display_t display, bool *outSupport);
  int32_t SetDisplayBrightness(hwc2_display_t display, float brightness);
  void WaitForResources(bool wait_for_resources, hwc2_display_t active_builtin_id,
                        hwc2_display_t display_id);

  // newly added
  int32_t GetDisplayType(hwc2_display_t display, int32_t *out_type);
  int32_t GetDisplayAttribute(hwc2_display_t display, hwc2_config_t config,
                              int32_t int_attribute, int32_t *out_value);
  int32_t GetActiveConfig(hwc2_display_t display, hwc2_config_t *out_config);
  int32_t GetColorModes(hwc2_display_t display, uint32_t *out_num_modes,
                        int32_t /*ColorMode*/ *int_out_modes);
  int32_t GetRenderIntents(hwc2_display_t display, int32_t /*ColorMode*/ int_mode,
                           uint32_t *out_num_intents, int32_t /*RenderIntent*/ *int_out_intents);
  int32_t GetHdrCapabilities(hwc2_display_t display, uint32_t* out_num_types, int32_t* out_types,
                             float* out_max_luminance, float* out_max_average_luminance,
                             float* out_min_luminance);
  int32_t GetPerFrameMetadataKeys(hwc2_display_t display, uint32_t *out_num_keys,
                                  int32_t *int_out_keys);
  int32_t GetClientTargetSupport(hwc2_display_t display, uint32_t width, uint32_t height,
                                 int32_t format, int32_t dataspace);
  int32_t GetDisplayName(hwc2_display_t display, uint32_t *out_size, char *out_name);
  int32_t SetActiveConfig(hwc2_display_t display, hwc2_config_t config);
  int32_t GetChangedCompositionTypes(hwc2_display_t display, uint32_t *out_num_elements,
                                     hwc2_layer_t *out_layers, int32_t *out_types);
  int32_t GetDisplayRequests(hwc2_display_t display, int32_t *out_display_requests,
                             uint32_t *out_num_elements, hwc2_layer_t *out_layers,
                             int32_t *out_layer_requests);
  int32_t GetReleaseFences(hwc2_display_t display, uint32_t *out_num_elements,
                           hwc2_layer_t *out_layers, int32_t *out_fences);
  int32_t SetClientTarget(hwc2_display_t display, buffer_handle_t target, int32_t acquire_fence,
                          int32_t dataspace, hwc_region_t damage);
  int32_t SetCursorPosition(hwc2_display_t display, hwc2_layer_t layer, int32_t x, int32_t y);
  int32_t GetDataspaceSaturationMatrix(int32_t /*Dataspace*/ int_dataspace, float *out_matrix);
  int32_t SetDisplayBrightnessScale(const android::Parcel *input_parcel);

  // Layer functions
  int32_t SetLayerBuffer(hwc2_display_t display, hwc2_layer_t layer, buffer_handle_t buffer,
                         int32_t acquire_fence);
  int32_t SetLayerBlendMode(hwc2_display_t display, hwc2_layer_t layer, int32_t int_mode);
  int32_t SetLayerDisplayFrame(hwc2_display_t display, hwc2_layer_t layer, hwc_rect_t frame);
  int32_t SetLayerPlaneAlpha(hwc2_display_t display, hwc2_layer_t layer, float alpha);
  int32_t SetLayerSourceCrop(hwc2_display_t display, hwc2_layer_t layer, hwc_frect_t crop);
  int32_t SetLayerTransform(hwc2_display_t display, hwc2_layer_t layer, int32_t int_transform);
  int32_t SetLayerZOrder(hwc2_display_t display, hwc2_layer_t layer, uint32_t z);
  int32_t SetLayerType(hwc2_display_t display, hwc2_layer_t layer,
                       IQtiComposerClient::LayerType type);
  int32_t SetLayerSurfaceDamage(hwc2_display_t display, hwc2_layer_t layer, hwc_region_t damage);
  int32_t SetLayerVisibleRegion(hwc2_display_t display, hwc2_layer_t layer, hwc_region_t damage);
  int32_t SetLayerCompositionType(hwc2_display_t display, hwc2_layer_t layer, int32_t int_type);
  int32_t SetLayerColor(hwc2_display_t display, hwc2_layer_t layer, hwc_color_t color);
  int32_t SetLayerDataspace(hwc2_display_t display, hwc2_layer_t layer, int32_t dataspace);
  int32_t SetLayerPerFrameMetadata(hwc2_display_t display, hwc2_layer_t layer,
                                   uint32_t num_elements, const int32_t *int_keys,
                                   const float *metadata);
  int32_t SetLayerColorTransform(hwc2_display_t display, hwc2_layer_t layer, const float *matrix);
  int32_t SetLayerPerFrameMetadataBlobs(hwc2_display_t display, hwc2_layer_t layer,
                                        uint32_t num_elements, const int32_t *int_keys,
                                        const uint32_t *sizes, const uint8_t *metadata);
  int32_t SetDisplayedContentSamplingEnabled(hwc2_display_t display, int32_t enabled,
                                             uint8_t component_mask, uint64_t max_frames);
  int32_t GetDisplayedContentSamplingAttributes(hwc2_display_t display, int32_t *format,
                                                int32_t *dataspace, uint8_t *supported_components);
  int32_t GetDisplayedContentSample(hwc2_display_t display, uint64_t max_frames, uint64_t timestamp,
                                    uint64_t *numFrames,
                                    int32_t samples_size[NUM_HISTOGRAM_COLOR_COMPONENTS],
                                    uint64_t *samples[NUM_HISTOGRAM_COLOR_COMPONENTS]);
  int32_t SetDisplayElapseTime(hwc2_display_t display, uint64_t time);

  // HWCDisplayEventHandler
  virtual void DisplayPowerReset();

  int32_t SetVsyncEnabled(hwc2_display_t display, int32_t int_enabled);
  int32_t GetDozeSupport(hwc2_display_t display, int32_t *out_support);
  int32_t GetDisplayConfigs(hwc2_display_t display, uint32_t *out_num_configs,
                            hwc2_config_t *out_configs);
  int GetVsyncPeriod(int disp);
  void Refresh(hwc2_display_t display);

  static Locker locker_[HWCCallbacks::kNumDisplays];
  static Locker power_state_[HWCCallbacks::kNumDisplays];
  static Locker hdr_locker_[HWCCallbacks::kNumDisplays];
  static Locker display_config_locker_;

 private:
#ifndef DISPLAY_CONFIG_VERSION_OPTIMAL
  class CWB {
   public:
    explicit CWB(HWCSession *hwc_session) : hwc_session_(hwc_session) { }
    void PresentDisplayDone(hwc2_display_t disp_id);

    int32_t PostBuffer(const sp<IDisplayCWBCallback> &callback, bool post_processed,
                       const hidl_handle& buffer);

   private:
    struct QueueNode {
      QueueNode(const sp<IDisplayCWBCallback> &cb, bool pp, const hidl_handle& buf)
        : callback(cb), post_processed(pp), buffer(buf) { }

      const android::sp<IDisplayCWBCallback> callback;
      bool post_processed = false;
      const hidl_handle buffer;
    };

    void ProcessRequests();
    static void AsyncTask(CWB *cwb);

    std::queue<QueueNode *> queue_;

    std::future<void> future_;
    Locker queue_lock_;
    std::mutex mutex_;
    std::condition_variable cv_;
    HWCSession *hwc_session_ = nullptr;
  };
#endif

  struct DisplayMapInfo {
    hwc2_display_t client_id = HWCCallbacks::kNumDisplays;        // mapped sf id for this display
    int32_t sdm_id = -1;                                         // sdm id for this display
    sdm:: DisplayType disp_type = kDisplayTypeMax;              // sdm display type
    bool test_pattern = false;                                 // display will show test pattern
    void Reset() {
      // Do not clear client id
      sdm_id = -1;
      disp_type = kDisplayTypeMax;
      test_pattern = false;
    }
  };

  static const int kExternalConnectionTimeoutMs = 500;
  static const int kCommitDoneTimeoutMs = 100;
  uint32_t throttling_refresh_rate_ = 60;
  std::mutex hotplug_mutex_;
  std::condition_variable hotplug_cv_;
  void UpdateThrottlingRate();
  void SetNewThrottlingRate(uint32_t new_rate);

  void ResetPanel();
  void InitSupportedDisplaySlots();
  void InitSupportedNullDisplaySlots();
  int GetDisplayIndex(int dpy);
  int CreatePrimaryDisplay();
  void CreateDummyDisplay(hwc2_display_t client_id);
  int HandleBuiltInDisplays();
  int HandlePluggableDisplays(bool delay_hotplug);
  int HandleConnectedDisplays(HWDisplaysInfo *hw_displays_info, bool delay_hotplug);
  int HandleDisconnectedDisplays(HWDisplaysInfo *hw_displays_info);
  void DestroyDisplay(DisplayMapInfo *map_info);
  void DestroyPluggableDisplay(DisplayMapInfo *map_info);
  void DestroyNonPluggableDisplay(DisplayMapInfo *map_info);
  int32_t GetConfigCount(int disp_id, uint32_t *count);
  int32_t GetActiveConfigIndex(int disp_id, uint32_t *config);
  int32_t SetActiveConfigIndex(int disp_id, uint32_t config);
  int32_t ControlPartialUpdate(int dpy, bool enable);
  int32_t DisplayBWTransactionPending(bool *status);
  int32_t SetSecondaryDisplayStatus(int disp_id, HWCDisplay::DisplayStatus status);
  int32_t MinHdcpEncryptionLevelChanged(int disp_id, uint32_t min_enc_level);
  int32_t IsWbUbwcSupported(int *value);
  int32_t SetDynamicDSIClock(int64_t disp_id, uint32_t bitrate);
  int32_t getDisplayBrightness(uint32_t display, float *brightness);
  int32_t setDisplayBrightness(uint32_t display, float brightness);
  int32_t getDisplayMaxBrightness(uint32_t display, uint32_t *max_brightness_level);
  bool HasHDRSupport(HWCDisplay *hwc_display);
  void PostInit();

  // Uevent handler
  virtual void UEventHandler(const char *uevent_data, int length);

  // service methods
  void StartServices();

  // Methods from ::android::hardware::display::config::V1_0::IDisplayConfig follow.
  Return<void> isDisplayConnected(IDisplayConfig::DisplayType dpy,
                                  isDisplayConnected_cb _hidl_cb) override;
  Return<int32_t> setSecondayDisplayStatus(IDisplayConfig::DisplayType dpy,
                                           IDisplayConfig::DisplayExternalStatus status) override;
  Return<int32_t> configureDynRefeshRate(IDisplayConfig::DisplayDynRefreshRateOp op,
                                         uint32_t refreshRate) override;
  Return<void> getConfigCount(IDisplayConfig::DisplayType dpy,
                              getConfigCount_cb _hidl_cb) override;
  Return<void> getActiveConfig(IDisplayConfig::DisplayType dpy,
                               getActiveConfig_cb _hidl_cb) override;
  Return<int32_t> setActiveConfig(IDisplayConfig::DisplayType dpy, uint32_t config) override;
  Return<void> getDisplayAttributes(uint32_t configIndex, IDisplayConfig::DisplayType dpy,
                                    getDisplayAttributes_cb _hidl_cb) override;
  Return<int32_t> setPanelBrightness(uint32_t level) override;
  Return<void> getPanelBrightness(getPanelBrightness_cb _hidl_cb) override;
  Return<int32_t> minHdcpEncryptionLevelChanged(IDisplayConfig::DisplayType dpy,
                                                uint32_t min_enc_level) override;
  Return<int32_t> refreshScreen() override;
  Return<int32_t> controlPartialUpdate(IDisplayConfig::DisplayType dpy, bool enable) override;
  Return<int32_t> toggleScreenUpdate(bool on) override;
  Return<int32_t> setIdleTimeout(uint32_t value) override;
  Return<void> getHDRCapabilities(IDisplayConfig::DisplayType dpy,
                                  getHDRCapabilities_cb _hidl_cb) override;
  Return<int32_t> setCameraLaunchStatus(uint32_t on) override;
  Return<void> displayBWTransactionPending(displayBWTransactionPending_cb _hidl_cb) override;
  Return<int32_t> IdlePowerCollapse(bool enable, bool synchronous);

#ifndef DISPLAY_CONFIG_VERSION_OPTIMAL
  Return<int32_t> setDisplayAnimating(uint64_t display_id, bool animating) override;
  Return<int32_t> setDisplayIndex(IDisplayConfig::DisplayTypeExt disp_type,
                                  uint32_t base, uint32_t count) override;
  Return<int32_t> controlIdlePowerCollapse(bool enable, bool synchronous) override;
  Return<void> getWriteBackCapabilities(getWriteBackCapabilities_cb _hidl_cb) override;
  Return<int32_t> SetDisplayDppsAdROI(uint32_t dispaly_id, uint32_t h_start, uint32_t h_end,
                                      uint32_t v_start, uint32_t v_end, uint32_t factor_in,
                                      uint32_t factor_out) override;
  Return<int32_t> updateVSyncSourceOnPowerModeOff() override;
  Return<int32_t> updateVSyncSourceOnPowerModeDoze() override;
  Return<int32_t> setPowerMode(uint32_t disp_id, PowerMode power_mode) override;
  Return<bool> isPowerModeOverrideSupported(uint32_t disp_id) override;
  Return<bool> isHDRSupported(uint32_t disp_id) override;
  Return<bool> isWCGSupported(uint32_t disp_id) override;
  Return<int32_t> setLayerAsMask(uint32_t disp_id, uint64_t layer_id) override;
  Return<void> getDebugProperty(const hidl_string &prop_name,
                                getDebugProperty_cb _hidl_cb) override;
  Return<void> getActiveBuiltinDisplayAttributes(getDisplayAttributes_cb _hidl_cb) override;
  Return<int32_t> setPanelLuminanceAttributes(uint32_t disp_id, float min_lum,
                                              float max_lum) override;
  Return<bool> isBuiltInDisplay(uint32_t disp_id) override;
  Return<void> getSupportedDSIBitClks(uint32_t disp_id,
                                      getSupportedDSIBitClks_cb _hidl_cb) override;
  Return<uint64_t> getDSIClk(uint32_t disp_id) override;
  Return<int32_t> setDSIClk(uint32_t disp_id, uint64_t bit_clk) override;
  Return<int32_t> setCWBOutputBuffer(const sp<IDisplayCWBCallback> &callback,
                                     uint32_t disp_id, const Rect &rect, bool post_processed,
                                     const hidl_handle& buffer) override;
  Return<int32_t> setQsyncMode(uint32_t disp_id, IDisplayConfig::QsyncMode mode) override;
  Return<bool> isSmartPanelConfig(uint32_t disp_id, uint32_t config_id) override;
  Return<bool> isAsyncVDSCreationSupported() override;
  Return<int32_t> createVirtualDisplay(uint32_t width, uint32_t height, int32_t format) override;
  Return<bool> isRotatorSupportedFormat(int hal_format, bool ubwc) override;
  Return<int32_t> registerQsyncCallback(const sp<IDisplayQsyncCallback> &callback) override;
#endif

  // QClient methods
  virtual android::status_t notifyCallback(uint32_t command, const android::Parcel *input_parcel,
                                           android::Parcel *output_parcel);
  void DynamicDebug(const android::Parcel *input_parcel);
  android::status_t SetFrameDumpConfig(const android::Parcel *input_parcel);
  android::status_t SetMaxMixerStages(const android::Parcel *input_parcel);
  android::status_t SetDisplayMode(const android::Parcel *input_parcel);
  android::status_t ConfigureRefreshRate(const android::Parcel *input_parcel);
  android::status_t QdcmCMDHandler(const android::Parcel *input_parcel,
                                   android::Parcel *output_parcel);
  android::status_t QdcmCMDDispatch(uint32_t display_id,
                                    const PPDisplayAPIPayload &req_payload,
                                    PPDisplayAPIPayload *resp_payload,
                                    PPPendingParams *pending_action);
  android::status_t GetDisplayAttributesForConfig(const android::Parcel *input_parcel,
                                                  android::Parcel *output_parcel);
  android::status_t GetVisibleDisplayRect(const android::Parcel *input_parcel,
                                          android::Parcel *output_parcel);
  android::status_t SetMixerResolution(const android::Parcel *input_parcel);
  android::status_t SetColorModeOverride(const android::Parcel *input_parcel);
  android::status_t SetColorModeWithRenderIntentOverride(const android::Parcel *input_parcel);

  android::status_t SetColorModeById(const android::Parcel *input_parcel);
  android::status_t SetColorModeFromClient(const android::Parcel *input_parcel);
  android::status_t getComposerStatus();
  android::status_t SetQSyncMode(const android::Parcel *input_parcel);
  android::status_t SetIdlePC(const android::Parcel *input_parcel);
  android::status_t RefreshScreen(const android::Parcel *input_parcel);
  android::status_t SetAd4RoiConfig(const android::Parcel *input_parcel);
  android::status_t SetDsiClk(const android::Parcel *input_parcel);
  android::status_t GetDsiClk(const android::Parcel *input_parcel, android::Parcel *output_parcel);
  android::status_t GetSupportedDsiClk(const android::Parcel *input_parcel,
                                       android::Parcel *output_parcel);
  android::status_t SetFrameTriggerMode(const android::Parcel *input_parcel);
  android::status_t SetPanelLuminanceAttributes(const android::Parcel *input_parcel);
  android::status_t setColorSamplingEnabled(const android::Parcel *input_parcel);

  // Internal methods
  HWC2::Error ValidateDisplayInternal(hwc2_display_t display, uint32_t *out_num_types,
                                      uint32_t *out_num_requests);
  HWC2::Error PresentDisplayInternal(hwc2_display_t display, int32_t *out_retire_fence);
  void HandleSecureSession();
  void HandlePendingPowerMode(hwc2_display_t display, int retire_fence);
  void HandlePendingHotplug(hwc2_display_t disp_id, int retire_fence);
  bool IsPluggableDisplayConnected();
  hwc2_display_t GetActiveBuiltinDisplay();
  void HandlePendingRefresh();
  void NotifyClientStatus(bool connected);
  int32_t GetVirtualDisplayId();
  void PerformQsyncCallback(hwc2_display_t display);

  CoreInterface *core_intf_ = nullptr;
  HWCDisplay *hwc_display_[HWCCallbacks::kNumDisplays] = {nullptr};
  HWCCallbacks callbacks_;
  HWCBufferAllocator buffer_allocator_;
  HWCBufferSyncHandler buffer_sync_handler_;
  HWCVirtualDisplayFactory virtual_display_factory_;
  HWCColorManager *color_mgr_ = nullptr;
  DisplayMapInfo map_info_primary_;                 // Primary display (either builtin or pluggable)
  std::vector<DisplayMapInfo> map_info_builtin_;    // Builtin displays excluding primary
  std::vector<DisplayMapInfo> map_info_pluggable_;  // Pluggable displays excluding primary
  std::vector<DisplayMapInfo> map_info_virtual_;    // Virtual displays
  bool update_vsync_on_power_off_ = false;
  bool update_vsync_on_doze_ = false;
  std::vector<bool> is_hdr_display_;    // info on HDR supported
  std::map <hwc2_display_t, hwc2_display_t> map_hwc_display_;  // Real and dummy display pairs.
  bool reset_panel_ = false;
  bool client_connected_ = false;
  bool new_bw_mode_ = false;
  bool need_invalidate_ = false;
  int bw_mode_release_fd_ = -1;
  qService::QService *qservice_ = nullptr;
  HWCSocketHandler socket_handler_;
  bool hdmi_is_primary_ = false;
  bool is_composer_up_ = false;
  std::mutex mutex_lum_;
  int hpd_bpp_ = 0;
  int hpd_pattern_ = 0;
  static bool pending_power_mode_[HWCCallbacks::kNumDisplays];
  static int null_display_mode_;
  HotPlugEvent pending_hotplug_event_ = kHotPlugNone;
  hwc2_display_t virtual_id_ = HWCCallbacks::kNumDisplays;
  Locker pluggable_handler_lock_;
  bool destroy_virtual_disp_pending_ = false;
  uint32_t idle_pc_ref_cnt_ = 0;
  int32_t disable_hotplug_bwcheck_ = 0;
  int32_t disable_mask_layer_hint_ = 0;
  float set_max_lum_ = -1.0;
  float set_min_lum_ = -1.0;
  std::bitset<HWCCallbacks::kNumDisplays> pending_refresh_;
#ifndef DISPLAY_CONFIG_VERSION_OPTIMAL
  CWB cwb_;
  android::sp<IDisplayQsyncCallback> qsync_callback_ = nullptr;
#endif
  bool async_powermode_ = false;
  bool async_vds_creation_ = false;
  bool power_state_transition_[HWCCallbacks::kNumDisplays] = {};
  std::bitset<HWCCallbacks::kNumDisplays> display_ready_;
  bool secure_session_active_ = false;
};
}  // namespace sdm

#endif  // __HWC_SESSION_H__
