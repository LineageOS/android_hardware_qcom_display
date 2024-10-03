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
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef __HWC_SESSION_H__
#define __HWC_SESSION_H__

#include <aidl/android/hardware/graphics/composer3/BnComposerClient.h>
#include <aidl/android/hardware/graphics/composer3/IComposer.h>
#include <config/device_interface.h>
#include <aidl/vendor/qti/hardware/display/config/BnDisplayConfig.h>
#include <aidl/vendor/qti/hardware/display/config/BnDisplayConfigCallback.h>
#include <binder/Status.h>

#include <core/core_interface.h>
#include <core/ipc_interface.h>
#include <utils/locker.h>
#include <utils/constants.h>
#include <display_config.h>
#include <vector>
#include <queue>
#include <utility>
#include <future>  // NOLINT
#include <map>
#include <unordered_map>
#include <string>
#include <memory>
#include <atomic>
#include <core/display_interface.h>

#include "hwc_callbacks.h"
#include "hwc_layers.h"
#include "hwc_display.h"
#include "hwc_display_builtin.h"
#include "hwc_display_pluggable.h"
#include "hwc_display_virtual.h"
#include "hwc_display_pluggable_test.h"
#include "hwc_color_manager.h"
#include "hwc_socket_handler.h"
#include "hwc_display_event_handler.h"
#include "hwc_buffer_sync_handler.h"
#include "hwc_display_virtual_factory.h"

using ::android::sp;
using android::hardware::hidl_handle;
using ::android::hardware::hidl_string;
using ::android::hardware::hidl_vec;
using ::android::hardware::Return;
using ::android::hardware::Void;
namespace composer_V3 = aidl::android::hardware::graphics::composer3;
using HwcDisplayCapability = composer_V3::DisplayCapability;
using HwcDisplayConnectionType = composer_V3::DisplayConnectionType;
using DisplayConfiguration = composer_V3::DisplayConfiguration;
using HwcClientTargetProperty = composer_V3::ClientTargetProperty;
using ::aidl::vendor::qti::hardware::display::config::Attributes;
using ::aidl::vendor::qti::hardware::display::config::CameraSmoothOp;
using ::aidl::vendor::qti::hardware::display::config::DisplayPortType;
using ::aidl::vendor::qti::hardware::display::config::IDisplayConfig;
using ::aidl::vendor::qti::hardware::display::config::IDisplayConfigCallback;
using ::aidl::vendor::qti::hardware::display::config::TUIEventType;

namespace aidl::vendor::qti::hardware::display::config {
class DisplayConfigAIDL;
}

namespace sdm {

using composer_V3::IComposerClient;

int32_t GetDataspaceFromColorMode(ColorMode mode);

typedef DisplayConfig::DisplayType DispType;

constexpr int32_t kDataspaceSaturationMatrixCount = 16;
constexpr int32_t kDataspaceSaturationPropertyElements = 9;
constexpr int32_t kPropertyMax = 256;

class HWCUEvent {
 public:
  virtual ~HWCUEvent() {}
  // hpd handling
  virtual void HpdInit() = 0;
  virtual void HpdDeinit() = 0;
  virtual void ParseUEvent(char *uevent_data, int length) = 0;

  std::mutex hpd_mutex_;
  std::condition_variable hpd_cv_;

  // top thread needs to be detached as we can't control
  // to wake it up to terminate it.
  virtual void HpdThreadTop() = 0;

 protected:
  int hpd_connected_ = -1;
  int hpd_bpp_ = 0;
  int hpd_pattern_ = 0;
  bool hpd_thread_should_terminate_ = false;
  std::atomic<int> uevent_counter_ = 0;
};

class HWCSession : public HWCUEvent,
                   public qClient::BnQClient,
                   public HWCDisplayEventHandler,
                   public DisplayConfig::ClientContext {
  friend class aidl::vendor::qti::hardware::display::config::DisplayConfigAIDL;

 public:
  enum HotPlugEvent {
    kHotPlugNone,
    kHotPlugEvent,
    kHotPlugProcessing,
  };

  enum ClientCommitDone {
    kClientPartialUpdate,
    kClientIdlepowerCollapse,
    kClientTeardownCWB,
    kClientTrustedUI,
    kClientMax
  };

  enum CwbConfigFlag {
    kCwbFlagPuAsCwbROI,
    kCwbFlagAvoidRefresh,
  };

  HWCSession();
  int Init();
  int Deinit();
  HWC3::Error CreateVirtualDisplayObj(uint32_t width, uint32_t height, int32_t *format,
                                      Display *out_display_id);

  template <typename... Args>
  HWC3::Error CallDisplayFunction(Display display, HWC3::Error (HWCDisplay::*member)(Args...),
                                  Args... args) {
    if (display >= HWCCallbacks::kNumDisplays) {
      return HWC3::Error::BadDisplay;
    }

    SCOPE_LOCK(locker_[display]);
    auto status = HWC3::Error::BadDisplay;
    if (hwc_display_[display]) {
      auto hwc_display = hwc_display_[display];
      status = (hwc_display->*member)(std::forward<Args>(args)...);
    }
    return status;
  }

  template <typename... Args>
  HWC3::Error CallLayerFunction(Display display, LayerId layer,
                                HWC3::Error (HWCLayer::*member)(Args...), Args... args) {
    if (display >= HWCCallbacks::kNumDisplays) {
      return HWC3::Error::BadDisplay;
    }

    SCOPE_LOCK(locker_[display]);
    auto status = HWC3::Error::BadDisplay;
    if (hwc_display_[display]) {
      status = HWC3::Error::BadLayer;
      auto hwc_layer = hwc_display_[display]->GetHWCLayer(layer);
      if (hwc_layer != nullptr) {
        status = (hwc_layer->*member)(std::forward<Args>(args)...);
      }
    }
    return status;
  }

  // HWC3 Functions that require a concrete implementation in hwc session
  // and hence need to be member functions
  static HWCSession *GetInstance();
  void GetCapabilities(uint32_t *outCount, int32_t *outCapabilities);
  void Dump(uint32_t *out_size, char *out_buffer);

  HWC3::Error AcceptDisplayChanges(Display display);
  HWC3::Error CreateLayer(Display display, LayerId *out_layer_id);
  HWC3::Error CreateVirtualDisplay(uint32_t width, uint32_t height, int32_t *format,
                                   Display *out_display_id);
  HWC3::Error DestroyLayer(Display display, LayerId layer);
  HWC3::Error DestroyVirtualDisplay(Display display);
  HWC3::Error PresentDisplay(Display display, shared_ptr<Fence> *out_retire_fence);
  void RegisterCallback(CallbackCommand descriptor, void *callback_data, void *callback_fn);
  HWC3::Error SetOutputBuffer(Display display, buffer_handle_t buffer,
                              const shared_ptr<Fence> &release_fence);
  HWC3::Error SetPowerMode(Display display, int32_t int_mode);
  HWC3::Error SetColorMode(Display display, int32_t /*ColorMode*/ int_mode);
  HWC3::Error SetColorModeWithRenderIntent(Display display, int32_t /*ColorMode*/ int_mode,
                                           int32_t /*RenderIntent*/ int_render_intent);
  HWC3::Error SetColorTransform(Display display, const std::vector<float> &matrix);
  HWC3::Error getDisplayDecorationSupport(Display display, PixelFormat_V3 *format,
                                          AlphaInterpretation *alpha);
  HWC3::Error GetReadbackBufferAttributes(Display display, int32_t *format, int32_t *dataspace);
  HWC3::Error SetReadbackBuffer(Display display, const native_handle_t *buffer,
                                const shared_ptr<Fence> &acquire_fence);
  HWC3::Error GetReadbackBufferFence(Display display, shared_ptr<Fence> *release_fence);
  uint32_t GetMaxVirtualDisplayCount();
  HWC3::Error GetDisplayIdentificationData(Display display, uint8_t *outPort, uint32_t *outDataSize,
                                           uint8_t *outData);
  HWC3::Error GetDisplayCapabilities(Display display, hidl_vec<HwcDisplayCapability> *capabilities);
  HWC3::Error GetDisplayBrightnessSupport(Display display, bool *outSupport);
  HWC3::Error SetDisplayBrightness(Display display, float brightness);
  HWC3::Error WaitForResources(bool wait_for_resources, Display active_builtin_id,
                               Display display_id);

  // newly added
  HWC3::Error GetDisplayType(Display display, int32_t *out_type);
  HWC3::Error GetDisplayAttribute(Display display, Config config, HwcAttribute attribute,
                                  int32_t *out_value);
  HWC3::Error GetDisplayConfigurations(Display display,
                                       std::vector<DisplayConfiguration> *out_configs);
  HWC3::Error GetActiveConfig(Display display, Config *out_config);
  HWC3::Error GetColorModes(Display display, uint32_t *out_num_modes,
                            int32_t /*ColorMode*/ *int_out_modes);
  HWC3::Error GetRenderIntents(Display display, int32_t /*ColorMode*/ int_mode,
                               uint32_t *out_num_intents,
                               int32_t /*RenderIntent*/ *int_out_intents);
  HWC3::Error GetHdrCapabilities(Display display, uint32_t *out_num_types, int32_t *out_types,
                                 float *out_max_luminance, float *out_max_average_luminance,
                                 float *out_min_luminance);
  HWC3::Error GetPerFrameMetadataKeys(Display display, uint32_t *out_num_keys,
                                      int32_t *int_out_keys);
  HWC3::Error GetClientTargetSupport(Display display, uint32_t width, uint32_t height,
                                     int32_t format, int32_t dataspace);
  HWC3::Error GetDisplayName(Display display, uint32_t *out_size, char *out_name);
  HWC3::Error SetActiveConfig(Display display, Config config);
  HWC3::Error GetChangedCompositionTypes(Display display, uint32_t *out_num_elements,
                                         LayerId *out_layers, int32_t *out_types);
  HWC3::Error GetDisplayRequests(Display display, int32_t *out_display_requests,
                                 uint32_t *out_num_elements, LayerId *out_layers,
                                 int32_t *out_layer_requests);
  HWC3::Error GetReleaseFences(Display display, uint32_t *out_num_elements, LayerId *out_layers,
                               std::vector<shared_ptr<Fence>> *out_fences);
  HWC3::Error SetClientTarget(Display display, buffer_handle_t target,
                              shared_ptr<Fence> acquire_fence, int32_t dataspace, Region damage);
  HWC3::Error SetClientTarget_3_1(Display display, buffer_handle_t target,
                                  shared_ptr<Fence> acquire_fence, int32_t dataspace,
                                  Region damage);
  HWC3::Error SetCursorPosition(Display display, LayerId layer, int32_t x, int32_t y);
  HWC3::Error GetDataspaceSaturationMatrix(int32_t /*Dataspace*/ int_dataspace, float *out_matrix);
  HWC3::Error SetDisplayBrightnessScale(const android::Parcel *input_parcel);
  HWC3::Error GetDisplayConnectionType(Display display, HwcDisplayConnectionType *type);
  HWC3::Error SetDimmingEnable(Display display, int32_t int_enabled);
  HWC3::Error SetDimmingMinBl(Display display, int32_t min_bl);
  HWC3::Error GetClientTargetProperty(Display display,
                                      HwcClientTargetProperty *outClientTargetProperty);
  HWC3::Error SetDemuraState(Display display, int32_t state);
  HWC3::Error SetDemuraConfig(Display display, int32_t demura_idx);

  // Layer functions
  HWC3::Error SetLayerBuffer(Display display, LayerId layer, buffer_handle_t buffer,
                             const shared_ptr<Fence> &acquire_fence);
  HWC3::Error SetLayerBlendMode(Display display, LayerId layer, int32_t int_mode);
  HWC3::Error SetLayerDisplayFrame(Display display, LayerId layer, Rect frame);
  HWC3::Error SetLayerPlaneAlpha(Display display, LayerId layer, float alpha);
  HWC3::Error SetLayerSourceCrop(Display display, LayerId layer, FRect crop);
  HWC3::Error SetLayerTransform(Display display, LayerId layer, Transform transform);
  HWC3::Error SetLayerZOrder(Display display, LayerId layer, uint32_t z);
  HWC3::Error SetLayerType(Display display, LayerId layer, LayerType type);
  HWC3::Error SetLayerFlag(Display display, LayerId layer, LayerFlag flag);
  HWC3::Error SetLayerSurfaceDamage(Display display, LayerId layer, Region damage);
  HWC3::Error SetLayerVisibleRegion(Display display, LayerId layer, Region damage);
  HWC3::Error SetLayerCompositionType(Display display, LayerId layer, int32_t int_type);
  HWC3::Error SetLayerColor(Display display, LayerId layer, Color color);
  HWC3::Error SetLayerDataspace(Display display, LayerId layer, int32_t dataspace);
  HWC3::Error SetLayerPerFrameMetadata(Display display, LayerId layer, uint32_t num_elements,
                                       const int32_t *int_keys, const float *metadata);
  HWC3::Error SetLayerColorTransform(Display display, LayerId layer, const float *matrix);
  HWC3::Error SetLayerPerFrameMetadataBlobs(Display display, LayerId layer, uint32_t num_elements,
                                            const int32_t *int_keys, const uint32_t *sizes,
                                            const uint8_t *metadata);
  HWC3::Error SetLayerBrightness(Display display, LayerId layer, float brightness);
  HWC3::Error SetDisplayedContentSamplingEnabled(Display display, bool enabled,
                                                 uint8_t component_mask, uint64_t max_frames);
  HWC3::Error GetDisplayedContentSamplingAttributes(Display display, int32_t *format,
                                                    int32_t *dataspace,
                                                    uint8_t *supported_components);
  HWC3::Error GetDisplayedContentSample(Display display, uint64_t max_frames, uint64_t timestamp,
                                        uint64_t *numFrames,
                                        int32_t samples_size[NUM_HISTOGRAM_COLOR_COMPONENTS],
                                        uint64_t *samples[NUM_HISTOGRAM_COLOR_COMPONENTS]);
  HWC3::Error SetDisplayElapseTime(Display display, uint64_t time);

  int SetCameraSmoothInfo(CameraSmoothOp op, int32_t fps);
  int RegisterCallbackClient(const std::shared_ptr<IDisplayConfigCallback> &callback,
                             int64_t *client_handle);
  int UnregisterCallbackClient(const int64_t client_handle);
  int NotifyResolutionChange(int32_t disp_id, Attributes &attr);
  int NotifyTUIEventDone(int disp_id, TUIEventType event_type);

  // HWCDisplayEventHandler
  virtual void DisplayPowerReset();
  virtual void PerformDisplayPowerReset();
  virtual void PerformQsyncCallback(Display display, bool qsync_enabled, uint32_t refresh_rate,
                                    uint32_t qsync_refresh_rate);
  virtual void VmReleaseDone(Display display);
  virtual int NotifyCwbDone(int dpy_index, int32_t status, uint64_t handle_id);
  virtual int NotifyIdleStatus(bool idle_status);

  HWC3::Error SetVsyncEnabled(Display display, bool enabled);
  HWC3::Error GetDozeSupport(Display display, int32_t *out_support);
  HWC3::Error GetDisplayConfigs(Display display, uint32_t *out_num_configs, Config *out_configs);
  HWC3::Error GetVsyncPeriod(Display disp, uint32_t *vsync_period);
  void Refresh(Display display);

  HWC3::Error GetDisplayVsyncPeriod(Display display, VsyncPeriodNanos *out_vsync_period);
  HWC3::Error SetActiveConfigWithConstraints(
      Display display, Config config,
      const VsyncPeriodChangeConstraints *vsync_period_change_constraints,
      VsyncPeriodChangeTimeline *out_timeline);
  HWC3::Error CommitOrPrepare(Display display, bool validate_only,
                              shared_ptr<Fence> *out_retire_fence, uint32_t *out_num_types,
                              uint32_t *out_num_requests, bool *needs_commit);
  HWC3::Error TryDrawMethod(Display display, DrawMethod drawMethod);
  HWC3::Error SetExpectedPresentTime(Display display, uint64_t expectedPresentTime);
  HWC3::Error GetOverlaySupport(OverlayProperties *supported_props);

  static Locker locker_[HWCCallbacks::kNumDisplays];
  static Locker hdr_locker_[HWCCallbacks::kNumDisplays];
  static Locker display_config_locker_;
  static std::mutex command_seq_mutex_;
  static std::bitset<kClientMax> clients_waiting_for_commit_[HWCCallbacks::kNumDisplays];
  static shared_ptr<Fence> retire_fence_[HWCCallbacks::kNumDisplays];
  static int commit_error_[HWCCallbacks::kNumDisplays];
  static Locker vm_release_locker_[HWCCallbacks::kNumDisplays];
  static std::bitset<HWCCallbacks::kNumDisplays> clients_waiting_for_vm_release_;
  static std::set<Display> active_displays_;

 private:
  class CWB {
   public:
    explicit CWB(HWCSession *hwc_session) : hwc_session_(hwc_session) {}

    int32_t PostBuffer(std::shared_ptr<IDisplayConfigCallback> callback,
                       const CwbConfig &cwb_config, const native_handle_t *buffer,
                       Display display_type, int dpy_index);
    int OnCWBDone(int dpy_index, int32_t status, uint64_t handle_id);

   private:
    enum CWBNotifiedStatus {
      kCwbNotifiedFailure = -1,
      kCwbNotifiedSuccess,
      kCwbNotifiedNone,
    };

    struct QueueNode {
      QueueNode(std::shared_ptr<IDisplayConfigCallback> cb, const CwbConfig &cwb_conf,
                const hidl_handle &buf, Display disp_type, uint64_t buf_id)
          : callback(cb),
            cwb_config(cwb_conf),
            buffer(buf),
            display_type(disp_type),
            handle_id(buf_id) {}

      std::shared_ptr<IDisplayConfigCallback> callback;
      CwbConfig cwb_config = {};
      const native_handle_t *buffer;
      Display display_type;
      uint64_t handle_id;
      CWBNotifiedStatus notified_status = kCwbNotifiedNone;
      bool request_completed = false;
    };

    struct DisplayCWBSession {
      std::deque<std::shared_ptr<QueueNode>> queue;
      std::mutex lock;
      std::condition_variable cv;
      std::future<void> future;
      bool async_thread_running = false;
    };

    static void AsyncTaskToProcessCWBStatus(CWB *cwb, int dpy_index);
    void ProcessCWBStatus(int dpy_index);
    void NotifyCWBStatus(int status, std::shared_ptr<QueueNode> cwb_node);

    std::map<int, DisplayCWBSession> display_cwb_session_map_;
    HWCSession *hwc_session_ = nullptr;
  };

  struct DisplayMapInfo {
    Display client_id = HWCCallbacks::kNumDisplays;  // mapped sf id for this display
    int32_t sdm_id = -1;                             // sdm id for this display
    sdm::DisplayType disp_type = kDisplayTypeMax;    // sdm display type
    bool test_pattern = false;                       // display will show test pattern
    void Reset() {
      // Do not clear client id
      sdm_id = -1;
      disp_type = kDisplayTypeMax;
      test_pattern = false;
    }
  };

  static const int kExternalConnectionTimeoutMs = 500;
  static const int kVmReleaseTimeoutMs = 100;
  static const int kCommitDoneTimeoutMs = 100;
  static const int kVmReleaseRetry = 3;
  static const int kDenomNstoMs = 1000000;
  static const int kNumDrawCycles = 3;

  uint32_t throttling_refresh_rate_ = 60;
  std::mutex hotplug_mutex_;
  std::condition_variable hotplug_cv_;
  bool resource_ready_ = false;
  Display active_display_id_ = 0;
  shared_ptr<Fence> cached_retire_fence_ = nullptr;
  void UpdateThrottlingRate();
  void SetNewThrottlingRate(uint32_t new_rate);

  void ResetPanel();
  void InitSupportedDisplaySlots();
  int GetDisplayIndex(int dpy);
  int CreatePrimaryDisplay();
  int HandleBuiltInDisplays();
  int HandlePluggableDisplays(bool delay_hotplug);
  int HandleConnectedDisplays(HWDisplaysInfo *hw_displays_info, bool delay_hotplug);
  int HandleDisconnectedDisplays(HWDisplaysInfo *hw_displays_info);
  void DestroyDisplay(DisplayMapInfo *map_info);
  void DestroyDisplayLocked(DisplayMapInfo *map_info);
  void DestroyPluggableDisplay(DisplayMapInfo *map_info);
  void DestroyPluggableDisplayLocked(DisplayMapInfo *map_info);
  void DestroyNonPluggableDisplay(DisplayMapInfo *map_info);
  void DestroyNonPluggableDisplayLocked(DisplayMapInfo *map_info);
  int GetConfigCount(int disp_id, uint32_t *count);
  int GetActiveConfigIndex(int disp_id, uint32_t *config);
  int SetActiveConfigIndex(int disp_id, uint32_t config);
  int SetNoisePlugInOverride(int32_t disp_id, bool override_en, int32_t attn, int32_t noise_zpos);
  int ControlPartialUpdate(int dpy, bool enable);
  int DisplayBWTransactionPending(bool *status);
  int SetDisplayStatus(int disp_id, HWCDisplay::DisplayStatus status);
  int MinHdcpEncryptionLevelChanged(int disp_id, uint32_t min_enc_level);
  int IsWbUbwcSupported(bool *value);
  int SetIdleTimeout(uint32_t value);
  int ToggleScreenUpdate(bool on);
  int SetCameraLaunchStatus(uint32_t on);
  int SetDisplayDppsAdROI(uint32_t display_id, uint32_t h_start, uint32_t h_end, uint32_t v_start,
                          uint32_t v_end, uint32_t factor_in, uint32_t factor_out);
  int ControlIdlePowerCollapse(bool enable, bool synchronous);
  int GetSupportedDisplayRefreshRates(int disp_id, std::vector<uint32_t> *supported_refresh_rates);
  HWC3::Error SetDynamicDSIClock(int64_t disp_id, uint32_t bitrate);
  int32_t getDisplayBrightness(uint32_t display, float *brightness);
  HWC3::Error setDisplayBrightness(uint32_t display, float brightness);
  int32_t getDisplayMaxBrightness(uint32_t display, uint32_t *max_brightness_level);
  bool HasHDRSupport(HWCDisplay *hwc_display);
  void PostInit();
  int GetDispTypeFromPhysicalId(uint64_t physical_disp_id, DispType *disp_type);
#ifdef PROFILE_COVERAGE_DATA
  android::status_t DumpCodeCoverage(const android::Parcel *input_parcel);
#endif
  DisplayError WaitForPrimaryHotplug(HWDisplayInterfaceInfo *hw_disp_info);
  void HandlePluggablePrimaryDisplay(HWDisplaysInfo *hw_displays_info);

  // Uevent handler
  virtual void UEventHandler();

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
  android::status_t QdcmCMDDispatch(uint32_t display_id, const PPDisplayAPIPayload &req_payload,
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
  android::status_t SetBppMode(const android::Parcel *input_parcel);
  android::status_t SetQSyncMode(const android::Parcel *input_parcel);
  android::status_t SetIdlePC(const android::Parcel *input_parcel);
  android::status_t RefreshScreen(const android::Parcel *input_parcel);
  android::status_t SetAd4RoiConfig(const android::Parcel *input_parcel);
  android::status_t SetJitterConfig(const android::Parcel *input_parcel);
  android::status_t SetDsiClk(const android::Parcel *input_parcel);
  android::status_t GetDsiClk(const android::Parcel *input_parcel, android::Parcel *output_parcel);
  android::status_t GetSupportedDsiClk(const android::Parcel *input_parcel,
                                       android::Parcel *output_parcel);
  android::status_t SetFrameTriggerMode(const android::Parcel *input_parcel);
  android::status_t SetPanelLuminanceAttributes(const android::Parcel *input_parcel);
  android::status_t setColorSamplingEnabled(const android::Parcel *input_parcel);
  android::status_t HandleTUITransition(int disp_id, int event);
  android::status_t TUIEventHandler(int disp_id, TUIEventType event_type);
  android::status_t GetDisplayPortId(uint32_t display, int *port_id);
  android::status_t UpdateTransferTime(const android::Parcel *input_parcel);
  android::status_t RetrieveDemuraTnFiles(const android::Parcel *input_parcel);

  // Internal methods
  void HandleSecureSession();
  void HandlePendingPowerMode(Display display, const shared_ptr<Fence> &retire_fence);
  void HandlePendingHotplug(Display disp_id, const shared_ptr<Fence> &retire_fence);
  bool IsPluggableDisplayConnected();
  bool IsVirtualDisplayConnected();
  Display GetActiveBuiltinDisplay();
  void HandlePendingRefresh();
  void NotifyClientStatus(bool connected);
  int32_t GetVirtualDisplayId(HWDisplayInfo &info);
  android::status_t TUITransitionPrepare(int disp_id);
  android::status_t TUITransitionStart(int disp_id);
  android::status_t TUITransitionEnd(int disp_id);
  android::status_t TUITransitionEndLocked(int disp_id);
  android::status_t TUITransitionUnPrepare(int disp_id);
  void PerformIdleStatusCallback(Display display);
  DispType GetDisplayConfigDisplayType(int qdutils_disp_type);
  HWC3::Error TeardownConcurrentWriteback(Display display);
  void PostCommitUnlocked(Display display, const shared_ptr<Fence> &retire_fence);
  void PostCommitLocked(Display display, shared_ptr<Fence> &retire_fence);
  int WaitForCommitDone(Display display, int client_id);
  int WaitForCommitDoneAsync(Display display, int client_id);
  void NotifyDisplayAttributes(Display display, Config config);
  int WaitForVmRelease(Display display, int timeout_ms);
  void GetVirtualDisplayList();
  bool IsHWDisplayConnected(Display client_id);
  int32_t ValidateFrameDumpConfig(uint32_t frame_dump_count, uint32_t bit_mask_disp_type,
                                  uint32_t bit_mask_layer_type);
  bool TeardownPluggableDisplays();
  int DisconnectPluggableDisplays(DisplayMapInfo &map_info);
  void RemoveDisconnectedPluggableDisplays();
  void AddGpuBasedVirtualDisplay(const HWDisplaysInfo *const hw_displays_info);

  CoreInterface *core_intf_ = nullptr;
  HWCDisplay *hwc_display_[HWCCallbacks::kNumDisplays] = {nullptr};
  QSyncMode hwc_display_qsync_[HWCCallbacks::kNumDisplays] = {QSyncMode::kQSyncModeNone};
  uint32_t idle_time_active_ms_ = 0;
  uint32_t idle_time_inactive_ms_ = 0;
  HWCCallbacks callbacks_;
  HWCBufferAllocator buffer_allocator_;
  HWCVirtualDisplayFactory virtual_display_factory_;
  HWCColorManager *color_mgr_ = nullptr;
  DisplayMapInfo map_info_primary_;                 // Primary display (either builtin or pluggable)
  std::vector<DisplayMapInfo> map_info_builtin_;    // Builtin displays excluding primary
  std::vector<DisplayMapInfo> map_info_pluggable_;  // Pluggable displays excluding primary
  std::vector<DisplayMapInfo> map_info_virtual_;    // Virtual displays
  bool update_vsync_on_power_off_ = false;
  bool update_vsync_on_doze_ = false;
  std::vector<bool> is_hdr_display_;            // info on HDR supported
  std::map<Display, Display> map_hwc_display_;  // Real and dummy display pairs.
  bool reset_panel_ = false;
  bool client_connected_ = false;
  bool new_bw_mode_ = false;
  int bw_mode_release_fd_ = -1;
  qService::QService *qservice_ = nullptr;
  HWCSocketHandler socket_handler_;
  bool hdmi_is_primary_ = false;
  bool is_composer_up_ = false;
  std::mutex mutex_lum_;
  static bool pending_power_mode_[HWCCallbacks::kNumDisplays];
  HotPlugEvent pending_hotplug_event_ = kHotPlugNone;

  struct VirtualDisplayData {
    uint32_t width;
    uint32_t height;
    int32_t format;
    bool in_use = false;
  };

  std::unordered_map<Display, VirtualDisplayData> virtual_id_map_;
  Locker pluggable_handler_lock_;
  int32_t idle_pc_ref_cnt_ = 0;
  int32_t disable_hotplug_bwcheck_ = 0;
  int32_t disable_mask_layer_hint_ = 0;
  int32_t enable_primary_reconfig_req_ = 0;
  float set_max_lum_ = -1.0;
  float set_min_lum_ = -1.0;
  std::bitset<HWCCallbacks::kNumDisplays> pending_refresh_;
  CWB cwb_;
  std::weak_ptr<DisplayConfig::ConfigCallback> qsync_callback_;
  std::weak_ptr<DisplayConfig::ConfigCallback> idle_callback_;
  std::mutex callbacks_lock_;
  std::unordered_map<int64_t, std::shared_ptr<IDisplayConfigCallback>> callback_clients_;
  uint64_t callback_client_id_ = 0;
  bool async_vds_creation_ = false;
  bool tui_state_transition_[HWCCallbacks::kNumDisplays] = {};
  std::bitset<HWCCallbacks::kNumDisplays> display_ready_;
  bool secure_session_active_ = false;
  bool is_client_up_ = false;
  std::shared_ptr<IPCIntf> ipc_intf_ = nullptr;
  bool primary_pending_ = true;
  Locker primary_display_lock_;
  std::map<Display, DisplayMapInfo *> map_active_displays_;
  vector<HWDisplayInfo> virtual_display_list_ = {};
  std::map<hwc2_display_t, std::future<int>> commit_done_future_;
  std::mutex tui_handler_lock_;
  std::future<int> tui_event_handler_future_;
  std::future<int> tui_callback_handler_future_;
  bool disable_get_screen_decorator_support_ = false;
  bool enable_aidl_idle_notification_ = false;

  // hpd handling
  void HpdInit() override;
  void HpdDeinit() override;
  void ParseUEvent(char *uevent_data, int length) override;
  void HpdThreadTop() override;

  // Bottom thread needs to be attached so we can wake
  // it up to terminate it before terminating hwc.
  void HpdThreadBottom();
  std::thread hpd_thread_;

  std::vector<Display> pending_hotplugs_{};
};

}  // namespace sdm

#endif  // __HWC_SESSION_H__
