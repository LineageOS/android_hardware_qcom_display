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

#ifndef __HWC_SESSION_H__
#define __HWC_SESSION_H__

#include <vendor/qti/hardware/display/composer/3.1/IQtiComposerClient.h>
#include <config/device_interface.h>
#include <aidl/vendor/qti/hardware/display/config/BnDisplayConfig.h>
#include <aidl/vendor/qti/hardware/display/config/BnDisplayConfigCallback.h>
#include <binder/Status.h>

#include <core/core_interface.h>
#include <core/ipc_interface.h>
#include <utils/locker.h>
#include <utils/constants.h>
#include <qd_utils.h>
#include <display_config.h>
#include <vector>
#include <queue>
#include <utility>
#include <future>   // NOLINT
#include <map>
#include <unordered_map>
#include <string>
#include <memory>
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

using ::android::hardware::Return;
using ::android::hardware::hidl_string;
using android::hardware::hidl_handle;
using ::android::hardware::hidl_vec;
using ::android::sp;
using ::android::hardware::Void;
namespace composer_V2_3 = ::android::hardware::graphics::composer::V2_3;
namespace composer_V2_4 = ::android::hardware::graphics::composer::V2_4;
using HwcDisplayCapability = composer_V2_4::IComposerClient::DisplayCapability;
using HwcDisplayConnectionType = composer_V2_4::IComposerClient::DisplayConnectionType;
using HwcClientTargetProperty = composer_V2_4::IComposerClient::ClientTargetProperty;
using ::aidl::vendor::qti::hardware::display::config::IDisplayConfig;
using ::aidl::vendor::qti::hardware::display::config::IDisplayConfigCallback;
using ::aidl::vendor::qti::hardware::display::config::CameraSmoothOp;
using ::aidl::vendor::qti::hardware::display::config::Attributes;
using ::aidl::vendor::qti::hardware::display::config::DisplayPortType;
using ::aidl::vendor::qti::hardware::display::config::Concurrency;

namespace aidl::vendor::qti::hardware::display::config {
  class DisplayConfigAIDL;
}

namespace sdm {

using vendor::qti::hardware::display::composer::V3_1::IQtiComposerClient;

int32_t GetDataspaceFromColorMode(ColorMode mode);

typedef DisplayConfig::DisplayType DispType;

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

class HWCSession : hwc2_device_t, HWCUEventListener, public qClient::BnQClient,
                   public HWCDisplayEventHandler, public DisplayConfig::ClientContext {

 friend class aidl::vendor::qti::hardware::display::config::DisplayConfigAIDL;

 public:
  enum HotPlugEvent {
    kHotPlugNone,
    kHotPlugEvent,
  };

  enum ClientCommitDone {
    kClientPartialUpdate,
    kClientIdlepowerCollapse,
    kClientTeardownCWB,
    kClientTrustedUI,
    kClientConcurrency,
    kClientVirtualDisplay,
    kClientMax
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

    SCOPE_LOCK(locker_[display]);
    auto status = HWC2::Error::BadDisplay;
    if (hwc_display_[display]) {
      status = HWC2::Error::BadLayer;
      auto hwc_layer = hwc_display_[display]->GetHWCLayer(layer);
      if (hwc_layer != nullptr) {
        status = (hwc_layer->*member)(std::forward<Args>(args)...);
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
  int32_t PresentDisplay(hwc2_display_t display, shared_ptr<Fence> *out_retire_fence);
  void RegisterCallback(int32_t descriptor, hwc2_callback_data_t callback_data,
                        hwc2_function_pointer_t pointer);
  int32_t SetOutputBuffer(hwc2_display_t display, buffer_handle_t buffer,
                          const shared_ptr<Fence> &release_fence);
  int32_t SetPowerMode(hwc2_display_t display, int32_t int_mode);
  int32_t SetColorMode(hwc2_display_t display, int32_t /*ColorMode*/ int_mode);
  int32_t SetColorModeWithRenderIntent(hwc2_display_t display, int32_t /*ColorMode*/ int_mode,
                                       int32_t /*RenderIntent*/ int_render_intent);
  int32_t SetColorTransform(hwc2_display_t display, const float *matrix,
                            int32_t /*android_color_transform_t*/ hint);
  int32_t GetReadbackBufferAttributes(hwc2_display_t display,
                                      int32_t *format, int32_t *dataspace);
  int32_t SetReadbackBuffer(hwc2_display_t display, const native_handle_t *buffer,
                            const shared_ptr<Fence> &acquire_fence);
  int32_t GetReadbackBufferFence(hwc2_display_t display, shared_ptr<Fence> *release_fence);
  uint32_t GetMaxVirtualDisplayCount();
  int32_t GetDisplayIdentificationData(hwc2_display_t display, uint8_t *outPort,
                                       uint32_t *outDataSize, uint8_t *outData);
  int32_t GetDisplayCapabilities(hwc2_display_t display,
                                 hidl_vec<HwcDisplayCapability> *capabilities);
  int32_t GetDisplayBrightnessSupport(hwc2_display_t display, bool *outSupport);
  int32_t SetDisplayBrightness(hwc2_display_t display, float brightness);
  int32_t WaitForResources(bool wait_for_resources, hwc2_display_t active_builtin_id,
                        hwc2_display_t display_id);

  // newly added
  int32_t GetDisplayType(hwc2_display_t display, int32_t *out_type);
  int32_t GetDisplayAttribute(hwc2_display_t display, hwc2_config_t config, HwcAttribute attribute,
                              int32_t *out_value);
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
                           hwc2_layer_t *out_layers, std::vector<shared_ptr<Fence>> *out_fences);
  int32_t SetClientTarget(hwc2_display_t display, buffer_handle_t target,
                          shared_ptr<Fence> acquire_fence,
                          int32_t dataspace, hwc_region_t damage);
  int32_t SetClientTarget_3_1(hwc2_display_t display, buffer_handle_t target,
                              shared_ptr<Fence> acquire_fence,
                              int32_t dataspace, hwc_region_t damage);
  int32_t SetCursorPosition(hwc2_display_t display, hwc2_layer_t layer, int32_t x, int32_t y);
  int32_t GetDataspaceSaturationMatrix(int32_t /*Dataspace*/ int_dataspace, float *out_matrix);
  int32_t SetDisplayBrightnessScale(const android::Parcel *input_parcel);
  int32_t GetDisplayConnectionType(hwc2_display_t display, HwcDisplayConnectionType *type);
  int32_t SetDimmingEnable(hwc2_display_t display, int32_t int_enabled);
  int32_t SetDimmingMinBl(hwc2_display_t display, int32_t min_bl);
  int32_t GetClientTargetProperty(hwc2_display_t display,
                                  HwcClientTargetProperty *outClientTargetProperty);

  // Layer functions
  int32_t SetLayerBuffer(hwc2_display_t display, hwc2_layer_t layer, buffer_handle_t buffer,
                         const shared_ptr<Fence> &acquire_fence);
  int32_t SetLayerBlendMode(hwc2_display_t display, hwc2_layer_t layer, int32_t int_mode);
  int32_t SetLayerDisplayFrame(hwc2_display_t display, hwc2_layer_t layer, hwc_rect_t frame);
  int32_t SetLayerPlaneAlpha(hwc2_display_t display, hwc2_layer_t layer, float alpha);
  int32_t SetLayerSourceCrop(hwc2_display_t display, hwc2_layer_t layer, hwc_frect_t crop);
  int32_t SetLayerTransform(hwc2_display_t display, hwc2_layer_t layer, int32_t int_transform);
  int32_t SetLayerZOrder(hwc2_display_t display, hwc2_layer_t layer, uint32_t z);
  int32_t SetLayerType(hwc2_display_t display, hwc2_layer_t layer,
                       IQtiComposerClient::LayerType type);
  int32_t SetLayerFlag(hwc2_display_t display, hwc2_layer_t layer,
                       IQtiComposerClient::LayerFlag flag);
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

  int SetCameraSmoothInfo(CameraSmoothOp op, int32_t fps);
  int RegisterCallbackClient(const std::shared_ptr<IDisplayConfigCallback>& callback,
                             int64_t *client_handle);
  int UnregisterCallbackClient(const int64_t client_handle);
  int NotifyResolutionChange(int32_t disp_id, Attributes& attr);
  int NotifyFpsMitigation(int32_t disp_id, Attributes attr, Concurrency con);

  virtual int RegisterClientContext(std::shared_ptr<DisplayConfig::ConfigCallback> callback,
                                    DisplayConfig::ConfigInterface **intf);
  virtual void UnRegisterClientContext(DisplayConfig::ConfigInterface *intf);

  // HWCDisplayEventHandler
  virtual void DisplayPowerReset();
  virtual void PerformDisplayPowerReset();
  virtual void PerformQsyncCallback(hwc2_display_t display, bool qsync_enabled,
                                    uint32_t refresh_rate, uint32_t qsync_refresh_rate);
  virtual void VmReleaseDone(hwc2_display_t display);
  virtual void NotifyConcurrencyFps(const float fps, DisplayConcurrencyType concurrency,
                                    bool concurrency_begin);

  int32_t SetVsyncEnabled(hwc2_display_t display, int32_t int_enabled);
  int32_t GetDozeSupport(hwc2_display_t display, int32_t *out_support);
  int32_t GetDisplayConfigs(hwc2_display_t display, uint32_t *out_num_configs,
                            hwc2_config_t *out_configs);
  int32_t GetVsyncPeriod(hwc2_display_t disp, uint32_t *vsync_period);
  void Refresh(hwc2_display_t display);

  int32_t GetDisplayVsyncPeriod(hwc2_display_t display, VsyncPeriodNanos *out_vsync_period);
  int32_t SetActiveConfigWithConstraints(
      hwc2_display_t display, hwc2_config_t config,
      const VsyncPeriodChangeConstraints *vsync_period_change_constraints,
      VsyncPeriodChangeTimeline *out_timeline);
  HWC2::Error CommitOrPrepare(hwc2_display_t display, bool validate_only,
                              shared_ptr<Fence> *out_retire_fence,
                              uint32_t *out_num_types, uint32_t *out_num_requests,
                              bool *needs_commit);
  HWC2::Error TryDrawMethod(hwc2_display_t display, IQtiComposerClient::DrawMethod drawMethod);


  static Locker locker_[HWCCallbacks::kNumDisplays];
  static Locker hdr_locker_[HWCCallbacks::kNumDisplays];
  static Locker display_config_locker_;
  static std::mutex command_seq_mutex_;
  static std::bitset<kClientMax> clients_waiting_for_commit_[HWCCallbacks::kNumDisplays];
  static shared_ptr<Fence> retire_fence_[HWCCallbacks::kNumDisplays];
  static int commit_error_[HWCCallbacks::kNumDisplays];
  static Locker vm_release_locker_[HWCCallbacks::kNumDisplays];
  static std::bitset<HWCCallbacks::kNumDisplays> clients_waiting_for_vm_release_;
  static std::set<hwc2_display_t> active_displays_;

 private:
  class CWB {
   public:
    explicit CWB(HWCSession *hwc_session) : hwc_session_(hwc_session) { }
    void PresentDisplayDone(hwc2_display_t disp_id);

    int32_t PostBuffer(std::weak_ptr<DisplayConfig::ConfigCallback> callback,
                       const CwbConfig &cwb_config, const native_handle_t *buffer,
                       hwc2_display_t display_type);
    bool IsCwbActiveOnDisplay(hwc2_display_t disp_type);

   private:
    struct QueueNode {
      QueueNode(std::weak_ptr<DisplayConfig::ConfigCallback> cb, const CwbConfig &cwb_conf,
                const hidl_handle &buf, hwc2_display_t disp_type)
          : callback(cb), cwb_config(cwb_conf), buffer(buf), display_type(disp_type) {}

      ~QueueNode() {
        if (!buffer) {
          return;
        }

        native_handle_close(buffer);
        native_handle_delete(const_cast<native_handle_t *>(buffer));
      }

      std::weak_ptr<DisplayConfig::ConfigCallback> callback;
      CwbConfig cwb_config = {};
      const native_handle_t *buffer;
      hwc2_display_t display_type;
    };

    void ProcessRequests();
    void PerformFenceWaits();
    static void AsyncTask(CWB *cwb);
    static void AsyncFenceWaits(CWB *cwb);
    void NotifyCWBStatus(int status, shared_ptr<QueueNode> cwb_node);

    std::queue<shared_ptr<QueueNode>> queue_;
    std::queue<pair<shared_ptr<Fence>, shared_ptr<QueueNode>>> fence_wait_queue_;

    std::future<void> future_;
    std::future<void> fence_wait_future_;
    Locker queue_lock_;
    Locker fence_queue_lock_;
    std::mutex mutex_;
    std::condition_variable cv_;
    HWCSession *hwc_session_ = nullptr;
  };

  class DisplayConfigImpl: public DisplayConfig::ConfigInterface {
   public:
    explicit DisplayConfigImpl(std::weak_ptr<DisplayConfig::ConfigCallback> callback,
                               HWCSession *hwc_session);

   private:
    virtual int IsDisplayConnected(DispType dpy, bool *connected);
    virtual int SetDisplayStatus(DispType dpy, DisplayConfig::ExternalStatus status);
    virtual int ConfigureDynRefreshRate(DisplayConfig::DynRefreshRateOp op, uint32_t refresh_rate);
    virtual int GetConfigCount(DispType dpy, uint32_t *count);
    virtual int GetActiveConfig(DispType dpy, uint32_t *config);
    virtual int SetActiveConfig(DispType dpy, uint32_t config);
    virtual int GetDisplayAttributes(uint32_t config_index, DispType dpy,
                                     DisplayConfig::Attributes *attributes);
    virtual int SetPanelBrightness(uint32_t level);
    virtual int GetPanelBrightness(uint32_t *level);
    virtual int MinHdcpEncryptionLevelChanged(DispType dpy, uint32_t min_enc_level);
    virtual int RefreshScreen();
    virtual int ControlPartialUpdate(DispType dpy, bool enable);
    virtual int ToggleScreenUpdate(bool on);
    virtual int SetIdleTimeout(uint32_t value);
    virtual int GetHDRCapabilities(DispType dpy, DisplayConfig::HDRCapsParams *caps);
    virtual int SetCameraLaunchStatus(uint32_t on);
    virtual int DisplayBWTransactionPending(bool *status);
    virtual int SetDisplayAnimating(uint64_t display_id, bool animating);
    virtual int ControlIdlePowerCollapse(bool enable, bool synchronous);
    virtual int GetWriteBackCapabilities(bool *is_wb_ubwc_supported);
    virtual int SetDisplayDppsAdROI(uint32_t display_id, uint32_t h_start, uint32_t h_end,
                                    uint32_t v_start, uint32_t v_end, uint32_t factor_in,
                                    uint32_t factor_out);
    virtual int UpdateVSyncSourceOnPowerModeOff();
    virtual int UpdateVSyncSourceOnPowerModeDoze();
    virtual int IsHDRSupported(uint32_t disp_id, bool *supported);
    virtual int IsWCGSupported(uint32_t disp_id, bool *supported);
    virtual int SetLayerAsMask(uint32_t disp_id, uint64_t layer_id);
    virtual int GetDebugProperty(const std::string prop_name, std::string value) {return -EINVAL;}
    virtual int GetDebugProperty(const std::string prop_name, std::string *value);
    virtual int GetActiveBuiltinDisplayAttributes(DisplayConfig::Attributes *attr);
    virtual int SetPanelLuminanceAttributes(uint32_t disp_id, float min_lum, float max_lum);
    virtual int IsBuiltInDisplay(uint32_t disp_id, bool *is_builtin);
    virtual int IsAsyncVDSCreationSupported(bool *supported);
    virtual int CreateVirtualDisplay(uint32_t width, uint32_t height, int format);
    virtual int GetSupportedDSIBitClks(uint32_t disp_id,
                                       std::vector<uint64_t> bit_clks) {return -EINVAL;}
    virtual int GetSupportedDSIBitClks(uint32_t disp_id, std::vector<uint64_t> *bit_clks);
    virtual int GetDSIClk(uint32_t disp_id, uint64_t *bit_clk);
    virtual int SetDSIClk(uint32_t disp_id, uint64_t bit_clk);
    virtual int SetCWBOutputBuffer(uint32_t disp_id, const DisplayConfig::Rect rect,
                                   bool post_processed, const native_handle_t *buffer);
    virtual int SetQsyncMode(uint32_t disp_id, DisplayConfig::QsyncMode mode);
    virtual int IsSmartPanelConfig(uint32_t disp_id, uint32_t config_id, bool *is_smart);
    virtual int IsRotatorSupportedFormat(int hal_format, bool ubwc, bool *supported);
    virtual int ControlQsyncCallback(bool enable);
    virtual int GetDisplayHwId(uint32_t disp_id, uint32_t *display_hw_id);
    virtual int SendTUIEvent(DispType dpy, DisplayConfig::TUIEventType event_type);
    virtual int GetSupportedDisplayRefreshRates(DispType dpy,
                                                std::vector<uint32_t> *supported_refresh_rates);
    virtual int IsRCSupported(uint32_t disp_id, bool *supported);
    virtual int IsSupportedConfigSwitch(uint32_t disp_id, uint32_t config, bool *supported);
    virtual int ControlIdleStatusCallback(bool enable);
    virtual int GetDisplayType(uint64_t physical_disp_id, DispType *disp_type);
    virtual int AllowIdleFallback();

    std::weak_ptr<DisplayConfig::ConfigCallback> callback_;
    HWCSession *hwc_session_ = nullptr;
  };

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
  static const int kVmReleaseTimeoutMs = 100;
  static const int kCommitDoneTimeoutMs = 100;
  static const int kVmReleaseRetry = 3;
  static const int kDenomNstoMs = 1000000;
  static const int kNumDrawCycles = 3;

  uint32_t throttling_refresh_rate_ = 60;
  std::mutex hotplug_mutex_;
  std::condition_variable hotplug_cv_;
  bool resource_ready_ = false;
  hwc2_display_t active_display_id_ = 0;
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
  void DestroyPluggableDisplay(DisplayMapInfo *map_info);
  void DestroyNonPluggableDisplay(DisplayMapInfo *map_info);
  int GetConfigCount(int disp_id, uint32_t *count);
  int GetActiveConfigIndex(int disp_id, uint32_t *config);
  int SetActiveConfigIndex(int disp_id, uint32_t config);
  int SetNoisePlugInOverride(int32_t disp_id, bool override_en, int32_t attn,
                             int32_t noise_zpos);
  int ControlPartialUpdate(int dpy, bool enable);
  int DisplayBWTransactionPending(bool *status);
  int SetDisplayStatus(int disp_id, HWCDisplay::DisplayStatus status);
  int MinHdcpEncryptionLevelChanged(int disp_id, uint32_t min_enc_level);
  int IsWbUbwcSupported(bool *value);
  int SetIdleTimeout(uint32_t value);
  int ToggleScreenUpdate(bool on);
  int SetCameraLaunchStatus(uint32_t on);
  int SetDisplayDppsAdROI(uint32_t display_id, uint32_t h_start, uint32_t h_end,
                          uint32_t v_start, uint32_t v_end, uint32_t factor_in,
                          uint32_t factor_out);
  int ControlIdlePowerCollapse(bool enable, bool synchronous);
  int GetSupportedDisplayRefreshRates(int disp_id, std::vector<uint32_t> *supported_refresh_rates);
  int32_t SetDynamicDSIClock(int64_t disp_id, uint32_t bitrate);
  int32_t getDisplayBrightness(uint32_t display, float *brightness);
  int32_t setDisplayBrightness(uint32_t display, float brightness);
  int32_t getDisplayMaxBrightness(uint32_t display, uint32_t *max_brightness_level);
  bool HasHDRSupport(HWCDisplay *hwc_display);
  void PostInit();
  int GetDispTypeFromPhysicalId(uint64_t physical_disp_id, DispType *disp_type);
  DisplayError WaitForPrimaryHotplug(HWDisplayInterfaceInfo *hw_disp_info);
  void HandlePluggablePrimaryDisplay(HWDisplaysInfo *hw_displays_info);

  // Uevent handler
  virtual void UEventHandler(const char *uevent_data, int length);

  // service methods
  void StartServices();

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
  android::status_t HandleTUITransition(int disp_id, int event);
  android::status_t GetDisplayPortId(uint32_t display, int *port_id);

  // Internal methods
  void HandleSecureSession();
  void HandlePendingPowerMode(hwc2_display_t display, const shared_ptr<Fence> &retire_fence);
  void HandlePendingHotplug(hwc2_display_t disp_id, const shared_ptr<Fence> &retire_fence);
  bool IsPluggableDisplayConnected();
  bool IsVirtualDisplayConnected();
  hwc2_display_t GetActiveBuiltinDisplay();
  void HandlePendingRefresh();
  void NotifyClientStatus(bool connected);
  int32_t GetVirtualDisplayId();
  android::status_t TUITransitionPrepare(int disp_id);
  android::status_t TUITransitionStart(int disp_id);
  android::status_t TUITransitionEnd(int disp_id);
  android::status_t TUITransitionUnPrepare(int disp_id);
  void PerformIdleStatusCallback(hwc2_display_t display);
  DispType GetDisplayConfigDisplayType(int qdutils_disp_type);
  HWC2::Error TeardownConcurrentWriteback(hwc2_display_t display);
  void PostCommitUnlocked(hwc2_display_t display, const shared_ptr<Fence> &retire_fence,
                          HWC2::Error status);
  void PostCommitLocked(hwc2_display_t display, shared_ptr<Fence> &retire_fence);
  int WaitForCommitDone(hwc2_display_t display, int client_id);
  int WaitForCommitDoneAsync(hwc2_display_t display, int client_id);
  void NotifyDisplayAttributes(hwc2_display_t display, hwc2_config_t config);
  int WaitForVmRelease(hwc2_display_t display, int timeout_ms);

  CoreInterface *core_intf_ = nullptr;
  HWCDisplay *hwc_display_[HWCCallbacks::kNumDisplays] = {nullptr};
  QSyncMode  hwc_display_qsync_[HWCCallbacks::kNumDisplays] = {QSyncMode::kQSyncModeNone};
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
  std::vector<bool> is_hdr_display_;    // info on HDR supported
  std::map <hwc2_display_t, hwc2_display_t> map_hwc_display_;  // Real and dummy display pairs.
  bool reset_panel_ = false;
  bool client_connected_ = false;
  bool new_bw_mode_ = false;
  int bw_mode_release_fd_ = -1;
  qService::QService *qservice_ = nullptr;
  HWCSocketHandler socket_handler_;
  bool hdmi_is_primary_ = false;
  bool is_composer_up_ = false;
  std::mutex mutex_lum_;
  int hpd_bpp_ = 0;
  int hpd_pattern_ = 0;
  static bool pending_power_mode_[HWCCallbacks::kNumDisplays];
  HotPlugEvent pending_hotplug_event_ = kHotPlugNone;
  hwc2_display_t virtual_id_ = HWCCallbacks::kNumDisplays;
  Locker pluggable_handler_lock_;
  uint32_t idle_pc_ref_cnt_ = 0;
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
  bool async_vds_creation_requested_ = false;
  std::bitset<HWCCallbacks::kNumDisplays> display_ready_;
  bool secure_session_active_ = false;
  bool is_client_up_ = false;
  std::shared_ptr<IPCIntf> ipc_intf_ = nullptr;
  bool primary_pending_ = true;
  Locker primary_display_lock_;
  bool disable_non_wfd_vds_ = false;
  bool debug_enable_hwc_vds_ = false;
  bool tui_start_success_ = false;
  std::map <hwc2_display_t, std::future<int>> commit_done_future_;
  std::future<int> wfd_refresh_future_;
};
}  // namespace sdm

#endif  // __HWC_SESSION_H__
