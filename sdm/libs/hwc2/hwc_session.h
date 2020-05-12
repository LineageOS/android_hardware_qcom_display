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

#include <vendor/display/config/1.11/IDisplayConfig.h>

#include <core/core_interface.h>
#include <utils/locker.h>
#include <qd_utils.h>
#include <display_config.h>
#include <vector>
#include <utility>
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

namespace sdm {

using vendor::display::config::V1_11::IDisplayConfig;
using vendor::display::config::V1_10::IDisplayCWBCallback;

using ::android::hardware::Return;
using ::android::hardware::hidl_string;
using android::hardware::hidl_handle;
using ::android::hardware::hidl_vec;

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
  struct HWCModuleMethods : public hw_module_methods_t {
    HWCModuleMethods() { hw_module_methods_t::open = HWCSession::Open; }
  };

  enum HotPlugEvent {
    kHotPlugNone,
    kHotPlugEvent,
  };

  explicit HWCSession(const hw_module_t *module);
  int Init();
  int Deinit();
  HWC2::Error CreateVirtualDisplayObj(uint32_t width, uint32_t height, int32_t *format,
                                      hwc2_display_t *out_display_id);

  template <typename... Args>
  static int32_t CallDisplayFunction(hwc2_device_t *device, hwc2_display_t display,
                                     HWC2::Error (HWCDisplay::*member)(Args...), Args... args) {
    if (!device) {
      return HWC2_ERROR_BAD_PARAMETER;
    }

    if (display >= HWCCallbacks::kNumDisplays) {
      return HWC2_ERROR_BAD_DISPLAY;
    }

    HWCSession *hwc_session = static_cast<HWCSession *>(device);
    {
      // Power state transition start.
      SCOPE_LOCK(power_state_[display]);
      if (hwc_session->power_state_transition_[display]) {
        display = hwc_session->map_hwc_display_.find(display)->second;
      }
    }

    SCOPE_LOCK(locker_[display]);
    auto status = HWC2::Error::BadDisplay;
    if (hwc_session->hwc_display_[display]) {
      auto hwc_display = hwc_session->hwc_display_[display];
      status = (hwc_display->*member)(std::forward<Args>(args)...);
    }
    return INT32(status);
  }

  template <typename... Args>
  static int32_t CallLayerFunction(hwc2_device_t *device, hwc2_display_t display,
                                   hwc2_layer_t layer, HWC2::Error (HWCLayer::*member)(Args...),
                                   Args... args) {
    if (!device) {
      return HWC2_ERROR_BAD_PARAMETER;
    }

    if (display >= HWCCallbacks::kNumDisplays) {
      return HWC2_ERROR_BAD_DISPLAY;
    }

    HWCSession *hwc_session = static_cast<HWCSession *>(device);
    {
      // Power state transition start.
      SCOPE_LOCK(power_state_[display]);
      if (hwc_session->power_state_transition_[display]) {
        display = hwc_session->map_hwc_display_.find(display)->second;
      }
    }

    SCOPE_LOCK(locker_[display]);
    auto status = HWC2::Error::BadDisplay;
    if (hwc_session->hwc_display_[display]) {
      status = HWC2::Error::BadLayer;
      auto hwc_layer = hwc_session->hwc_display_[display]->GetHWCLayer(layer);
      if (hwc_layer != nullptr) {
        status = (hwc_layer->*member)(std::forward<Args>(args)...);
        if (hwc_session->hwc_display_[display]->GetGeometryChanges()) {
          hwc_session->hwc_display_[display]->ResetValidation();
        }
      }
    }
    return INT32(status);
  }

  // HWC2 Functions that require a concrete implementation in hwc session
  // and hence need to be member functions
  static int32_t AcceptDisplayChanges(hwc2_device_t *device, hwc2_display_t display);
  static int32_t CreateLayer(hwc2_device_t *device, hwc2_display_t display,
                             hwc2_layer_t *out_layer_id);
  static int32_t CreateVirtualDisplay(hwc2_device_t *device, uint32_t width, uint32_t height,
                                      int32_t *format, hwc2_display_t *out_display_id);
  static int32_t DestroyLayer(hwc2_device_t *device, hwc2_display_t display, hwc2_layer_t layer);
  static int32_t DestroyVirtualDisplay(hwc2_device_t *device, hwc2_display_t display);
  static void Dump(hwc2_device_t *device, uint32_t *out_size, char *out_buffer);
  static int32_t PresentDisplay(hwc2_device_t *device, hwc2_display_t display,
                                int32_t *out_retire_fence);
  static int32_t RegisterCallback(hwc2_device_t *device, int32_t descriptor,
                                  hwc2_callback_data_t callback_data,
                                  hwc2_function_pointer_t pointer);
  static int32_t SetOutputBuffer(hwc2_device_t *device, hwc2_display_t display,
                                 buffer_handle_t buffer, int32_t releaseFence);
  static int32_t SetPowerMode(hwc2_device_t *device, hwc2_display_t display, int32_t int_mode);
  static int32_t ValidateDisplay(hwc2_device_t *device, hwc2_display_t display,
                                 uint32_t *out_num_types, uint32_t *out_num_requests);
  static int32_t SetColorMode(hwc2_device_t *device, hwc2_display_t display,
                              int32_t /*ColorMode*/ int_mode);
  static int32_t SetColorModeWithRenderIntent(hwc2_device_t *device, hwc2_display_t display,
                                              int32_t /*ColorMode*/ int_mode,
                                              int32_t /*RenderIntent*/ int_render_intent);
  static int32_t SetColorTransform(hwc2_device_t *device, hwc2_display_t display,
                                   const float *matrix, int32_t /*android_color_transform_t*/ hint);
  static int32_t GetReadbackBufferAttributes(hwc2_device_t *device, hwc2_display_t display,
                                             int32_t *format, int32_t *dataspace);
  static int32_t SetReadbackBuffer(hwc2_device_t *device, hwc2_display_t display,
                                   const native_handle_t *buffer, int32_t acquire_fence);
  static int32_t GetReadbackBufferFence(hwc2_device_t *device, hwc2_display_t display,
                                        int32_t *release_fence);
  static uint32_t GetMaxVirtualDisplayCount(hwc2_device_t *device);
  static int32_t GetDisplayIdentificationData(hwc2_device_t *device, hwc2_display_t display,
                                              uint8_t *outPort, uint32_t *outDataSize,
                                              uint8_t *outData);
  static int32_t GetDisplayCapabilities(hwc2_device_t *device, hwc2_display_t display,
                                        uint32_t *outNumCapabilities, uint32_t *outCapabilities);
  static int32_t GetDisplayBrightnessSupport(hwc2_device_t *device, hwc2_display_t display,
                                             bool *outSupport);

  // HWCDisplayEventHandler
  virtual void DisplayPowerReset();

  static int32_t SetVsyncEnabled(hwc2_device_t *device, hwc2_display_t display,
                                 int32_t int_enabled);
  static int32_t GetDozeSupport(hwc2_device_t *device, hwc2_display_t display,
                                int32_t *out_support);

  static Locker locker_[HWCCallbacks::kNumDisplays];
  static Locker power_state_[HWCCallbacks::kNumDisplays];
  static Locker display_config_locker_;

 private:
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
  void UpdateThrottlingRate();
  void SetNewThrottlingRate(uint32_t new_rate);
  // hwc methods
  static int Open(const hw_module_t *module, const char *name, hw_device_t **device);
  static int Close(hw_device_t *device);
  static void GetCapabilities(struct hwc2_device *device, uint32_t *outCount,
                              int32_t *outCapabilities);
  static hwc2_function_pointer_t GetFunction(struct hwc2_device *device, int32_t descriptor);

  // Uevent handler
  virtual void UEventHandler(const char *uevent_data, int length);
  void ResetPanel();
  void InitSupportedDisplaySlots();
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
  int GetVsyncPeriod(int disp);
  int32_t GetConfigCount(int disp_id, uint32_t *count);
  int32_t GetActiveConfigIndex(int disp_id, uint32_t *config);
  int32_t SetActiveConfigIndex(int disp_id, uint32_t config);
  int32_t ControlPartialUpdate(int dpy, bool enable);
  int32_t DisplayBWTransactionPending(bool *status);
  int32_t SetSecondaryDisplayStatus(int disp_id, HWCDisplay::DisplayStatus status);
  int32_t GetPanelBrightness(int *level);
  int32_t MinHdcpEncryptionLevelChanged(int disp_id, uint32_t min_enc_level);
  int32_t IsWbUbwcSupported(int *value);
  int32_t SetDynamicDSIClock(int64_t disp_id, uint32_t bitrate);
  bool HasHDRSupport(HWCDisplay *hwc_display);

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
  Return<int32_t> setCWBOutputBuffer(const ::android::sp<IDisplayCWBCallback> &callback,
                                     uint32_t disp_id, const Rect &rect, bool post_processed,
                                     const hidl_handle& buffer) override;
  Return<int32_t> setQsyncMode(uint32_t disp_id, IDisplayConfig::QsyncMode mode) override;

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
  android::status_t SetPanelLuminanceAttributes(const android::Parcel *input_parcel);

  void Refresh(hwc2_display_t display);
  void HotPlug(hwc2_display_t display, HWC2::Connection state);

  // Internal methods
  HWC2::Error ValidateDisplayInternal(hwc2_display_t display, uint32_t *out_num_types,
                                      uint32_t *out_num_requests);
  HWC2::Error PresentDisplayInternal(hwc2_display_t display, int32_t *out_retire_fence);
  void HandleSecureSession();
  void HandlePowerOnPending(hwc2_display_t display, int retire_fence);
  void HandleHotplugPending(hwc2_display_t disp_id, int retire_fence);
  bool IsPluggableDisplayConnected();
  hwc2_display_t GetActiveBuiltinDisplay();
  void HandlePendingRefresh();
  void NotifyClientStatus(bool connected);

  CoreInterface *core_intf_ = nullptr;
  HWCDisplay *hwc_display_[HWCCallbacks::kNumDisplays] = {nullptr};
  HWCCallbacks callbacks_;
  HWCBufferAllocator buffer_allocator_;
  HWCBufferSyncHandler buffer_sync_handler_;
  HWCColorManager *color_mgr_ = nullptr;
  DisplayMapInfo map_info_primary_;                 // Primary display (either builtin or pluggable)
  std::vector<DisplayMapInfo> map_info_builtin_;    // Builtin displays excluding primary
  std::vector<DisplayMapInfo> map_info_pluggable_;  // Pluggable displays excluding primary
  std::vector<DisplayMapInfo> map_info_virtual_;    // Virtual displays
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
  Locker callbacks_lock_;
  std::mutex mutex_lum_;
  int hpd_bpp_ = 0;
  int hpd_pattern_ = 0;
  static bool power_on_pending_[HWCCallbacks::kNumDisplays];
  static int null_display_mode_;
  HotPlugEvent hotplug_pending_event_ = kHotPlugNone;
  Locker pluggable_handler_lock_;
  bool destroy_virtual_disp_pending_ = false;
  uint32_t idle_pc_ref_cnt_ = 0;
  int32_t disable_hotplug_bwcheck_ = 0;
  int32_t disable_mask_layer_hint_ = 0;
  float set_max_lum_ = -1.0;
  float set_min_lum_ = -1.0;
  std::bitset<HWCCallbacks::kNumDisplays> pending_refresh_;
  bool async_powermode_ = false;
  bool power_state_transition_[HWCCallbacks::kNumDisplays] = {};  // +1 to account for primary.
  std::bitset<HWCCallbacks::kNumDisplays> display_ready_;
};

}  // namespace sdm

#endif  // __HWC_SESSION_H__
