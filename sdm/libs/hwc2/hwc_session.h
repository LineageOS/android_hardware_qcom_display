/*
 * Copyright (c) 2014-2020, 2021, The Linux Foundation. All rights reserved.
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

#include <cutils/native_handle.h>
#include <config/device_interface.h>
#include <core/core_interface.h>
#include <utils/locker.h>
#include <qd_utils.h>
#include <display_config.h>
#include <vector>
#include <utility>
#include <map>
#include <string>

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
#include <android/hardware/graphics/composer/2.4/IComposerClient.h>
namespace sdm {
using ::android::hardware::Return;
using ::android::hardware::hidl_string;
using android::hardware::hidl_handle;
using ::android::hardware::hidl_vec;

namespace composer_V2_4 = ::android::hardware::graphics::composer::V2_4;
using HwcDisplayCapability = composer_V2_4::IComposerClient::DisplayCapability;

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
  static int32_t SetDisplayBrightness(hwc2_device_t *device, hwc2_display_t display,
                                      float brightness);
  static int32_t GetDisplayConnectionType(hwc2_device_t *device, hwc2_display_t display,
                                          uint32_t *outType);
  static int32_t GetDisplayVsyncPeriod(hwc2_device_t *device, hwc2_display_t display,
                                       hwc2_vsync_period_t *out_vsync_period);
  static int32_t SetActiveConfigWithConstraints(hwc2_device_t *device, hwc2_display_t display,
                    hwc2_config_t config,
                    hwc_vsync_period_change_constraints_t *vsync_period_change_constraints,
                    hwc_vsync_period_change_timeline_t *out_timeline);

  virtual int RegisterClientContext(std::shared_ptr<DisplayConfig::ConfigCallback> callback,
                                    DisplayConfig::ConfigInterface **intf);
  virtual void UnRegisterClientContext(DisplayConfig::ConfigInterface *intf);

  // HWCDisplayEventHandler
  virtual void DisplayPowerReset();

  static int32_t SetVsyncEnabled(hwc2_device_t *device, hwc2_display_t display,
                                 int32_t int_enabled);
  static int32_t GetDozeSupport(hwc2_device_t *device, hwc2_display_t display,
                                int32_t *out_support);
  static int32_t SetAutoLowLatencyMode(hwc2_device_t *device, hwc2_display_t display, bool on);
  static int32_t GetSupportedContentTypes(hwc2_device_t *device, hwc2_display_t display,
                                          uint32_t *count, uint32_t *contentTypes);
  static int32_t SetContentType(hwc2_device_t *device, hwc2_display_t display,int32_t type);
  static Locker locker_[HWCCallbacks::kNumDisplays];
  static Locker power_state_[HWCCallbacks::kNumDisplays];
  static Locker display_config_locker_;

 private:

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
    virtual int SetPowerMode(uint32_t disp_id, DisplayConfig::PowerMode power_mode);
    virtual int IsPowerModeOverrideSupported(uint32_t disp_id, bool *supported);
    virtual int IsHDRSupported(uint32_t disp_id, bool *supported);
    virtual int IsWCGSupported(uint32_t disp_id, bool *supported);
    virtual int SetLayerAsMask(uint32_t disp_id, uint64_t layer_id);
    virtual int GetDebugProperty(const std::string prop_name, std::string value) {return -EINVAL;}
    virtual int GetDebugProperty(const std::string prop_name, std::string *value);
    virtual int GetActiveBuiltinDisplayAttributes(DisplayConfig::Attributes *attr);
    virtual int SetPanelLuminanceAttributes(uint32_t disp_id, float min_lum, float max_lum);
    virtual int IsBuiltInDisplay(uint32_t disp_id, bool *is_builtin);
    virtual int GetSupportedDSIBitClks(uint32_t disp_id,
                                       std::vector<uint64_t> bit_clks) {return -EINVAL;}
    virtual int GetSupportedDSIBitClks(uint32_t disp_id, std::vector<uint64_t> *bit_clks);
    virtual int GetDSIClk(uint32_t disp_id, uint64_t *bit_clk);
    virtual int SetDSIClk(uint32_t disp_id, uint64_t bit_clk);
    virtual int SetCWBOutputBuffer(uint32_t disp_id, const DisplayConfig::Rect rect,
                                   bool post_processed, const native_handle_t *buffer);
    virtual int SetQsyncMode(uint32_t disp_id, DisplayConfig::QsyncMode mode);

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
  int GetConfigCount(int disp_id, uint32_t *count);
  int GetActiveConfigIndex(int disp_id, uint32_t *config);
  int SetActiveConfigIndex(int disp_id, uint32_t config);
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
  int32_t SetDynamicDSIClock(int64_t disp_id, uint32_t bitrate);
  bool HasHDRSupport(HWCDisplay *hwc_display);
  int32_t getDisplayBrightness(uint32_t display, float *brightness);
  int32_t setDisplayBrightness(uint32_t display, float brightness);
  bool isSmartPanelConfig(uint32_t disp_id, uint32_t config_id);

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
  bool pluggable_is_primary_ = false;
  bool null_display_active_ = false;
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
