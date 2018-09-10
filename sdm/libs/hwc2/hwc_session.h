/*
 * Copyright (c) 2014-2018, The Linux Foundation. All rights reserved.
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

#ifdef DISPLAY_CONFIG_1_2
#include <vendor/display/config/1.2/IDisplayConfig.h>
#elif DISPLAY_CONFIG_1_1
#include <vendor/display/config/1.1/IDisplayConfig.h>
#else
#include <vendor/display/config/1.0/IDisplayConfig.h>
#endif

#include <core/core_interface.h>
#include <utils/locker.h>
#include <qd_utils.h>
#include <display_config.h>
#include <vector>

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

namespace sdm {

#ifdef DISPLAY_CONFIG_1_2
using vendor::display::config::V1_2::IDisplayConfig;
#elif DISPLAY_CONFIG_1_1
using vendor::display::config::V1_1::IDisplayConfig;
#else
using ::vendor::display::config::V1_0::IDisplayConfig;
#endif
using ::android::hardware::Return;

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

class HWCSession : hwc2_device_t, HWCUEventListener, IDisplayConfig, public qClient::BnQClient {
 public:
  static const int kNumBuiltIn = 4;
  static const int kNumPluggable = 4;
  static const int kNumVirtual = 4;
  // Add 1 primary display which can be either a builtin or pluggable.
  static const int kNumDisplays = 1 + kNumBuiltIn + kNumPluggable + kNumVirtual;

  struct HWCModuleMethods : public hw_module_methods_t {
    HWCModuleMethods() { hw_module_methods_t::open = HWCSession::Open; }
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

    if (display >= kNumDisplays) {
      return HWC2_ERROR_BAD_DISPLAY;
    }

    SCOPE_LOCK(locker_[display]);
    HWCSession *hwc_session = static_cast<HWCSession *>(device);
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

    if (display >= kNumDisplays) {
      return HWC2_ERROR_BAD_DISPLAY;
    }

    SCOPE_LOCK(locker_[display]);
    HWCSession *hwc_session = static_cast<HWCSession *>(device);
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
                              int32_t /*android_color_mode_t*/ int_mode);
  static int32_t SetColorTransform(hwc2_device_t *device, hwc2_display_t display,
                                   const float *matrix, int32_t /*android_color_transform_t*/ hint);
  static int32_t SetVsyncEnabled(hwc2_device_t *device, hwc2_display_t display,
                                 int32_t int_enabled);
  static int32_t GetDozeSupport(hwc2_device_t *device, hwc2_display_t display,
                                int32_t *out_support);

  static Locker locker_[kNumDisplays];

 private:
  struct DisplayMapInfo {
    hwc2_display_t client_id = kNumDisplays;        // mapped sf id for this display
    int32_t sdm_id = -1;                            // sdm id for this display
    sdm:: DisplayType disp_type = kDisplayTypeMax;  // sdm display type
    bool test_pattern = false;                      // display will show test pattern
    void Reset() {
      // Do not clear client id
      sdm_id = -1;
      disp_type = kDisplayTypeMax;
      test_pattern = false;
    }
  };

  static const int kExternalConnectionTimeoutMs = 500;
  static const int kPartialUpdateControlTimeoutMs = 100;

  // hwc methods
  static int Open(const hw_module_t *module, const char *name, hw_device_t **device);
  static int Close(hw_device_t *device);
  static void GetCapabilities(struct hwc2_device *device, uint32_t *outCount,
                              int32_t *outCapabilities);
  static hwc2_function_pointer_t GetFunction(struct hwc2_device *device, int32_t descriptor);

  // Uevent handler
  virtual void UEventHandler(const char *uevent_data, int length);
  void ResetPanel();
  void InitDisplaySlots();
  int GetDisplayIndex(int dpy);
  int CreatePrimaryDisplay();
  void CreateNullDisplay();
  int CreateBuiltInDisplays();
  int CreatePluggableDisplays(bool delay_hotplug);
  int HandleConnectedDisplays(HWDisplaysInfo *hw_displays_info, bool delay_hotplug);
  int HandleDisconnectedDisplays(HWDisplaysInfo *hw_displays_info);
  void DestroyDisplay(DisplayMapInfo *map_info);
  int GetVsyncPeriod(int disp);
  int32_t GetConfigCount(int disp_id, uint32_t *count);
  int32_t GetActiveConfigIndex(int disp_id, uint32_t *config);
  int32_t SetActiveConfigIndex(int disp_id, uint32_t config);
  int32_t ControlPartialUpdate(int dpy, bool enable);
  int32_t DisplayBWTransactionPending(bool *status);
  int32_t SetSecondaryDisplayStatus(int disp_id, HWCDisplay::DisplayStatus status);
  int32_t GetPanelBrightness(int *level);
  int32_t MinHdcpEncryptionLevelChanged(int disp_id, uint32_t min_enc_level);

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

  // Methods from ::android::hardware::display::config::V1_1::IDisplayConfig follow.
#ifdef DISPLAY_CONFIG_1_1
  Return<int32_t> setDisplayAnimating(uint64_t display_id, bool animating) override;
#endif
  // Methods from ::android::hardware::display::config::V1_2::IDisplayConfig follow.
#ifdef DISPLAY_CONFIG_1_2
  Return<int32_t> setDisplayIndex(IDisplayConfig::DisplayTypeExt disp_type,
                                  uint32_t base, uint32_t count) override;
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

  android::status_t SetColorModeById(const android::Parcel *input_parcel);
  android::status_t getComposerStatus();

  void Refresh(hwc2_display_t display);
  void HotPlug(hwc2_display_t display, HWC2::Connection state);
  void UpdateVsyncSource(hwc2_display_t display);
  void HandleConcurrency(hwc2_display_t disp);
  void ActivateDisplay(hwc2_display_t disp, bool enable);
  void NonBuiltinConcurrency(hwc2_display_t disp, bool is_built_in_2_on);
  void MapBuiltInDisplays();
  bool GetSecondBuiltinStatus();
  hwc2_display_t GetNextBuiltinIndex();

  CoreInterface *core_intf_ = nullptr;
  HWCDisplay *hwc_display_[kNumDisplays] = {nullptr};
  HWCDisplay *hwc_display_builtin_[kNumBuiltIn] = {nullptr};
  HWCCallbacks callbacks_;
  HWCBufferAllocator buffer_allocator_;
  HWCBufferSyncHandler buffer_sync_handler_;
  HWCColorManager *color_mgr_ = nullptr;
  DisplayMapInfo map_info_primary_;                 // Primary display (either builtin or pluggable)
  std::vector<DisplayMapInfo> map_info_builtin_;    // Builtin displays excluding primary
  std::vector<DisplayMapInfo> map_info_pluggable_;  // Pluggable displays excluding primary
  std::vector<DisplayMapInfo> map_info_virtual_;    // Virtual displays
  bool reset_panel_ = false;
  bool secure_display_active_ = false;
  bool primary_ready_ = false;
  bool client_connected_ = false;
  bool new_bw_mode_ = false;
  bool need_invalidate_ = false;
  int bw_mode_release_fd_ = -1;
  qService::QService *qservice_ = nullptr;
  HWCSocketHandler socket_handler_;
  bool hdmi_is_primary_ = false;
  bool is_composer_up_ = false;
  Locker callbacks_lock_;
  int hpd_bpp_ = 0;
  int hpd_pattern_ = 0;
};

}  // namespace sdm

#endif  // __HWC_SESSION_H__
