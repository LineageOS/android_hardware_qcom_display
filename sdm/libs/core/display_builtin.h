/*
* Copyright (c) 2014 - 2022, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted
* provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright notice, this list of
*      conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright notice, this list of
*      conditions and the following disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation nor the names of its contributors may be used to
*      endorse or promote products derived from this software without specific prior written
*      permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef __DISPLAY_BUILTIN_H__
#define __DISPLAY_BUILTIN_H__

#include <sys/time.h>
#include <sys/stat.h>

#include <core/dpps_interface.h>
#include <core/ipc_interface.h>
#include <private/extension_interface.h>
#include <private/spr_intf.h>
#include <private/panel_feature_property_intf.h>
#include <private/panel_feature_factory_intf.h>
#include <string>
#include <vector>

#include "display_base.h"
#include "drm_interface.h"
#include "hw_events_interface.h"

namespace sdm {

struct DeferFpsConfig {
  uint32_t frame_count = 0;
  uint32_t frames_to_defer = 0;
  uint32_t fps = 0;
  uint32_t vsync_period_ns = 0;
  uint32_t transfer_time_us = 0;
  bool dirty = false;
  bool apply = false;

  void Init(uint32_t refresh_rate, uint32_t vsync_period, uint32_t transfer_time) {
    fps = refresh_rate;
    vsync_period_ns = vsync_period;
    transfer_time_us = transfer_time;
    frames_to_defer = frame_count;
    dirty = false;
    apply = false;
  }

  bool IsDeferredState() { return (frames_to_defer != 0); }

  bool CanApplyDeferredState() { return apply; }

  bool IsDirty() { return dirty; }

  void MarkDirty() { dirty = IsDeferredState(); }

  void UpdateDeferCount() {
    if (frames_to_defer > 0) {
      frames_to_defer--;
      apply = (frames_to_defer == 0);
    }
  }

  void Clear() {
    frames_to_defer = 0;
    dirty = false;
    apply = false;
  }
};

class DppsInfo {
 public:
  void Init(DppsPropIntf *intf, const std::string &panel_name, DisplayInterface *display_intf);
  void Deinit();
  void DppsNotifyOps(enum DppsNotifyOps op, void *payload, size_t size);
  bool disable_pu_ = false;

 private:
  const char *kDppsLib_ = "libdpps.so";
  DynLib dpps_impl_lib_;
  static DppsInterface *dpps_intf_;
  static std::vector<int32_t> display_id_;
  std::mutex lock_;
  DppsInterface *(*GetDppsInterface)() = NULL;
};

class DisplayIPCVmCallbackImpl : public IPCVmCallbackIntf {
 public:
  DisplayIPCVmCallbackImpl(BufferAllocator *buffer_allocator,
                               std::shared_ptr<IPCIntf> ipc_intf,
                               uint64_t panel_id, uint32_t width, uint32_t height);
  void Init();
  void Deinit();
  void OnServerReady();
  void OnServerExit();
  void ExportHFCBuffer();
  void FreeExportBuffer();
  virtual ~DisplayIPCVmCallbackImpl() {}

 private:
  IPCExportBufOutParams export_buf_out_params_ = {};
  BufferAllocator *buffer_allocator_ {};
  int *cb_hnd_out_ = nullptr;
  std::shared_ptr<IPCIntf> ipc_intf_ = nullptr;
  BufferInfo buffer_info_hfc_ = {};
  uint64_t panel_id_ = 0;
  bool server_ready_ = false;
  uint32_t hfc_buffer_width_ = 0;
  uint32_t hfc_buffer_height_ = 0;
  recursive_mutex cb_mutex_;
};

class DisplayBuiltIn : public DisplayBase, HWEventHandler, DppsPropIntf {
 public:
  DisplayBuiltIn(DisplayEventHandler *event_handler, HWInfoInterface *hw_info_intf,
                 BufferAllocator *buffer_allocator, CompManager *comp_manager,
                 std::shared_ptr<IPCIntf> ipc_intf);
  DisplayBuiltIn(int32_t display_id, DisplayEventHandler *event_handler,
                 HWInfoInterface *hw_info_intf, BufferAllocator *buffer_allocator,
                 CompManager *comp_manager, std::shared_ptr<IPCIntf> ipc_intf);
  virtual ~DisplayBuiltIn();

  DisplayError Init() override;
  DisplayError Deinit() override;
  DisplayError Prepare(LayerStack *layer_stack) override;
  DisplayError ControlPartialUpdate(bool enable, uint32_t *pending) override;
  DisplayError DisablePartialUpdateOneFrame() override;
  DisplayError DisablePartialUpdateOneFrameInternal() override;
  DisplayError SetDisplayState(DisplayState state, bool teardown,
                               shared_ptr<Fence> *release_fence) override;
  void SetIdleTimeoutMs(uint32_t active_ms, uint32_t inactive_ms) override;
  DisplayError SetDisplayMode(uint32_t mode) override;
  DisplayError GetRefreshRateRange(uint32_t *min_refresh_rate,
                                   uint32_t *max_refresh_rate) override;
  DisplayError SetRefreshRate(uint32_t refresh_rate, bool final_rate, bool idle_screen) override;
  DisplayError SetPanelBrightness(float brightness) override;
  DisplayError GetPanelBrightness(float *brightness) override;
  DisplayError GetPanelBrightnessFromLevel(float level, float *brightness);
  DisplayError GetPanelMaxBrightness(uint32_t *max_brightness_level) override;
  DisplayError GetRefreshRate(uint32_t *refresh_rate) override;
  DisplayError SetDisplayDppsAdROI(void *payload) override;
  DisplayError SetQSyncMode(QSyncMode qsync_mode) override;
  DisplayError ControlIdlePowerCollapse(bool enable, bool synchronous) override;
  DisplayError SetDynamicDSIClock(uint64_t bit_clk_rate) override;
  DisplayError GetDynamicDSIClock(uint64_t *bit_clk_rate) override;
  DisplayError GetSupportedDSIClock(std::vector<uint64_t> *bitclk_rates) override;
  DisplayError SetFrameTriggerMode(FrameTriggerMode mode) override;
  DisplayError SetBLScale(uint32_t level) override;
  DisplayError GetQSyncMode(QSyncMode *qsync_mode) override;
  DisplayError colorSamplingOn() override;
  DisplayError colorSamplingOff() override;
  DisplayError GetStcColorModes(snapdragoncolor::ColorModeList *mode_list) override;
  DisplayError SetStcColorMode(const snapdragoncolor::ColorMode &color_mode) override;
  DisplayError NotifyDisplayCalibrationMode(bool in_calibration) override;
  bool HasDemura() override { return demura_intended_; }
  std::string Dump() override;
  DisplayError GetConfig(DisplayConfigFixedInfo *fixed_info) override;
  DisplayError PrePrepare(LayerStack *layer_stack) override;
  DisplayError SetAlternateDisplayConfig(uint32_t *alt_config) override;
  DisplayError PostHandleSecureEvent(SecureEvent secure_event) override;
  void InitCWBBuffer();
  void AppendCWBLayer(LayerStack *layer_stack);
  uint32_t GetUpdatingAppLayersCount(LayerStack *layer_stack);
  DisplayError ChangeFps();
  uint32_t GetUpdatingLayersCount();
  uint32_t GetOptimalRefreshRate(bool one_updating_layer);
  uint32_t CalculateMetaDataRefreshRate();
  uint32_t SanitizeRefreshRate(uint32_t req_refresh_rate, uint32_t max_refresh_rate,
                               uint32_t min_refresh_rate);

  // Implement the HWEventHandlers
  DisplayError VSync(int64_t timestamp) override;
  DisplayError Blank(bool blank) override { return kErrorNone; }
  void IdleTimeout() override;
  void CECMessage(char *message) override {}
  void IdlePowerCollapse() override;
  void PingPongTimeout() override;
  void PanelDead() override;
  void HwRecovery(const HWRecoveryEvent sdm_event_code) override;
  void MMRMEvent(uint32_t clk) override;
  DisplayError ClearLUTs() override;
  void Histogram(int histogram_fd, uint32_t blob_id) override;
  void HandleBacklightEvent(float brightness_level) override;
  void HandlePowerEvent() override;
  void HandleVmReleaseEvent() override;

  // Implement the DppsPropIntf
  DisplayError DppsProcessOps(enum DppsOps op, void *payload, size_t size) override;
  DisplayError SetActiveConfig(uint32_t index) override;
  DisplayError ReconfigureDisplay() override;
  DisplayError CreatePanelfeatures();
  DisplayError CommitLocked(LayerStack *layer_stack) override;
  DisplayError SetUpCommit(LayerStack *layer_stack) override;
  DisplayError PostCommit(HWLayersInfo *hw_layers_info) override;
  DisplayError GetQsyncFps(uint32_t *qsync_fps) override;

 private:
  bool CanCompareFrameROI(LayerStack *layer_stack);
  bool CanSkipDisplayPrepare(LayerStack *layer_stack);
  HWAVRModes GetAvrMode(QSyncMode mode);
  bool CanDeferFpsConfig(uint32_t fps);
  void SetDeferredFpsConfig();
  void GetFpsConfig(HWDisplayAttributes *display_attributes, HWPanelInfo *panel_info);
  PrimariesTransfer GetBlendSpaceFromStcColorMode(const snapdragoncolor::ColorMode &color_mode);
  DisplayError SetupSPR();
  DisplayError SetupDemura();
  DisplayError SetupDemuraLayer();
  DisplayError BuildLayerStackStats(LayerStack *layer_stack) override;
  void UpdateDisplayModeParams();
  void HandleQsyncPostCommit();
  void UpdateQsyncMode();
  void SetVsyncStatus(bool enable);
  void SendBacklight();
  void SendDisplayConfigs();
  bool CanLowerFps(bool idle_screen);
  int SetDemuraIntfStatus(bool enable);
  DisplayError HandleSPR();
  void CacheFrameROI();
  void PreCommit(LayerStack *layer_stack);
  DisplayError ControlPartialUpdateLocked(bool enable, uint32_t *pending);
  DisplayError SetDppsFeatureLocked(void *payload, size_t size);
  DisplayError HandleDemuraLayer(LayerStack *layer_stack);
  void NotifyDppsHdrPresent(LayerStack *layer_stack);
  bool IdleFallbackLowerFps(bool idle_screen);

  const uint32_t kPuTimeOutMs = 1000;
  std::vector<HWEvent> event_list_;
  bool avr_prop_disabled_ = false;
  bool switch_to_cmd_ = false;
  bool commit_event_enabled_ = false;
  bool reset_panel_ = false;
  bool panel_feature_init_ = false;
  bool disable_dyn_fps_ = false;
  DppsInfo dpps_info_ = {};
  // Posted Start is default mode
  FrameTriggerMode trigger_mode_debug_ = kFrameTriggerPostedStart;
  float level_remainder_ = 0.0f;
  float cached_brightness_ = 0.0f;
  bool pending_brightness_ = false;
  recursive_mutex brightness_lock_;
  LayerRect left_frame_roi_ = {};
  LayerRect right_frame_roi_ = {};
  Locker dpps_pu_lock_;
  bool dpps_pu_nofiy_pending_ = false;
  enum class SamplingState { Off, On } samplingState = SamplingState::Off;
  DisplayError setColorSamplingState(SamplingState state);

  bool histogramSetup = false;
  sde_drm::DppsFeaturePayload histogramCtrl;
  sde_drm::DppsFeaturePayload histogramIRQ;
  void initColorSamplingState();
  DeferFpsConfig deferred_config_ = {};
  snapdragoncolor::ColorMode current_color_mode_ = {};
  snapdragoncolor::ColorModeList stc_color_modes_ = {};

  std::shared_ptr<SPRIntf> spr_ = nullptr;
  bool needs_validate_on_pu_enable_ = false;
  bool enable_qsync_idle_ = false;
  bool pending_vsync_enable_ = false;
  QSyncMode active_qsync_mode_ = kQSyncModeNone;
  std::shared_ptr<IPCIntf> ipc_intf_ = nullptr;
  bool enhance_idle_time_ = false;
  int idle_time_ms_ = 0;
  struct timespec idle_timer_start_;
  std::unique_ptr<DemuraIntf> demura_ = nullptr;
  Layer demura_layer_ = {};
  bool demura_intended_ = false;
  bool enable_dpps_dyn_fps_ = false;
  HWDisplayMode last_panel_mode_ = kModeDefault;
  bool hdr_present_ = false;
  bool qsync_enabled_ = false;
  uint32_t hfc_buffer_width_ = 0;
  uint32_t hfc_buffer_height_ = 0;
  int hfc_buffer_fd_ = -1;
  uint32_t hfc_buffer_size_ = 0;
  DisplayIPCVmCallbackImpl *vm_cb_intf_ = nullptr;
  Layer cwb_layer_ = {};
  bool lower_fps_ = false;
  bool cwb_buffer_inited_ = false;
};

}  // namespace sdm

#endif  // __DISPLAY_BUILTIN_H__
