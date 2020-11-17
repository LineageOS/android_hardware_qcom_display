/*
* Copyright (c) 2014 - 2020, The Linux Foundation. All rights reserved.
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

#ifndef __DISPLAY_BUILTIN_H__
#define __DISPLAY_BUILTIN_H__

#include <core/dpps_interface.h>
#include <string>
#include <vector>

#include "display_base.h"
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
  void Init(DppsPropIntf *intf, const std::string &panel_name);
  void Deinit();
  void DppsNotifyOps(enum DppsNotifyOps op, void *payload, size_t size);

 private:
  const char *kDppsLib = "libdpps.so";
  DynLib dpps_impl_lib;
  DppsInterface *dpps_intf = NULL;
  DppsInterface *(*GetDppsInterface)() = NULL;
  bool dpps_initialized_ = false;
};

class DisplayBuiltIn : public DisplayBase, HWEventHandler, DppsPropIntf {
 public:
  DisplayBuiltIn(DisplayEventHandler *event_handler, HWInfoInterface *hw_info_intf,
                 BufferSyncHandler *buffer_sync_handler, BufferAllocator *buffer_allocator,
                 CompManager *comp_manager);
  DisplayBuiltIn(int32_t display_id, DisplayEventHandler *event_handler,
                 HWInfoInterface *hw_info_intf, BufferSyncHandler *buffer_sync_handler,
                 BufferAllocator *buffer_allocator, CompManager *comp_manager);
  virtual ~DisplayBuiltIn();

  virtual DisplayError Init();
  virtual DisplayError Deinit();
  virtual DisplayError Prepare(LayerStack *layer_stack);
  virtual DisplayError Commit(LayerStack *layer_stack);
  virtual DisplayError ControlPartialUpdate(bool enable, uint32_t *pending);
  virtual DisplayError DisablePartialUpdateOneFrame();
  virtual DisplayError SetDisplayState(DisplayState state, bool teardown,
                                       int *release_fence);
  virtual void SetIdleTimeoutMs(uint32_t active_ms);
  virtual DisplayError SetDisplayMode(uint32_t mode);
  virtual DisplayError GetRefreshRateRange(uint32_t *min_refresh_rate, uint32_t *max_refresh_rate);
  virtual DisplayError SetRefreshRate(uint32_t refresh_rate, bool final_rate);
  virtual DisplayError GetRefreshRate(uint32_t *refresh_rate);
  virtual DisplayError SetPanelBrightness(float brightness);
  virtual DisplayError GetPanelBrightness(float *brightness);
  virtual DisplayError HandleSecureEvent(SecureEvent secure_event, LayerStack *layer_stack);
  virtual DisplayError SetDisplayDppsAdROI(void *payload);
  virtual DisplayError SetQSyncMode(QSyncMode qsync_mode);
  virtual DisplayError ControlIdlePowerCollapse(bool enable, bool synchronous);
  virtual DisplayError SetDynamicDSIClock(uint64_t bit_clk_rate);
  virtual DisplayError GetDynamicDSIClock(uint64_t *bit_clk_rate);
  virtual DisplayError GetSupportedDSIClock(std::vector<uint64_t> *bitclk_rates);
  virtual DisplayError GetConfig(DisplayConfigFixedInfo *fixed_info);

  // Implement the HWEventHandlers
  virtual DisplayError VSync(int64_t timestamp);
  virtual DisplayError Blank(bool blank) { return kErrorNone; }
  virtual void IdleTimeout();
  virtual void ThermalEvent(int64_t thermal_level);
  virtual void CECMessage(char *message) {}
  virtual void IdlePowerCollapse();
  virtual void PingPongTimeout();
  virtual void PanelDead();
  virtual void HwRecovery(const HWRecoveryEvent sdm_event_code);
  virtual DisplayError TeardownConcurrentWriteback(void);

  // Implement the DppsPropIntf
  virtual DisplayError DppsProcessOps(enum DppsOps op, void *payload, size_t size);
  virtual DisplayError SetActiveConfig(uint32_t index);
  virtual DisplayError ReconfigureDisplay();

 private:
  bool CanCompareFrameROI(LayerStack *layer_stack);
  bool CanSkipDisplayPrepare(LayerStack *layer_stack);
  HWAVRModes GetAvrMode(QSyncMode mode);
  bool CanDeferFpsConfig(uint32_t fps);
  void SetDeferredFpsConfig();
  void GetFpsConfig(HWDisplayAttributes *display_attributes, HWPanelInfo *panel_info);
  void UpdateDisplayModeParams();

  std::vector<HWEvent> event_list_;
  bool avr_prop_disabled_ = false;
  bool switch_to_cmd_ = false;
  bool handle_idle_timeout_ = false;
  bool commit_event_enabled_ = false;
  DppsInfo dpps_info_ = {};
  LayerRect left_frame_roi_ = {};
  LayerRect right_frame_roi_ = {};
  bool first_cycle_ = true;
  int previous_retire_fence_ = -1;
  DeferFpsConfig deferred_config_ = {};
  float level_remainder_ = 0.0f;
  recursive_mutex brightness_lock_;
  float cached_brightness_ = 0.0f;
  bool pending_brightness_ = false;
};

}  // namespace sdm

#endif  // __DISPLAY_BUILTIN_H__
