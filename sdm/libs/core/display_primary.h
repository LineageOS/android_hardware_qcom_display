/*
* Copyright (c) 2014 - 2018, The Linux Foundation. All rights reserved.
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

#ifndef __DISPLAY_PRIMARY_H__
#define __DISPLAY_PRIMARY_H__

#include <core/dpps_interface.h>
#include <vector>
#include <string>

#include "display_base.h"
#include "hw_events_interface.h"

namespace sdm {

class HWPrimaryInterface;

class DppsInfo {
 public:
  void Init(DppsPropIntf* intf, const std::string &panel_name);
  void Deinit();
  void DppsNotifyOps(enum DppsNotifyOps op, void *payload, size_t size);

 private:
  const char *kDppsLib = "libdpps.so";
  DynLib dpps_impl_lib;
  DppsInterface* dpps_intf = NULL;
  DppsInterface* (*GetDppsInterface)() = NULL;
  bool dpps_initialized_ = false;
};

class DisplayPrimary : public DisplayBase, HWEventHandler, DppsPropIntf {
 public:
  DisplayPrimary(DisplayEventHandler *event_handler, HWInfoInterface *hw_info_intf,
                 BufferSyncHandler *buffer_sync_handler, BufferAllocator *buffer_allocator,
                 CompManager *comp_manager);
  virtual DisplayError Init();
  virtual DisplayError Deinit();
  virtual DisplayError Prepare(LayerStack *layer_stack);
  virtual DisplayError Commit(LayerStack *layer_stack);
  virtual DisplayError ControlPartialUpdate(bool enable, uint32_t *pending);
  virtual DisplayError DisablePartialUpdateOneFrame();
  virtual DisplayError SetDisplayState(DisplayState state, int *release_fence);
  virtual void SetIdleTimeoutMs(uint32_t active_ms);
  virtual DisplayError SetDisplayMode(uint32_t mode);
  virtual DisplayError GetRefreshRateRange(uint32_t *min_refresh_rate, uint32_t *max_refresh_rate);
  virtual DisplayError SetRefreshRate(uint32_t refresh_rate, bool final_rate);
  virtual DisplayError SetPanelBrightness(int level);
  virtual DisplayError GetPanelBrightness(int *level);
  virtual DisplayError HandleSecureEvent(SecureEvent secure_event);
  virtual DisplayError SetQSyncMode(QSyncMode qsync_mode);

  // Implement the HWEventHandlers
  virtual DisplayError VSync(int64_t timestamp);
  virtual DisplayError Blank(bool blank) { return kErrorNone; }
  virtual void IdleTimeout();
  virtual void ThermalEvent(int64_t thermal_level);
  virtual void CECMessage(char *message) { }
  virtual void IdlePowerCollapse();
  virtual void PingPongTimeout();
  virtual void PanelDead();
  virtual void HwRecovery(const HWRecoveryEvent sdm_event_code);

  // Implement the DppsPropIntf
  virtual DisplayError DppsProcessOps(enum DppsOps op, void *payload, size_t size);

 private:
  bool NeedsAVREnable();
  void ResetPanel();

  std::vector<HWEvent> event_list_;
  bool avr_prop_disabled_ = false;
  bool switch_to_cmd_ = false;
  bool handle_idle_timeout_ = false;
  uint32_t current_refresh_rate_ = 0;
  bool reset_panel_ = false;
  bool commit_event_enabled_ = false;
  DppsInfo dpps_info_ = {};
  QSyncMode qsync_mode_ = kQSyncModeNone;
};

}  // namespace sdm

#endif  // __DISPLAY_PRIMARY_H__

