/*
* Copyright (c) 2014 - 2021, The Linux Foundation. All rights reserved.
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

/* Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef __COMP_MANAGER_H__
#define __COMP_MANAGER_H__

#include <core/display_interface.h>
#include <private/extension_interface.h>
#include <utils/locker.h>
#include <bitset>
#include <set>
#include <vector>
#include <string>
#include <map>
#include <mutex>

#include "strategy.h"
#include "resource_default.h"
#include "hw_interface.h"

namespace sdm {

class CompManager {
 public:
  DisplayError Init(const HWResourceInfo &hw_res_info_, ExtensionInterface *extension_intf,
                    BufferAllocator *buffer_allocator, SocketHandler *socket_handler);
  DisplayError Deinit();
  DisplayError ReserveDisplay(DisplayType type);
  DisplayError RegisterDisplay(int32_t display_id, DisplayType type,
                               const HWDisplayAttributes &display_attributes,
                               const HWPanelInfo &hw_panel_info,
                               const HWMixerAttributes &mixer_attributes,
                               const DisplayConfigVariableInfo &fb_config, Handle *display_ctx,
                               HWQosData *qos_data);
  DisplayError UnregisterDisplay(Handle display_ctx);
  DisplayError ReconfigureDisplay(Handle display_ctx, const HWDisplayAttributes &display_attributes,
                                  const HWPanelInfo &hw_panel_info,
                                  const HWMixerAttributes &mixer_attributes,
                                  const DisplayConfigVariableInfo &fb_config,
                                  HWQosData *qos_data);
  DisplayError PrePrepare(Handle display_ctx, DispLayerStack *disp_layer_stack);
  DisplayError Prepare(Handle display_ctx, DispLayerStack *disp_layer_stack);
  DisplayError Commit(Handle display_ctx, DispLayerStack *disp_layer_stack);
  DisplayError PostPrepare(Handle display_ctx, DispLayerStack *disp_layer_stack);
  DisplayError PostCommit(Handle display_ctx, DispLayerStack *disp_layer_stack);
  void Purge(Handle display_ctx);
  DisplayError SetIdleTimeoutMs(Handle display_ctx, uint32_t active_ms, uint32_t inactive_ms);
  void ProcessIdleTimeout(Handle display_ctx);
  void ProcessThermalEvent(Handle display_ctx, int64_t thermal_level);
  void ProcessIdlePowerCollapse(Handle display_ctx);
  DisplayError SetMaxMixerStages(Handle display_ctx, uint32_t max_mixer_stages);
  void ControlPartialUpdate(Handle display_ctx, bool enable);
  DisplayError ValidateScaling(const LayerRect &crop, const LayerRect &dst, bool rotate90);
  DisplayError ValidateAndSetCursorPosition(Handle display_ctx, DispLayerStack *disp_layer_stack,
                                            int x, int y);
  bool SetDisplayState(Handle display_ctx, DisplayState state, const SyncPoints &sync_points);
  DisplayError SetMaxBandwidthMode(HWBwModes mode);
  DisplayError GetScaleLutConfig(HWScaleLutInfo *lut_info);
  DisplayError SetDetailEnhancerData(Handle display_ctx, const DisplayDetailEnhancerData &de_data);
  DisplayError SetCompositionState(Handle display_ctx, LayerComposition composition_type,
                                   bool enable);
  DisplayError ControlDpps(bool enable);
  DisplayError SetColorModesInfo(Handle display_ctx,
                                 const std::vector<PrimariesTransfer> &colormodes_cs);
  DisplayError SetBlendSpace(Handle display_ctx, const PrimariesTransfer &blend_space);
  void HandleSecureEvent(Handle display_ctx, SecureEvent secure_event);
  void SetSafeMode(bool enable);
  bool IsSafeMode();
  void GenerateROI(Handle display_ctx, DispLayerStack *disp_layer_stack);
  DisplayError CheckEnforceSplit(Handle comp_handle, uint32_t new_refresh_rate);
  DppsControlInterface* GetDppsControlIntf();
  bool CheckResourceState(Handle display_ctx, bool *res_exhausted, HWDisplayAttributes attr);
  DisplayError GetConcurrencyFps(DisplayConcurrencyType type, float *fps);
  bool IsRotatorSupportedFormat(LayerBufferFormat format);
  DisplayError SetDrawMethod(Handle display_ctx, const DisplayDrawMethod &draw_method);
  DisplayError FreeDemuraFetchResources(const uint32_t &display_id);
  DisplayError GetDemuraFetchResourceCount(std::map<uint32_t, uint8_t> *fetch_resource_cnt);
  DisplayError ReserveDemuraFetchResources(const uint32_t &display_id,
                                           const int8_t &preferred_rect);
  DisplayError GetDemuraFetchResources(Handle display_ctx, FetchResourceList *frl);
  void SetDemuraStatus(bool status);
  bool GetDemuraStatus();
  void SetDemuraStatusForDisplay(const int32_t &display_id, bool status);
  bool GetDemuraStatusForDisplay(const int32_t &display_id);
  DisplayError SetMaxSDEClk(Handle display_ctx, uint32_t clk);
  void GetRetireFence(Handle display_ctx, shared_ptr<Fence> *retire_fence);
  void NeedsValidate(Handle display_ctx, bool *needs_validate);
  DisplayError SetBacklightLevel(Handle display_ctx, const uint32_t &backlight_level);
  DisplayError GetHDR10PlusCapability(bool *hdr_plus_support);
  DisplayError ForceToneMapConfigure(Handle display_ctx, DispLayerStack *disp_layer_stack);
  DisplayError GetDefaultQosData(Handle display_ctx, HWQosData *qos_data);
  DisplayError HandleCwbFrequencyBoost(bool isRequest);
  bool IsDisplayHWAvailable();

 private:
  static const int kMaxThermalLevel = 3;
  static const int kSafeModeThreshold = 4;

  void PrepareStrategyConstraints(Handle display_ctx, DispLayerStack *disp_layer_stack);
  void UpdateStrategyConstraints(bool is_primary, bool disabled);
  std::string StringDisplayList(const std::set<int32_t> &displays);

  struct DisplayCompositionContext {
    Strategy *strategy = NULL;
    StrategyConstraints constraints;
    Handle display_resource_ctx = NULL;
    int32_t display_id = -1;
    DisplayType display_type = kBuiltIn;
    uint32_t max_strategies = 0;
    uint32_t remaining_strategies = 0;
    bool idle_fallback = false;
    // Using primary panel flag of hw panel to configure Constraints. We do not need other hw
    // panel parameters for now.
    bool is_primary_panel = false;
    PUConstraints pu_constraints = {};
    DisplayConfigVariableInfo fb_config = {};
    bool first_cycle_ = true;
    uint32_t dest_scaler_blocks_used = 0;
  };

  std::recursive_mutex comp_mgr_mutex_;
  ResourceInterface *resource_intf_ = NULL;
  std::set<int32_t> registered_displays_;  // List of registered displays
  std::set<int32_t> configured_displays_;  // List of sucessfully configured displays
  std::set<int32_t> powered_on_displays_;  // List of powered on displays.
  bool safe_mode_ = false;              // Flag to notify all displays to be in resource crunch
                                        // mode, where strategy manager chooses the best strategy
                                        // that uses optimal number of pipes for each display
  HWResourceInfo hw_res_info_;
  BufferAllocator *buffer_allocator_ = NULL;
  ExtensionInterface *extension_intf_ = NULL;
  CapabilitiesInterface *cap_intf_ = nullptr;
  uint32_t max_sde_secondary_fetch_layers_ = 2;
  uint32_t max_sde_builtin_fetch_layers_ = 2;
  uint32_t max_primary_layers_ = 0;
  DppsControlInterface *dpps_ctrl_intf_ = NULL;
  bool demura_enabled_ = false;
  std::map<int32_t /* display_id */, bool> display_demura_status_;
  SecureEvent secure_event_ = kSecureEventMax;
  bool force_gpu_comp_ = false;
};

}  // namespace sdm

#endif  // __COMP_MANAGER_H__
