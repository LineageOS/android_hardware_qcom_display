/*
* Copyright (c) 2015 - 2021, The Linux Foundation. All rights reserved.
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

#ifndef __RESOURCE_INTERFACE_H__
#define __RESOURCE_INTERFACE_H__

#include <core/display_interface.h>
#include <map>
#include <vector>
#include "hw_info_types.h"
#include "layer_feedback.h"

namespace sdm {

class ResourceInterface {
 public:
  enum ResourceCmd {
    kCmdResetLUT,
    kCmdGetDefaultQosData,
    kCmdDisableRotatorOneFrame,
    kCmdSetDisplayState,
    kCmdUpdateSyncHandle,
    kCmdCheckEnforceSplit,
    kCmdDedicatePipes,
    kCmdGetResourceStatus,
    kCmdGetRetireFence,
    kCmdNeedsValidate,
    kCmdSetBacklightLevel,
    kCmdSetCwbBoost,
    kCmdGetResourceConstraints,
    kCmdMax,
  };

  virtual DisplayError ReserveDisplay(DisplayType type) = 0;
  virtual DisplayError RegisterDisplay(int32_t display_id, DisplayType type,
                                       const HWDisplayAttributes &display_attributes,
                                       const HWPanelInfo &hw_panel_info,
                                       const HWMixerAttributes &mixer_attributes,
                                       const Resolution &fb_resolution,
                                       Handle *display_ctx) = 0;
  virtual DisplayError UnregisterDisplay(Handle display_ctx) = 0;
  virtual DisplayError ReconfigureDisplay(Handle display_ctx,
                                          const HWDisplayAttributes &display_attributes,
                                          const HWPanelInfo &hw_panel_info,
                                          const HWMixerAttributes &mixer_attributes,
                                          const Resolution &fb_resolution) = 0;
  virtual DisplayError Start(Handle display_ctx, LayerStack *layer_stack) = 0;
  virtual DisplayError Precheck(Handle display_ctx, DispLayerStack* disp_layer_stack,
                                LayerFeedback* feedback) = 0;
  virtual DisplayError Stop(Handle display_ctx, DispLayerStack *disp_layer_stack) = 0;
  virtual DisplayError SetDrawMethod(Handle display_ctx, const DisplayDrawMethod &draw_method) = 0;
  virtual DisplayError Prepare(Handle display_ctx, DispLayerStack *disp_layer_stack,
                               LayerFeedback *feedback) = 0;
  virtual DisplayError PostPrepare(Handle display_ctx, DispLayerStack *disp_layer_stack) = 0;
  virtual DisplayError Commit(Handle display_ctx, DispLayerStack *disp_layer_stack) = 0;
  virtual DisplayError PostCommit(Handle display_ctx, DispLayerStack *disp_layer_stack) = 0;
  virtual void Purge(Handle display_ctx) = 0;
  virtual DisplayError SetMaxMixerStages(Handle display_ctx, uint32_t max_mixer_stages) = 0;
  virtual DisplayError ValidateScaling(const LayerRect &crop, const LayerRect &dst, bool rotate90,
                                       BufferLayout layout, bool use_rotator_downscale) = 0;
  virtual DisplayError ValidateAndSetCursorPosition(Handle display_ctx,
                                                    DispLayerStack *disp_layer_stack, int x, int y,
                                                    DisplayConfigVariableInfo *fb_config) = 0;
  virtual DisplayError SetMaxBandwidthMode(HWBwModes mode) = 0;
  virtual DisplayError GetScaleLutConfig(HWScaleLutInfo *lut_info) = 0;
  virtual DisplayError SetDetailEnhancerData(Handle display_ctx,
                                             const DisplayDetailEnhancerData &de_data) = 0;
  virtual DisplayError UpdateSyncHandle(Handle display_ctx, const SyncPoints &sync_points) = 0;
  virtual DisplayError Perform(int cmd, ...) = 0;
  virtual bool IsRotatorSupportedFormat(LayerBufferFormat format) = 0;
  virtual DisplayError FreeDemuraFetchResources(const int32_t &display_id) = 0;
  virtual DisplayError GetDemuraFetchResourceCount(
                       std::map<uint32_t, uint8_t> *fetch_resource_cnt) = 0;
  virtual DisplayError ReserveDemuraFetchResources(const int32_t &display_id,
                                                   const int8_t &preferred_rect) = 0;
  virtual DisplayError GetDemuraFetchResources(Handle display_ctx, FetchResourceList *frl) = 0;
  virtual ~ResourceInterface() {}
  virtual DisplayError SetMaxSDEClk(Handle display_ctx, uint32_t clk) = 0;
  virtual DisplayError ForceToneMapConfigure(Handle display_ctx,
                                             DispLayerStack *disp_layer_stack) = 0;
  virtual bool IsDisplayHWAvailable() = 0;
};

}  // namespace sdm

#endif  // __RESOURCE_INTERFACE_H__
