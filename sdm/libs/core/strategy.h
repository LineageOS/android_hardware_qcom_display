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

#ifndef __STRATEGY_H__
#define __STRATEGY_H__

#include <core/display_interface.h>
#include <private/extension_interface.h>
#include <core/buffer_allocator.h>
#include <vector>

namespace sdm {

class Strategy {
 public:
  Strategy(ExtensionInterface *extension_intf, BufferAllocator *buffer_allocator,
           int32_t display_id, DisplayType type, const HWResourceInfo &hw_resource_info,
           const HWPanelInfo &hw_panel_info, const HWMixerAttributes &mixer_attributes,
           const HWDisplayAttributes &display_attributes,
           const DisplayConfigVariableInfo &fb_config);

  DisplayError Init();
  DisplayError Deinit();

  DisplayError Start(HWLayersInfo *hw_layers_info, uint32_t *max_attempts);
  DisplayError GetNextStrategy(StrategyConstraints *constraints);
  DisplayError Stop();
  DisplayError Reconfigure(const HWPanelInfo &hw_panel_info,
                           const HWDisplayAttributes &hw_display_attributes,
                           const HWMixerAttributes &mixer_attributes,
                           const DisplayConfigVariableInfo &fb_config);
  DisplayError SetCompositionState(LayerComposition composition_type, bool enable);
  DisplayError Purge();
  DisplayError SetIdleTimeoutMs(uint32_t active_ms, uint32_t inactive_ms);
  DisplayError SetColorModesInfo(const std::vector<PrimariesTransfer> &colormodes_cs);
  DisplayError SetBlendSpace(const PrimariesTransfer &blend_space);
  bool CanSkipValidate(bool *needs_buffer_swap);
  void GenerateROI(HWLayersInfo *hw_layers_info, const PUConstraints &pu_constraints);
  DisplayError SwapBuffers();

 private:
  void GenerateROI();

  ExtensionInterface *extension_intf_ = NULL;
  StrategyInterface *strategy_intf_ = NULL;
  PartialUpdateInterface *partial_update_intf_ = NULL;
  int32_t display_id_;
  DisplayType display_type_;
  HWResourceInfo hw_resource_info_;
  HWPanelInfo hw_panel_info_;
  HWLayersInfo *hw_layers_info_ = NULL;
  HWMixerAttributes mixer_attributes_ = {};
  HWDisplayAttributes display_attributes_ = {};
  DisplayConfigVariableInfo fb_config_ = {};
  bool extn_start_success_ = false;
  bool disable_gpu_comp_ = false;
  BufferAllocator *buffer_allocator_ = NULL;
};

}  // namespace sdm

#endif  // __STRATEGY_H__

