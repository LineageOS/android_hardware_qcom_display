/*
* Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
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

#include <core/buffer_allocator.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <set>
#include <string>
#include <vector>
#include <map>
#include <utility>

#include "comp_manager.h"
#include "strategy.h"

#define __CLASS__ "CompManager"

namespace sdm {

DisplayError CompManager::Init(const HWResourceInfo &hw_res_info,
                               ExtensionInterface *extension_intf,
                               BufferAllocator *buffer_allocator,
                               SocketHandler *socket_handler) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  DisplayError error = kErrorNone;

  if (extension_intf) {
    error = extension_intf->CreateResourceExtn(hw_res_info, buffer_allocator, &resource_intf_);
    extension_intf->CreateDppsControlExtn(&dpps_ctrl_intf_, socket_handler);
    extension_intf->CreateCapabilitiesExtn(&cap_intf_);
  } else {
    error = ResourceDefault::CreateResourceDefault(hw_res_info, &resource_intf_);
  }

  if (error != kErrorNone) {
    if (extension_intf) {
      extension_intf->DestroyDppsControlExtn(dpps_ctrl_intf_);
      extension_intf->DestroyCapabilitiesExtn(cap_intf_);
    }
    return error;
  }

  hw_res_info_ = hw_res_info;
  buffer_allocator_ = buffer_allocator;
  extension_intf_ = extension_intf;

  int value = 0;
  if (DebugHandler::Get()->GetProperty(MAX_PRIMARY_LAYERS, &value) == kErrorNone) {
    max_primary_layers_ = value;
    DLOGI("Max layers on primary limited to %d", max_primary_layers_);
  }

  return error;
}

DisplayError CompManager::Deinit() {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  if (extension_intf_) {
    extension_intf_->DestroyResourceExtn(resource_intf_);
    extension_intf_->DestroyDppsControlExtn(dpps_ctrl_intf_);
    extension_intf_->DestroyCapabilitiesExtn(cap_intf_);
  } else {
    ResourceDefault::DestroyResourceDefault(resource_intf_);
  }

  return kErrorNone;
}

DisplayError CompManager::ReserveDisplay(DisplayType type) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  return resource_intf_->ReserveDisplay(type);
}

DisplayError CompManager::RegisterDisplay(int32_t display_id, DisplayType type,
                                          const HWDisplayAttributes &display_attributes,
                                          const HWPanelInfo &hw_panel_info,
                                          const HWMixerAttributes &mixer_attributes,
                                          const DisplayConfigVariableInfo &fb_config,
                                          Handle *display_ctx, HWQosData*default_qos_data) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  DisplayError error = kErrorNone;

  DisplayCompositionContext *display_comp_ctx = new DisplayCompositionContext();
  if (!display_comp_ctx) {
    return kErrorMemory;
  }

  Strategy *&strategy = display_comp_ctx->strategy;
  strategy = new Strategy(extension_intf_, buffer_allocator_, display_id, type,
                          hw_res_info_, hw_panel_info, mixer_attributes, display_attributes,
                          fb_config);
  if (!strategy) {
    DLOGE("Unable to create strategy");
    delete display_comp_ctx;
    return kErrorMemory;
  }

  error = strategy->Init();
  if (error != kErrorNone) {
    delete strategy;
    delete display_comp_ctx;
    return error;
  }

  Resolution fb_resolution = {fb_config.x_pixels, fb_config.y_pixels};

  error = resource_intf_->RegisterDisplay(display_id, type, display_attributes, hw_panel_info,
                                          mixer_attributes, fb_resolution,
                                          &display_comp_ctx->display_resource_ctx);
  if (error != kErrorNone) {
    strategy->Deinit();
    delete strategy;
    delete display_comp_ctx;
    display_comp_ctx = NULL;
    return error;
  }

  error = resource_intf_->Perform(ResourceInterface::kCmdGetDefaultQosData,
                                  display_comp_ctx->display_resource_ctx, default_qos_data);
  if (error != kErrorNone) {
    strategy->Deinit();
    delete strategy;
    resource_intf_->UnregisterDisplay(display_comp_ctx->display_resource_ctx);
    delete display_comp_ctx;
    display_comp_ctx = NULL;
    return error;
  }

  error = resource_intf_->Perform(ResourceInterface::kCmdDedicatePipes,
                                  display_comp_ctx->display_resource_ctx);
  if (error != kErrorNone) {
    strategy->Deinit();
    delete strategy;
    resource_intf_->UnregisterDisplay(display_comp_ctx->display_resource_ctx);
    delete display_comp_ctx;
    display_comp_ctx = NULL;
    return error;
  }

  registered_displays_.insert(display_id);
  display_comp_ctx->is_primary_panel = hw_panel_info.is_primary_panel;
  display_comp_ctx->display_id = display_id;
  display_comp_ctx->display_type = type;
  display_comp_ctx->fb_config = fb_config;
  display_comp_ctx->dest_scaler_blocks_used = mixer_attributes.dest_scaler_blocks_used;
  *display_ctx = display_comp_ctx;
  // New non-primary display device has been added, so move the composition mode to safe mode until
  // resources for the added display is configured properly.
  if (!display_comp_ctx->is_primary_panel) {
    max_sde_secondary_fetch_layers_ = UINT32(Debug::GetSecondaryMaxFetchLayers());
  }

  display_demura_status_[display_id] = false;

  DLOGV_IF(kTagCompManager, "Registered displays [%s], display %d-%d",
           StringDisplayList(registered_displays_).c_str(), display_comp_ctx->display_id,
           display_comp_ctx->display_type);


  int force_gpu_comp = 0;
  if (DebugHandler::Get()->GetProperty(FORCE_GPU_COMPOSITION, &force_gpu_comp) == kErrorNone) {
    DLOGV_IF(kTagCompManager, "Force GPU composition: %d", force_gpu_comp);
  }

  if (force_gpu_comp) {
    int display_count = registered_displays_.size();

    // enable GPU comp for 1) mirror mode with two displays 2) dual LM
    if ((display_count > 1 && display_attributes.topology == kSingleLM) ||
        (display_attributes.topology == kDualLM)) {
      force_gpu_comp_ = true;
    }
  }

  return kErrorNone;
}

DisplayError CompManager::UnregisterDisplay(Handle display_ctx) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  if (!display_comp_ctx) {
    return kErrorParameters;
  }

  resource_intf_->UnregisterDisplay(display_comp_ctx->display_resource_ctx);

  Strategy *&strategy = display_comp_ctx->strategy;
  strategy->Deinit();
  delete strategy;

  registered_displays_.erase(display_comp_ctx->display_id);
  powered_on_displays_.erase(display_comp_ctx->display_id);

  DLOGV_IF(kTagCompManager, "Registered displays [%s], display %d-%d",
           StringDisplayList(registered_displays_).c_str(), display_comp_ctx->display_id,
           display_comp_ctx->display_type);

  delete display_comp_ctx;
  display_comp_ctx = NULL;
  return kErrorNone;
}

DisplayError CompManager::CheckEnforceSplit(Handle comp_handle,
                                            uint32_t new_refresh_rate) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayError error = kErrorNone;
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(comp_handle);

  error = resource_intf_->Perform(ResourceInterface::kCmdCheckEnforceSplit,
                                  display_comp_ctx->display_resource_ctx, new_refresh_rate);
  return error;
}

DisplayError CompManager::ReconfigureDisplay(Handle comp_handle,
                                             const HWDisplayAttributes &display_attributes,
                                             const HWPanelInfo &hw_panel_info,
                                             const HWMixerAttributes &mixer_attributes,
                                             const DisplayConfigVariableInfo &fb_config,
                                             HWQosData*default_qos_data) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DTRACE_SCOPED();

  DisplayError error = kErrorNone;
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(comp_handle);

  Resolution fb_resolution = {fb_config.x_pixels, fb_config.y_pixels};

  error = resource_intf_->ReconfigureDisplay(display_comp_ctx->display_resource_ctx,
                                             display_attributes, hw_panel_info, mixer_attributes,
                                             fb_resolution);
  if (error != kErrorNone) {
    DLOGW("ReconfigureDisplay returned error=%d", error);
    return error;
  }

  error = resource_intf_->Perform(ResourceInterface::kCmdGetDefaultQosData,
                                  display_comp_ctx->display_resource_ctx, default_qos_data);
  if (error != kErrorNone) {
    DLOGW("GetDefaultQosData Data returned error=%d", error);
    return error;
  }

  error = resource_intf_->Perform(ResourceInterface::kCmdCheckEnforceSplit,
                                  display_comp_ctx->display_resource_ctx, display_attributes.fps);
  if (error != kErrorNone) {
    DLOGW("CheckEnforceSplit returned error=%d", error);
    return error;
  }

  if (display_comp_ctx->strategy) {
    error = display_comp_ctx->strategy->Reconfigure(hw_panel_info, display_attributes,
                                                    mixer_attributes, fb_config);
    if (error != kErrorNone) {
      DLOGE("Unable to Reconfigure strategy.");
      display_comp_ctx->strategy->Deinit();
      delete display_comp_ctx->strategy;
      display_comp_ctx->strategy = NULL;
      return error;
    }
  }

  // Update new resolution.
  display_comp_ctx->fb_config = fb_config;
  return error;
}

void CompManager::PrepareStrategyConstraints(Handle comp_handle,
                                             DispLayerStack *disp_layer_stack) {
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(comp_handle);
  StrategyConstraints *constraints = &display_comp_ctx->constraints;
  Handle &display_resource_ctx = display_comp_ctx->display_resource_ctx;

  // Call Layer Precheck to get feedback
  LayerFeedback feedback(disp_layer_stack->info.app_layer_count);
  if (resource_intf_)
    resource_intf_->Precheck(display_resource_ctx, disp_layer_stack, &feedback);

  constraints->safe_mode = safe_mode_;
  constraints->max_layers = hw_res_info_.num_blending_stages;
  constraints->feedback = feedback;

  // Limit 2 layer SDE Comp if its not a Primary Display.
  // Safe mode is the policy for External display on a low end device.
  if (!display_comp_ctx->is_primary_panel) {
    bool low_end_hw = ((hw_res_info_.num_vig_pipe + hw_res_info_.num_rgb_pipe +
                        hw_res_info_.num_dma_pipe) <= kSafeModeThreshold);
    constraints->max_layers = display_comp_ctx->display_type == kBuiltIn ?
                              max_sde_builtin_fetch_layers_ : max_sde_secondary_fetch_layers_;
    constraints->safe_mode = (low_end_hw && !hw_res_info_.separate_rotator) ? true : safe_mode_;
  } else {
    constraints->max_layers = max_primary_layers_ > 0 ? max_primary_layers_ :
                              hw_res_info_.num_blending_stages;
  }

  // If a strategy fails after successfully allocating resources, then set safe mode
  if (display_comp_ctx->remaining_strategies != display_comp_ctx->max_strategies) {
    constraints->safe_mode = true;
  }

  if (secure_event_ == kTUITransitionStart) {
    constraints->max_layers = 1;
  }

  uint32_t size_ff = 1;  // gpu target layer always present
  if (disp_layer_stack->info.stitch_present)
    size_ff++;
  if (disp_layer_stack->info.demura_present)
    size_ff++;
  if (disp_layer_stack->info.cwb_present)
    size_ff++;
  uint32_t app_layer_count = UINT32(disp_layer_stack->stack->layers.size()) - size_ff;
  if (display_comp_ctx->idle_fallback) {
    // Handle the GPU based idle timeout by falling back
    constraints->safe_mode = true;
  }

  // Avoid safe mode, if there is only one app layer.
  if (app_layer_count == 1) {
     constraints->safe_mode = false;
  }

  constraints->force_gpu_comp = force_gpu_comp_;
}

void CompManager::GenerateROI(Handle display_ctx, DispLayerStack *disp_layer_stack) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayCompositionContext *disp_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  return disp_comp_ctx->strategy->GenerateROI(disp_layer_stack, disp_comp_ctx->pu_constraints);
}

DisplayError CompManager::PrePrepare(Handle display_ctx, DispLayerStack *disp_layer_stack) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  if (display_comp_ctx->idle_fallback) {
    display_comp_ctx->constraints.idle_timeout = true;
  }

  DisplayError error = display_comp_ctx->strategy->Start(disp_layer_stack,
                                                         &display_comp_ctx->max_strategies,
                                                         &display_comp_ctx->constraints);
  display_comp_ctx->remaining_strategies = display_comp_ctx->max_strategies;

  // Select a composition strategy, and try to allocate resources for it.
  resource_intf_->Start(display_comp_ctx->display_resource_ctx, disp_layer_stack->stack);

  return error;
}

DisplayError CompManager::Prepare(Handle display_ctx, DispLayerStack *disp_layer_stack) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  DTRACE_SCOPED();
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  Handle &display_resource_ctx = display_comp_ctx->display_resource_ctx;
  DisplayError error = kErrorUndefined;

  PrepareStrategyConstraints(display_ctx, disp_layer_stack);
  // Select a composition strategy, and try to allocate resources for it.
  resource_intf_->Start(display_resource_ctx, disp_layer_stack->stack);

  bool exit = false;
  uint32_t &count = display_comp_ctx->remaining_strategies;
  for (; !exit && count > 0; count--) {
    error = display_comp_ctx->strategy->GetNextStrategy();
    if (error != kErrorNone) {
      // Composition strategies exhausted. Resource Manager could not allocate resources even for
      // GPU composition. This will never happen.
      exit = true;
    }

    if (!exit) {
      LayerFeedback updated_feedback(disp_layer_stack->info.app_layer_count);
      error = resource_intf_->Prepare(display_resource_ctx, disp_layer_stack, &updated_feedback);
      // Exit if successfully prepared resource, else try next strategy.
      exit = (error == kErrorNone);
      if (!exit)
        display_comp_ctx->constraints.feedback = updated_feedback;
    }
  }

  if (error != kErrorNone) {
    resource_intf_->Stop(display_resource_ctx, disp_layer_stack);
    DLOGE("Composition strategies exhausted for display = %d-%d. (first frame = %s)",
          display_comp_ctx->display_id, display_comp_ctx->display_type,
          display_comp_ctx->first_cycle_ ? "True" : "False");
    return error;
  }

  return error;
}

DisplayError CompManager::PostPrepare(Handle display_ctx, DispLayerStack *disp_layer_stack) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  Handle &display_resource_ctx = display_comp_ctx->display_resource_ctx;

  DisplayError error = kErrorNone;

  display_comp_ctx->strategy->Stop();

  error = resource_intf_->Stop(display_resource_ctx, disp_layer_stack);
  if (error != kErrorNone) {
    DLOGE("Resource stop failed for display = %d", display_comp_ctx->display_type);
  }

  error = resource_intf_->PostPrepare(display_resource_ctx, disp_layer_stack);
  if (error != kErrorNone) {
    return error;
  }

  return kErrorNone;
}

DisplayError CompManager::Commit(Handle display_ctx, DispLayerStack *disp_layer_stack) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  DisplayError error = resource_intf_->Commit(display_comp_ctx->display_resource_ctx,
                                              disp_layer_stack);
  if (error != kErrorNone) {
    return error;
  }
  if (secure_event_ == kTUITransitionStart) {
    return GetDefaultQosData(display_ctx, &disp_layer_stack->info.qos_data);
  }
  return kErrorNone;
}

DisplayError CompManager::PostCommit(Handle display_ctx, DispLayerStack *disp_layer_stack) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  DisplayError error = kErrorNone;
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  error = resource_intf_->PostCommit(display_comp_ctx->display_resource_ctx, disp_layer_stack);
  if (error != kErrorNone) {
    return error;
  }

  display_comp_ctx->idle_fallback = false;
  display_comp_ctx->first_cycle_ = false;
  display_comp_ctx->constraints.idle_timeout = false;

  DLOGV_IF(kTagCompManager, "Registered displays [%s], display %d-%d",
           StringDisplayList(registered_displays_).c_str(), display_comp_ctx->display_id,
           display_comp_ctx->display_type);

  return kErrorNone;
}

void CompManager::Purge(Handle display_ctx) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  resource_intf_->Purge(display_comp_ctx->display_resource_ctx);

  display_comp_ctx->strategy->Purge();
}

DisplayError CompManager::SetIdleTimeoutMs(Handle display_ctx, uint32_t active_ms,
                                           uint32_t inactive_ms) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  return display_comp_ctx->strategy->SetIdleTimeoutMs(active_ms, inactive_ms);
}

void CompManager::ProcessIdleTimeout(Handle display_ctx) {
  DTRACE_SCOPED();
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  if (!display_comp_ctx) {
    return;
  }

  display_comp_ctx->idle_fallback = true;
}

void CompManager::ProcessIdlePowerCollapse(Handle display_ctx) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  DisplayCompositionContext *display_comp_ctx =
          reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  if (display_comp_ctx) {
    resource_intf_->Perform(ResourceInterface::kCmdResetLUT,
                            display_comp_ctx->display_resource_ctx);
  }
}

DisplayError CompManager::SetMaxMixerStages(Handle display_ctx, uint32_t max_mixer_stages) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  DisplayError error = kErrorNone;
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  if (display_comp_ctx) {
    error = resource_intf_->SetMaxMixerStages(display_comp_ctx->display_resource_ctx,
                                              max_mixer_stages);
  }

  return error;
}

DisplayError CompManager::GetHDR10PlusCapability(bool *hdr_plus_support) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayError error = kErrorNone;
  if (cap_intf_) {
    DLOGD_IF(kTagCompManager, "Attempting to get HDR10+ capability");
    error = cap_intf_->GetCapability(kHDR10PlusCapability, hdr_plus_support);
  }
  if (error != kErrorNone || !cap_intf_) {
    DLOGW("Failed to get HDR10+ capability");
  }
  return error;
}

void CompManager::ControlPartialUpdate(Handle display_ctx, bool enable) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  display_comp_ctx->pu_constraints.enable = enable;
}

DisplayError CompManager::ValidateScaling(const LayerRect &crop, const LayerRect &dst,
                                          bool rotate90) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  BufferLayout layout = Debug::IsUbwcTiledFrameBuffer() ? kUBWC : kLinear;
  return resource_intf_->ValidateScaling(crop, dst, rotate90, layout, true);
}

DisplayError CompManager::ValidateAndSetCursorPosition(Handle display_ctx,
                                                       DispLayerStack *disp_layer_stack,
                                                       int x, int y) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  Handle &display_resource_ctx = display_comp_ctx->display_resource_ctx;
  return resource_intf_->ValidateAndSetCursorPosition(display_resource_ctx, disp_layer_stack, x, y,
                                                      &display_comp_ctx->fb_config);
}

DisplayError CompManager::SetMaxBandwidthMode(HWBwModes mode) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayError error = kErrorNotSupported;
  if (mode >= kBwModeMax) {
    return error;
  }

  if (resource_intf_) {
    return resource_intf_->SetMaxBandwidthMode(mode);
  }

  return error;
}

DisplayError CompManager::GetScaleLutConfig(HWScaleLutInfo *lut_info) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  return resource_intf_->GetScaleLutConfig(lut_info);
}

DisplayError CompManager::SetDetailEnhancerData(Handle display_ctx,
                                                const DisplayDetailEnhancerData &de_data) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  if (!display_comp_ctx->dest_scaler_blocks_used) {
    return kErrorResources;
  }

  return resource_intf_->SetDetailEnhancerData(display_comp_ctx->display_resource_ctx, de_data);
}

DisplayError CompManager::SetCompositionState(Handle display_ctx,
                                              LayerComposition composition_type, bool enable) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  return display_comp_ctx->strategy->SetCompositionState(composition_type, enable);
}

DisplayError CompManager::ControlDpps(bool enable) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  // DPPS feature and HDR using SSPP tone mapping can co-exist
  // DPPS feature and HDR using DSPP tone mapping are mutually exclusive
  if (dpps_ctrl_intf_ && hw_res_info_.src_tone_map.none()) {
    int err = 0;
    if (enable) {
      err = dpps_ctrl_intf_->On();
    } else {
      err = dpps_ctrl_intf_->Off();
    }
    if (err) {
      return kErrorUndefined;
    }
  }

  return kErrorNone;
}

bool CompManager::SetDisplayState(Handle display_ctx, DisplayState state,
                                  const SyncPoints &sync_points) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  resource_intf_->Perform(ResourceInterface::kCmdSetDisplayState,
                          display_comp_ctx->display_resource_ctx, state);

  switch (state) {
  case kStateOff:
    Purge(display_ctx);
    powered_on_displays_.erase(display_comp_ctx->display_id);
    break;

  case kStateOn:
  case kStateDoze:
    resource_intf_->Perform(ResourceInterface::kCmdDedicatePipes,
                            display_comp_ctx->display_resource_ctx);
    powered_on_displays_.insert(display_comp_ctx->display_id);
    break;

  case kStateDozeSuspend:
    powered_on_displays_.erase(display_comp_ctx->display_id);
    break;

  default:
    break;
  }

  bool inactive = (state == kStateOff) || (state == kStateDozeSuspend);
  UpdateStrategyConstraints(display_comp_ctx->is_primary_panel, inactive);

  resource_intf_->UpdateSyncHandle(display_comp_ctx->display_resource_ctx, sync_points);

  return true;
}

DisplayError CompManager::SetColorModesInfo(Handle display_ctx,
                                            const std::vector<PrimariesTransfer> &colormodes_cs) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  display_comp_ctx->strategy->SetColorModesInfo(colormodes_cs);

  return kErrorNone;
}

std::string CompManager::StringDisplayList(const std::set<int32_t> &displays) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  std::string displays_str;
  for (auto disps : displays) {
    if (displays_str.empty()) {
      displays_str = std::to_string(disps);
    } else {
      displays_str += ", " + std::to_string(disps);
    }
  }
  return displays_str;
}

DisplayError CompManager::SetBlendSpace(Handle display_ctx, const PrimariesTransfer &blend_space) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  display_comp_ctx->strategy->SetBlendSpace(blend_space);

  return kErrorNone;
}

void CompManager::HandleSecureEvent(Handle display_ctx, SecureEvent secure_event) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  // Disable rotator for non secure layers at the end of secure display session, because scm call
  // has been made to end secure display session during the display commit. Since then access to
  // non secure memory is unavailable. So this results in smmu page fault when rotator tries to
  // access the non secure memory.
  if (secure_event == kSecureDisplayEnd) {
    resource_intf_->Perform(ResourceInterface::kCmdDisableRotatorOneFrame,
                            display_comp_ctx->display_resource_ctx);
  }
  if (secure_event == kTUITransitionEnd) {
    resource_intf_->Perform(ResourceInterface::kCmdResetLUT,
                            display_comp_ctx->display_resource_ctx);
    safe_mode_ = false;
  }
  safe_mode_ = (secure_event == kTUITransitionStart) ? true : safe_mode_;
  secure_event_ = secure_event;
}

void CompManager::UpdateStrategyConstraints(bool is_primary, bool disabled) {
  if (!is_primary) {
    return;
  }

  // Allow builtin display to use all pipes when primary is suspended.
  // Restore it back to 2 after primary poweron.
  max_sde_builtin_fetch_layers_ = (disabled && (powered_on_displays_.size() <= 1)) ?
                                   kMaxSDELayers : max_sde_secondary_fetch_layers_;
}

bool CompManager::CheckResourceState(Handle display_ctx, bool *res_exhausted,
                                     HWDisplayAttributes attr) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  bool res_wait_needed = false;

  resource_intf_->Perform(ResourceInterface::kCmdGetResourceStatus,
                          display_comp_ctx->display_resource_ctx, res_exhausted, &attr,
                          &res_wait_needed);
  return res_wait_needed;
}

DisplayError CompManager::GetConcurrencyFps(DisplayConcurrencyType type, float *fps) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  ResourceConstraintsIn res_constraints_in;
  res_constraints_in.concurrency_type = type;
  ResourceConstraintsOut res_constraints_out;

  auto error = resource_intf_->Perform(ResourceInterface::kCmdGetResourceConstraints,
                                       &res_constraints_in, &res_constraints_out);
  *fps = res_constraints_out.fps;
  return error;
}

DisplayError CompManager::SetDrawMethod(Handle display_ctx, const DisplayDrawMethod &draw_method) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  auto error = display_comp_ctx->strategy->SetDrawMethod(draw_method);
  if (error != kErrorNone) {
    return error;
  }
  error = resource_intf_->SetDrawMethod(display_comp_ctx->display_resource_ctx, draw_method);
  if (error != kErrorNone) {
    return error;
  }

  return kErrorNone;
}

bool CompManager::IsRotatorSupportedFormat(LayerBufferFormat format) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  if (resource_intf_) {
    return resource_intf_->IsRotatorSupportedFormat(format);
  }

  return false;
}

bool CompManager::IsDisplayHWAvailable() {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  if (resource_intf_) {
    return resource_intf_->IsDisplayHWAvailable();
  }

  return false;
}


DisplayError CompManager::FreeDemuraFetchResources(const uint32_t &display_id) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  return resource_intf_->FreeDemuraFetchResources(display_id);
}

DisplayError CompManager::GetDemuraFetchResourceCount(
                          std::map<uint32_t, uint8_t> *fetch_resource_cnt) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  return resource_intf_->GetDemuraFetchResourceCount(fetch_resource_cnt);
}

DisplayError CompManager::ReserveDemuraFetchResources(const uint32_t &display_id,
                                                      const int8_t &preferred_rect) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  return resource_intf_->ReserveDemuraFetchResources(display_id, preferred_rect);
}

DisplayError CompManager::GetDemuraFetchResources(Handle display_ctx, FetchResourceList *frl) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  return resource_intf_->GetDemuraFetchResources(display_comp_ctx->display_resource_ctx, frl);
}

DisplayError CompManager::SetMaxSDEClk(Handle display_ctx, uint32_t clk) {
  DTRACE_SCOPED();
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  if (resource_intf_) {
    DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);
    return resource_intf_->SetMaxSDEClk(display_comp_ctx->display_resource_ctx, clk);
  }

  return kErrorNotSupported;
}

void CompManager::GetRetireFence(Handle display_ctx, shared_ptr<Fence> *retire_fence) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  if (resource_intf_ == nullptr) {
    return;
  }

  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  resource_intf_->Perform(ResourceInterface::kCmdGetRetireFence,
                          display_comp_ctx->display_resource_ctx, retire_fence);
}

void CompManager::NeedsValidate(Handle display_ctx, bool *needs_validate) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  if (resource_intf_ == nullptr) {
    return;
  }

  DisplayCompositionContext *display_comp_ctx =
        reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  resource_intf_->Perform(ResourceInterface::kCmdNeedsValidate,
                          display_comp_ctx->display_resource_ctx, needs_validate);
}

DisplayError CompManager::SetBacklightLevel(Handle display_ctx,
    const uint32_t &backlight_level) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  return resource_intf_->Perform(ResourceInterface::kCmdSetBacklightLevel,
                                  display_comp_ctx->display_resource_ctx,
                                  backlight_level);
}

DisplayError CompManager::ForceToneMapConfigure(Handle display_ctx,
    DispLayerStack *disp_layer_stack) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  return resource_intf_->ForceToneMapConfigure(display_comp_ctx->display_resource_ctx,
                                               disp_layer_stack);
}

DisplayError CompManager::GetDefaultQosData(Handle display_ctx, HWQosData *qos_data) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  return resource_intf_->Perform(ResourceInterface::kCmdGetDefaultQosData,
                                 display_comp_ctx->display_resource_ctx, qos_data);
}

DisplayError CompManager::HandleCwbFrequencyBoost(bool isRequest) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  DisplayError error = kErrorNone;
  error = resource_intf_->Perform(ResourceInterface::kCmdSetCwbBoost, &isRequest);
  return error;
}

void CompManager::SetSafeMode(bool enable) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  safe_mode_ = enable;
}

bool CompManager::IsSafeMode() {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  return safe_mode_;
}

DppsControlInterface* CompManager::GetDppsControlIntf() {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  return dpps_ctrl_intf_;
}

void CompManager::SetDemuraStatus(bool status) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  demura_enabled_ = status;
}

bool CompManager::GetDemuraStatus() {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  return demura_enabled_;
}

void CompManager::SetDemuraStatusForDisplay(const int32_t &display_id, bool status) {
  std::lock_guard<std::recursive_mutex> obj(comp_mgr_mutex_);
  display_demura_status_[display_id] = status;
}

bool CompManager::GetDemuraStatusForDisplay(const int32_t &display_id) {
  return display_demura_status_[display_id];
}

}  // namespace sdm
