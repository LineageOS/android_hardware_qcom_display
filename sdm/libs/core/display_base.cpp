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

/*
* Changes from Qualcomm Innovation Center are provided under the following license:
*
* Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the
* disclaimer below) provided that the following conditions are met:
*
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*
*    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <stdio.h>
#include <malloc.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/formats.h>
#include <utils/rect.h>
#include <utils/utils.h>
#include <drm_interface.h>

#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

#include "display_base.h"
#include "hw_info_interface.h"

#define __CLASS__ "DisplayBase"

namespace sdm {

bool DisplayBase::display_power_reset_pending_ = false;
bool DisplayBase::primary_active_ = false;
Locker DisplayBase::display_power_reset_lock_;
int32_t DisplayBase::mmrm_floor_clk_vote_ = 100000000;

static bool ValidNoisePluginDebugOverride(uint32_t override_param) {
  return ((override_param >= kNoisePlugInDebugPropertyStart) &&
          (override_param < kNoisePlugInDebugPropertyEnd));
}

static ColorPrimaries GetColorPrimariesFromAttribute(const std::string &gamut) {
  if (gamut.find(kDisplayP3) != std::string::npos || gamut.find(kDcip3) != std::string::npos) {
    return ColorPrimaries_DCIP3;
  } else if (gamut.find(kHdr) != std::string::npos || gamut.find("bt2020") != std::string::npos ||
             gamut.find("BT2020") != std::string::npos) {
    // BT2020 is hdr, but the dynamicrange of kHdr means its BT2020
    return ColorPrimaries_BT2020;
  } else if (gamut.find(kSrgb) != std::string::npos) {
    return ColorPrimaries_BT709_5;
  } else if (gamut.find(kNative) != std::string::npos) {
    DLOGW("Native Gamut found, returning default: sRGB");
    return ColorPrimaries_BT709_5;
  }

  return ColorPrimaries_BT709_5;
}

// TODO(user): Have a single structure handle carries all the interface pointers and variables.
DisplayBase::DisplayBase(DisplayType display_type, DisplayEventHandler *event_handler,
                         HWDeviceType hw_device_type, BufferAllocator *buffer_allocator,
                         CompManager *comp_manager, HWInfoInterface *hw_info_intf)
  : display_type_(display_type), event_handler_(event_handler), hw_device_type_(hw_device_type),
    buffer_allocator_(buffer_allocator), comp_manager_(comp_manager), hw_info_intf_(hw_info_intf) {
  // Kick off worker thread and block the caller thread until worker thread has started and
  // ready to process commit requests.
  lock_guard<recursive_mutex> client_lock(disp_mutex_.client_mutex);

  // Start commit worker thread and wait for thread response.
  DLOGI("Starting commit thread for display: %d", display_type);

  std::thread commit_thread(&DisplayBase::CommitThread, this);
  disp_mutex_.client_cv.wait(disp_mutex_.client_mutex);
  commit_thread_.swap(commit_thread);

  DLOGI("Commit thread started for display: %d", display_type);
}

DisplayBase::DisplayBase(int32_t display_id, DisplayType display_type,
                         DisplayEventHandler *event_handler, HWDeviceType hw_device_type,
                         BufferAllocator *buffer_allocator, CompManager *comp_manager,
                         HWInfoInterface *hw_info_intf)
  : DisplayBase(display_type, event_handler, hw_device_type,
                buffer_allocator, comp_manager, hw_info_intf) {
  display_id_ = display_id;
}

DisplayBase::~DisplayBase() {
  // Signal worker thread and wait for it to terminate.
  {
    ClientLock lock(disp_mutex_);
    disp_mutex_.worker_exit = true;
    lock.NotifyWorker();
  }

  commit_thread_.join();
}

DisplayError DisplayBase::Init() {
  ClientLock lock(disp_mutex_);
  DisplayError error = kErrorNone;
  hw_panel_info_ = HWPanelInfo();
  hw_intf_->GetHWPanelInfo(&hw_panel_info_);
  if (hw_info_intf_) {
    hw_info_intf_->GetHWResourceInfo(&hw_resource_info_);
  }
  auto max_mixer_stages = hw_resource_info_.num_blending_stages;
  int property_value = Debug::GetMaxPipesPerMixer(display_type_);

  uint32_t active_index = 0;
  int drop_vsync = 0;
  int hw_recovery_threshold = 1;
  int32_t prop = 0;
  hw_intf_->GetActiveConfig(&active_index);
  hw_intf_->GetDisplayAttributes(active_index, &display_attributes_);
  fb_config_ = display_attributes_;
  active_refresh_rate_ = display_attributes_.fps;

  if (!Debug::GetMixerResolution(&mixer_attributes_.width, &mixer_attributes_.height)) {
    if (hw_intf_->SetMixerAttributes(mixer_attributes_) == kErrorNone) {
      custom_mixer_resolution_ = true;
    }
  }

  error = hw_intf_->GetMixerAttributes(&mixer_attributes_);
  if (error != kErrorNone) {
    return error;
  }

  // Override x_pixels and y_pixels of frame buffer with mixer width and height
  fb_config_.x_pixels = mixer_attributes_.width;
  fb_config_.y_pixels = mixer_attributes_.height;

  if (IsPrimaryDisplayLocked()) {
    HWScaleLutInfo lut_info = {};
    error = comp_manager_->GetScaleLutConfig(&lut_info);
    if (error == kErrorNone) {
      error = hw_intf_->SetScaleLutConfig(&lut_info);
      if (error != kErrorNone) {
        goto CleanupOnError;
      }
    }
  }

  // ColorManager supported for built-in display.
  if (kBuiltIn == display_type_) {
    DppsControlInterface *dpps_intf = comp_manager_->GetDppsControlIntf();
    color_mgr_ = ColorManagerProxy::CreateColorManagerProxy(display_type_, hw_intf_,
                                                            display_attributes_, hw_panel_info_,
                                                            dpps_intf, this);
  }

  error = comp_manager_->RegisterDisplay(display_id_, display_type_, display_attributes_,
                                         hw_panel_info_, mixer_attributes_, fb_config_,
                                         &display_comp_ctx_, &cached_qos_data_);
  if (error != kErrorNone) {
    DLOGW("Display %d comp manager registration failed!", display_id_);
    goto CleanupOnError;
  }
  default_clock_hz_ = cached_qos_data_.clock_hz;

  if (color_modes_cs_.size() > 0) {
    error = comp_manager_->SetColorModesInfo(display_comp_ctx_, color_modes_cs_);
    if (error) {
      DLOGW("SetColorModesInfo failed on display = %d", display_type_);
    }
  }

  if (property_value >= 0) {
    max_mixer_stages = std::min(UINT32(property_value), hw_resource_info_.num_blending_stages);
  }
  DisplayBase::SetMaxMixerStages(max_mixer_stages);

  // TODO(user): Temporary changes, to be removed when DRM driver supports
  // Partial update with Destination scaler enabled.
  SetPUonDestScaler();

  Debug::GetProperty(DISABLE_HW_RECOVERY_DUMP_PROP, &disable_hw_recovery_dump_);
  DLOGI("disable_hw_recovery_dump_ set to %d", disable_hw_recovery_dump_);

  Debug::Get()->GetProperty(DROP_SKEWED_VSYNC, &drop_vsync);
  drop_skewed_vsync_ = (drop_vsync == 1);

  Debug::GetProperty(HW_RECOVERY_THRESHOLD, &hw_recovery_threshold);
  DLOGI("hw_recovery_threshold_ set to %d", hw_recovery_threshold);
  if (hw_recovery_threshold > 0) {
    hw_recovery_threshold_ = (UINT32(hw_recovery_threshold));
  }
  if (Debug::Get()->GetProperty(MMRM_FLOOR_CLK_VOTE, &prop) == kErrorNone) {
    mmrm_floor_clk_vote_ = prop;
  }

  SetupPanelFeatureFactory();

  InitBorderLayers();
  // Assume unified draw is supported.
  unified_draw_supported_ = true;

  prop = 0;
  Debug::GetProperty(TRACK_INPUT_FENCES, &prop);
  track_input_fences_ = (prop == 1);
  DLOGI("track_input_fences_:%d %d-%d", track_input_fences_, display_id_, display_type_);

  return kErrorNone;

CleanupOnError:
  ClearColorInfo();
  if (display_comp_ctx_) {
    comp_manager_->UnregisterDisplay(display_comp_ctx_);
  }

  return error;
}

DisplayError DisplayBase::InitBorderLayers() {
  // Feature is limited to primary.
  if (!hw_panel_info_.is_primary_panel) {
    return kErrorNone;
  }

  windowed_display_ = Debug::GetWindowRect(true /*is_primary_*/ , &window_rect_.left,
                                           &window_rect_.top, &window_rect_.right,
                                           &window_rect_.bottom) == 0;
  if (!windowed_display_) {
    return kErrorNone;
  }

  int value = 0;
  Debug::GetProperty(ENABLE_WINDOW_RECT_MASK, &value);
  enable_win_rect_mask_ = (value == 1);
  if (!enable_win_rect_mask_) {
    DLOGI("Window rect enabled without RC on %d-%d", display_id_, display_type_);
    return kErrorNone;
  }

  DLOGI("Generating border layers for %d-%d", display_id_, display_type_);

  std::vector<LayerRect> border_rects = GetBorderRects();
  for (auto &rect : border_rects) {
    DLOGI("Rect: %f %f %f %f", rect.left, rect.top, rect.right, rect.bottom);
  }

  GenerateBorderLayers(border_rects);

  return kErrorNone;
}

std::vector<LayerRect> DisplayBase::GetBorderRects() {
  // Window rect can result 4 regions(max) to be blacked out.
  // Horizontal strip at top and bottom, pillar-box on each side.
  float display_width = FLOAT(display_attributes_.x_pixels);
  float display_height = FLOAT(display_attributes_.y_pixels);
  LayerRect win_rect = window_rect_;
  std::vector<LayerRect> border_rects;
  if (win_rect.left) {
    LayerRect rect = {0, 0, win_rect.left, display_height};
    border_rects.push_back(rect);
  }

  if (win_rect.right) {
    LayerRect rect = {display_width - win_rect.right, 0, display_width, display_height};
    border_rects.push_back(rect);
  }

  if (win_rect.top) {
    LayerRect rect = {0, 0, display_width, win_rect.top};
    border_rects.push_back(rect);
  }

  if (win_rect.bottom) {
    LayerRect rect = {0, display_height - win_rect.bottom, display_width, display_height};
    border_rects.push_back(rect);
  }

  return border_rects;
}

void DisplayBase::GenerateBorderLayers(const std::vector<LayerRect> &border_rects) {
  for (auto &border_rect : border_rects) {
    Layer layer;
    layer.src_rect = {0, 0, border_rect.right - border_rect.left,
                       border_rect.bottom - border_rect.top};
    layer.dst_rect = border_rect;
    LayerBuffer &layer_buffer = layer.input_buffer;
    layer_buffer.width = UINT32(layer.dst_rect.right - layer.dst_rect.left);
    layer_buffer.height = UINT32(layer.dst_rect.bottom - layer.dst_rect.top);
    layer_buffer.unaligned_width = layer_buffer.width;
    layer_buffer.unaligned_height = layer_buffer.height;
    layer_buffer.format = kFormatRGBA8888;
    layer_buffer.flags.mask_layer = true;
    layer.flags.solid_fill = 1;

    // 32 bit ARGB
    uint32_t a = UINT32(255) << 24;
    uint32_t r = UINT32(0) << 16;
    uint32_t g = UINT32(0) << 8;
    uint32_t b = UINT32(0);
    uint32_t color = a | r | g | b;
    layer.solid_fill_color = color;

    border_layers_.push_back(layer);
  }
}

DisplayError DisplayBase::Deinit() {
  {  // Scope for lock
    ClientLock lock(disp_mutex_);
    ClearColorInfo();
    if (IsPrimaryDisplayLocked()) {
      hw_intf_->UnsetScaleLutConfig();
    }
  }
  HWEventsInterface::Destroy(hw_events_intf_);
  HWInterface::Destroy(hw_intf_);

  {  // Scope for lock
    ClientLock lock(disp_mutex_);
    comp_manager_->UnregisterDisplay(display_comp_ctx_);
  }

  if (rc_panel_feature_init_) {
    rc_core_->Deinit();
    rc_panel_feature_init_ = false;
  }

  if (noise_plugin_intf_) {
    noise_plugin_intf_->Deinit();
    noise_plugin_intf_ = nullptr;
  }

  CloseFd(&cached_framebuffer_.planes[0].fd);
#ifdef TRUSTED_VM
  // release free memory from the heap, needed for Trusted_VM due to the limited
  // carveout size
  malloc_trim(0);
#endif
  return kErrorNone;
}

DisplayError DisplayBase::SetupPanelFeatureFactory() {
  if (pf_factory_ && prop_intf_) {
    return kErrorNone;
  }

  DynLib feature_impl_lib;
  GetPanelFeatureFactory get_factory_f_ptr = nullptr;
  if (feature_impl_lib.Open(EXTENSION_LIBRARY_NAME)) {
    if (!feature_impl_lib.Sym(GET_PANEL_FEATURE_FACTORY,
                              reinterpret_cast<void **>(&get_factory_f_ptr))) {
      DLOGE("Unable to load symbols, error = %s", feature_impl_lib.Error());
      return kErrorUndefined;
    }
  } else {
    DLOGW("Unable to load = %s, error = %s", EXTENSION_LIBRARY_NAME, feature_impl_lib.Error());
    DLOGW("SDM Extension is not supported");
    return kErrorNone;
  }

  pf_factory_ = get_factory_f_ptr();
  if (!pf_factory_) {
    DLOGE("Failed to create PanelFeatureFactory");
    return kErrorResources;
  }

  prop_intf_ = hw_intf_->GetPanelFeaturePropertyIntf();
  if (!prop_intf_) {
    DLOGW("Failed to create PanelFeaturePropertyIntf");
    pf_factory_ = nullptr;
    return kErrorResources;
  }

  DLOGI("Setup pf factory and prop intf for Panel Features");
  return kErrorNone;
}

DisplayError DisplayBase::NoiseInit() {
  if (!hw_resource_info_.has_noise_layer || noise_disable_prop_) {
    DLOGW("Noise Layer disabled on display %d-%d has_noise = %d noise_disable_prop = %d",
          display_id_, display_type_, hw_resource_info_.has_noise_layer, noise_disable_prop_);
    return kErrorNone;
  }

  noise_plugin_factory_intf_ = GetNoisePlugInFactoryIntf();
  if (!noise_plugin_factory_intf_) {
    DLOGE("Failed to create noise plugin factory for display %d-%d", display_id_, display_type_);
    return kErrorNotSupported;
  }

  noise_plugin_intf_ = noise_plugin_factory_intf_->CreateNoisePlugInIntf(
        NOISE_PLUGIN_VERSION_MAJOR, NOISE_PLUGIN_VERSION_MINOR);
  if (!noise_plugin_intf_) {
    DLOGE("CreateNoisePluginIntf failed! for display %d-%d", display_id_, display_type_);
    return kErrorNotSupported;
  }

  int ret = noise_plugin_intf_->Init();
  if (ret) {
    DLOGE("NoisePlugin Init failed! for display %d-%d", display_id_, display_type_);
    noise_plugin_intf_ = nullptr;
    return kErrorNotSupported;
  }

  return kErrorNone;
}

// Query the dspp capabilities and enable the RC feature.
DisplayError DisplayBase::InitRC() {
  if (!rc_core_ && !first_cycle_ && rc_enable_prop_ && pf_factory_ && prop_intf_) {
    RCInputConfig input_cfg = {};
    input_cfg.display_id = display_id_;
    input_cfg.display_type = display_type_;
    input_cfg.display_xres = display_attributes_.x_pixels;
    input_cfg.display_yres = display_attributes_.y_pixels;
    input_cfg.max_mem_size = hw_resource_info_.rc_total_mem_size;
    rc_core_ = pf_factory_->CreateRCIntf(input_cfg, prop_intf_);
    GenericPayload dummy;
    int err = 0;
    if (!rc_core_) {
      DLOGE("Failed to create RC Intf");
      return kErrorUndefined;
    }
    err = rc_core_->GetParameter(kRCFeatureQueryDspp, &dummy);
    if (!err) {
      // Since the query succeeded, this display has a DSPP.
      if (rc_core_->Init() != 0) {
        DLOGW("Failed to initialize RC");
        return kErrorNotSupported;
      }
    } else {
      DLOGW("RC HW block is not present for display %d-%d.", display_id_, display_type_);
      return kErrorResources;
    }

    rc_panel_feature_init_ = true;
  }

  return kErrorNone;
}

DisplayError DisplayBase::GetCwbBufferResolution(CwbConfig *cwb_config, uint32_t *x_pixels,
                                                 uint32_t *y_pixels) {
  DisplayError error = kErrorNotSupported;
  DisplayConfigVariableInfo display_config;

  CwbTapPoint cwb_tappoint = cwb_config->tap_point;
  bool pu_as_cwb_roi = cwb_config->pu_as_cwb_roi;
  if (cwb_tappoint == CwbTapPoint::kDsppTapPoint || cwb_tappoint == CwbTapPoint::kDemuraTapPoint) {
    // To dump post-processed (DSPP) output for CWB, use Panel resolution.
    uint32_t active_index = 0;
    error = GetActiveConfig(&active_index);
    if (error == kErrorNone) {
      error = GetRealConfig(active_index, &display_config);
      if (error == kErrorNone) {
        cwb_config->cwb_full_rect.right = display_config.x_pixels;
        cwb_config->cwb_full_rect.bottom = display_config.y_pixels;
        LayerRect cwb_roi = cwb_config->cwb_roi;
        if (pu_as_cwb_roi) {
          *x_pixels = display_config.x_pixels;
          *y_pixels = display_config.y_pixels;
        } else if (IsValidCwbRoi(cwb_roi, cwb_config->cwb_full_rect)) {
          *x_pixels = cwb_roi.right - cwb_roi.left;
          *y_pixels = cwb_roi.bottom - cwb_roi.top;
        } else {
          *x_pixels = display_config.x_pixels;
          *y_pixels = display_config.y_pixels;
        }
      }
    }
  } else if (cwb_tappoint == CwbTapPoint::kLmTapPoint) {
    // To dump LM output for CWB, use FB resolution. If LM resolution differs from FB resolution in
    // a CWB active frame, then LM resolution is reconfigured to FB resolution in PrePrepare phase.
    error = GetFrameBufferConfig(&display_config);
    if (error == kErrorNone) {
      cwb_config->cwb_full_rect.right = display_config.x_pixels;
      cwb_config->cwb_full_rect.bottom = display_config.y_pixels;
      LayerRect cwb_roi = cwb_config->cwb_roi;
      if (pu_as_cwb_roi) {
        *x_pixels = display_config.x_pixels;
        *y_pixels = display_config.y_pixels;
      } else if (IsValidCwbRoi(cwb_roi, cwb_config->cwb_full_rect)) {
        *x_pixels = cwb_roi.right - cwb_roi.left;
        *y_pixels = cwb_roi.bottom - cwb_roi.top;
        } else {
          *x_pixels = display_config.x_pixels;
          *y_pixels = display_config.y_pixels;
        }
    }
  }
  return error;
}

DisplayError DisplayBase::ConfigureCwb(LayerStack *layer_stack) {
  DisplayError error = kErrorNone;
  if (hw_resource_info_.has_concurrent_writeback && layer_stack->output_buffer) {  // CWB requested
    comp_manager_->HandleCwbFrequencyBoost(true);

    if (!cwb_config_) {  // Instantiate cwb_config_ if cwb was not enabled in previous draw cycle.
      cwb_config_ = new CwbConfig;
      needs_validate_ = true;  // Do not skip Validate in CWB setup frame.
    }
    *cwb_config_ = {};  // Reset cwb_config_ so as to set it to new cwb config passed by the client

    if (layer_stack->cwb_config == NULL) {
      // If Cwb client doesn't set Cwb config in LayerStack.cwb_config, then we consider full frame
      // ROI and recognize tppt. from post-processed flag (demura tappoint is not suopported then).

      // set tappoint based on post-processed flag.
      cwb_config_->tap_point = (layer_stack->flags.post_processed_output)
                                   ? CwbTapPoint::kDsppTapPoint
                                   : CwbTapPoint::kLmTapPoint;

      uint32_t buffer_width = 0, buffer_height = 0;
      error = GetCwbBufferResolution(cwb_config_, &buffer_width, &buffer_height);
      if (error != kErrorNone) {
        DLOGE("GetCwbBufferResolution failed for tap_point = %d .", cwb_config_->tap_point);
        return error;
      }
      DLOGW("Layerstack.cwb_config isn't set by CWB client. Thus, falling back to Full frame ROI.");
      cwb_config_->cwb_roi = cwb_config_->cwb_full_rect;
    } else {  // Cwb client has set the cwb config in LayerStack.cwb_config .
      *cwb_config_ = *(layer_stack->cwb_config);
    }

    // Config dither data
    cwb_config_->dither_info = nullptr;
    if (cwb_config_->tap_point != CwbTapPoint::kLmTapPoint && color_mgr_) {
      error = color_mgr_->ConfigureCWBDither(cwb_config_, false);
      if (error != kErrorNone) {
        DLOGE("CWB dither config failed, error %d", error);
      }
    }

    disp_layer_stack_.info.hw_cwb_config = cwb_config_;
    error = ValidateCwbConfigInfo(disp_layer_stack_.info.hw_cwb_config,
                                  layer_stack->output_buffer->format);
    if (error != kErrorNone) {
      DLOGE("CWB_config validation failed.");
      return error;
    }
    if (!needs_validate_) {
      if (cwb_config_->pu_as_cwb_roi) {
        needs_validate_ = true;
        DLOGI_IF(kTagDisplay, "pu_as_cwb_roi: true. Validate call needed for CWB.");
      } else if (cwb_config_->tap_point == CwbTapPoint::kLmTapPoint ||
                 !disable_pu_on_dest_scaler_) {
        // Either if cwb tppt is LM or if cwb tppt is DSPP/Demura with destin scalar disabled, then
        // check whether PU ROI contains CWB ROI. If it doesn't, then set needs_validate_ to true.
        // Note: If destin scalar is enabled, then there would be full frame update and the check
        // whether PU ROI contains CWB ROI isn't needed. CWB doesn't requires Validate call then.
        bool cwb_needs_validate = true;
        for (uint32_t i = 0; i < disp_layer_stack_.info.left_frame_roi.size(); i++) {
          auto &pu_roi = disp_layer_stack_.info.left_frame_roi.at(i);
          if (Contains(pu_roi, cwb_config_->cwb_roi)) {  // checking whether PU roi contain CWB roi
            DLOGI_IF(kTagDisplay, "PU ROI contains CWB ROI. Validate not needed for CWB.");
            cwb_needs_validate = false;
            break;
          }
        }
        needs_validate_ = cwb_needs_validate;
      }
    }
  } else if (cwb_config_) {  // CWB isn't requested in the current draw cycle.
    // Release dither data
    if (color_mgr_) {
      error = color_mgr_->ConfigureCWBDither(cwb_config_, true);
      if (error != kErrorNone) {
        DLOGE("Release dither data failed.");
      }
    }
    // Check and release cwb_config_ if it was instantiated in the previous draw cycle.
    delete cwb_config_;
    cwb_config_ = NULL;
    disp_layer_stack_.info.hw_cwb_config = NULL;
    needs_validate_ = true;  // Do not skip Validate in CWB teardown frame.

    comp_manager_->HandleCwbFrequencyBoost(false);
  }
  return error;
}

bool DisplayBase::IsWriteBackSupportedFormat(const LayerBufferFormat &format) {
  // check whether writeback supported for parameter color format or not.
  std::map<HWSubBlockType, std::vector<LayerBufferFormat>>::iterator it =
      hw_resource_info_.supported_formats_map.find(HWSubBlockType::kHWWBIntfOutput);
  if (it == hw_resource_info_.supported_formats_map.end()) {
    return false;
  }
  std::vector<LayerBufferFormat> &supported_sdm_formats = it->second;
  if (supported_sdm_formats.empty()) {
    return false;
  }
  for (int i = 0; i < supported_sdm_formats.size(); i++) {
    if (supported_sdm_formats[i] == format) {
      return true;
    }
  }
  return false;
}

DisplayError DisplayBase::BuildLayerStackStats(LayerStack *layer_stack) {
  DTRACE_SCOPED();
  std::vector<Layer *> &layers = layer_stack->layers;
  HWLayersInfo &hw_layers_info = disp_layer_stack_.info;
  hw_layers_info.app_layer_count = 0;
  hw_layers_info.gpu_target_index = -1;
  hw_layers_info.stitch_target_index = -1;
  hw_layers_info.noise_layer_index = -1;

  disp_layer_stack_.stack = layer_stack;
  hw_layers_info.flags = layer_stack->flags;
  hw_layers_info.blend_cs = layer_stack->blend_cs;
  hw_layers_info.wide_color_primaries.clear();

  int index = 0;
  for (auto &layer : layers) {
    if (layer->buffer_map == nullptr) {
      layer->buffer_map = std::make_shared<LayerBufferMap>();
    }
    if (layer->composition == kCompositionGPUTarget) {
      hw_layers_info.gpu_target_index = hw_layers_info.app_layer_count;
    } else if (layer->composition == kCompositionStitchTarget) {
      hw_layers_info.stitch_target_index = index;
    } else if (layer->flags.is_noise) {
      hw_layers_info.flags.noise_present = true;
      hw_layers_info.noise_layer_index = index;
      hw_layers_info.noise_layer_info = noise_layer_info_;
      DLOGV_IF(kTagDisplay, "Display %d-%d requested Noise at index = %d with zpos_n = %d",
                display_id_, display_type_, index, noise_layer_info_.zpos_noise);
    } else {
      hw_layers_info.app_layer_count++;
    }
    if (IsWideColor(layer->input_buffer.color_metadata.colorPrimaries)) {
      hw_layers_info.wide_color_primaries.push_back(
          layer->input_buffer.color_metadata.colorPrimaries);
    }
    if (layer->flags.is_game) {
      hw_layers_info.game_present = true;
    }
    index++;
  }

  DLOGD_IF(kTagDisplay, "LayerStack layer_count: %zu, app_layer_count: %d, "
                        "gpu_target_index: %d, stitch_index: %d game_present: %d "
                        " noise_present: %d display: %d-%d", layers.size(),
                        hw_layers_info.app_layer_count, hw_layers_info.gpu_target_index,
                        hw_layers_info.stitch_target_index, hw_layers_info.game_present,
                        hw_layers_info.flags.noise_present, display_id_, display_type_);

  if (!hw_layers_info.app_layer_count) {
    DLOGW("Layer count is zero");
    return kErrorNoAppLayers;
  }

  if (hw_layers_info.gpu_target_index > 0) {
    return ValidateGPUTargetParams();
  }

  return kErrorNone;
}

DisplayError DisplayBase::ValidateGPUTargetParams() {
  HWLayersInfo &hw_layers_info = disp_layer_stack_.info;
  Layer *gpu_target_layer = disp_layer_stack_.stack->layers.at(hw_layers_info.gpu_target_index);

  if (!IsValid(gpu_target_layer->src_rect)) {
    DLOGE("Invalid src rect for GPU target layer");
    return kErrorParameters;
  }

  if (!IsValid(gpu_target_layer->dst_rect)) {
    DLOGE("Invalid dst rect for GPU target layer");
    return kErrorParameters;
  }

  float layer_mixer_width = FLOAT(mixer_attributes_.width);
  float layer_mixer_height = FLOAT(mixer_attributes_.height);
  float fb_width = FLOAT(fb_config_.x_pixels);
  float fb_height = FLOAT(fb_config_.y_pixels);
  LayerRect src_domain = (LayerRect){0.0f, 0.0f, fb_width, fb_height};
  LayerRect dst_domain = (LayerRect){0.0f, 0.0f, layer_mixer_width, layer_mixer_height};
  LayerRect out_rect = gpu_target_layer->dst_rect;

  MapRect(src_domain, dst_domain, gpu_target_layer->dst_rect, &out_rect);
  Normalize(1, 1, &out_rect);

  auto gpu_target_layer_dst_xpixels = out_rect.right - out_rect.left;
  auto gpu_target_layer_dst_ypixels = out_rect.bottom - out_rect.top;

  if (gpu_target_layer_dst_xpixels > mixer_attributes_.width ||
    gpu_target_layer_dst_ypixels > mixer_attributes_.height) {
    DLOGE("GPU target layer dst rect is not with in limits gpu wxh %fx%f, mixer wxh %dx%d",
                  gpu_target_layer_dst_xpixels, gpu_target_layer_dst_ypixels,
                  mixer_attributes_.width, mixer_attributes_.height);
    return kErrorParameters;
  }

  return kErrorNone;
}

bool DisplayBase::IsValidateNeeded() {
  // This api checks on special cases for which Validate call may be needed.
  if (pu_pending_ && partial_update_control_ && !disable_pu_one_frame_ &&
      !disable_pu_on_dest_scaler_ && !(color_mgr_ && color_mgr_->NeedsPartialUpdateDisable())) {
    // If a PU request is pending and PU is enabled for the current frame,
    // then prevent Skip Validate in order to recalculate PU.
    pu_pending_ = false;
    return true;
  }
  return false;
}

DisplayError DisplayBase::PrePrepare(LayerStack *layer_stack) {
  DTRACE_SCOPED();
  ClientLock lock(disp_mutex_);

  // Allow prepare as pending doze/pending_power_on is handled as a part of draw cycle
  if (!active_ && (pending_power_state_ == kPowerStateNone)) {
    return kErrorPermission;
  }

  DisplayError error = InitRC();
  if (error != kErrorNone) {
    // Non-fatal but not expected, log error
    DLOGE("RC Failed to initialize. Error = %d", error);
  }

  error = HandleNoiseLayer(layer_stack);
  if (error != kErrorNone) {
    DLOGW("HandleNoiseLayer returned Error for display %d-%d", display_id_, display_type_);
  }

  error = BuildLayerStackStats(layer_stack);
  if (error != kErrorNone) {
    return error;
  }

  error = PrepareRC(layer_stack);
  if (error == kErrorNeedsValidate) {
    needs_validate_ |= true;
  } else  if (error != kErrorNone) {
    DLOGE("PrepareRC returned error = %d for display %d-%d", error, display_id_, display_type_);
  }

  needs_validate_ |= IsValidateNeeded();

  error = ConfigureCwb(layer_stack);
  if (error != kErrorNone) {
    return error;
  }

  layer_stack->needs_validate = !validated_ || needs_validate_;
  if (!layer_stack->needs_validate) {
    // Check for validation in case of new display connected, other displays exiting off state etc.
    bool needs_validate = false;
    comp_manager_->NeedsValidate(display_comp_ctx_, &needs_validate);
    layer_stack->needs_validate = needs_validate;
  }
  return comp_manager_->PrePrepare(display_comp_ctx_, &disp_layer_stack_);
}

DisplayError DisplayBase::ForceToneMapUpdate(LayerStack *layer_stack) {
  DTRACE_SCOPED();
  DisplayError error = kErrorNotSupported;


  for (size_t hw_index = 0; hw_index < disp_layer_stack_.info.index.size(); hw_index++) {
    size_t layer_index = disp_layer_stack_.info.index.at(hw_index);

    if (layer_index >= layer_stack->layers.size()) {
      DLOGE("Error forcing TM update. Layer stack appears to have changed");
      return error;
    }

    Layer *stack_layer = layer_stack->layers.at(layer_index);
    Layer &cached_layer = disp_layer_stack_.info.hw_layers.at(hw_index);
    HWLayerConfig &hw_config = disp_layer_stack_.info.config[hw_index];

    cached_layer.input_buffer.hist_data = stack_layer->input_buffer.hist_data;
    cached_layer.input_buffer.color_metadata = stack_layer->input_buffer.color_metadata;
    hw_config.left_pipe.lut_info.clear();
    hw_config.right_pipe.lut_info.clear();
  }

  error = comp_manager_->ForceToneMapConfigure(display_comp_ctx_, &disp_layer_stack_);
  if (error == kErrorNone) {
    validated_ = true;
  }

  return error;
}

DisplayError DisplayBase::Prepare(LayerStack *layer_stack) {
  DTRACE_SCOPED();
  ClientLock lock(disp_mutex_);
  DisplayError error = kErrorNone;
  needs_validate_ = true;

  if (!layer_stack) {
    return kErrorParameters;
  }

  disp_layer_stack_.info.output_buffer = layer_stack->output_buffer;

  // Allow prepare as pending doze/pending_power_on is handled as a part of draw cycle
  if (!active_ && (pending_power_state_ == kPowerStateNone)) {
    return kErrorPermission;
  }

  DLOGI_IF(kTagDisplay, "Entering Prepare for display: %d-%d", display_id_, display_type_);
  error = BuildLayerStackStats(layer_stack);
  if (error != kErrorNone) {
    return error;
  }

  // This call in Prepare will return the cached value during PrePrepare()
  PrepareRC(layer_stack);

  error = ConfigureCwb(layer_stack);
  if (error != kErrorNone) {
    return error;
  }

  if (color_mgr_) {
    color_mgr_->Prepare();
    // apply pending DE config
    PPPendingParams pending_action;
    PPDisplayAPIPayload req_payload;
    pending_action.action = kGetDetailedEnhancerData;
    pending_action.params = NULL;
    error = color_mgr_->ColorSVCRequestRoute(req_payload, NULL, &pending_action);
    if (!error && pending_action.action == kConfigureDetailedEnhancer) {
      error = SetHWDetailedEnhancerConfig(pending_action.params);
    }
  }

  if (color_mgr_ && color_mgr_->NeedsPartialUpdateDisable()) {
    DisablePartialUpdateOneFrameInternal();
  }
  // TODO(user): Temporary changes, to be removed when DRM driver supports
  // Partial update with Destination scaler enabled.
  if (!partial_update_control_ || disable_pu_one_frame_ ||
      disable_pu_on_dest_scaler_) {
    comp_manager_->ControlPartialUpdate(display_comp_ctx_, false /* enable */);
    disable_pu_one_frame_ = false;
  }

  disp_layer_stack_.info.updates_mask.set(kUpdateResources);
  comp_manager_->GenerateROI(display_comp_ctx_, &disp_layer_stack_);

  CheckMMRMState();

  while (true) {
    error = comp_manager_->Prepare(display_comp_ctx_, &disp_layer_stack_);
    if (error != kErrorNone) {
      break;
    }

    // Trigger validate only if needed.
    if (disp_layer_stack_.info.do_hw_validate) {
      error = hw_intf_->Validate(&disp_layer_stack_.info);
    }

    if (error == kErrorNone) {
      // Strategy is successful now, wait for Commit().
      validated_ = true;
      needs_validate_ = false;
      break;
    }
    if (error == kErrorShutDown) {
      comp_manager_->PostPrepare(display_comp_ctx_, &disp_layer_stack_);
      return error;
    }
  }

  if (color_mgr_)
    color_mgr_->Validate(&disp_layer_stack_);

  comp_manager_->PostPrepare(display_comp_ctx_, &disp_layer_stack_);

  CacheDisplayComposition();

  if (error == kErrorNone) {
    error = ConfigureCwbForIdleFallback(layer_stack);
    if (error != kErrorNone) {
      return error;
    }
  }

  if (disp_layer_stack_.info.enable_self_refresh) {
    hw_intf_->EnableSelfRefresh();
  }

  DLOGI_IF(kTagDisplay, "Exiting Prepare for display type : %d error: %d", display_type_, error);

  return error;
}

void DisplayBase::FlushConcurrentWriteback() {
  hw_intf_->FlushConcurrentWriteback();
}

DisplayError DisplayBase::HandleNoiseLayer(LayerStack *layer_stack) {
  if (noise_disable_prop_ || !noise_plugin_intf_) {
    return kErrorNone;
  }

  if (!layer_stack) {
    DLOGE("layer_stack is null");
    return kErrorParameters;
  }

  DisplayError error = GetNoisePluginParams(layer_stack);
  if (error) {
    DLOGW("Noise Plugin Failed for display %d-%d", display_id_, display_type_);
    return error;
  }

  HWLayersInfo &hw_layers_info = disp_layer_stack_.info;
  if (!noise_layer_info_.enable) {
    if (hw_layers_info.noise_layer_index != -1) {
      DLOGV_IF(kTagDisplay, "Noise layer disabled for display %d-%d", display_id_, display_type_);
      needs_validate_ = true;
    }
    return kErrorNone;
  } else {
    if (hw_layers_info.noise_layer_index == -1) {
      DLOGV_IF(kTagDisplay, "Noise layer Enabled for display %d-%d", display_id_, display_type_);
      needs_validate_ = true;
    }
    InsertNoiseLayer(layer_stack);
  }

  return error;
}

DisplayError DisplayBase::InsertNoiseLayer(LayerStack *layer_stack) {
  std::vector<Layer *> &layers = layer_stack->layers;
  uint32_t primary_width = 0, primary_height = 0;
  GetMixerResolution(&primary_width, &primary_height);

  noise_layer_.flags.is_noise = true;
  noise_layer_.composition = kCompositionSDE;
  LayerBuffer *layer_buffer = &noise_layer_.input_buffer;
  layer_buffer->width = primary_width;
  layer_buffer->height = primary_height;
  layer_buffer->unaligned_width = layer_buffer->width;
  layer_buffer->unaligned_height = layer_buffer->height;
  noise_layer_.src_rect.left = 0;
  noise_layer_.src_rect.top = 0;
  noise_layer_.src_rect.right = layer_buffer->width;
  noise_layer_.src_rect.bottom = layer_buffer->height;
  noise_layer_.dst_rect = noise_layer_.src_rect;

  layers.push_back(&noise_layer_);

  return kErrorNone;
}

DisplayError DisplayBase::GetNoisePluginParams(LayerStack *layer_stack) {
  NoisePlugInInputParams *noise_plugin_in = nullptr;
  NoisePlugInOutputParams *noise_plugin_out = nullptr;
  GenericPayload in_payload, out_payload;
  int ret = in_payload.CreatePayload<NoisePlugInInputParams>(noise_plugin_in);
  if (ret) {
    DLOGE("Failed to create input payload, ret = %d", ret);
    return kErrorUndefined;
  }
  ret = out_payload.CreatePayload<NoisePlugInOutputParams>(noise_plugin_out);
  if (ret) {
    DLOGE("Failed to create output payload, ret = %d", ret);
    return kErrorUndefined;
  }

  if (noise_plugin_override_en_) {
    DLOGI_IF(kTagDisplay, "Display %d-%d Override enabled with noise_override_zpos_ = %d",
             display_id_, display_type_, noise_override_zpos_);
  }
  std::vector<Layer *> &layers = layer_stack->layers;
  int idx = 0;
  int32_t skip_layer_count = 0;
  for (auto &layer : layers) {
    NoisePlugInInputLayers in_layer = {};
    if (layer->flags.skip) {
      skip_layer_count++;
    }
    if (noise_plugin_override_en_ &&
       ((idx == noise_override_zpos_) && !layer->input_buffer.flags.mask_layer)) {
      // NoisePlugin needs sde_preferred to be set, which is determined by debug Override
      // If override(zpos) is set, then mark the layer as preferred if its not a mask layer
      DLOGV_IF(kTagDisplay, "For display %d-%d, Setting sde_preferred flag from override for "
               "idx = %d at z_pos = %d", display_id_, display_type_, idx, noise_override_zpos_);
      layer->flags.sde_preferred = true;
    }
    if (layer->composition == kCompositionGPUTarget) {
      // Avoid sending GPU target to Plugin and break as its the last layer from client
      break;
    } else if (layer->flags.sde_preferred && !layer->flags.skip) {
      in_layer.layer_type = kFodLayer;
    } else if (layer->flags.is_game) {
      in_layer.layer_type = kGameLayer;
    } else if (layer->input_buffer.flags.mask_layer) {
      in_layer.layer_type = kMaskLayer;
    } else if (layer->flags.solid_fill) {
      in_layer.layer_type = kDimLayer;
    } else {
      in_layer.layer_type = kGraphicsLayer;
    }
    in_layer.zorder = idx;
    noise_plugin_in->layers.push_back(in_layer);
    idx++;
  }
  noise_layer_info_ = {};  // clear the noise layer info as new values must be set after Plugin
  bool disable_noise_plugin = false;  // Need to disable noiseplugin when all layers are skip
  if (skip_layer_count == idx) {
    // All app layers are marked as skip
    disable_noise_plugin = true;
    DLOGV_IF(kTagDisplay, "Display %d-%d All layers are skip, disable NoisePlugin", display_id_,
             display_type_);
  }
  GenericPayload payload;
  int32_t *val = nullptr;
  ret = payload.CreatePayload<int32_t>(val);
  if (ret) {
    DLOGE("Display %d-%d CreatePayload failed for NoisePlugInDisable", display_id_,
          display_type_, ret);
    return kErrorUndefined;
  }
  *val = disable_noise_plugin ? 1 : 0;
  ret = noise_plugin_intf_->SetParameter(kNoisePlugInDisable, payload);
  if (ret) {
    DLOGE("Display %d-%d Disabling NoisePlugin for Full frame skip failed", display_id_,
          display_type_, ret);
    return kErrorUndefined;
  }
  ret = noise_plugin_intf_->ProcessOps(kOpsRunNoisePlugIn, in_payload, &out_payload);
  if (!ret && noise_plugin_out->enabled) {
    noise_layer_info_.enable = noise_plugin_out->enabled;
    noise_layer_info_.zpos_noise = noise_plugin_out->zpos[0];
    noise_layer_info_.zpos_attn = noise_plugin_out->zpos[1];
    noise_layer_info_.attenuation_factor = noise_plugin_out->attn;
    noise_layer_info_.noise_strength = noise_plugin_out->strength;
    noise_layer_info_.alpha_noise = noise_plugin_out->alpha_noise;
    noise_layer_info_.temporal_en = noise_plugin_out->temporal_en;
    DLOGV_IF(kTagDisplay, "For display %d-%d, Noise enabled by Plugin, zpos_noise = %d "
             "zpos_attn = %d, attn = %d, Noise strength = %d, alpha noise = %d, temporal_en = %d",
             display_id_, display_type_, noise_layer_info_.zpos_noise,
             noise_layer_info_.zpos_attn, noise_layer_info_.attenuation_factor,
             noise_layer_info_.noise_strength, noise_layer_info_.alpha_noise,
             noise_layer_info_.temporal_en);
  }

  return ret ? kErrorUndefined : kErrorNone;
}

// Send layer stack to RC core to generate and configure the mask on HW.
DisplayError DisplayBase::PrepareRC(LayerStack *layer_stack) {
  if (!rc_panel_feature_init_) {
    return kErrorNone;
  }

  if (rc_prepared_) {
    // Set the RC data into LayerStack which was generated in PrePrepare()
    disp_layer_stack_.info.rc_config = rc_config_enable_;
    disp_layer_stack_.info.rc_layers_info = rc_info_;
    if (rc_config_enable_) {
      DLOGV_IF(kTagDisplay, "RC is prepared, top_height = %d, RC bot_height = %d",
               rc_info_.top_height, rc_info_.bottom_height);
    }
    return kErrorNone;
  }

  DTRACE_SCOPED();
  int ret = -1;
  HWLayersInfo &hw_layers_info = disp_layer_stack_.info;
  hw_layers_info.spr_enable = spr_enable_;
  DLOGI_IF(kTagDisplay, "Display resolution: %dx%d", display_attributes_.x_pixels,
           display_attributes_.y_pixels);
  if (rc_cached_res_width_ != display_attributes_.x_pixels) {
    GenericPayload in;
    uint32_t *display_xres = nullptr;
    ret = in.CreatePayload<uint32_t>(display_xres);
    if (ret) {
      DLOGE("failed to create the payload. Error:%d", ret);
      return kErrorUndefined;
    }
    *display_xres = rc_cached_res_width_ = display_attributes_.x_pixels;
    ret = rc_core_->SetParameter(kRCFeatureDisplayXRes, in);
    if (ret) {
      DLOGE("failed to set display X resolution. Error:%d", ret);
      return kErrorUndefined;
    }
  }

  if (rc_cached_res_height_ != display_attributes_.y_pixels) {
    GenericPayload in;
    uint32_t *display_yres = nullptr;
    ret = in.CreatePayload<uint32_t>(display_yres);
    if (ret) {
      DLOGE("failed to create the payload. Error:%d", ret);
      return kErrorUndefined;
    }
    *display_yres = rc_cached_res_height_ = display_attributes_.y_pixels;
    ret = rc_core_->SetParameter(kRCFeatureDisplayYRes, in);
    if (ret) {
      DLOGE("failed to set display Y resolution. Error:%d", ret);
      return kErrorUndefined;
    }
  }

  if (rc_cached_mixer_width_ != mixer_attributes_.width) {
    GenericPayload in;
    uint32_t *mixer_width = nullptr;
    ret = in.CreatePayload<uint32_t>(mixer_width);
    if (ret) {
      DLOGE("failed to create the payload. Error:%d", ret);
      return kErrorUndefined;
    }
    *mixer_width = rc_cached_mixer_width_ = mixer_attributes_.width;
    ret = rc_core_->SetParameter(kRCFeatureMixerWidth, in);
    if (ret) {
      DLOGE("failed to set mixer width. Error:%d", ret);
      return kErrorUndefined;
    }
  }

  if (rc_cached_mixer_height_ != mixer_attributes_.height) {
    GenericPayload in;
    uint32_t *mixer_height = nullptr;
    ret = in.CreatePayload<uint32_t>(mixer_height);
    if (ret) {
      DLOGE("failed to create the payload. Error:%d", ret);
      return kErrorUndefined;
    }
    *mixer_height = rc_cached_mixer_height_ = mixer_attributes_.height;
    ret = rc_core_->SetParameter(kRCFeatureMixerHeight, in);
    if (ret) {
      DLOGE("failed to set mixer height. Error:%d", ret);
      return kErrorUndefined;
    }
  }

  if (rc_cached_fb_width_ != fb_config_.x_pixels) {
    GenericPayload in;
    uint32_t *fb_width = nullptr;
    ret = in.CreatePayload<uint32_t>(fb_width);
    if (ret) {
      DLOGE("failed to create the payload. Error:%d", ret);
      return kErrorUndefined;
    }
    *fb_width = rc_cached_fb_width_ = fb_config_.x_pixels;
    ret = rc_core_->SetParameter(kRCFeatureFbWidth, in);
    if (ret) {
      DLOGE("failed to set fb width. Error:%d", ret);
      return kErrorUndefined;
    }
  }

  if (rc_cached_fb_height_ != fb_config_.y_pixels) {
    GenericPayload in;
    uint32_t *fb_height = nullptr;
    ret = in.CreatePayload<uint32_t>(fb_height);
    if (ret) {
      DLOGE("failed to create the payload. Error:%d", ret);
      return kErrorUndefined;
    }
    *fb_height = rc_cached_fb_height_ = fb_config_.y_pixels;
    ret = rc_core_->SetParameter(kRCFeatureFbHeight, in);
    if (ret) {
      DLOGE("failed to set mixer height. Error:%d", ret);
      return kErrorUndefined;
    }
  }

  GenericPayload in;
  LayerStack **layer_stack_ptr = nullptr;
  ret = in.CreatePayload<LayerStack *>(layer_stack_ptr);
  if (ret) {
    DLOGE("failed to create the payload. Error:%d", ret);
    return kErrorUndefined;
  }
  *layer_stack_ptr = layer_stack;

  LayerStack rc_stack;
  if (enable_win_rect_mask_) {
    // ToDo: Append RC layers as well. Handle Rc + Window rect.
    // Append border layers.
    for (auto &layer : border_layers_) {
      rc_stack.layers.push_back(&layer);
    }

    *layer_stack_ptr = &rc_stack;
  }


  GenericPayload out;
  RCOutputConfig *rc_out_config = nullptr;
  ret = out.CreatePayload<RCOutputConfig>(rc_out_config);
  if (ret) {
    DLOGE("failed to create the payload. Error:%d", ret);
    return kErrorUndefined;
  }

  hw_layers_info.rc_layers_info.mask_layer_idx.clear();
  hw_layers_info.rc_layers_info.rc_hw_layer_idx.clear();

  ret = rc_core_->ProcessOps(kRCFeaturePrepare, in, &out);
  if (!ret) {
    DLOGD_IF(kTagDisplay, "RC top_height = %d, RC bot_height = %d", rc_out_config->top_height,
             rc_out_config->bottom_height);
    if (rc_out_config->rc_needs_full_roi) {
      DisablePartialUpdateOneFrameInternal();
    }
    hw_layers_info.rc_config = true;
    hw_layers_info.rc_layers_info.top_width = rc_out_config->top_width;
    hw_layers_info.rc_layers_info.top_height = rc_out_config->top_height;
    hw_layers_info.rc_layers_info.bottom_width = rc_out_config->bottom_width;
    hw_layers_info.rc_layers_info.bottom_height = rc_out_config->bottom_height;

    rc_config_enable_ = true;
    rc_info_ = hw_layers_info.rc_layers_info;
  } else {
    rc_config_enable_ = false;
    rc_info_ = {};
  }

  for (const auto &layer : layer_stack->layers) {
    if (layer->input_buffer.flags.mask_layer) {
      hw_layers_info.rc_layers_info.mask_layer_idx.push_back(UINT32(layer->layer_id));
      rc_info_.mask_layer_idx.push_back(UINT32(layer->layer_id));
      if (layer->request.flags.rc && !ret) {
        hw_layers_info.rc_layers_info.rc_hw_layer_idx.push_back(UINT32(layer->layer_id));
        rc_info_.rc_hw_layer_idx.push_back(UINT32(layer->layer_id));
      }
    }
  }

  DisplayError error = kErrorNone;
  GenericPayload input, output;
  RCMaskCfgState *mask_status = nullptr;
  ret = output.CreatePayload<RCMaskCfgState>(mask_status);
  if (ret) {
    DLOGE("failed to create the payload. Error:%d", ret);
    return kErrorUndefined;
  }
  ret = rc_core_->ProcessOps(kRCFeaturePostPrepare, input, &output);
  if (ret) {
    // If RC commit failed, fall back to default (GPU/SDE pipes) drawing of "handled" mask layers.
    if ((*mask_status).rc_mask_state == kStatusRcMaskStackHandled) {
      DLOGW("Couldn't Commit RC in kRCFeaturePostPrepare for display: %d-%d Error: %d, status: %d"
             " Needs Validate", display_id_, display_type_, ret, (*mask_status).rc_mask_state);
      rc_config_enable_ = false;
      rc_info_ = {};
      for (auto &layer : layer_stack->layers) {
        if (layer->input_buffer.flags.mask_layer) {
          layer->request.flags.rc = false;
        }
      }
      error = kErrorNeedsValidate;
    }
  } else {
    DLOGI_IF(kTagDisplay, "Status of RC mask data: %d.", (*mask_status).rc_mask_state);
    if ((*mask_status).rc_mask_state == kStatusRcMaskStackDirty) {
      DisablePartialUpdateOneFrameInternal();
      DLOGI_IF(kTagDisplay, "Mask is ready for display %d-%d, call Corresponding Prepare()",
               display_id_, display_type_);
      error = kErrorNeedsValidate;
    }
  }

  rc_prepared_ = true;

  return error;
}

DisplayError DisplayBase::CommitOrPrepare(LayerStack *layer_stack) {
  DTRACE_SCOPED();
  ClientLock lock(disp_mutex_);
  DisplayError error = kErrorNone;
  // Perform prepare
  error = Prepare(layer_stack);
  if (error != kErrorNone) {
    if (error == kErrorPermission) {
      DLOGW("Prepare failed: %d", error);
    } else {
      DLOGE("Prepare failed: %d", error);
    }
    // Clear fences
    DLOGI("Clearing fences on input layers on display %d-%d", display_id_, display_type_);
    for (auto &layer : layer_stack->layers) {
      layer->input_buffer.release_fence = nullptr;
    }
    layer_stack->retire_fence = nullptr;

    return error;
  }

  // Trigger commit based on draw outcome.
  bool async_commit = disp_layer_stack_.info.trigger_async_commit;
  DLOGV_IF(kTagDisplay, "Trigger async commit: %d", async_commit);
  if (async_commit) {
    // Copy layer stack attributes needed for commit.
    error = SetUpCommit(layer_stack);
    if (error != kErrorNone) {
      return error;
    }

    PrepareForAsyncTransition();

    // Notify worker to do hw commit.
    lock.NotifyWorker();
  }

  return async_commit ? kErrorNone : kErrorNeedsCommit;
}

void DisplayBase::HandleAsyncCommit() {
  // Do not acquire mutexes here.
  // Perform hw commit here.
  PerformHwCommit(&disp_layer_stack_.info);
}

void DisplayBase::CommitThread() {
  // Acquire worker mutex and wait for events.
  lock_guard<recursive_mutex> worker_lock(disp_mutex_.worker_mutex);

  DLOGI("Commit thread entered. %d-%d", display_id_, display_type_);

  // Commit thread need to run with real time priority ie; similar to composer thread.
  SetRealTimePriority();

  // Notify client thread that the thread has started listening to events.
  {
    DLOGI("Notify client.");
    lock_guard<recursive_mutex> client_lock(disp_mutex_.client_mutex);
    disp_mutex_.client_cv.notify_one();
  }

  DLOGI("Commit thread started.");

  while (1) {
    // Reset busy status and notify. There may be some thread waiting for the status to reset.
    disp_mutex_.worker_busy = false;
    disp_mutex_.worker_cv.notify_one();

    // Wait for client thread to signal. Handle spurious interrupts.
    if (!(disp_mutex_.worker_cv.wait_until(disp_mutex_.worker_mutex, WaitUntil(), [this] {
      return (disp_mutex_.worker_busy);
    }))) {
      DLOGI("Received idle timeout");
      event_handler_->HandleEvent(kIdleTimeout);
      IdleTimeout();
      continue;
    }

    if (disp_mutex_.worker_exit) {
      DLOGI("Terminate commit thread.");
      break;
    }

    HandleAsyncCommit();
  }
}

DisplayError DisplayBase::SetUpCommit(LayerStack *layer_stack) {
  DTRACE_SCOPED();
  DisplayError error = kErrorNone;

  if (!layer_stack) {
    return kErrorParameters;
  }

#ifdef TRUSTED_VM
  // Register all hw events on first commit for trusted vm only as the hw acquire happens as a
  // part of first validate
  if (first_cycle_) {
    hw_events_intf_->SetEventState(HWEvent::PANEL_DEAD, true);
    hw_events_intf_->SetEventState(HWEvent::IDLE_POWER_COLLAPSE, true);
    hw_events_intf_->SetEventState(HWEvent::HW_RECOVERY, true);
    hw_events_intf_->SetEventState(HWEvent::HISTOGRAM, true);
    hw_events_intf_->SetEventState(HWEvent::MMRM, true);
  }
#endif

  disp_layer_stack_.info.output_buffer = layer_stack->output_buffer;
  if (layer_stack->request_flags.trigger_refresh) {
    if (!disable_cwb_idle_fallback_ && disp_layer_stack_.info.output_buffer) {
      cwb_fence_wait_ = true;
    }
    layer_stack->output_buffer = nullptr;
  }

  disp_layer_stack_.info.retire_fence_offset = retire_fence_offset_;
  // Regiser for power events on first cycle in unified draw.
  if (first_cycle_ && (draw_method_ != kDrawDefault) && (display_type_ != kVirtual) &&
      !hw_panel_info_.is_primary_panel && (display_type_ != kHDMI)) {
    DLOGI("Registering for power events");
    hw_events_intf_->SetEventState(HWEvent::POWER_EVENT, true);
  }

  // Allow commit as pending doze/pending_power_on is handled as a part of draw cycle
  if (!active_ && (pending_power_state_ == kPowerStateNone)) {
    validated_ = false;
    return kErrorPermission;
  }

  if (needs_validate_) {
    DLOGE("Commit: Corresponding Prepare() is not called for display %d-%d", display_id_,
          display_type_);
    validated_ = false;
    return kErrorNotValidated;
  }

  DLOGI_IF(kTagDisplay, "Entering commit for display: %d-%d", display_id_, display_type_);
  CommitLayerParams(layer_stack);

  error = comp_manager_->Commit(display_comp_ctx_, &disp_layer_stack_);
  if (error != kErrorNone) {
    return error;
  }

  // check if feature list cache is dirty and pending.
  // If dirty, need program to hardware blocks.
  if (color_mgr_)
    error = color_mgr_->Commit();
  if (error != kErrorNone) {  // won't affect this execution path.
    DLOGW("ColorManager::Commit(...) isn't working");
  }

  return error;
}

DisplayError DisplayBase::PerformCommit(HWLayersInfo *hw_layers_info) {
  DTRACE_SCOPED();
  TrackInputFences();
  DisplayError error = hw_intf_->Commit(hw_layers_info);
  if (error != kErrorNone) {
    DLOGE("COMMIT failed: %d ", error);
  }

  return error;
}

DisplayError DisplayBase::Commit(LayerStack *layer_stack) {
  DTRACE_SCOPED();
  ClientLock lock(disp_mutex_);

  disp_layer_stack_.stack = layer_stack;

  if (draw_method_ == kDrawDefault) {
    return CommitLocked(layer_stack);
  }

  // Copy layer stack attributes needed for commit.
  DisplayError error = SetUpCommit(layer_stack);
  if (error != kErrorNone) {
    return error;
  }

  PrepareForAsyncTransition();

  // Trigger async commit.
  lock.NotifyWorker();

  return kErrorNone;
}

DisplayError DisplayBase::CommitLocked(LayerStack *layer_stack) {
  DTRACE_SCOPED();
  DisplayError error = SetUpCommit(layer_stack);
  if (error != kErrorNone) {
    DLOGW("SetUpCommit failed %d", error);
    return error;
  }

  error = PerformHwCommit(&disp_layer_stack_.info);
  if (error != kErrorNone) {
    DLOGE("HwCommit failed %d", error);
  }

  return error;
}

DisplayError DisplayBase::PerformHwCommit(HWLayersInfo *hw_layers_info) {
  DTRACE_SCOPED();
  DisplayError error = PerformCommit(hw_layers_info);
  if (error != kErrorNone) {
    DLOGE("Commit IOCTL failed %d", error);
    CleanupOnError();
    DLOGI("Triggering flush to release fences");
    DisplayError flush_err = FlushLocked(nullptr);
    if (flush_err != kErrorNone) {
      DLOGE("flush_err: %d", flush_err);
      return flush_err;
    }
  }

  // TODO(user): Workaround for messenger app flicker issue in CWB idle fallback,
  // to be removed when issue is fixed.
  if (cwb_fence_wait_ && hw_layers_info->output_buffer &&
      (hw_layers_info->output_buffer->release_fence != nullptr)) {
    if (Fence::Wait(hw_layers_info->output_buffer->release_fence) != kErrorNone) {
      DLOGW("sync_wait error errno = %d, desc = %s", errno, strerror(errno));
    }
  }
  cwb_fence_wait_ = false;

  error = PostCommit(hw_layers_info);
  if (error != kErrorNone) {
    DLOGE("Post Commit failed %d", error);
    return error;
  }

  DLOGI_IF(kTagDisplay, "Exiting commit for display: %d-%d", display_id_, display_type_);

  return kErrorNone;
}

void DisplayBase::CleanupOnError() {
  // Buffer Fd's are duped for async thread operation.
  for (auto &hw_layer : disp_layer_stack_.info.hw_layers) {
    CloseFd(&hw_layer.input_buffer.planes[0].fd);
  }
}

DisplayError DisplayBase::PostCommit(HWLayersInfo *hw_layers_info) {
  DTRACE_SCOPED();
  // Store retire fence to track commit start.
  CacheRetireFence();
  if (secure_event_ == kSecureDisplayEnd || secure_event_ == kTUITransitionEnd ||
      secure_event_ == kTUITransitionUnPrepare) {
    secure_event_ = kSecureEventMax;
  }

  int level = 0;
  if (first_cycle_ && (hw_intf_->GetPanelBrightness(&level) == kErrorNone)) {
    comp_manager_->SetBacklightLevel(display_comp_ctx_, level);
  }

  PostCommitLayerParams();

  rc_prepared_ = false;
  avoid_qync_mode_change_ = false;

  if (partial_update_control_) {
    comp_manager_->ControlPartialUpdate(display_comp_ctx_, true /* enable */);
  }

  DisplayError error = comp_manager_->PostCommit(display_comp_ctx_, &disp_layer_stack_);
  if (error != kErrorNone) {
    return error;
  }

  // Stop dropping vsync when first commit is received after idle fallback.
  drop_hw_vsync_ = false;

  // Reset pending power state if any after the commit.
  error = ResetPendingPowerState(retire_fence_);
  if (error != kErrorNone) {
    return error;
  }

  // Handle pending vsync enable if any after the commit
  error = HandlePendingVSyncEnable(retire_fence_);
  if (error != kErrorNone) {
    return error;
  }

  comp_manager_->SetSafeMode(false);

  CacheFrameBuffer();

  for (auto &hw_layer : disp_layer_stack_.info.hw_layers) {
    CloseFd(&hw_layer.input_buffer.planes[0].fd);
  }

  first_cycle_ = false;

  return error;
}

void DisplayBase::CacheFrameBuffer() {
  if (draw_method_ != kDrawUnifiedWithGPUTarget) {
    return;
  }

  if (!gpu_comp_frame_) {
    return;
  }

  // Close current fd.
  CloseFd(&cached_framebuffer_.planes[0].fd);
  for (auto &hw_layer : disp_layer_stack_.info.hw_layers) {
    if (hw_layer.composition == kCompositionGPUTarget) {
      cached_framebuffer_ = hw_layer.input_buffer;
      break;
    }
  }

  // Replace buffer fd with duped fd.
  int new_fd = Sys::dup_(cached_framebuffer_.planes[0].fd);
  cached_framebuffer_.planes[0].fd = new_fd;
}

void DisplayBase::CacheDisplayComposition() {
  // Bail out if GPU composed layers aren't present.
  gpu_comp_frame_ = false;
  for (auto &layer : disp_layer_stack_.stack->layers) {
    if (layer->composition == kCompositionGPU) {
      gpu_comp_frame_ = true;
      break;
    }
  }
}

DisplayError DisplayBase::Flush(LayerStack *layer_stack) {
  ClientLock lock(disp_mutex_);

  return FlushLocked(layer_stack);
}

DisplayError DisplayBase::FlushLocked(LayerStack *layer_stack) {
  DisplayError error = kErrorNone;

  validated_ = false;
  if (!active_) {
    return kErrorPermission;
  }
  disp_layer_stack_.info.hw_layers.clear();
  disp_layer_stack_.stack = layer_stack;
  error = hw_intf_->Flush(&disp_layer_stack_.info);
  if (error == kErrorNone) {
    comp_manager_->Purge(display_comp_ctx_);
    validated_ = false;
    needs_validate_ = true;
  } else {
    DLOGW("Unable to flush display %d-%d", display_id_, display_type_);
  }
  if (layer_stack) {
    layer_stack->retire_fence = disp_layer_stack_.info.retire_fence;
  }

  return error;
}

DisplayError DisplayBase::GetDisplayState(DisplayState *state) {
  ClientLock lock(disp_mutex_);
  if (!state) {
    return kErrorParameters;
  }

  *state = state_;
  return kErrorNone;
}

DisplayError DisplayBase::GetNumVariableInfoConfigs(uint32_t *count) {
  ClientLock lock(disp_mutex_);
  return hw_intf_->GetNumDisplayAttributes(count);
}

DisplayError DisplayBase::GetConfig(uint32_t index, DisplayConfigVariableInfo *variable_info) {
  ClientLock lock(disp_mutex_);
  HWDisplayAttributes attrib;
  if (hw_intf_->GetDisplayAttributes(index, &attrib) == kErrorNone) {
    *variable_info = attrib;
    if (custom_mixer_resolution_) {
      variable_info->x_pixels = fb_config_.x_pixels;
      variable_info->y_pixels = fb_config_.y_pixels;
    }
    return kErrorNone;
  }

  return kErrorNotSupported;
}

DisplayError DisplayBase::GetConfig(DisplayConfigFixedInfo *fixed_info) {
  ClientLock lock(disp_mutex_);
  fixed_info->is_cmdmode = (hw_panel_info_.mode == kModeCommand);

  HWResourceInfo hw_resource_info = HWResourceInfo();
  hw_info_intf_->GetHWResourceInfo(&hw_resource_info);
  bool hdr_supported = hw_resource_info.has_hdr;
  bool hdr_plus_supported = false;
  HWDisplayInterfaceInfo hw_disp_info = {};
  hw_info_intf_->GetFirstDisplayInterfaceType(&hw_disp_info);
  if (hw_disp_info.type == kHDMI) {
    hdr_supported = (hdr_supported && hw_panel_info_.hdr_enabled);
  }

  // Checking library support for HDR10+
  comp_manager_->GetHDR10PlusCapability(&hdr_plus_supported);

  fixed_info->hdr_supported = hdr_supported;
  // For non-builtin displays, check panel capability for HDR10+
  fixed_info->hdr_plus_supported =
      hdr_supported && hw_panel_info_.hdr_plus_enabled && hdr_plus_supported;
  // Populate luminance values only if hdr will be supported on that display
  fixed_info->max_luminance = fixed_info->hdr_supported ? hw_panel_info_.peak_luminance: 0;
  fixed_info->average_luminance = fixed_info->hdr_supported ? hw_panel_info_.average_luminance : 0;
  fixed_info->min_luminance = fixed_info->hdr_supported ?  hw_panel_info_.blackness_level: 0;
  fixed_info->hdr_eotf = hw_panel_info_.hdr_eotf;
  fixed_info->hdr_metadata_type_one = hw_panel_info_.hdr_metadata_type_one;
  fixed_info->partial_update = hw_panel_info_.partial_update;
  fixed_info->readback_supported = hw_resource_info.has_concurrent_writeback;
  fixed_info->supports_unified_draw = unified_draw_supported_;

  return kErrorNone;
}

DisplayError DisplayBase::GetRealConfig(uint32_t index, DisplayConfigVariableInfo *variable_info) {
  ClientLock lock(disp_mutex_);
  HWDisplayAttributes attrib;
  if (hw_intf_->GetDisplayAttributes(index, &attrib) == kErrorNone) {
    *variable_info = attrib;
    return kErrorNone;
  }

  return kErrorNotSupported;
}

DisplayError DisplayBase::GetActiveConfig(uint32_t *index) {
  ClientLock lock(disp_mutex_);
  return hw_intf_->GetActiveConfig(index);
}

DisplayError DisplayBase::GetVSyncState(bool *enabled) {
  ClientLock lock(disp_mutex_);
  if (!enabled) {
    return kErrorParameters;
  }

  *enabled = vsync_enable_;

  return kErrorNone;
}

DisplayError DisplayBase::SetDrawMethod(DisplayDrawMethod draw_method) {
  DTRACE_SCOPED();
  ClientLock lock(disp_mutex_);
  if (draw_method_set_ || !first_cycle_) {
    DLOGW("Draw method set = %d or commits already started: %d", draw_method_, !first_cycle_);
    return kErrorNotSupported;
  }

  auto error = comp_manager_->SetDrawMethod(display_comp_ctx_, draw_method);
  if (error != kErrorNone) {
    DLOGE("Failed to set method: %d for %d-%d", draw_method, display_id_, display_type_);
    retire_fence_offset_ = 0;
    draw_method_ = kDrawDefault;
    draw_method_set_ = true;
    return error;
  }

  retire_fence_offset_ = (draw_method != kDrawDefault) && (display_type_ != kVirtual) ? 1 : 0;
  draw_method_ = draw_method;
  draw_method_set_ = true;
  DLOGI("method: %d", draw_method);

  return kErrorNone;
}

DisplayError DisplayBase::SetDisplayState(DisplayState state, bool teardown,
                                          shared_ptr<Fence> *release_fence) {
  ClientLock lock(disp_mutex_);
  DisplayError error = kErrorNone;
  bool active = false;

  DLOGI("Set state = %d, display %d-%d, teardown = %d", state, display_id_,
        display_type_, teardown);

  if (state == state_) {
    if (pending_power_state_ != kPowerStateNone) {
      hw_intf_->CancelDeferredPowerMode();
      pending_power_state_ = kPowerStateNone;
    }
    DLOGI("Same state transition is requested.");
    return kErrorNone;
  }

  validated_ = false;
  // If vsync is enabled, disable vsync before power off/Doze suspend
  if (vsync_enable_ && (state == kStateOff || state == kStateDozeSuspend)) {
    error = SetVSyncState(false /* enable */);
    if (error == kErrorNone) {
      vsync_enable_pending_ = true;
    }
  }

  SyncPoints sync_points = {};

  switch (state) {
  case kStateOff:
    disp_layer_stack_.info.hw_layers.clear();
    error = hw_intf_->PowerOff(teardown, &sync_points);
    if (error != kErrorNone) {
      if (error == kErrorDeferred) {
        pending_power_state_ = kPowerStateOff;
        error = kErrorNone;
      } else {
        return error;
      }
    } else {
      pending_power_state_ = kPowerStateNone;
    }
    cached_qos_data_ = {};
    cached_qos_data_.clock_hz = default_clock_hz_;
    break;

  case kStateOn:
    if (display_type_ == kHDMI && first_cycle_) {
      hw_events_intf_->SetEventState(HWEvent::POWER_EVENT, true);
    }

    error = hw_intf_->PowerOn(cached_qos_data_, &sync_points);
    if (error != kErrorNone) {
      if (error == kErrorDeferred) {
        pending_power_state_ = kPowerStateOn;
        error = kErrorNone;
      } else {
        return error;
      }
    } else {
      pending_power_state_ = kPowerStateNone;
    }

    error = comp_manager_->ReconfigureDisplay(display_comp_ctx_, display_attributes_,
                                              hw_panel_info_, mixer_attributes_, fb_config_,
                                              &cached_qos_data_);
    if (error != kErrorNone) {
      return error;
    }
    default_clock_hz_ = cached_qos_data_.clock_hz;

    active = true;
    break;

  case kStateDoze:
    error = hw_intf_->Doze(cached_qos_data_, &sync_points);
    if (error != kErrorNone) {
      if (error == kErrorDeferred) {
        pending_power_state_ = kPowerStateDoze;
        error = kErrorNone;
      } else {
        return error;
      }
    } else {
      pending_power_state_ = kPowerStateNone;
    }
    active = true;
    break;

  case kStateDozeSuspend:
    error = hw_intf_->DozeSuspend(cached_qos_data_, &sync_points);
    if (error != kErrorNone) {
      if (error == kErrorDeferred) {
        pending_power_state_ = kPowerStateDozeSuspend;
        error = kErrorNone;
      } else {
        return error;
      }
    } else {
      pending_power_state_ = kPowerStateNone;
    }

    if (display_type_ != kBuiltIn) {
      active = true;
    }
    break;

  case kStateStandby:
    error = hw_intf_->Standby(&sync_points);
    break;

  default:
    DLOGE("Spurious state = %d transition requested.", state);
    return kErrorParameters;
  }

  if ((pending_power_state_ == kPowerStateNone) && (!first_cycle_ || display_type_ == kHDMI)) {
    SyncPoints sync = {};
    if (draw_method_ == kDrawDefault || display_type_ == kVirtual) {
      // Wait on current retire fence.
      sync.retire_fence = sync_points.retire_fence;
    } else {
      // For displays in unified draw, wait on cached retire fence in steady state.
      comp_manager_->GetRetireFence(display_comp_ctx_, &retire_fence_);
      sync.retire_fence = retire_fence_;
    }
    WaitForCompletion(&sync);
  }

  error = ReconfigureDisplay();
  if (error != kErrorNone) {
    return error;
  }

  DisablePartialUpdateOneFrameInternal();

  if (error == kErrorNone) {
    if (pending_power_state_ == kPowerStateNone) {
      active_ = active;
      state_ = state;
      if (IsPrimaryDisplayLocked()) {
        primary_active_ = active;
      }
      // Handle vsync pending on resume, Since the power on commit is synchronous we pass -1 as
      // retire fence otherwise pass valid retire fence
      if (state == kStateOn) {
        HandlePendingVSyncEnable(nullptr /* retire fence */);
      }
    }
    comp_manager_->SetDisplayState(display_comp_ctx_, state, sync_points);
  }
  DLOGI("active %d-%d state %d-%d pending_power_state_ %d", active, active_, state, state_,
        pending_power_state_);

  if (release_fence) {
    *release_fence = sync_points.release_fence;
  }

  return error;
}

DisplayError DisplayBase::SetActiveConfig(uint32_t index) {
  ClientLock lock(disp_mutex_);
  DisplayError error = kErrorNone;
  uint32_t active_index = 0;

  validated_ = false;
  hw_intf_->GetActiveConfig(&active_index);

  // Cache the last refresh rate set by SF
  HWDisplayAttributes display_attributes = {};
  hw_intf_->GetDisplayAttributes(index, &display_attributes);

  if (active_index == index) {
    active_refresh_rate_ = display_attributes.fps;
    return kErrorNone;
  }

  error = hw_intf_->SetDisplayAttributes(index);
  if (error != kErrorNone) {
    return error;
  }

  avoid_qync_mode_change_ = true;
  active_refresh_rate_ = display_attributes.fps;

  return ReconfigureDisplay();
}

DisplayError DisplayBase::SetMaxMixerStages(uint32_t max_mixer_stages) {
  ClientLock lock(disp_mutex_);
  DisplayError error = kErrorNone;
  validated_ = false;

  error = comp_manager_->SetMaxMixerStages(display_comp_ctx_, max_mixer_stages);

  if (error == kErrorNone) {
    max_mixer_stages_ = max_mixer_stages;
  }

  return error;
}

void DisplayBase::AppendRCMaskData(std::ostringstream &os) {
  uint32_t num_mask_layers = disp_layer_stack_.info.rc_layers_info.mask_layer_idx.size();
  uint32_t num_rc_hw_layers = disp_layer_stack_.info.rc_layers_info.rc_hw_layer_idx.size();
  if (num_mask_layers && rc_enable_prop_) {
    os << "\nRC HW Mask Layer Idx: [";
    for (uint32_t i = 0; i < num_rc_hw_layers; i++) {
      os << disp_layer_stack_.info.rc_layers_info.rc_hw_layer_idx.at(i);
      if (i < (num_rc_hw_layers - 1)) {
        os << ", ";
      }
    }
    os << "] of [";
    for (uint32_t i = 0; i < num_mask_layers; i++) {
      os << disp_layer_stack_.info.rc_layers_info.mask_layer_idx.at(i);
      if (i < (num_mask_layers - 1)) {
        os << ", ";
      }
    }
    os << "]";
  }
}

std::string DisplayBase::Dump() {
  ClientLock lock(disp_mutex_);
  HWDisplayAttributes attrib;
  uint32_t active_index = 0;
  uint32_t num_modes = 0;
  std::ostringstream os;

  hw_intf_->GetNumDisplayAttributes(&num_modes);
  hw_intf_->GetActiveConfig(&active_index);
  hw_intf_->GetDisplayAttributes(active_index, &attrib);

  os << "device type:" << display_type_;
  os << " DrawMethod: " << draw_method_;
  os << "\nstate: " << state_ << " vsync on: " << vsync_enable_
     << " max. mixer stages: " << max_mixer_stages_;
  if (disp_layer_stack_.info.noise_layer_info.enable) {
    os << "\nNoise z-orders: [" << disp_layer_stack_.info.noise_layer_info.zpos_noise << "," <<
        disp_layer_stack_.info.noise_layer_info.zpos_attn << "]";
  }
  os << "\nnum configs: " << num_modes << " active config index: " << active_index;
  os << "\nDisplay Attributes:";
  os << "\n Mode:" << (hw_panel_info_.mode == kModeVideo ? "Video" : "Command");
  os << std::boolalpha;
  os << " Primary:" << hw_panel_info_.is_primary_panel;
  os << " DynFPS:" << hw_panel_info_.dynamic_fps;
  os << "\n HDR Panel:" << hw_panel_info_.hdr_enabled;
  os << " QSync:" << hw_panel_info_.qsync_support;
  os << " DynBitclk:" << hw_panel_info_.dyn_bitclk_support;
  os << "\n Left Split:" << hw_panel_info_.split_info.left_split
     << " Right Split:" << hw_panel_info_.split_info.right_split;
  os << "\n PartialUpdate:" << hw_panel_info_.partial_update;
  if (hw_panel_info_.partial_update) {
    os << "\n ROI Min w:" << hw_panel_info_.min_roi_width;
    os << " Min h:" << hw_panel_info_.min_roi_height;
    os << " NeedsMerge: " << hw_panel_info_.needs_roi_merge;
    os << " Alignment: l:" << hw_panel_info_.left_align << " w:" << hw_panel_info_.width_align;
    os << " t:" << hw_panel_info_.top_align << " b:" << hw_panel_info_.height_align;
  }
  os << "\n FPS min:" << hw_panel_info_.min_fps << " max:" << hw_panel_info_.max_fps
     << " cur:" << display_attributes_.fps;
  os << " TransferTime: " << hw_panel_info_.transfer_time_us << "us";
  os << " MaxBrightness:" << hw_panel_info_.panel_max_brightness;
  os << "\n Display WxH: " << display_attributes_.x_pixels << "x" << display_attributes_.y_pixels;
  os << " MixerWxH: " << mixer_attributes_.width << "x" << mixer_attributes_.height;
  os << " DPI: " << display_attributes_.x_dpi << "x" << display_attributes_.y_dpi;
  os << " LM_Split: " << display_attributes_.is_device_split;
  os << "\n vsync_period " << display_attributes_.vsync_period_ns;
  os << " v_back_porch: " << display_attributes_.v_back_porch;
  os << " v_front_porch: " << display_attributes_.v_front_porch;
  os << " v_pulse_width: " << display_attributes_.v_pulse_width;
  os << "\n v_total: " << display_attributes_.v_total;
  os << " h_total: " << display_attributes_.h_total;
  os << " clk: " << display_attributes_.clock_khz;
  os << " Topology: " << display_attributes_.topology;
  os << std::noboolalpha;

  os << "\nCurrent Color Mode: " << current_color_mode_.c_str();
  os << "\nAvailable Color Modes:\n";
  for (auto it : color_mode_map_) {
    os << "  " << it.first << " " << std::setw(35 - INT(it.first.length())) <<
       it.second->id;
    os << " ";
    for (auto attr_it : color_mode_attr_map_[it.first]) {
      os << std::right << " " << attr_it.first << ": " << attr_it.second;
    }
    os << "\n";
  }

  uint32_t num_hw_layers = UINT32(disp_layer_stack_.info.hw_layers.size());

  if (num_hw_layers == 0) {
    os << "\nNo hardware layers programmed";
    return os.str();
  }

  LayerBuffer *out_buffer = disp_layer_stack_.info.output_buffer;
  if (out_buffer) {
    os << "\n Output buffer res: " << out_buffer->width << "x" << out_buffer->height
       << " format: " << GetFormatString(out_buffer->format);
  }
  HWLayersInfo &layer_info = disp_layer_stack_.info;
  for (uint32_t i = 0; i < layer_info.left_frame_roi.size(); i++) {
    LayerRect &l_roi = layer_info.left_frame_roi.at(i);
    LayerRect &r_roi = layer_info.right_frame_roi.at(i);

    os << "\nROI(LTRB)#" << i << " LEFT(" << INT(l_roi.left) << " " << INT(l_roi.top) << " " <<
      INT(l_roi.right) << " " << INT(l_roi.bottom) << ")";
    if (IsValid(r_roi)) {
    os << " RIGHT(" << INT(r_roi.left) << " " << INT(r_roi.top) << " " << INT(r_roi.right) << " "
      << INT(r_roi.bottom) << ")";
    }
  }

  LayerRect &fb_roi = layer_info.partial_fb_roi;
  if (IsValid(fb_roi)) {
    os << "\nPartial FB ROI(LTRB):(" << INT(fb_roi.left) << " " << INT(fb_roi.top) << " " <<
      INT(fb_roi.right) << " " << INT(fb_roi.bottom) << ")";
  }

  AppendRCMaskData(os);

  const char *header  = "\n| Idx |   Comp Type   |   Split   | Pipe |    W x H    |          Format          |  Src Rect (L T R B) |  Dst Rect (L T R B) |  Z | Pipe Flags | Deci(HxV) | CS | Rng | Tr |";  //NOLINT
  const char *newline = "\n|-----|---------------|-----------|------|-------------|--------------------------|---------------------|---------------------|----|------------|-----------|----|-----|----|";  //NOLINT
  const char *format  = "\n| %3s | %13s | %9s | %4d | %4d x %4d | %24s | %4d %4d %4d %4d | %4d %4d %4d %4d | %2s | %10s | %9s | %2s | %3s | %2s |";  //NOLINT

  os << "\n";
  os << newline;
  os << header;
  os << newline;

  for (uint32_t i = 0; i < num_hw_layers; i++) {
    uint32_t layer_index = disp_layer_stack_.info.index.at(i);
    // hw-layer from hw layers info
    Layer &hw_layer = disp_layer_stack_.info.hw_layers.at(i);
    LayerBuffer *input_buffer = &hw_layer.input_buffer;
    HWLayerConfig &layer_config = disp_layer_stack_.info.config[i];
    HWRotatorSession &hw_rotator_session = layer_config.hw_rotator_session;

    const char *comp_type = GetName(hw_layer.composition);
    const char *buffer_format = GetFormatString(input_buffer->format);
    const char *pipe_split[2] = { "Pipe-1", "Pipe-2" };
    const char *rot_pipe[2] = { "Rot-inl-1", "Rot-inl-2" };
    char idx[8];

    snprintf(idx, sizeof(idx), "%d", layer_index);

    for (uint32_t count = 0; count < hw_rotator_session.hw_block_count; count++) {
      char row[1024];
      HWRotateInfo &rotate = hw_rotator_session.hw_rotate_info[count];
      LayerRect &src_roi = rotate.src_roi;
      LayerRect &dst_roi = rotate.dst_roi;
      char rot[12] = { 0 };

      snprintf(rot, sizeof(rot), "Rot-%s-%d", layer_config.use_inline_rot ?
               "inl" : "off", count + 1);

      snprintf(row, sizeof(row), format, idx, comp_type, rot,
               0, input_buffer->width, input_buffer->height, buffer_format,
               INT(src_roi.left), INT(src_roi.top), INT(src_roi.right), INT(src_roi.bottom),
               INT(dst_roi.left), INT(dst_roi.top), INT(dst_roi.right), INT(dst_roi.bottom),
               "-", "-    ", "-    ", "-", "-", "-");
      os << row;
      // print the below only once per layer block, fill with spaces for rest.
      idx[0] = 0;
      comp_type = "";
    }

    if (hw_rotator_session.hw_block_count > 0) {
      input_buffer = &hw_rotator_session.output_buffer;
      buffer_format = GetFormatString(input_buffer->format);
    }

    if (layer_config.use_solidfill_stage) {
      LayerRect src_roi = layer_config.hw_solidfill_stage.roi;
      const char *decimation = "";
      char flags[16] = { 0 };
      char z_order[8] = { 0 };
      const char *color_primary = "";
      const char *range = "";
      const char *transfer = "";
      char row[1024] = { 0 };

      snprintf(z_order, sizeof(z_order), "%d", layer_config.hw_solidfill_stage.z_order);
      snprintf(flags, sizeof(flags), "0x%08x", hw_layer.flags.flags);
      snprintf(row, sizeof(row), format, idx, comp_type, pipe_split[0],
               0, INT(src_roi.right), INT(src_roi.bottom),
               buffer_format, INT(src_roi.left), INT(src_roi.top),
               INT(src_roi.right), INT(src_roi.bottom), INT(src_roi.left),
               INT(src_roi.top), INT(src_roi.right), INT(src_roi.bottom),
               z_order, flags, decimation, color_primary, range, transfer);
      os << row;
      continue;
    }

    for (uint32_t count = 0; count < 2; count++) {
      char decimation[16] = { 0 };
      char flags[16] = { 0 };
      char z_order[8] = { 0 };
      char color_primary[8] = { 0 };
      char range[8] = { 0 };
      char transfer[8] = { 0 };
      bool rot = layer_config.use_inline_rot;

      HWPipeInfo &pipe = (count == 0) ? layer_config.left_pipe : layer_config.right_pipe;

      if (!pipe.valid) {
        continue;
      }

      LayerRect src_roi = pipe.src_roi;
      LayerRect &dst_roi = pipe.dst_roi;

      snprintf(z_order, sizeof(z_order), "%d", pipe.z_order);
      snprintf(flags, sizeof(flags), "0x%08x", pipe.flags);
      snprintf(decimation, sizeof(decimation), "%3d x %3d", pipe.horizontal_decimation,
               pipe.vertical_decimation);
      ColorMetaData &color_metadata = hw_layer.input_buffer.color_metadata;
      snprintf(color_primary, sizeof(color_primary), "%d", color_metadata.colorPrimaries);
      snprintf(range, sizeof(range), "%d", color_metadata.range);
      snprintf(transfer, sizeof(transfer), "%d", color_metadata.transfer);

      char row[1024];
      snprintf(row, sizeof(row), format, idx, comp_type, rot ? rot_pipe[count] : pipe_split[count],
               pipe.pipe_id, input_buffer->width, input_buffer->height,
               buffer_format, INT(src_roi.left), INT(src_roi.top),
               INT(src_roi.right), INT(src_roi.bottom), INT(dst_roi.left),
               INT(dst_roi.top), INT(dst_roi.right), INT(dst_roi.bottom),
               z_order, flags, decimation, color_primary, range, transfer);

      os << row;
      // print the below only once per layer block, fill with spaces for rest.
      idx[0] = 0;
      comp_type = "";
    }
  }

  os << newline << "\n";

  return os.str();
}

const char * DisplayBase::GetName(const LayerComposition &composition) {
  switch (composition) {
  case kCompositionGPU:           return "GPU";
  case kCompositionSDE:           return "SDE";
  case kCompositionCursor:        return "CURSOR";
  case kCompositionStitch:        return "STITCH";
  case kCompositionGPUTarget:     return "GPU_TARGET";
  case kCompositionStitchTarget:  return "STITCH_TARGET";
  case kCompositionDemura:        return "DEMURA";
  case kCompositionCWBTarget:     return "CWB_TARGET";
  default:                        return "UNKNOWN";
  }
}

DisplayError DisplayBase::ColorSVCRequestRoute(const PPDisplayAPIPayload &in_payload,
                                               PPDisplayAPIPayload *out_payload,
                                               PPPendingParams *pending_action) {
  ClientLock lock(disp_mutex_);
  if (color_mgr_)
    return color_mgr_->ColorSVCRequestRoute(in_payload, out_payload, pending_action);
  else
    return kErrorParameters;
}

DisplayError DisplayBase::GetColorModeCount(uint32_t *mode_count) {
  ClientLock lock(disp_mutex_);
  if (!mode_count) {
    return kErrorParameters;
  }

  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  DLOGV_IF(kTagQDCM, "Display = %d Number of modes from color manager = %d", display_type_,
           num_color_modes_);

  *mode_count = num_color_modes_;

  return kErrorNone;
}

DisplayError DisplayBase::GetColorModes(uint32_t *mode_count,
                                        std::vector<std::string> *color_modes) {
  ClientLock lock(disp_mutex_);
  if (!mode_count || !color_modes) {
    return kErrorParameters;
  }

  if (!color_mgr_) {
    return kErrorNotSupported;
  }
  uint32_t i = 0;
  for (ColorModeAttrMap::iterator it = color_mode_attr_map_.begin();
       ((i < num_color_modes_) && (it != color_mode_attr_map_.end())); i++, it++) {
    DLOGI("ColorMode name = %s", it->first.c_str());
    color_modes->at(i) = it->first.c_str();
  }

  return kErrorNone;
}

DisplayError DisplayBase::GetColorModeAttr(const std::string &color_mode, AttrVal *attr) {
  ClientLock lock(disp_mutex_);
  if (!attr) {
    return kErrorParameters;
  }

  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  auto it = color_mode_attr_map_.find(color_mode);
  if (it == color_mode_attr_map_.end()) {
    DLOGI("Mode %s has no attribute", color_mode.c_str());
    return kErrorNotSupported;
  }
  *attr = it->second;

  return kErrorNone;
}

DisplayError DisplayBase::SetColorMode(const std::string &color_mode) {
  ClientLock lock(disp_mutex_);
  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  if (color_mode.empty()) {
    return kErrorParameters;
  }

  DisplayError error = kErrorNone;
  std::string dynamic_range = kSdr, str_render_intent;
  if (IsSupportColorModeAttribute(color_mode)) {
    auto it_mode = color_mode_attr_map_.find(color_mode);
    GetValueOfModeAttribute(it_mode->second, kDynamicRangeAttribute, &dynamic_range);
    GetValueOfModeAttribute(it_mode->second, kRenderIntentAttribute, &str_render_intent);
  }

  current_color_mode_ = color_mode;
  PrimariesTransfer blend_space = {};
  blend_space = GetBlendSpaceFromColorMode();
  error = comp_manager_->SetBlendSpace(display_comp_ctx_, blend_space);
  if (error != kErrorNone) {
    DLOGE("SetBlendSpace failed, error = %d display_type_ = %d", error, display_type_);
  }

  error = hw_intf_->SetBlendSpace(blend_space);
  if (error != kErrorNone) {
    DLOGE("Failed to pass blend space, error = %d display_type_ = %d", error, display_type_);
  }

  error = SetColorModeInternal(color_mode, str_render_intent,  blend_space);
  if (error != kErrorNone) {
    return error;
  }

  comp_manager_->ControlDpps(dynamic_range != kHdr);

  return error;
}

DisplayError DisplayBase::SetColorModeById(int32_t color_mode_id) {
  validated_ = false;
  for (auto it : color_mode_map_) {
    if (it.second->id == color_mode_id) {
      return SetColorMode(it.first);
    }
  }

  return kErrorNotSupported;
}

DisplayError DisplayBase::SetColorModeInternal(const std::string &color_mode,
                                               const std::string &str_render_intent,
                                               const PrimariesTransfer &pt) {
  DLOGV_IF(kTagQDCM, "Color Mode = %s", color_mode.c_str());

  ColorModeMap::iterator it = color_mode_map_.find(color_mode);
  if (it == color_mode_map_.end()) {
    DLOGE("Failed: Unknown Mode : %s", color_mode.c_str());
    return kErrorNotSupported;
  }

  SDEDisplayMode *sde_display_mode = it->second;

  DLOGV_IF(kTagQDCM, "Color Mode Name = %s corresponding mode_id = %d", sde_display_mode->name,
           sde_display_mode->id);
  DisplayError error = kErrorNone;
  int32_t render_intent = 0;
  if (!str_render_intent.empty()) {
    render_intent = std::stoi(str_render_intent);
  }

  if (render_intent < 0 || render_intent > MAX_EXTENDED_RENDER_INTENT) {
    DLOGW("Invalid render intent %d for mode id = %d", render_intent, sde_display_mode->id);
    return kErrorNotSupported;
  }

  error = color_mgr_->ColorMgrSetMode(sde_display_mode->id);
  if (error != kErrorNone) {
    DLOGE("Failed for mode id = %d", sde_display_mode->id);
    return error;
  }

  error = color_mgr_->ColorMgrSetModeWithRenderIntent(sde_display_mode->id, pt, render_intent);
  if (error != kErrorNone) {
    DLOGE("Failed for mode id = %d", sde_display_mode->id);
    return error;
  }

  return error;
}

DisplayError DisplayBase::GetColorModeName(int32_t mode_id, std::string *mode_name) {
  if (!mode_name) {
    DLOGE("Invalid parameters");
    return kErrorParameters;
  }
  for (uint32_t i = 0; i < num_color_modes_; i++) {
    if (color_modes_[i].id == mode_id) {
      *mode_name = color_modes_[i].name;
      return kErrorNone;
    }
  }

  DLOGE("Failed to get color mode name for mode id = %d", mode_id);
  return kErrorUndefined;
}

DisplayError DisplayBase::GetValueOfModeAttribute(const AttrVal &attr, const std::string &type,
                                                  std::string *value) {
  if (!value) {
    return kErrorParameters;
  }
  for (auto &it : attr) {
    if (it.first.find(type) != std::string::npos) {
      *value = it.second;
    }
  }

  return kErrorNone;
}

bool DisplayBase::IsSupportColorModeAttribute(const std::string &color_mode) {
  auto it = color_mode_attr_map_.find(color_mode);
  if (it == color_mode_attr_map_.end()) {
    return false;
  }
  return true;
}

DisplayError DisplayBase::SetColorTransform(const uint32_t length, const double *color_transform) {
  ClientLock lock(disp_mutex_);
  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  if (!color_transform) {
    return kErrorParameters;
  }

  DisplayError error = color_mgr_->ColorMgrSetColorTransform(length, color_transform);
  if (error) {
    return error;
  }
  validated_ = false;
  DisablePartialUpdateOneFrameInternal();
  return kErrorNone;
}

DisplayError DisplayBase::GetDefaultColorMode(std::string *color_mode) {
  ClientLock lock(disp_mutex_);
  if (!color_mode) {
    return kErrorParameters;
  }

  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  int32_t default_id = kInvalidModeId;
  DisplayError error = color_mgr_->ColorMgrGetDefaultModeID(&default_id);
  if (error != kErrorNone) {
    DLOGE("Failed for get default color mode id");
    return error;
  }

  for (uint32_t i = 0; i < num_color_modes_; i++) {
    if (color_modes_[i].id == default_id) {
      *color_mode = color_modes_[i].name;
      return kErrorNone;
    }
  }

  return kErrorNotSupported;
}

DisplayError DisplayBase::ApplyDefaultDisplayMode() {
  ClientLock lock(disp_mutex_);
  DisplayError error = kErrorNone;
  if (color_mgr_) {
    error = color_mgr_->ApplyDefaultDisplayMode();
    // Apply default mode failed
    if (error != kErrorNone) {
      DLOGI("default mode not found");
      return error;
    }
    DeInitializeColorModes();
    // Default mode apply is called during first frame, if file system
    // where mode files is present, ColorManager will not find any modes.
    // Once boot animation is complete we re-try to apply the modes, since
    // file system should be mounted. InitColorModes needs to called again
    error = InitializeColorModes();
    if (error != kErrorNone) {
      DLOGE("failed to initial modes\n");
      return error;
    }
    if (color_modes_cs_.size() > 0) {
      error = comp_manager_->SetColorModesInfo(display_comp_ctx_, color_modes_cs_);
      if (error) {
        DLOGW("SetColorModesInfo failed on display = %d", display_type_);
      }
    }
  } else {
    return kErrorParameters;
  }
  return kErrorNone;
}

bool DisplayBase::IsValidCwbRoi(const LayerRect &roi, const LayerRect &full_frame) {
  bool is_valid = true;
  if (!IsValid(roi) || roi.left < 0 || roi.top < 0 || roi.right < 0 || roi.bottom < 0) {
    is_valid = false;
  } else if (roi.top < full_frame.top || roi.top >= full_frame.bottom ||
             roi.bottom <= full_frame.top || roi.bottom > full_frame.bottom) {
    is_valid = false;
  } else if (roi.left < full_frame.left || roi.left >= full_frame.right ||
             roi.right <= full_frame.left || roi.right > full_frame.right) {
    is_valid = false;
  }
  return is_valid;
}

DisplayError DisplayBase::ValidateCwbConfigInfo(CwbConfig *cwb_config,
                                                const LayerBufferFormat &format) {
  CwbTapPoint &tap_point = cwb_config->tap_point;
  if (tap_point < CwbTapPoint::kLmTapPoint || tap_point > CwbTapPoint::kDemuraTapPoint) {
    DLOGE("Invalid CWB tappoint. %d ", tap_point);
    return kErrorParameters;
  } else if (tap_point == CwbTapPoint::kDemuraTapPoint) {
    // Check whether demura tap-point is supported for CWB.
    uint32_t demura_tappoint_supported = 0;
    IsSupportedOnDisplay(kCwbDemuraTapPoint, &demura_tappoint_supported);
    if (!demura_tappoint_supported) {
      DLOGW("Demura tap-point is not supported for CWB. Falling back to DSPP tap-point.");
      tap_point = CwbTapPoint::kDsppTapPoint;
    }
  }

  LayerRect &roi = cwb_config->cwb_roi;
  LayerRect &full_frame = cwb_config->cwb_full_rect;
  uint32_t cwb_roi_supported = 0;  // Check whether CWB ROI is supported.
  IsSupportedOnDisplay(kCwbCrop, &cwb_roi_supported);
  if (!cwb_roi_supported) {
    roi = full_frame;
    return kErrorNone;  // below checks are not needed if CWB ROI isn't supported.
  }

  if (!IsRgbFormat(format)) {  // CWB ROI is supported only on RGB color formats. Thus, in-case of
    // other color formats, fallback to Full frame ROI.
    DLOGW("CWB ROI is not suopported on color format : %s , thus falling back to Full frame ROI.",
          GetFormatString(format));
    roi = full_frame;
  }

  bool &pu_as_cwb_roi = cwb_config->pu_as_cwb_roi;
  bool is_valid_cwb_roi = IsValidCwbRoi(roi, full_frame);
  if (is_valid_cwb_roi && !pu_as_cwb_roi) {
    // If client passed valid ROI and PU ROI not to be included in CWB ROI, then
    // make Client ROI's (width * height) as 256B aligned.
    int cwb_alignment_factor = GetCwbAlignmentFactor(format);
    if (!cwb_alignment_factor) {
      DLOGE("Output buffer has invalid color format.");
      return kErrorParameters;
    }
    ApplyCwbRoiRestrictions(roi, full_frame, cwb_alignment_factor, format);
  }

  // For cmd mode : Incase CWB Client sets cwb_config.pu_as_cwb_roi as true, then PU ROI would be
  // included in CWB ROI. Incase client passes invalid ROI, only PU ROI generated from dirty rects
  // of app layers would be taken as CWB ROI. Incase client passes valid ROI, then union of PU ROI
  // and client's ROI would be taken as CWB ROI.
  // For video mode : PU ROI would be full frame rect. If either pu_as_cwb_roi is true or client
  // passes invalid ROI, then CWB ROI would also be set as full frame. Incase pu_as_cwb_roi is
  // False and client passes a valid ROI, then the client's ROI would be set as CWB ROI.
  if (pu_as_cwb_roi) {
    DLOGI_IF(kTagDisplay, "PU ROI would be included in CWB ROI.");
  } else if (!is_valid_cwb_roi) {
    DLOGI_IF(kTagDisplay, "Client provided invalid ROI. Going for Full frame CWB.");
    roi = full_frame;
  }

  DLOGI_IF(kTagDisplay, "Cwb_config: tap_point %d, CWB ROI Rect(%f %f %f %f), PU_as_CWB_ROI %d",
           tap_point, roi.left, roi.top, roi.right, roi.bottom, pu_as_cwb_roi);

  return kErrorNone;
}

DisplayError DisplayBase::SetCursorPosition(int x, int y) {
  ClientLock lock(disp_mutex_);
  if (state_ != kStateOn) {
    return kErrorNotSupported;
  }

  DisplayError error = comp_manager_->ValidateAndSetCursorPosition(display_comp_ctx_,
                                                                   &disp_layer_stack_, x, y);
  if (error == kErrorNone) {
    return hw_intf_->SetCursorPosition(&disp_layer_stack_.info, x, y);
  }

  return kErrorNone;
}

DisplayError DisplayBase::GetRefreshRateRange(uint32_t *min_refresh_rate,
                                              uint32_t *max_refresh_rate) {
  ClientLock lock(disp_mutex_);
  // The min and max refresh rates will be same when the HWPanelInfo does not contain valid rates.
  // Usually for secondary displays, command mode panels
  HWDisplayAttributes display_attributes;
  uint32_t active_index = 0;
  hw_intf_->GetActiveConfig(&active_index);
  DisplayError error = hw_intf_->GetDisplayAttributes(active_index, &display_attributes);
  if (error) {
    return error;
  }

  *min_refresh_rate = display_attributes.fps;
  *max_refresh_rate = display_attributes.fps;

  return error;
}

DisplayError DisplayBase::HandlePendingVSyncEnable(const shared_ptr<Fence> &retire_fence) {
  if (vsync_enable_pending_) {
    // Retire fence signalling confirms that CRTC enabled, hence wait for retire fence before
    // we enable vsync
    Fence::Wait(retire_fence_);

    DisplayError error = SetVSyncStateLocked(true /* enable */);
    if (error != kErrorNone) {
      return error;
    }
    vsync_enable_pending_ = false;
  }
  return kErrorNone;
}

DisplayError DisplayBase::SetVSyncState(bool enable) {
  ClientLock lock(disp_mutex_);

  return SetVSyncStateLocked(enable);
}

DisplayError DisplayBase::SetVSyncStateLocked(bool enable) {
  if ((state_ == kStateOff || secure_event_ != kSecureEventMax) && enable) {
    DLOGW("Can't enable vsync when display %d-%d is powered off or SecureDisplay/TUI in progress",
          display_id_, display_type_);
    vsync_enable_pending_ = true;
    return kErrorNone;
  }
  DisplayError error = kErrorNone;
  if (vsync_enable_ != enable) {
    error = hw_intf_->SetVSyncState(enable);
    if (error == kErrorNotSupported) {
      if (drop_skewed_vsync_ && (hw_panel_info_.mode == kModeVideo) &&
        enable && (current_refresh_rate_ < hw_panel_info_.max_fps)) {
        drop_hw_vsync_ = true;
      }
      error = hw_events_intf_->SetEventState(HWEvent::VSYNC, enable);
    }
    if (error == kErrorNone) {
      vsync_enable_ = enable;
    } else {
      vsync_enable_pending_ = true;
    }
  }
  vsync_enable_pending_ = !enable ? false : vsync_enable_pending_;

  return error;
}

DisplayError DisplayBase::SetNoisePlugInOverride(bool override_en, int32_t attn,
                                                 int32_t noise_zpos) {
  if (!noise_plugin_intf_) {
    DLOGW("Noise Layer Feature not enabled for Display %d-%d",  display_id_, display_type_);
    return kErrorNone;
  }
  ClientLock lock(disp_mutex_);
  GenericPayload payload;
  int32_t *val = nullptr;
  int32_t ret = payload.CreatePayload<int32_t>(val);
  if (ret) {
    DLOGE("failed to create the payload. ret = %d", ret);
    return kErrorUndefined;
  }

  *val = override_en;
  DLOGI("Display %d-%d override_en:%d", display_id_, display_type_, override_en);
  NoisePlugInParams param = {};
  if (ValidNoisePluginDebugOverride(kNoisePlugInDebugOverride)) {
    *val = override_en;
    param = static_cast<NoisePlugInParams>(kNoisePlugInDebugOverride);
    ret = noise_plugin_intf_->SetParameter(param, payload);
    if (ret) {
      DLOGW("Display %d-%d Failed to set Override = %d ret = %d", display_id_, display_type_,
            override_en, ret);
      return kErrorUndefined;
    }
  }
  noise_override_zpos_ = -1;
  if (override_en) {
    // over-ride enable case
    noise_plugin_override_en_ = true;
    if (ValidNoisePluginDebugOverride(kNoisePlugInDebugAttn)) {
      // override attenuation factor
      param = static_cast<sdm::NoisePlugInParams>(sdm::kNoisePlugInDebugAttn);
      *val = attn;
      DLOGI("Display %d-%d  attenuation = %d", display_id_, display_type_, attn);
      param = static_cast<NoisePlugInParams>(kNoisePlugInDebugAttn);
      ret = noise_plugin_intf_->SetParameter(param, payload);
      if (ret && (attn > 0)) {
        DLOGE("Display %d-%d Failed to set Attn = %d ret = %d", display_id_, display_type_, attn,
               ret);
      }
    }
    if (ValidNoisePluginDebugOverride(kNoisePlugInDebugNoiseZpos)) {
      // override noise layer z position
      param = static_cast<NoisePlugInParams>(kNoisePlugInDebugNoiseZpos);
      *val = noise_zpos;
      DLOGI("Display %d-%d noise_zpos = %d", display_id_, display_type_, noise_zpos);
      ret = noise_plugin_intf_->SetParameter(param, payload);
      if (ret && (noise_zpos > 0)) {
        DLOGE("Display %d-%d Failed to set Zpos = %d ret = %d", display_id_, display_type_,
              noise_zpos, ret);
      } else {
        noise_override_zpos_ = noise_zpos;
      }
    }
  }

  if (!override_en) {
    noise_plugin_override_en_ = false;
    noise_override_zpos_ = -1;
  }
  validated_ = false;

  return kErrorNone;
}

DisplayError DisplayBase::ReconfigureDisplay() {
  ClientLock lock(disp_mutex_);
  DisplayError error = kErrorNone;
  HWDisplayAttributes display_attributes;
  HWMixerAttributes mixer_attributes;
  HWPanelInfo hw_panel_info;
  uint32_t active_index = 0;

  DTRACE_SCOPED();

  error = hw_intf_->GetActiveConfig(&active_index);
  if (error != kErrorNone) {
    return error;
  }

  error = hw_intf_->GetDisplayAttributes(active_index, &display_attributes);
  if (error != kErrorNone) {
    return error;
  }

  error = hw_intf_->GetMixerAttributes(&mixer_attributes);
  if (error != kErrorNone) {
    return error;
  }

  error = hw_intf_->GetHWPanelInfo(&hw_panel_info);
  if (error != kErrorNone) {
    return error;
  }

  bool display_unchanged = (display_attributes == display_attributes_);
  bool mixer_unchanged = (mixer_attributes == mixer_attributes_);
  bool panel_unchanged = (hw_panel_info == hw_panel_info_);
  if (display_unchanged && mixer_unchanged && panel_unchanged) {
    return kErrorNone;
  }

  error = comp_manager_->ReconfigureDisplay(display_comp_ctx_, display_attributes, hw_panel_info,
                                            mixer_attributes, fb_config_, &cached_qos_data_);
  if (error != kErrorNone) {
    return error;
  }
  default_clock_hz_ = cached_qos_data_.clock_hz;

  // Disable Partial Update for one frame as PU not supported during modeset.
  DisablePartialUpdateOneFrameInternal();

  display_attributes_ = display_attributes;
  mixer_attributes_ = mixer_attributes;
  hw_panel_info_ = hw_panel_info;

  // TODO(user): Temporary changes, to be removed when DRM driver supports
  // Partial update with Destination scaler enabled.
  SetPUonDestScaler();
  if (hw_panel_info_.partial_update && !disable_pu_on_dest_scaler_) {
    // If current panel supports Partial Update and destination scalar isn't enabled, then add
    // a pending PU request to be served in the first PU enable frame after the modeset frame.
    // Because if first PU enable frame, after transition, has a partial Frame-ROI and
    // is followed by Skip Validate frames, then it can benefit those frames.
    pu_pending_ = true;
  }

  return kErrorNone;
}

DisplayError DisplayBase::SetMixerResolution(uint32_t width, uint32_t height) {
  ClientLock lock(disp_mutex_);

  validated_ = false;
  DisplayError error = ReconfigureMixer(width, height);
  if (error != kErrorNone) {
    return error;
  }

  req_mixer_width_ = width;
  req_mixer_height_ = height;

  return kErrorNone;
}

DisplayError DisplayBase::GetMixerResolution(uint32_t *width, uint32_t *height) {
  ClientLock lock(disp_mutex_);
  if (!width || !height) {
    return kErrorParameters;
  }

  *width = mixer_attributes_.width;
  *height = mixer_attributes_.height;

  return kErrorNone;
}

DisplayError DisplayBase::ReconfigureMixer(uint32_t width, uint32_t height) {
  ClientLock lock(disp_mutex_);
  DisplayError error = kErrorNone;

  DTRACE_SCOPED();
  if (!width || !height) {
    return kErrorParameters;
  }

  DLOGD_IF(kTagQDCM, "Reconfiguring mixer with width : %d, height : %d", width, height);

  LayerRect fb_rect = { 0.0f, 0.0f, FLOAT(fb_config_.x_pixels), FLOAT(fb_config_.y_pixels) };
  LayerRect mixer_rect = { 0.0f, 0.0f, FLOAT(width), FLOAT(height) };

  error = comp_manager_->ValidateScaling(fb_rect, mixer_rect, false /* rotate90 */);
  if (error != kErrorNone) {
    return error;
  }

  HWMixerAttributes mixer_attributes;
  mixer_attributes.width = width;
  mixer_attributes.height = height;

  error = hw_intf_->SetMixerAttributes(mixer_attributes);
  if (error != kErrorNone) {
    return error;
  }

  return ReconfigureDisplay();
}

bool DisplayBase::NeedsDownScale(const LayerRect &src_rect, const LayerRect &dst_rect,
                                 bool needs_rotation) {
  float src_width = FLOAT(src_rect.right - src_rect.left);
  float src_height = FLOAT(src_rect.bottom - src_rect.top);
  float dst_width = FLOAT(dst_rect.right - dst_rect.left);
  float dst_height = FLOAT(dst_rect.bottom - dst_rect.top);

  if (needs_rotation) {
    std::swap(src_width, src_height);
  }

  if ((src_width > dst_width) || (src_height > dst_height)) {
    return true;
  }

  return false;
}

bool DisplayBase::NeedsMixerReconfiguration(LayerStack *layer_stack, uint32_t *new_mixer_width,
                                            uint32_t *new_mixer_height) {
  ClientLock lock(disp_mutex_);
  uint32_t mixer_width = mixer_attributes_.width;
  uint32_t mixer_height = mixer_attributes_.height;
  uint32_t fb_width = fb_config_.x_pixels;
  uint32_t fb_height = fb_config_.y_pixels;
  uint32_t display_width = display_attributes_.x_pixels;
  uint32_t display_height = display_attributes_.y_pixels;

  if (hw_resource_info_.has_concurrent_writeback && layer_stack->output_buffer) {
    DLOGV_IF(kTagDisplay, "Found concurrent writeback, configure LM width:%d height:%d",
             fb_width, fb_height);
    *new_mixer_width = fb_width;
    *new_mixer_height = fb_height;
    return ((*new_mixer_width != mixer_width) || (*new_mixer_height != mixer_height));
  }

  if (secure_event_ == kSecureDisplayStart || secure_event_ == kTUITransitionStart) {
    *new_mixer_width = display_width;
    *new_mixer_height = display_height;
    return ((*new_mixer_width != mixer_width) || (*new_mixer_height != mixer_height));
  }

  if (req_mixer_width_ && req_mixer_height_) {
    DLOGD_IF(kTagDisplay, "Required mixer width : %d, height : %d",
             req_mixer_width_, req_mixer_height_);
    *new_mixer_width = req_mixer_width_;
    *new_mixer_height = req_mixer_height_;
    return (req_mixer_width_ != mixer_width || req_mixer_height_ != mixer_height);
  }

  // Reconfigure mixer if display size is not equal to avoid quality loss in videoplayback
  // usecase due to video upscaling to fit display after downscaling at LM
  if (!custom_mixer_resolution_ && display_width == fb_width && display_height == fb_height
      && mixer_width == fb_width && mixer_height == fb_height) {
    DLOGV_IF(kTagDisplay, "Custom mixer resolution not enabled. Mixer size is same as"
                          "framebuffer and display resolution. Reconfiguration not needed");
    return false;
  }

  uint32_t layer_count = UINT32(layer_stack->layers.size());
  uint32_t fb_area = fb_width * fb_height;
  LayerRect fb_rect = (LayerRect) {0.0f, 0.0f, FLOAT(fb_width), FLOAT(fb_height)};

  RectOrientation fb_orientation = GetOrientation(fb_rect);
  uint32_t max_layer_area = 0;
  uint32_t max_area_layer_index = 0;
  std::vector<Layer *> layers = layer_stack->layers;
  uint32_t align_x = display_attributes_.is_3d_mux_used ? 4 : 2;
  uint32_t align_y = 2;

  for (uint32_t i = 0; i < layer_count; i++) {
    Layer *layer = layers.at(i);
    if (layer->flags.is_demura) {
      continue;
    }

    uint32_t layer_width = UINT32(layer->src_rect.right - layer->src_rect.left);
    uint32_t layer_height = UINT32(layer->src_rect.bottom - layer->src_rect.top);
    uint32_t layer_area = layer_width * layer_height;

    if (layer_area > max_layer_area) {
      max_layer_area = layer_area;
      max_area_layer_index = i;
    }
  }
  DLOGV_IF(kTagDisplay, "Max area layer at index : %d", max_area_layer_index);

  // TODO(user): Mark layer which needs downscaling on GPU fallback as priority layer and use MDP
  // for composition to avoid quality mismatch between GPU and MDP switch(idle timeout usecase).
  if (max_layer_area >= fb_area) {
    Layer *layer = layers.at(max_area_layer_index);
    bool needs_rotation = (layer->transform.rotation == 90.0f);

    uint32_t layer_width = UINT32(layer->src_rect.right - layer->src_rect.left);
    uint32_t layer_height = UINT32(layer->src_rect.bottom - layer->src_rect.top);
    LayerRect layer_dst_rect = {};

    RectOrientation layer_orientation = GetOrientation(layer->src_rect);
    if (layer_orientation != kOrientationUnknown &&
        fb_orientation != kOrientationUnknown) {
      if (layer_orientation != fb_orientation) {
        std::swap(layer_width, layer_height);
      }
    }

    // Align the width and height according to fb's aspect ratio
    *new_mixer_width = FloorToMultipleOf(UINT32((FLOAT(fb_width) / FLOAT(fb_height)) *
                                         layer_height), align_x);
    *new_mixer_height = FloorToMultipleOf(layer_height, align_y);

    LayerRect dst_domain = {0.0f, 0.0f, FLOAT(*new_mixer_width), FLOAT(*new_mixer_height)};

    MapRect(fb_rect, dst_domain, layer->dst_rect, &layer_dst_rect);
    if (NeedsDownScale(layer->src_rect, layer_dst_rect, needs_rotation)) {
      *new_mixer_width = display_width;
      *new_mixer_height = display_height;
    }
    if (*new_mixer_width > display_width || *new_mixer_height > display_height) {
      *new_mixer_width = display_width;
      *new_mixer_height = display_height;
    }
    return ((*new_mixer_width != mixer_width) || (*new_mixer_height != mixer_height));
  }

  return false;
}

DisplayError DisplayBase::SetFrameBufferConfig(const DisplayConfigVariableInfo &variable_info) {
  ClientLock lock(disp_mutex_);
  uint32_t width = variable_info.x_pixels;
  uint32_t height = variable_info.y_pixels;

  if (width == 0 || height == 0) {
    DLOGE("Unsupported resolution: (%dx%d)", width, height);
    return kErrorParameters;
  }

  // Create rects to represent the new source and destination crops
  LayerRect crop = LayerRect(0, 0, FLOAT(width), FLOAT(height));
  LayerRect dst = LayerRect(0, 0, FLOAT(mixer_attributes_.width), FLOAT(mixer_attributes_.height));
  // Set rotate90 to false since this is taken care of during regular composition.
  bool rotate90 = false;

  DisplayError error = comp_manager_->ValidateScaling(crop, dst, rotate90);
  if (error != kErrorNone) {
    DLOGE("Unsupported resolution: (%dx%d)", width, height);
    return kErrorParameters;
  }

  error =  comp_manager_->ReconfigureDisplay(display_comp_ctx_, display_attributes_, hw_panel_info_,
                                             mixer_attributes_, variable_info, &cached_qos_data_);
  if (error != kErrorNone) {
    return error;
  }
  default_clock_hz_ = cached_qos_data_.clock_hz;

  fb_config_.x_pixels = width;
  fb_config_.y_pixels = height;

  DLOGI("New framebuffer resolution (%dx%d)", fb_config_.x_pixels, fb_config_.y_pixels);

  return kErrorNone;
}

DisplayError DisplayBase::GetFrameBufferConfig(DisplayConfigVariableInfo *variable_info) {
  DTRACE_SCOPED();
  ClientLock lock(disp_mutex_);
  if (!variable_info) {
    return kErrorParameters;
  }

  *variable_info = fb_config_;

  return kErrorNone;
}

DisplayError DisplayBase::SetDetailEnhancerData(const DisplayDetailEnhancerData &de_data) {
  ClientLock lock(disp_mutex_);
  validated_ = false;
  DisplayError error = comp_manager_->SetDetailEnhancerData(display_comp_ctx_, de_data);
  if (error != kErrorNone) {
    return error;
  }
  // TODO(user): Temporary changes, to be removed when DRM driver supports
  // Partial update with Destination scaler enabled.
  if (de_data.enable) {
    de_enabled_  = true;
  } else {
    de_enabled_ = false;
  }
  SetPUonDestScaler();

  return kErrorNone;
}

DisplayError DisplayBase::GetDisplayPort(DisplayPort *port) {
  ClientLock lock(disp_mutex_);

  if (!port) {
    return kErrorParameters;
  }

  *port = hw_panel_info_.port;

  return kErrorNone;
}

DisplayError DisplayBase::GetDisplayId(int32_t *display_id) {
  ClientLock lock(disp_mutex_);

  if (!display_id) {
    return kErrorParameters;
  }

  *display_id = display_id_;

  return kErrorNone;
}

DisplayError DisplayBase::GetDisplayType(DisplayType *display_type) {
  ClientLock lock(disp_mutex_);

  if (!display_type) {
    return kErrorParameters;
  }

  *display_type = display_type_;

  return kErrorNone;
}

bool DisplayBase::IsPrimaryDisplay() {
  ClientLock lock(disp_mutex_);

  return IsPrimaryDisplayLocked();
}

bool DisplayBase::IsPrimaryDisplayLocked() {
  return hw_panel_info_.is_primary_panel;
}

DisplayError DisplayBase::SetCompositionState(LayerComposition composition_type, bool enable) {
  ClientLock lock(disp_mutex_);

  return comp_manager_->SetCompositionState(display_comp_ctx_, composition_type, enable);
}

void DisplayBase::CommitLayerParams(LayerStack *layer_stack) {
  if (!layer_stack) {
    DLOGW("Invalid layer stack found");
    return;
  }

  // Copy the acquire fence from clients layers  to HWLayers
  uint32_t hw_layers_count = UINT32(disp_layer_stack_.info.hw_layers.size());

  for (uint32_t i = 0; i < hw_layers_count; i++) {
    uint32_t sdm_layer_index = disp_layer_stack_.info.index.at(i);
    Layer *sdm_layer = layer_stack->layers.at(sdm_layer_index);
    Layer &hw_layer = disp_layer_stack_.info.hw_layers.at(i);

    hw_layer.input_buffer.planes[0].fd = Sys::dup_(sdm_layer->input_buffer.planes[0].fd);
    hw_layer.input_buffer.planes[0].offset = sdm_layer->input_buffer.planes[0].offset;
    hw_layer.input_buffer.planes[0].stride = sdm_layer->input_buffer.planes[0].stride;
    hw_layer.input_buffer.size = sdm_layer->input_buffer.size;
    hw_layer.input_buffer.acquire_fence = sdm_layer->input_buffer.acquire_fence;
    hw_layer.input_buffer.handle_id = sdm_layer->input_buffer.handle_id;
    // All app buffer handles are set prior to prepare.
    // TODO(user): Other FBT layer attributes like surface damage, dataspace, secure camera and
    // secure display flags are also updated during SetClientTarget() called between validate and
    // commit. Need to revist this and update it accordingly for FBT layer.
    if (disp_layer_stack_.info.gpu_target_index > 0 &&
        (static_cast<uint32_t>(disp_layer_stack_.info.gpu_target_index) == sdm_layer_index)) {
      hw_layer.input_buffer.flags.secure = sdm_layer->input_buffer.flags.secure;
      hw_layer.input_buffer.format = sdm_layer->input_buffer.format;
      hw_layer.input_buffer.width = sdm_layer->input_buffer.width;
      hw_layer.input_buffer.height = sdm_layer->input_buffer.height;
      hw_layer.input_buffer.unaligned_width = sdm_layer->input_buffer.unaligned_width;
      hw_layer.input_buffer.unaligned_height = sdm_layer->input_buffer.unaligned_height;
    }
  }

  UpdateFrameBuffer();

  if (layer_stack->elapse_timestamp) {
    disp_layer_stack_.info.elapse_timestamp = layer_stack->elapse_timestamp;
  }

  return;
}

void DisplayBase::UpdateFrameBuffer() {
  if (draw_method_ != kDrawUnifiedWithGPUTarget) {
    return;
  }

  bool client_target_present = false;
  for (auto &hw_layer : disp_layer_stack_.info.hw_layers) {
    if (hw_layer.composition == kCompositionGPUTarget) {
      client_target_present = true;
      break;
    }
  }

  bool need_cached_fb = !gpu_comp_frame_ && client_target_present;
  if (!need_cached_fb) {
    return;
  }

  uint32_t hw_layers_count = disp_layer_stack_.info.hw_layers.size();
  for (uint32_t i = 0; i < hw_layers_count; i++) {
    uint32_t sdm_layer_index = disp_layer_stack_.info.index.at(i);
    Layer &hw_layer = disp_layer_stack_.info.hw_layers.at(i);
    if (disp_layer_stack_.info.gpu_target_index == sdm_layer_index) {
      // Update GPU target buffer with cached fd.
      CloseFd(&hw_layer.input_buffer.planes[0].fd);
      hw_layer.input_buffer = cached_framebuffer_;
      hw_layer.input_buffer.planes[0].fd = Sys::dup_(hw_layer.input_buffer.planes[0].fd);
    }
  }
}

void DisplayBase::PostCommitLayerParams() {
  cached_qos_data_ = disp_layer_stack_.info.qos_data;
}

DisplayError DisplayBase::InitializeColorModes() {
  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  DisplayError error = color_mgr_->ColorMgrGetNumOfModes(&num_color_modes_);
  if (error != kErrorNone || !num_color_modes_) {
    DLOGV_IF(kTagQDCM, "GetNumModes failed = %d count = %d", error, num_color_modes_);
    return kErrorNotSupported;
  }
  DLOGI("Number of Color Modes = %d", num_color_modes_);

  if (!color_modes_.size()) {
    color_modes_.resize(num_color_modes_);

    DisplayError error = color_mgr_->ColorMgrGetModes(&num_color_modes_, color_modes_.data());
    if (error != kErrorNone) {
      color_modes_.clear();
      DLOGE("Failed");
      return error;
    }

    AttrVal var;
    uint32_t num_insert_color_modes = 0;
    for (uint32_t i = 0; i < num_color_modes_; i++) {
      DLOGV_IF(kTagQDCM, "Color Mode[%d]: Name = %s mode_id = %d", i, color_modes_[i].name,
               color_modes_[i].id);
      auto it = color_mode_map_.find(color_modes_[i].name);
      if (it != color_mode_map_.end()) {
        if (it->second->id < color_modes_[i].id) {
          color_mode_map_.erase(it);
          color_mode_map_.insert(std::make_pair(color_modes_[i].name, &color_modes_[i]));
        }
      } else {
        color_mode_map_.insert(std::make_pair(color_modes_[i].name, &color_modes_[i]));
      }

      var.clear();
      error = color_mgr_->ColorMgrGetModeInfo(color_modes_[i].id, &var);
      if (error != kErrorNone) {
        DLOGE("Failed for get attributes of mode_id = %d", color_modes_[i].id);
        continue;
      }
      if (!var.empty()) {
        auto it = color_mode_attr_map_.find(color_modes_[i].name);
        if (it == color_mode_attr_map_.end()) {
          color_mode_attr_map_.insert(std::make_pair(color_modes_[i].name, var));
          // If target doesn't support SSPP tone maping and color mode is HDR,
          // add bt2020pq and bt2020hlg color modes.
          if (hw_resource_info_.src_tone_map.none() && IsHdrMode(var)) {
            std::string str_render_intent;
            GetValueOfModeAttribute(var, kRenderIntentAttribute, &str_render_intent);
            color_mode_map_.insert(std::make_pair(kBt2020Pq, &color_modes_[i]));
            color_mode_map_.insert(std::make_pair(kBt2020Hlg, &color_modes_[i]));
            num_insert_color_modes = 2;
            InsertBT2020PqHlgModes(str_render_intent);
          }
        }
        std::vector<PrimariesTransfer> pt_list = {};
        GetColorPrimaryTransferFromAttributes(var, &pt_list);
        for (const PrimariesTransfer &pt : pt_list) {
          if (std::find(color_modes_cs_.begin(), color_modes_cs_.end(), pt) ==
              color_modes_cs_.end()) {
            color_modes_cs_.push_back(pt);
          }
        }
      }
    }
    PrimariesTransfer pt = {};
    if (std::find(color_modes_cs_.begin(), color_modes_cs_.end(), pt) ==
        color_modes_cs_.end()) {
      color_modes_cs_.push_back(pt);
    }

    num_color_modes_ += num_insert_color_modes;
  }

  return kErrorNone;
}

DisplayError DisplayBase::GetDisplayIdentificationData(uint8_t *out_port, uint32_t *out_data_size,
                                                       uint8_t *out_data) {
  if (!out_port || !out_data_size) {
    return kErrorParameters;
  }

  return hw_intf_->GetDisplayIdentificationData(out_port, out_data_size, out_data);
}

DisplayError DisplayBase::GetClientTargetSupport(uint32_t width, uint32_t height,
                                                 LayerBufferFormat format,
                                                 const ColorMetaData &color_metadata) {
  if (format != kFormatRGBA8888 && format != kFormatRGBA1010102) {
    DLOGW("Unsupported format = %d", format);
    return kErrorNotSupported;
  } else if (ValidateScaling(width, height) != kErrorNone) {
    DLOGW("Unsupported width = %d height = %d", width, height);
    return kErrorNotSupported;
  } else if (color_metadata.transfer && color_metadata.colorPrimaries) {
    DisplayError error = ValidateDataspace(color_metadata);
    if (error != kErrorNone) {
      DLOGW("Unsupported Transfer Request = %d Color Primary = %d",
             color_metadata.transfer, color_metadata.colorPrimaries);
      return error;
    }

    // Check for BT2020 support
    if (color_metadata.colorPrimaries == ColorPrimaries_BT2020) {
      DLOGW("Unsupported Color Primary = %d", color_metadata.colorPrimaries);
      return kErrorNotSupported;
    }
  }

  return kErrorNone;
}

bool DisplayBase::IsSupportSsppTonemap() {
  if (hw_resource_info_.src_tone_map.none()) {
    return false;
  } else {
    return true;
  }
}

DisplayError DisplayBase::ValidateScaling(uint32_t width, uint32_t height) {
  uint32_t display_width = display_attributes_.x_pixels;
  uint32_t display_height = display_attributes_.y_pixels;

  float max_scale_down = FLOAT(hw_resource_info_.max_scale_down);
  float max_scale_up = FLOAT(hw_resource_info_.max_scale_up);

  float scale_x = FLOAT(width / display_width);
  float scale_y = FLOAT(height / display_height);

  if (scale_x > max_scale_down || scale_y > max_scale_down) {
    return kErrorNotSupported;
  }

  if (UINT32(scale_x) < 1 && scale_x > 0.0f) {
    if ((1.0f / scale_x) > max_scale_up) {
      return kErrorNotSupported;
    }
  }

  if (UINT32(scale_y) < 1 && scale_y > 0.0f) {
    if ((1.0f / scale_y) > max_scale_up) {
      return kErrorNotSupported;
    }
  }

  return kErrorNone;
}

DisplayError DisplayBase::ValidateDataspace(const ColorMetaData &color_metadata) {
  // Handle transfer
  switch (color_metadata.transfer) {
    case Transfer_sRGB:
    case Transfer_SMPTE_170M:
    case Transfer_SMPTE_ST2084:
    case Transfer_HLG:
    case Transfer_Linear:
    case Transfer_Gamma2_2:
      break;
    default:
      DLOGW("Unsupported Transfer Request = %d", color_metadata.transfer);
      return kErrorNotSupported;
  }

  // Handle colorPrimaries
  switch (color_metadata.colorPrimaries) {
    case ColorPrimaries_BT709_5:
    case ColorPrimaries_BT601_6_525:
    case ColorPrimaries_BT601_6_625:
    case ColorPrimaries_DCIP3:
    case ColorPrimaries_BT2020:
      break;
    default:
      DLOGW("Unsupported Color Primary = %d", color_metadata.colorPrimaries);
      return kErrorNotSupported;
  }

  return kErrorNone;
}

// TODO(user): Temporary changes, to be removed when DRM driver supports
// Partial update with Destination scaler enabled.
void DisplayBase::SetPUonDestScaler() {
  uint32_t mixer_width = mixer_attributes_.width;
  uint32_t mixer_height = mixer_attributes_.height;
  uint32_t display_width = display_attributes_.x_pixels;
  uint32_t display_height = display_attributes_.y_pixels;

  disable_pu_on_dest_scaler_ = (mixer_width != display_width ||
                                mixer_height != display_height) || de_enabled_;
}

void DisplayBase::ClearColorInfo() {
  color_modes_.clear();
  color_mode_map_.clear();
  color_mode_attr_map_.clear();
  color_modes_cs_.clear();

  if (color_mgr_) {
    delete color_mgr_;
    color_mgr_ = NULL;
  }
}

void DisplayBase::DeInitializeColorModes() {
    color_mode_map_.clear();
    color_modes_.clear();
    color_mode_attr_map_.clear();
    num_color_modes_ = 0;
}

void DisplayBase::GetColorPrimaryTransferFromAttributes(const AttrVal &attr,
    std::vector<PrimariesTransfer> *supported_pt) {
  std::string attribute_field = {};
  if (attr.empty()) {
    return;
  }

  for (auto &it : attr) {
    if ((it.first.find(kColorGamutAttribute) != std::string::npos) ||
        (it.first.find(kDynamicRangeAttribute) != std::string::npos)) {
      attribute_field = it.second;
      PrimariesTransfer pt = {};
      pt.primaries = GetColorPrimariesFromAttribute(attribute_field);
      if (pt.primaries == ColorPrimaries_BT709_5) {
        pt.transfer = Transfer_sRGB;
        supported_pt->push_back(pt);
      } else if (pt.primaries == ColorPrimaries_DCIP3) {
        pt.transfer = Transfer_sRGB;
        supported_pt->push_back(pt);
      } else if (pt.primaries == ColorPrimaries_BT2020) {
        pt.transfer = Transfer_SMPTE_ST2084;
        supported_pt->push_back(pt);
        pt.transfer = Transfer_HLG;
        supported_pt->push_back(pt);
      }
    }
  }
}

void DisplayBase::HwRecovery(const HWRecoveryEvent sdm_event_code) {
  DLOGI("Handling event = %" PRIu32, sdm_event_code);
  if (DisplayPowerResetPending()) {
    DLOGI("Skipping handling for display = %d, display power reset in progress", display_type_);
    return;
  }
  switch (sdm_event_code) {
    case HWRecoveryEvent::kSuccess:
      hw_recovery_logs_captured_ = false;
      hw_recovery_count_ = 0;
      break;
    case HWRecoveryEvent::kCapture:
#ifndef TRUSTED_VM
      if (!disable_hw_recovery_dump_ && !hw_recovery_logs_captured_) {
        hw_intf_->DumpDebugData();
        hw_recovery_logs_captured_ = true;
        DLOGI("Captured debugfs data for display = %d", display_type_);
      } else if (!disable_hw_recovery_dump_) {
        DLOGI("Multiple capture events without intermediate success event, skipping debugfs"
              "capture for display = %d", display_type_);
      } else {
        DLOGI("Debugfs data dumping is disabled for display = %d", display_type_);
      }
      hw_recovery_count_++;
      if (hw_recovery_count_ >= hw_recovery_threshold_) {
        DLOGI("display = %d attempting to start display power reset", display_type_);
        if (StartDisplayPowerReset()) {
          DLOGI("display = %d allowed to start display power reset", display_type_);
          {
            ClientLock lock(disp_mutex_);
            validated_ = false;
          }
          event_handler_->HandleEvent(kDisplayPowerResetEvent);
          EndDisplayPowerReset();
          DLOGI("display = %d has finished display power reset", display_type_);
          hw_recovery_count_ = 0;
        }
      }
#else
      {
        ClientLock lock(disp_mutex_);
        validated_ = false;
      }
      event_handler_->HandleEvent(kDisplayPowerResetEvent);
#endif
      break;
    case HWRecoveryEvent::kDisplayPowerReset:
#ifndef TRUSTED_VM
      DLOGI("display = %d attempting to start display power reset", display_type_);
      if (StartDisplayPowerReset()) {
        DLOGI("display = %d allowed to start display power reset", display_type_);
        {
          ClientLock lock(disp_mutex_);
          validated_ = false;
        }
        event_handler_->HandleEvent(kDisplayPowerResetEvent);
        EndDisplayPowerReset();
        DLOGI("display = %d has finished display power reset", display_type_);
      }
#else
      {
        ClientLock lock(disp_mutex_);
        validated_ = false;
      }
      event_handler_->HandleEvent(kDisplayPowerResetEvent);
#endif
      break;
    default:
      return;
  }
}

bool DisplayBase::DisplayPowerResetPending() {
  SCOPE_LOCK(display_power_reset_lock_);
  return display_power_reset_pending_;
}

bool DisplayBase::StartDisplayPowerReset() {
  SCOPE_LOCK(display_power_reset_lock_);
  if (!display_power_reset_pending_) {
    display_power_reset_pending_ = true;
    return true;
  }
  return false;
}

void DisplayBase::EndDisplayPowerReset() {
  SCOPE_LOCK(display_power_reset_lock_);
  display_power_reset_pending_ = false;
}

bool DisplayBase::SetHdrModeAtStart(LayerStack *layer_stack) {
  return (hw_resource_info_.src_tone_map.none() && layer_stack->flags.hdr_present);
}

PrimariesTransfer DisplayBase::GetBlendSpaceFromColorMode() {
  PrimariesTransfer pt = {};
  auto current_color_attr_ = color_mode_attr_map_.find(current_color_mode_);
  if (current_color_attr_ == color_mode_attr_map_.end()) {
    DLOGE("The attritbutes is not present in color mode: %s", current_color_mode_.c_str());
    return pt;
  }

  AttrVal attr = current_color_attr_->second;
  std::string color_gamut = kNative, dynamic_range = kSdr, pic_quality = kStandard;
  std::string transfer = {};

  if (attr.begin() != attr.end()) {
    for (auto &it : attr) {
      if (it.first.find(kColorGamutAttribute) != std::string::npos) {
        color_gamut = it.second;
      } else if (it.first.find(kDynamicRangeAttribute) != std::string::npos) {
        dynamic_range = it.second;
      } else if (it.first.find(kPictureQualityAttribute) != std::string::npos) {
        pic_quality = it.second;
      } else if (it.first.find(kGammaTransferAttribute) != std::string::npos) {
        transfer = it.second;
      }
    }
  }
  // TODO(user): Check is if someone calls with hal_display_p3
  if (hw_resource_info_.src_tone_map.none() &&
      (pic_quality == kStandard && color_gamut == kBt2020)) {
    pt.primaries = GetColorPrimariesFromAttribute(color_gamut);
    if (transfer == kHlg) {
      pt.transfer = Transfer_HLG;
    } else {
      pt.transfer = Transfer_SMPTE_ST2084;
    }
  } else if (color_gamut == kDcip3) {
    pt.primaries = GetColorPrimariesFromAttribute(color_gamut);
    pt.transfer = Transfer_sRGB;
  }

  return pt;
}

void DisplayBase::InsertBT2020PqHlgModes(const std::string &str_render_intent) {
  AttrVal hdr_var = {};
  hdr_var.push_back(std::make_pair(kColorGamutAttribute, kBt2020));
  hdr_var.push_back(std::make_pair(kPictureQualityAttribute, kStandard));
  if (!str_render_intent.empty()) {
    hdr_var.push_back(std::make_pair(kRenderIntentAttribute, str_render_intent));
  }
  hdr_var.push_back(std::make_pair(kGammaTransferAttribute, kSt2084));
  color_mode_attr_map_.insert(std::make_pair(kBt2020Pq, hdr_var));
  hdr_var.pop_back();
  hdr_var.push_back(std::make_pair(kGammaTransferAttribute, kHlg));
  color_mode_attr_map_.insert(std::make_pair(kBt2020Hlg, hdr_var));

  return;
}

bool DisplayBase::IsHdrMode(const AttrVal &attr) {
  std::string color_gamut, dynamic_range;
  GetValueOfModeAttribute(attr, kColorGamutAttribute, &color_gamut);
  GetValueOfModeAttribute(attr, kDynamicRangeAttribute, &dynamic_range);
  if (color_gamut == kDcip3 && dynamic_range == kHdr) {
    return true;
  }

  return false;
}

DisplayError DisplayBase::ResetPendingPowerState(const shared_ptr<Fence> &retire_fence) {
  if (pending_power_state_ != kPowerStateNone) {
    // Retire fence signalling confirms that CRTC enabled, hence wait for retire fence before
    // we enable vsync
    SyncPoints sync_points = {};
    sync_points.retire_fence = retire_fence;
    WaitForCompletion(&sync_points);

    DisplayState pending_state;
    GetPendingDisplayState(&pending_state);
    if (IsPrimaryDisplayLocked() &&
     (pending_power_state_ != kPowerStateOff)) {
      primary_active_ = true;
    } else {
      primary_active_ = false;
    }

    state_ = pending_state;
    active_ = true;

    pending_power_state_ = kPowerStateNone;
  }
  return kErrorNone;
}

bool DisplayBase::CheckResourceState(bool *res_exhausted) {
  return comp_manager_->CheckResourceState(display_comp_ctx_, res_exhausted, display_attributes_);
}
DisplayError DisplayBase::colorSamplingOn() {
  return kErrorNone;
}

DisplayError DisplayBase::colorSamplingOff() {
  return kErrorNone;
}

bool DisplayBase::GameEnhanceSupported() {
  if (color_mgr_) {
    return color_mgr_->GameEnhanceSupported();
  }
  return false;
}

DisplayError DisplayBase::GetPendingDisplayState(DisplayState *disp_state) {
  if (!disp_state) {
    return kErrorParameters;
  }
  DLOGI("pending_power_state %d for display %d-%d", pending_power_state_, display_id_,
        display_type_);
  switch (pending_power_state_) {
    case kPowerStateOn:
      *disp_state = kStateOn;
      break;
    case kPowerStateOff:
      *disp_state = kStateOff;
      break;
    case kPowerStateDoze: {
      *disp_state = kStateDoze;
      DisplayError error = ReconfigureDisplay();
      if (error != kErrorNone) {
        return error;
      }
      event_handler_->Refresh();
      break;
    }
    case kPowerStateDozeSuspend:
      *disp_state = kStateDozeSuspend;
      break;
    default:
      return kErrorParameters;
  }
  return kErrorNone;
}

DisplayError DisplayBase::IsSupportedOnDisplay(const SupportedDisplayFeature feature,
                                               uint32_t *supported) {
  DisplayError error = kErrorNone;

  if (!supported) {
    return kErrorParameters;
  }

  switch (feature) {
    case kSupportedModeSwitch: {
      ClientLock lock(disp_mutex_);
      error = hw_intf_->GetFeatureSupportStatus(kAllowedModeSwitch, supported);
      break;
    }
    case kDestinationScalar:
      *supported = custom_mixer_resolution_;
      break;
    case kCwbDemuraTapPoint: {
      std::vector<CwbTapPoint> &tappoints = hw_resource_info_.tap_points;
      *supported = UINT32(std::find(tappoints.begin(), tappoints.end(),
                                    CwbTapPoint::kDemuraTapPoint) != tappoints.end());
      break;
    }
    case kCwbCrop:
      error = hw_intf_->GetFeatureSupportStatus(kHasCwbCrop, supported);
      break;
    case kDedicatedCwb:
      error = hw_intf_->GetFeatureSupportStatus(kHasDedicatedCwb, supported);
      break;
    default:
      DLOGW("Feature:%d is not present for display %d:%d", feature, display_id_, display_type_);
      error = kErrorParameters;
      break;
  }

  return error;
}

void DisplayBase::SetPendingPowerState(DisplayState state) {
  switch (state) {
    case kStateOn:
      pending_power_state_ = kPowerStateOn;
      break;
    case kStateOff:
      pending_power_state_ = kPowerStateOff;
      break;
    case kStateDoze:
      pending_power_state_ = kPowerStateDoze;
      break;
    case kStateDozeSuspend:
      pending_power_state_ = kPowerStateDozeSuspend;
      break;
    default:
      return;
  }
  DLOGI("pending_power_state %d for display %d-%d", pending_power_state_, display_id_,
        display_type_);
}

DisplayError DisplayBase::HandleSecureEvent(SecureEvent secure_event, bool *needs_refresh) {
  ClientLock lock(disp_mutex_);
  if (!needs_refresh) {
    return kErrorParameters;
  }
  DisplayError err = kErrorNone;
  *needs_refresh = false;

  DLOGI("Secure event %d for display %d-%d", secure_event, display_id_, display_type_);

  if (secure_event == kTUITransitionStart &&
      (state_ != kStateOn || (pending_power_state_ != kPowerStateNone))) {
    DLOGW("Cannot start TUI session when display state is %d or pending_power_state %d",
          state_, pending_power_state_);
    return kErrorPermission;
  }
  shared_ptr<Fence> release_fence = nullptr;
  if (secure_event == kTUITransitionStart) {
    if (vsync_enable_) {
      err = SetVSyncState(false /* enable */);
      if (err != kErrorNone) {
        return err;
      }
      vsync_enable_pending_ = true;
    }
    *needs_refresh = (hw_panel_info_.mode == kModeCommand);
    DisablePartialUpdateOneFrameInternal();
    err = hw_events_intf_->SetEventState(HWEvent::BACKLIGHT_EVENT, true);
    if (err != kErrorNone) {
      return err;
    }
    comp_manager_->GetDefaultQosData(display_comp_ctx_, &cached_qos_data_);
  } else if (secure_event == kTUITransitionPrepare) {
    DisplayState state = state_;
    err = SetDisplayState(kStateOff, true /* teardown */, &release_fence);
    if (err != kErrorNone) {
      DLOGE("SetDisplay state off failed for %d err %d", display_id_, err);
      return err;
    }
    SetPendingPowerState(state);
  }

  err = hw_intf_->HandleSecureEvent(secure_event, cached_qos_data_);
  if (err != kErrorNone) {
    return err;
  }

  comp_manager_->HandleSecureEvent(display_comp_ctx_, secure_event);
  secure_event_ = secure_event;
  if (secure_event == kTUITransitionEnd) {
    DisplayState pending_state;
    *needs_refresh = true;
    if (GetPendingDisplayState(&pending_state) == kErrorNone) {
      if (pending_state == kStateOff) {
        shared_ptr<Fence> release_fence = nullptr;
        DisplayError err = SetDisplayState(pending_state, false /* teardown */, &release_fence);
        if (err != kErrorNone) {
          DLOGE("SetDisplay state %d failed for %d err %d", pending_state, display_id_, err);
          return err;
        }
        *needs_refresh = false;
      }
    }
    DisablePartialUpdateOneFrameInternal();
    err = hw_events_intf_->SetEventState(HWEvent::BACKLIGHT_EVENT, false);
    if (err != kErrorNone) {
      return err;
    }
  } else if (secure_event == kTUITransitionUnPrepare) {
    // Trigger refresh on non targetted display to update the screen after TUI end
    *needs_refresh = true;
    DisplayState state = kStateOff;
    if (GetPendingDisplayState(&state) == kErrorNone) {
      shared_ptr<Fence> release_fence = nullptr;
      err = SetDisplayState(state, false /* teardown */, &release_fence);
      if (err != kErrorNone) {
        DLOGE("SetDisplay state %d failed for %d err %d", state, display_id_, err);
        return err;
      }
    }
    if (state == kStateOff) {
      *needs_refresh = false;
    }
  }
  if (*needs_refresh) {
    validated_ = false;
  }
  return kErrorNone;
}

DisplayError DisplayBase::GetOutputBufferAcquireFence(shared_ptr<Fence> *out_fence) {
  ClientLock lock(disp_mutex_);
  LayerBuffer *out_buffer = disp_layer_stack_.info.output_buffer;
  if (out_buffer == nullptr) {
    return kErrorNotSupported;
  }

  *out_fence = out_buffer->release_fence;

  return kErrorNone;
}

DisplayError DisplayBase::OnMinHdcpEncryptionLevelChange(uint32_t min_enc_level) {
  ClientLock lock(disp_mutex_);
  validated_ = false;
  return hw_intf_->OnMinHdcpEncryptionLevelChange(min_enc_level);
}

void DisplayBase::CheckMMRMState() {
  ClientLock lock(disp_mutex_);
  if (!mmrm_updated_) {
    return;
  }
  DTRACE_SCOPED();
  DLOGI("Handling updated MMRM request");
  mmrm_updated_ = false;
  bool reduced_clk = (mmrm_requested_clk_ < hw_resource_info_.max_sde_clk) ? true : false;

  // Check layers if clock is less than max
  LayerStack *stack = disp_layer_stack_.stack;
  if (reduced_clk && stack) {
    if (stack->flags.hdr_present || stack->flags.secure_present) {
      DLOGW("Cannot lower clock, hdr_present=%d, secure_present=%d",
        stack->flags.hdr_present, stack->flags.secure_present);
      return;
    } else {
      for (auto &layer : stack->layers) {
        if (layer->flags.sde_preferred) {
          DLOGW("Cannot lower clock. SDE Preferred layer");
          return;
        }
      }
    }
  }

  if (comp_manager_->SetMaxSDEClk(display_comp_ctx_, mmrm_requested_clk_) != kErrorNone) {
    DLOGW("Could not set max sde clk");
    return;
  }

  // Set flag to reject new ext. display creation/power change.Refresh all displays.
  event_handler_->MMRMEvent(reduced_clk);
}

void DisplayBase::MMRMEvent(uint32_t clk) {
  ClientLock lock(disp_mutex_);
  DTRACE_SCOPED();
  if (clk < mmrm_floor_clk_vote_) {
    DLOGW("Clk vote of %u is lower than floor clock %d. Bail.", clk, mmrm_floor_clk_vote_);
    return;
  }

  // Only support primary. If off, allow secondary.
  if (!IsPrimaryDisplayLocked() && primary_active_) {
    DLOGV("Ignoring event on secondary");
    return;
  }

  mmrm_requested_clk_ = clk;
  mmrm_updated_ = true;
  DLOGI("MMRM state has been updated, clk requested=%u", clk);

  // Invalidate to retrigger clk calculation
  validated_ = false;
  event_handler_->Refresh();
}

void DisplayBase::WaitForCompletion(SyncPoints *sync_points) {
  DTRACE_SCOPED();
  // Wait on current retire fence.
  if (draw_method_ == kDrawDefault || display_type_ == kVirtual) {
    Fence::Wait(sync_points->retire_fence);
    return;
  }

  // Wait for CRTC power event on first cycle.
  if (first_cycle_) {
    if (hw_panel_info_.is_primary_panel) {
      DLOGI("Sync commit on primary");
      return;
    }
    std::unique_lock<std::mutex> lck(power_mutex_);
    while (!transition_done_) {
      cv_.wait(lck);
    }

    // Unregister power events.
    hw_events_intf_->SetEventState(HWEvent::POWER_EVENT, false);
    return;
  }

  // For displays in unified draw, wait on cached retire fence in steady state.
  shared_ptr<Fence> retire_fence = sync_points->retire_fence;
  Fence::Wait(retire_fence, kPowerStateTimeout);
}

void DisplayBase::ProcessPowerEvent() {
  DTRACE_SCOPED();
  std::unique_lock<std::mutex> lck(power_mutex_);
  transition_done_ = true;
  cv_.notify_one();
}

void DisplayBase::Abort() {
  std::unique_lock<std::mutex> lck(power_mutex_);

  if (display_type_ == kHDMI && first_cycle_) {
    DLOGI("Abort!");
    transition_done_ = true;
    cv_.notify_one();
  }
}

void DisplayBase::CacheRetireFence() {
  if (draw_method_ == kDrawDefault) {
    retire_fence_ = disp_layer_stack_.info.retire_fence;
  } else {
    // For displays in unified draw, wait on cached retire fence in steady state.
    comp_manager_->GetRetireFence(display_comp_ctx_, &retire_fence_);
  }
}

DisplayError DisplayBase::SetHWDetailedEnhancerConfig(void *params) {
  DisplayError err = kErrorNone;
  DisplayDetailEnhancerData de_data;

  PPDETuningCfgData *de_tuning_cfg_data = reinterpret_cast<PPDETuningCfgData*>(params);
  if (de_tuning_cfg_data->cfg_pending) {
    if (!de_tuning_cfg_data->cfg_en) {
      de_data.enable = 0;
      DLOGV_IF(kTagQDCM, "Disable DE config");
    } else {
      de_data.override_flags = kOverrideDEEnable;
      de_data.enable = 1;

      DLOGV_IF(kTagQDCM, "Enable DE: flags %u, sharp_factor %d, thr_quiet %d, thr_dieout %d, "
        "thr_low %d, thr_high %d, clip %d, quality %d, content_type %d, de_blend %d",
        de_tuning_cfg_data->params.flags, de_tuning_cfg_data->params.sharp_factor,
        de_tuning_cfg_data->params.thr_quiet, de_tuning_cfg_data->params.thr_dieout,
        de_tuning_cfg_data->params.thr_low, de_tuning_cfg_data->params.thr_high,
        de_tuning_cfg_data->params.clip, de_tuning_cfg_data->params.quality,
        de_tuning_cfg_data->params.content_type, de_tuning_cfg_data->params.de_blend);

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagSharpFactor) {
        de_data.override_flags |= kOverrideDESharpen1;
        de_data.sharp_factor = de_tuning_cfg_data->params.sharp_factor;
      }

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagClip) {
        de_data.override_flags |= kOverrideDEClip;
        de_data.clip = de_tuning_cfg_data->params.clip;
      }

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagThrQuiet) {
        de_data.override_flags |= kOverrideDEThrQuiet;
        de_data.thr_quiet = de_tuning_cfg_data->params.thr_quiet;
      }

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagThrDieout) {
        de_data.override_flags |= kOverrideDEThrDieout;
        de_data.thr_dieout = de_tuning_cfg_data->params.thr_dieout;
      }

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagThrLow) {
        de_data.override_flags |= kOverrideDEThrLow;
        de_data.thr_low = de_tuning_cfg_data->params.thr_low;
      }

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagThrHigh) {
        de_data.override_flags |= kOverrideDEThrHigh;
        de_data.thr_high = de_tuning_cfg_data->params.thr_high;
      }

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagContentQualLevel) {
        switch (de_tuning_cfg_data->params.quality) {
          case kDeContentQualLow:
            de_data.quality_level = kContentQualityLow;
            break;
          case kDeContentQualMedium:
            de_data.quality_level = kContentQualityMedium;
            break;
          case kDeContentQualHigh:
            de_data.quality_level = kContentQualityHigh;
            break;
          case kDeContentQualUnknown:
          default:
            de_data.quality_level = kContentQualityUnknown;
            break;
        }
      }

      switch (de_tuning_cfg_data->params.content_type) {
        case kDeContentTypeVideo:
          de_data.content_type = kContentTypeVideo;
          break;
        case kDeContentTypeGraphics:
          de_data.content_type = kContentTypeGraphics;
          break;
        case kDeContentTypeUnknown:
        default:
          de_data.content_type = kContentTypeUnknown;
          break;
      }

      if (de_tuning_cfg_data->params.flags & kDeTuningFlagDeBlend) {
        de_data.override_flags |= kOverrideDEBlend;
        de_data.de_blend = de_tuning_cfg_data->params.de_blend;
      }
    }

    err = comp_manager_->SetDetailEnhancerData(display_comp_ctx_, de_data);
    if (err != kErrorNone) {
      DLOGW("SetDetailEnhancerConfig failed. err = %d", err);
      return err;
    }
    // TODO(user): Temporary changes, to be removed when DRM driver supports
    // Partial update with Destination scaler enabled.
    if (de_data.enable) {
      de_enabled_  = true;
    } else {
      de_enabled_  = false;
    }
    SetPUonDestScaler();

    de_tuning_cfg_data->cfg_pending = false;
  }

  return err;
}

DisplayError DisplayBase::GetPanelBlMaxLvl(uint32_t *max_level) {
  ClientLock lock(disp_mutex_);

  DisplayError err = hw_intf_->GetPanelBlMaxLvl(max_level);
  if (err) {
    DLOGE("Failed to get panel max backlight level %d", err);
  } else {
    DLOGI_IF(kTagDisplay, "Got panel max backlight %d", *(max_level));
  }
  return err;
}

DisplayError DisplayBase::SetPPConfig(void *payload, size_t size) {
  ClientLock lock(disp_mutex_);

  DisplayError err = hw_intf_->SetPPConfig(payload, size);
  if (err) {
    DLOGE("Failed to set PP Event %d", err);
  } else {
    DLOGI_IF(kTagDisplay, "PP Event is set successfully");
    event_handler_->Refresh();
  }
  return err;
}

DisplayError DisplayBase::SetDimmingEnable(int int_enabled) {
  struct sde_drm::DRMPPFeatureInfo info = {};
  GenericPayload payload;
  bool *bl_ctrl = nullptr;

  int ret = payload.CreatePayload(bl_ctrl);
  if (ret || !bl_ctrl) {
    DLOGE("Create Payload failed with ret %d", ret);
    return kErrorUndefined;
  }

  *bl_ctrl = int_enabled? true : false;
  info.object_type = DRM_MODE_OBJECT_CONNECTOR;
  info.id = sde_drm::kFeatureDimmingDynCtrl;
  info.type = sde_drm::kPropRange;
  info.version = 0;
  info.payload = bl_ctrl;
  info.payload_size = sizeof(bool);
  info.is_event = false;

  DLOGV_IF(kTagDisplay, "Display %d-%d set dimming enable %d", display_id_,
    display_type_, int_enabled);
  return SetPPConfig(reinterpret_cast<void *>(&info), sizeof(info));
}

DisplayError DisplayBase::SetDimmingMinBl(int min_bl) {
  struct sde_drm::DRMPPFeatureInfo info = {};
  GenericPayload payload;
  int *bl = nullptr;

  int ret = payload.CreatePayload(bl);
  if (ret || !bl) {
    DLOGE("Create Payload failed with ret %d", ret);
    return kErrorUndefined;
  }

  *bl = min_bl;
  info.object_type = DRM_MODE_OBJECT_CONNECTOR;
  info.id = sde_drm::kFeatureDimmingMinBl;
  info.type = sde_drm::kPropRange;
  info.version = 0;
  info.payload = bl;
  info.payload_size = sizeof(int);
  info.is_event = false;

  DLOGV_IF(kTagDisplay, "Display %d-%d set dimming min_bl %d", display_id_,
    display_type_, min_bl);
  return SetPPConfig(reinterpret_cast<void *>(&info), sizeof(info));
}

/* this func is called by DC dimming feature only after PCC updates */
void DisplayBase::ScreenRefresh() {
  {
    ClientLock lock(disp_mutex_);
    /* do not skip validate */
    validated_ = false;
  }
  event_handler_->Refresh();
}

void DisplayBase::PrepareForAsyncTransition() {
  // Caution:
  // Structures which are owned by caller or main thread must not be referenced by async execution.
  //    Caller is free to reuse the passed structures for next draw cycle preparation.
  // To prevent accidental usage, reset all such internal pointers referring to caller structures
  //    so that an instant fatal error is observed in place of prolonged corruption.
  disp_layer_stack_.stack = nullptr;
}

std::chrono::system_clock::time_point DisplayBase::WaitUntil() {
  int idle_time_ms = disp_layer_stack_.info.set_idle_time_ms;
  std::chrono::system_clock::time_point timeout_time;

  DLOGV_IF(kTagDisplay, "Off: %d, time: %d, timeout:%d, panel: %s",
        state_ == kStateOff, idle_time_ms, handle_idle_timeout_,
        hw_panel_info_.mode == kModeVideo ? "video" : "cmd");

  // Indefinite wait if state is off or idle timeout has triggered
  if (state_ == kStateOff || idle_time_ms <= 0 || handle_idle_timeout_) {
    timeout_time = std::chrono::system_clock::from_time_t(INT_MAX);
  } else {
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    timeout_time = current_time + std::chrono::milliseconds(idle_time_ms);
  }
  return timeout_time;
}

DisplayError DisplayBase::ConfigureCwbForIdleFallback(LayerStack *layer_stack) {
  DisplayError error = kErrorNone;
  if (!layer_stack->request_flags.trigger_refresh) {
    return error;
  }

  comp_manager_->HandleCwbFrequencyBoost(true);

  cwb_config_ = new CwbConfig;
  if (layer_stack->cwb_config == NULL) {
    cwb_config_->tap_point = CwbTapPoint::kLmTapPoint;
    // Setting full frame ROI
    cwb_config_->cwb_full_rect = LayerRect(0.0f, 0.0f, FLOAT(mixer_attributes_.width),
                                           FLOAT(mixer_attributes_.height));
    cwb_config_->cwb_roi = cwb_config_->cwb_full_rect;
  }

  disp_layer_stack_.info.hw_cwb_config = cwb_config_;
  error = ValidateCwbConfigInfo(disp_layer_stack_.info.hw_cwb_config,
                                layer_stack->output_buffer->format);
  if (error != kErrorNone) {
    DLOGE("CWB_config validation failed.");
    return error;
  }

  return error;
}

void DisplayBase::TrackInputFences() {
  if (!track_input_fences_) {
    return;
  }
  // Check if async task is in progress.
  // Wait until it finishes.
  if (fence_wait_future_.valid()) {
    fence_wait_future_.get();
  }

  lock_guard<mutex> scope_lock(fence_track_mutex_);
  // Copy & Wait on all fences.
  acquire_fences_ = {};
  for (auto &hw_layer : disp_layer_stack_.info.hw_layers) {
    acquire_fences_.push_back(hw_layer.input_buffer.acquire_fence);
  }
  // Start async task to wait on fences.
  fence_wait_future_ = std::async(std::launch::async, [&](){
                                  WaitOnFences();
                                  });
}

void DisplayBase::WaitOnFences() {
  lock_guard<mutex> scope_lock(fence_track_mutex_);
  const int kFenceWaitTimeoutMs = 500;
  for (auto &acquire_fence : acquire_fences_) {
    if (Fence::Wait(acquire_fence, kFenceWaitTimeoutMs) == kErrorNone) {
      continue;
    }
    // Fence did not signal in 500 ms.
    DLOGI("Dumping stack trace for %d-%d", display_id_, display_type_);
    event_handler_->HandleEvent(kDumpStacktrace);
    usleep(kFenceWaitTimeoutMs * 1000);
    DLOGI("Dumping stack trace after 500 ms sleep %d-%d", display_id_, display_type_);
    event_handler_->HandleEvent(kDumpStacktrace);
  }
}

}  // namespace sdm
