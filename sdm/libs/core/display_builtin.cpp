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

/*
* Changes from Qualcomm Innovation Center are provided under the following license:
*
* Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/rect.h>
#include <utils/utils.h>
#include <utils/formats.h>
#include <core/buffer_allocator.h>
#include <sys/mman.h>
#include <private/hw_interface.h>
#include <private/hw_info_interface.h>
#include <iomanip>
#include <algorithm>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "display_builtin.h"
#include "drm_interface.h"
#include "drm_master.h"

#define __CLASS__ "DisplayBuiltIn"

namespace sdm {

DisplayBuiltIn::DisplayBuiltIn(DisplayEventHandler *event_handler, HWInfoInterface *hw_info_intf,
                               BufferAllocator *buffer_allocator, CompManager *comp_manager,
                               std::shared_ptr<IPCIntf> ipc_intf)
  : DisplayBase(kBuiltIn, event_handler, kDeviceBuiltIn, buffer_allocator, comp_manager,
                hw_info_intf), ipc_intf_(ipc_intf) {}

DisplayBuiltIn::DisplayBuiltIn(int32_t display_id, DisplayEventHandler *event_handler,
                               HWInfoInterface *hw_info_intf,
                               BufferAllocator *buffer_allocator, CompManager *comp_manager,
                               std::shared_ptr<IPCIntf> ipc_intf)
  : DisplayBase(display_id, kBuiltIn, event_handler, kDeviceBuiltIn, buffer_allocator, comp_manager,
                hw_info_intf), ipc_intf_(ipc_intf) {}

DisplayBuiltIn::~DisplayBuiltIn() {
}

static uint64_t GetTimeInMs(struct timespec ts) {
  return (ts.tv_sec * 1000 + (ts.tv_nsec + 500000) / 1000000);
}

DisplayError DisplayBuiltIn::Init() {
  ClientLock lock(disp_mutex_);

  DisplayError error = HWInterface::Create(display_id_, kBuiltIn, hw_info_intf_,
                                           buffer_allocator_, &hw_intf_);
  if (error != kErrorNone) {
    DLOGE("Failed to create hardware interface on. Error = %d", error);
    return error;
  }

  if (-1 == display_id_) {
    hw_intf_->GetDisplayId(&display_id_);
  }

  error = DisplayBase::Init();
  if (error != kErrorNone) {
    HWInterface::Destroy(hw_intf_);
    return error;
  }

  if (color_mgr_) {
    color_mgr_->ColorMgrGetStcModes(&stc_color_modes_);
  }

  if (hw_panel_info_.mode == kModeCommand && Debug::IsVideoModeEnabled()) {
    error = hw_intf_->SetDisplayMode(kModeVideo);
    if (error != kErrorNone) {
      DLOGW("Retaining current display mode. Current = %d, Requested = %d", hw_panel_info_.mode,
            kModeVideo);
    }
  }

#ifdef TRUSTED_VM
  event_list_ = {HWEvent::VSYNC, HWEvent::EXIT, HWEvent::PINGPONG_TIMEOUT, HWEvent::PANEL_DEAD,
                 HWEvent::HW_RECOVERY};
#else
  event_list_ = {HWEvent::VSYNC,            HWEvent::EXIT,
                 HWEvent::SHOW_BLANK_EVENT, HWEvent::THERMAL_LEVEL,
                 HWEvent::PINGPONG_TIMEOUT, HWEvent::PANEL_DEAD,
                 HWEvent::HW_RECOVERY,      HWEvent::HISTOGRAM,
                 HWEvent::BACKLIGHT_EVENT,  HWEvent::POWER_EVENT,
                 HWEvent::MMRM,             HWEvent::VM_RELEASE_EVENT};
  if (hw_panel_info_.mode == kModeCommand) {
    event_list_.push_back(HWEvent::IDLE_POWER_COLLAPSE);
  }
#endif
  event_list_.push_back(HWEvent::POWER_EVENT);
  avr_prop_disabled_ = Debug::IsAVRDisabled();

  error = HWEventsInterface::Create(display_id_, kBuiltIn, this, event_list_, hw_intf_,
                                    &hw_events_intf_);
  if (error != kErrorNone) {
    DisplayBase::Deinit();
    HWInterface::Destroy(hw_intf_);
    DLOGE("Failed to create hardware events interface on. Error = %d", error);
  }

  current_refresh_rate_ = hw_panel_info_.max_fps;

  int value = 0;
  Debug::Get()->GetProperty(ENABLE_HISTOGRAM_INTR, &value);
  if (value == 1) {
    initColorSamplingState();
  }

  value = 0;
  Debug::Get()->GetProperty(DEFER_FPS_FRAME_COUNT, &value);
  deferred_config_.frame_count = (value > 0) ? UINT32(value) : 0;

  if (pf_factory_ && prop_intf_) {
    // Get status of RC enablement property. Default RC is disabled.
    int rc_prop_value = 0;
    Debug::GetProperty(ENABLE_ROUNDED_CORNER, &rc_prop_value);
    if (rc_prop_value && hw_panel_info_.is_primary_panel) {
      // TODO(user): Get the RC count from driver and decide if RC can be enabled for
      // sec built-ins.  Currently client sends RC layers only for first builtin.
      rc_enable_prop_ = true;
    }
    DLOGI("RC feature %s.", rc_enable_prop_ ? "enabled" : "disabled");

    if ((error = SetupSPR()) != kErrorNone) {
      DLOGE("SPR Failed to initialize. Error = %d", error);
      DisplayBase::Deinit();
      HWInterface::Destroy(hw_intf_);
      HWEventsInterface::Destroy(hw_events_intf_);
      return error;
    }

    if ((error = HandleSPR()) != kErrorNone) {
      DLOGE("Failed to get SPR status. Error = %d", error);
      DisplayBase::Deinit();
      HWInterface::Destroy(hw_intf_);
      HWEventsInterface::Destroy(hw_events_intf_);
      return error;
    }

    SetupDemuraT0AndTn();
  } else {
    DLOGW("Skipping Panel Feature Setups!");
  }
  value = 0;
  DebugHandler::Get()->GetProperty(DISABLE_DYNAMIC_FPS, &value);
  disable_dyn_fps_ = (value == 1);

  value = 0;
  DebugHandler::Get()->GetProperty(ENABLE_QSYNC_IDLE, &value);
  enable_qsync_idle_ = hw_panel_info_.qsync_support && (value == 1);
  if (enable_qsync_idle_) {
    DLOGI("Enabling qsync on idling");

    if (hw_panel_info_.transfer_time_us_min) {
      DLOGI("Setting transfer time to min: %d", hw_panel_info_.transfer_time_us_min);
      UpdateTransferTime(hw_panel_info_.transfer_time_us_min);
    }
  }

  value = 0;
  DebugHandler::Get()->GetProperty(ENHANCE_IDLE_TIME, &value);
  enhance_idle_time_ = (value == 1);

  value = 0;
  DebugHandler::Get()->GetProperty(ENABLE_DPPS_DYNAMIC_FPS, &value);
  enable_dpps_dyn_fps_ = (value == 1);

  value = 0;
  Debug::Get()->GetProperty(DISABLE_NOISE_LAYER, &value);
  noise_disable_prop_ = (value == 1);
  DLOGI("Noise Layer Feature is %s for display = %d-%d", noise_disable_prop_ ? "Disabled" :
        "Enabled", display_id_, display_type_);

  value = 0;
  DebugHandler::Get()->GetProperty(DISABLE_CWB_IDLE_FALLBACK, &value);
  disable_cwb_idle_fallback_ = (value == 1);

#ifdef TRUSTED_VM
  disable_cwb_idle_fallback_ = 1;
#endif

  value = 0;
  DebugHandler::Get()->GetProperty(FORCE_LM_TO_FB_CONFIG, &value);
  force_lm_to_fb_config_ = (value == 1);

  NoiseInit();
  InitCWBBuffer();

  return error;
}

DisplayError DisplayBuiltIn::Deinit() {
  {
    ClientLock lock(disp_mutex_);

    if (vm_cb_intf_) {
      vm_cb_intf_->Deinit();
      delete vm_cb_intf_;
    }

    dpps_info_.Deinit();

    if (demura_) {
      SetDemuraIntfStatus(false);

      if (demura_->Deinit() != 0) {
        DLOGE("Unable to DeInit Demura on Display %d-%d", display_id_, display_type_);
      }
    }
    if (demuratn_) {
      EnableDemuraTn(false);
      if (demuratn_->Deinit() != 0) {
        DLOGE("Unable to DeInit DemuraTn on Display %d", display_id_);
      }
    }
    demura_dynamic_enabled_ = true;

    DeinitCWBBuffer();
  }
  return DisplayBase::Deinit();
}

DisplayError DisplayBuiltIn::PrePrepare(LayerStack *layer_stack) {
  DTRACE_SCOPED();
  uint32_t new_mixer_width = 0;
  uint32_t new_mixer_height = 0;
  uint32_t display_width = display_attributes_.x_pixels;
  uint32_t display_height = display_attributes_.y_pixels;

  DisplayError error = HandleDemuraLayer(layer_stack);
  if (error != kErrorNone) {
    return error;
  }

  AppendCWBLayer(layer_stack);
  // Do not skip validate if needs update PP features.
  if (color_mgr_) {
    needs_validate_ |= color_mgr_->IsValidateNeeded();
  }

  error = DisplayBase::PrePrepare(layer_stack);
  if (error == kErrorNone || error == kErrorNeedsLutRegen) {
    return error;
  }

  if (NeedsMixerReconfiguration(layer_stack, &new_mixer_width, &new_mixer_height)) {
    error = ReconfigureMixer(new_mixer_width, new_mixer_height);
    if (error != kErrorNone) {
      ReconfigureMixer(display_width, display_height);
    }
  } else {
    if (CanSkipDisplayPrepare(layer_stack)) {
      UpdateQsyncMode();
      return kErrorNone;
    }
  }
  error = ChangeFps();
  lower_fps_ = disp_layer_stack_.info.lower_fps;

  return kErrorNotValidated;
}

DisplayError DisplayBuiltIn::HandleSPR() {
  if (spr_) {
    GenericPayload out;
    uint32_t *enable = nullptr;
    int ret = out.CreatePayload<uint32_t>(enable);
    if (ret) {
      DLOGE("Failed to create the payload. Error:%d", ret);
      validated_ = false;
      return kErrorUndefined;
    }
    ret = spr_->GetParameter(kSPRFeatureEnable, &out);
    if (ret) {
      DLOGE("Failed to get the spr status. Error:%d", ret);
      validated_ = false;
      return kErrorUndefined;
    }
    spr_enable_ = *enable;
  }

  return kErrorNone;
}

DisplayError DisplayBuiltIn::Prepare(LayerStack *layer_stack) {
  DTRACE_SCOPED();
  ClientLock lock(disp_mutex_);

  DisplayError error = PrePrepare(layer_stack);
  if (error == kErrorNone) {
    return kErrorNone;
  }

  if (error == kErrorNeedsLutRegen && (ForceToneMapUpdate(layer_stack) == kErrorNone)) {
    return kErrorNone;
  }

  DTRACE_BEGIN("Reset DispLayerStack");
  // Clean display layer stack for reuse.
  disp_layer_stack_ = DispLayerStack();
  DTRACE_END();

  error = HandleSPR();
  if (error != kErrorNone) {
    return error;
  }

  error = DisplayBase::Prepare(layer_stack);
  if (error != kErrorNone) {
    return error;
  }

  UpdateQsyncMode();

  CacheFrameROI();

  NotifyDppsHdrPresent(layer_stack);

  pending_commit_ = true;

  return kErrorNone;
}

void DisplayBuiltIn::NotifyDppsHdrPresent(LayerStack *layer_stack) {
  if (hdr_present_ != layer_stack->flags.hdr_present) {
    hdr_present_ = layer_stack->flags.hdr_present;
    DLOGV_IF(kTagDisplay, "Notify DPPS hdr_present %d on display %d-%d", hdr_present_,
             display_id_, display_type_);
    DppsNotifyPayload info = {};
    info.is_primary = IsPrimaryDisplay();
    info.payload = &hdr_present_;
    info.payload_size = sizeof(hdr_present_);
    dpps_info_.DppsNotifyOps(kDppsHdrPresentEvent, &info, sizeof(info));
  }
}

void DisplayBuiltIn::CacheFrameROI() {
  left_frame_roi_ = {};
  right_frame_roi_ = {};

  // Cache the Frame ROI.
  if (disp_layer_stack_.info.left_frame_roi.size() &&
      disp_layer_stack_.info.right_frame_roi.size()) {
    left_frame_roi_ = disp_layer_stack_.info.left_frame_roi.at(0);
    right_frame_roi_ = disp_layer_stack_.info.right_frame_roi.at(0);
  }
}

void DisplayBuiltIn::UpdateQsyncMode() {
  if (!hw_panel_info_.qsync_support || avoid_qsync_mode_change_) {
    return;
  }

  QSyncMode mode = kQSyncModeNone;
  if (lower_fps_ && enable_qsync_idle_) {
    // Override to continuous mode upon idling.
    mode = kQSyncModeContinuous;
    DLOGV_IF(kTagDisplay, "Qsync entering continuous mode");
  } else {
    // Set Qsync mode requested by client.
    mode = qsync_mode_;
    DLOGV_IF(kTagDisplay, "Restoring display %d-%d client's qsync mode: %d", display_id_,
             display_type_, mode);
  }

  disp_layer_stack_.info.hw_avr_info.update = (mode != active_qsync_mode_) || needs_avr_update_;
  disp_layer_stack_.info.hw_avr_info.mode = GetAvrMode(mode);

  DLOGV_IF(kTagDisplay, "display %d-%d update: %d mode: %d",
           display_id_, display_type_, disp_layer_stack_.info.hw_avr_info.update, mode);

  // Store active mde.
  active_qsync_mode_ = mode;
}

HWAVRModes DisplayBuiltIn::GetAvrMode(QSyncMode mode) {
  switch (mode) {
     case kQSyncModeNone:
       return kQsyncNone;
     case kQSyncModeContinuous:
       return kContinuousMode;
     case kQsyncModeOneShot:
     case kQsyncModeOneShotContinuous:
       return kOneShotMode;
     default:
       return kQsyncNone;
  }
}

void DisplayBuiltIn::initColorSamplingState() {
  samplingState = SamplingState::Off;
  histogramCtrl.object_type = DRM_MODE_OBJECT_CRTC;
  histogramCtrl.feature_id = sde_drm::DRMDPPSFeatureID::kFeatureAbaHistCtrl;
  histogramCtrl.value = sde_drm::HistModes::kHistDisabled;

  histogramIRQ.object_type = DRM_MODE_OBJECT_CRTC;
  histogramIRQ.feature_id = sde_drm::DRMDPPSFeatureID::kFeatureAbaHistIRQ;
  histogramIRQ.value = sde_drm::HistModes::kHistDisabled;
  histogramSetup = true;
}

DisplayError DisplayBuiltIn::setColorSamplingState(SamplingState state) {
  samplingState = state;
  if (samplingState == SamplingState::On) {
    histogramCtrl.value = sde_drm::HistModes::kHistEnabled;
    histogramIRQ.value = sde_drm::HistModes::kHistEnabled;
    if (hw_panel_info_.mode == kModeCommand) {
      uint32_t pending;
      ControlPartialUpdate(false /* enable */, &pending);
    }
  } else {
    histogramCtrl.value = sde_drm::HistModes::kHistDisabled;
    histogramIRQ.value = sde_drm::HistModes::kHistDisabled;
    if (hw_panel_info_.mode == kModeCommand) {
      uint32_t pending;
      ControlPartialUpdate(true /* enable */, &pending);
    }
  }

  // effectively drmModeAtomicAddProperty for the SDE_DSPP_HIST_CTRL_V1
  return DppsProcessOps(kDppsSetFeature, &histogramCtrl, sizeof(histogramCtrl));
}

DisplayError DisplayBuiltIn::colorSamplingOn() {
  if (!histogramSetup) {
    return kErrorParameters;
  }
  return setColorSamplingState(SamplingState::On);
}

DisplayError DisplayBuiltIn::colorSamplingOff() {
  if (!histogramSetup) {
    return kErrorParameters;
  }
  return setColorSamplingState(SamplingState::Off);
}

DisplayError DisplayBuiltIn::SetupSPR() {
  int spr_prop_value = 0;
  int spr_bypass_prop_value = 0;
  Debug::GetProperty(ENABLE_SPR, &spr_prop_value);
  Debug::GetProperty(ENABLE_SPR_BYPASS, &spr_bypass_prop_value);

  if (spr_prop_value) {
    SPRInputConfig spr_cfg;
    spr_cfg.panel_name = std::string(hw_panel_info_.panel_name);
    spr_cfg.spr_bypassed = (spr_bypass_prop_value) ? true : false;
    spr_ = pf_factory_->CreateSPRIntf(spr_cfg, prop_intf_);

    if (spr_ == nullptr) {
      DLOGE("Failed to create SPR interface");
      return kErrorResources;
    }

    if (spr_->Init() != 0) {
      DLOGE("Failed to initialize SPR");
      return kErrorResources;
    }

    spr_bypassed_ = spr_cfg.spr_bypassed;
    if (color_mgr_) {
      color_mgr_->ColorMgrSetSprIntf(spr_);
    }
  }

  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetupDemura() {
  DemuraInputConfig input_cfg;
  input_cfg.secure_session = false;  // TODO(user): Integrate with secure solution
  std::string brightness_base;
  hw_intf_->GetPanelBrightnessBasePath(&brightness_base);
  input_cfg.brightness_path = brightness_base+"brightness";

  FetchResourceList frl;
  comp_manager_->GetDemuraFetchResources(display_comp_ctx_, &frl);
  for (auto &fr : frl) {
    int i = std::get<1>(fr);  // fetch resource index
    input_cfg.resources.set(i);
  }

#ifdef TRUSTED_VM
  int ret = 0;
  GenericPayload out;
  IPCImportBufOutParams *buf_out_params = nullptr;
  if ((ret = out.CreatePayload<IPCImportBufOutParams>(buf_out_params))) {
    DLOGE("Failed to create output payload error = %d", ret);
    return kErrorUndefined;
  }

  GenericPayload in;
  IPCImportBufInParams *buf_in_params = nullptr;
  if ((ret = in.CreatePayload<IPCImportBufInParams>(buf_in_params))) {
    DLOGE("Failed to create input payload error = %d", ret);
    return kErrorUndefined;
  }
  buf_in_params->req_buf_type = kIpcBufferTypeDemuraHFC;

  if ((ret = ipc_intf_->ProcessOps(kIpcOpsImportBuffers, in, &out))) {
    DLOGE("Failed to kIpcOpsImportBuffers payload error = %d", ret);
    return kErrorUndefined;
  }
  DLOGI("DemuraHFC buffer fd %d size %d", buf_out_params->buffers[0].fd,
    buf_out_params->buffers[0].size);

  if (buf_out_params->buffers[0].fd < 0) {
    DLOGE("HFC buffer import error fd :%d ", buf_out_params->buffers[0].fd);
    return kErrorUndefined;
  }

  input_cfg.secure_hfc_fd = buf_out_params->buffers[0].fd;
  input_cfg.secure_hfc_size = buf_out_params->buffers[0].size;
  input_cfg.panel_id = buf_out_params->buffers[0].panel_id;
  input_cfg.secure_session = true;
  hfc_buffer_fd_ = buf_out_params->buffers[0].fd;
  hfc_buffer_size_ = buf_out_params->buffers[0].size;
#endif
  input_cfg.panel_id = panel_id_;
  DLOGI("panel id %lx\n", input_cfg.panel_id);
  std::unique_ptr<DemuraIntf> demura =
      pf_factory_->CreateDemuraIntf(input_cfg, prop_intf_, buffer_allocator_, spr_);
  if (!demura) {
    DLOGE("Unable to create Demura on Display %d-%d", display_id_, display_type_);
    return kErrorMemory;
  }

  demura_ = std::move(demura);
  if (demura_->Init() != 0) {
    DLOGE("Unable to initialize Demura on Display %d-%d", display_id_, display_type_);
    return kErrorUndefined;
  }

  if (SetupDemuraLayer() != kErrorNone) {
    DLOGE("Unable to setup Demura layer on Display %d-%d", display_id_, display_type_);
    return kErrorUndefined;
  }

  if (SetDemuraIntfStatus(true)) {
    return kErrorUndefined;
  }

  comp_manager_->SetDemuraStatusForDisplay(display_id_, true);
  demura_intended_ = true;
  DLOGI("Enabled Demura Core!");

#ifndef TRUSTED_VM
  GenericPayload pl;
  uint64_t *panel_id_ptr = nullptr;
  int rc = 0;
  if ((rc = pl.CreatePayload<uint64_t>(panel_id_ptr))) {
    DLOGE("Failed to create payload for Paneld, error = %d", rc);
  }
  demura_->GetParameter(kDemuraFeatureParamPanelId, &pl);
  vm_cb_intf_ = new DisplayIPCVmCallbackImpl(buffer_allocator_, ipc_intf_,
      *panel_id_ptr, hfc_buffer_width_, hfc_buffer_height_);
  vm_cb_intf_->Init();
#endif
  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetupDemuraLayer() {
  int ret = 0;
  GenericPayload pl;
  BufferInfo* buffer = nullptr;
  if ((ret = pl.CreatePayload<BufferInfo>(buffer))) {
    DLOGE("Failed to create payload for BufferInfo, error = %d", ret);
    return kErrorResources;
  }
  if ((ret = demura_->GetParameter(kDemuraFeatureParamCorrectionBuffer, &pl))) {
    DLOGE("Failed to get BufferInfo, error = %d", ret);
    return kErrorResources;
  }
#ifndef TRUSTED_VM
  demura_layer_.input_buffer.size = buffer->alloc_buffer_info.size;
  demura_layer_.input_buffer.buffer_id = buffer->alloc_buffer_info.id;
  demura_layer_.input_buffer.format = buffer->alloc_buffer_info.format;
  demura_layer_.input_buffer.width = buffer->alloc_buffer_info.aligned_width;
  demura_layer_.input_buffer.unaligned_width = buffer->alloc_buffer_info.aligned_width;
  demura_layer_.input_buffer.height = buffer->alloc_buffer_info.aligned_height;
  demura_layer_.input_buffer.unaligned_height = buffer->alloc_buffer_info.aligned_height;
  demura_layer_.input_buffer.planes[0].fd = buffer->alloc_buffer_info.fd;
  demura_layer_.input_buffer.planes[0].stride = buffer->alloc_buffer_info.stride;
  hfc_buffer_width_ = buffer->alloc_buffer_info.aligned_width;
  hfc_buffer_height_ = buffer->alloc_buffer_info.aligned_height;
#else
  uint32_t aligned_width = ALIGN(static_cast<int>(buffer->buffer_config.width), 32);
  uint32_t aligned_height = ALIGN(static_cast<int>(buffer->buffer_config.height), 32);
  float bpp = GetBufferFormatBpp(kFormatRGB888);
  uint32_t stride = static_cast<uint32_t>(aligned_width * bpp);
  demura_layer_.input_buffer.size = hfc_buffer_size_;
  demura_layer_.input_buffer.width = aligned_width;
  demura_layer_.input_buffer.unaligned_width = aligned_width;
  demura_layer_.input_buffer.height = aligned_height;
  demura_layer_.input_buffer.unaligned_height = aligned_height;
  demura_layer_.input_buffer.planes[0].fd = hfc_buffer_fd_;
  demura_layer_.input_buffer.planes[0].stride = stride;
#endif
  demura_layer_.input_buffer.planes[0].offset = 0;
  demura_layer_.input_buffer.flags.demura = 1;
  demura_layer_.composition = kCompositionDemura;
  demura_layer_.blending = kBlendingSkip;
  demura_layer_.flags.is_demura = 1;
  // ROI must match input dimensions
  demura_layer_.src_rect.top = 0;
  demura_layer_.src_rect.left = 0;
  demura_layer_.src_rect.right = buffer->buffer_config.width;
  demura_layer_.src_rect.bottom = buffer->buffer_config.height;
  LogI(kTagNone, "Demura src: ", demura_layer_.src_rect);
  demura_layer_.dst_rect.top = 0;
  demura_layer_.dst_rect.left = 0;
  demura_layer_.dst_rect.right = buffer->buffer_config.width;
  demura_layer_.dst_rect.bottom = buffer->buffer_config.height;
  LogI(kTagNone, "Demura dst: ", demura_layer_.dst_rect);
  demura_layer_.buffer_map = std::make_shared<LayerBufferMap>();
  return kErrorNone;
}

void DisplayBuiltIn::PreCommit(LayerStack *layer_stack) {
  uint32_t app_layer_count = disp_layer_stack_.info.app_layer_count;

  // Enabling auto refresh is async and needs to happen before commit ioctl
  if (hw_panel_info_.mode == kModeCommand) {
    bool enable = (app_layer_count == 1) && layer_stack->flags.single_buffered_layer_present;
    bool need_refresh = layer_stack->flags.single_buffered_layer_present && (app_layer_count > 1);

    hw_intf_->SetAutoRefresh(enable);
    if (need_refresh) {
      event_handler_->Refresh();
    }
  }

  if (trigger_mode_debug_ != kFrameTriggerMax) {
    DisplayError error = hw_intf_->SetFrameTrigger(trigger_mode_debug_);
    if (error != kErrorNone) {
      DLOGE("Failed to set frame trigger mode %d, err %d", (int)trigger_mode_debug_, error);
    } else {
      DLOGV_IF(kTagDisplay, "Set frame trigger mode %d on display %d-%d", trigger_mode_debug_,
               display_id_, display_type_);
      trigger_mode_debug_ = kFrameTriggerMax;
    }
  }

  // effectively drmModeAtomicAddProperty for SDE_DSPP_HIST_IRQ_V1
  if (histogramSetup) {
    SetDppsFeatureLocked(&histogramIRQ, sizeof(histogramIRQ));
  }
}

DisplayError DisplayBuiltIn::SetupDemuraT0AndTn() {
  DisplayError error = kErrorNone;
  int ret = 0, value = 0, panel_id_w = 0;
  uint64_t panel_id = 0;
  bool demura_allowed = false, demuratn_allowed = false;

  if (!comp_manager_->GetDemuraStatus()) {
    comp_manager_->FreeDemuraFetchResources(display_id_);
    comp_manager_->SetDemuraStatusForDisplay(display_id_, false);
    return kErrorNone;
  }

  if (IsPrimaryDisplay()) {
    Debug::Get()->GetProperty(DEMURA_PRIMARY_PANEL_OVERRIDE_LOW, &panel_id_w);
    panel_id = static_cast<uint32_t>(panel_id_w);
    Debug::Get()->GetProperty(DEMURA_PRIMARY_PANEL_OVERRIDE_HIGH, &panel_id_w);
    panel_id |= ((static_cast<uint64_t>(panel_id_w)) << 32);
    Debug::Get()->GetProperty(DISABLE_DEMURA_PRIMARY, &value);
    DLOGI("panel overide total value %lx\n", panel_id);
  } else {
    Debug::Get()->GetProperty(DEMURA_SECONDARY_PANEL_OVERRIDE_LOW, &panel_id_w);
    panel_id = static_cast<uint32_t>(panel_id_w);
    Debug::Get()->GetProperty(DEMURA_SECONDARY_PANEL_OVERRIDE_HIGH, &panel_id_w);
    panel_id |= ((static_cast<uint64_t>(panel_id_w)) << 32);
    Debug::Get()->GetProperty(DISABLE_DEMURA_SECONDARY, &value);
    DLOGI("panel overide total value %lx\n", panel_id);
  }

  if (value > 0) {
    comp_manager_->FreeDemuraFetchResources(display_id_);
    comp_manager_->SetDemuraStatusForDisplay(display_id_, false);
    return kErrorNone;
  } else if (value < 0) {
    return kErrorUndefined;
  }

  PanelFeaturePropertyInfo info;
  if (!panel_id) {
    info.prop_ptr = reinterpret_cast<uint64_t>(&panel_id);
    info.prop_id = kPanelFeatureDemuraPanelId;
    ret = prop_intf_->GetPanelFeature(&info);
    if (ret) {
      DLOGE("Failed to get panel id, error = %d", ret);
      return kErrorUndefined;
    }
  }
  panel_id_ = panel_id;
  DLOGI("panel_id 0x%lx", panel_id_);

#if defined SDM_UNIT_TESTING || defined TRUSTED_VM
  demura_allowed = true;
  demuratn_allowed = true;
#else
  if (!feature_license_factory_) {
    DLOGI("Feature license factory is not available");
    return kErrorNone;
  }

  std::shared_ptr<FeatureLicenseIntf> feat_license_intf =
      feature_license_factory_->CreateFeatureLicenseIntf();
  if (!feat_license_intf) {
    feature_license_factory_ = nullptr;
    DLOGE("Failed to create FeatureLicenseIntf");
    return kErrorUndefined;
  }
  ret = feat_license_intf->Init();
  if (ret) {
    DLOGE("Failed to init FeatureLicenseIntf");
    return kErrorUndefined;
  }

  GenericPayload demura_pl, aa_pl, out_pl;
  DemuraValidatePermissionInput *demura_input = nullptr;
  ret = demura_pl.CreatePayload<DemuraValidatePermissionInput>(demura_input);
  if (ret) {
    DLOGE("Failed to create the payload. Error:%d", ret);
    return kErrorUndefined;
  }

  bool *allowed = nullptr;
  ret = out_pl.CreatePayload<bool>(allowed);
  if (ret) {
    DLOGE("Failed to create the payload. Error:%d", ret);
    return kErrorUndefined;
  }

  demura_input->id = kDemura;
  demura_input->panel_id = panel_id_;
  ret = feat_license_intf->ProcessOps(kValidatePermission, demura_pl, &out_pl);
  if (ret) {
    DLOGE("Failed to get the license permission for Demura. Error:%d", ret);
    return kErrorUndefined;
  }
  demura_allowed = *allowed;

  AntiAgingValidatePermissionInput *aa_input = nullptr;
  ret = aa_pl.CreatePayload<AntiAgingValidatePermissionInput>(aa_input);
  if (ret) {
    DLOGE("Failed to create the payload. Error:%d", ret);
    return kErrorUndefined;
  }

  aa_input->id = kAntiAging;
  ret = feat_license_intf->ProcessOps(kValidatePermission, aa_pl, &out_pl);
  if (ret) {
    DLOGE("Failed to get the license permission for Anti-aging. Error:%d", ret);
    return kErrorUndefined;
  }
  demuratn_allowed = *allowed;
#endif

  DLOGI("Demura enable allowed %d, Anti-aging enable allowed %d", demura_allowed, demuratn_allowed);
  if (demura_allowed) {
    error = SetupDemura();
    if (error != kErrorNone) {
      // Non-fatal but not expected, log error
      DLOGE("Demura failed to initialize on display %d-%d, Error = %d", display_id_, display_type_,
            error);
      comp_manager_->FreeDemuraFetchResources(display_id_);
      comp_manager_->SetDemuraStatusForDisplay(display_id_, false);
      if (demura_) {
        SetDemuraIntfStatus(false);
      }
    } else if (demuratn_allowed && demuratn_factory_) {
      error = SetupDemuraTn();
      if (error != kErrorNone) {
        DLOGW("Failed to setup DemuraTn, Error = %d", error);
      }
    }
  }
  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetupDemuraTn() {
  int ret = 0;

  if (!demura_) {
    DLOGI("Demura is not enabled, cannot setup DemuraTn");
    return kErrorNone;
  }

  demuratn_ = demuratn_factory_->CreateDemuraTnCoreUvmIntf(demura_, buffer_allocator_, this);
  if (!demuratn_) {
    demuratn_factory_ = nullptr;
    DLOGE("Failed to create demuraTnCoreUvmIntf");
    return kErrorUndefined;
  }

  ret = demuratn_->Init();
  if (ret) {
    DLOGE("Failed to init demuraTnCoreUvmIntf, ret %d", ret);
    demuratn_factory_ = nullptr;
    demuratn_.reset();
    demuratn_ = nullptr;
    return kErrorUndefined;
  }

  return kErrorNone;
}

DisplayError DisplayBuiltIn::EnableDemuraTn(bool enable) {
  int ret = 0;
  bool *en = nullptr;
  GenericPayload payload;

  if (!demuratn_) {
    DLOGE("demuratn_ is nullptr");
    return kErrorUndefined;
  }

  if (demuratn_enabled_ == enable)
    return kErrorNone;

  ret = payload.CreatePayload(en);
  if (ret) {
    DLOGE("Failed to create enable payload");
    return kErrorUndefined;
  }
  *en = enable;

  if (enable) {  // make sure init is ready before enabling
    DemuraTnCoreState *init_ready = nullptr;
    GenericPayload ready_pl;
    ret = ready_pl.CreatePayload<DemuraTnCoreState>(init_ready);
    if (ret) {
      DLOGE("failed to create the payload. Error:%d", ret);
      return kErrorUndefined;
    }

    ret = demuratn_->GetParameter(kDemuraTnCoreUvmParamInitReady, &ready_pl);
    if (ret) {
      DLOGE("GetParameter for InitReady failed ret %d", ret);
      return kErrorUndefined;
    }
    if (*init_ready == kDemuraTnCoreNotReady) {
      return kErrorNone;
    } else if (*init_ready == kDemuraTnCoreError) {
      DLOGE("DemuraTn init ready state returns error");
      int rc = demuratn_->Deinit();
      if (rc)
        DLOGE("Failed to deinit DemuraTn ret %d", rc);
      demuratn_factory_ = nullptr;
      demuratn_.reset();
      demuratn_ = nullptr;
      return kErrorUndefined;
    } else if (*init_ready == kDemuraTnCoreReady) {
      ret = demuratn_->SetParameter(kDemuraTnCoreUvmParamEnable, payload);
      if (ret) {
        DLOGE("SetParameter for enable failed ret %d", ret);
        return kErrorUndefined;
      }
      demuratn_enabled_ = true;
    }
  } else {
    ret = demuratn_->SetParameter(kDemuraTnCoreUvmParamEnable, payload);
    if (ret) {
      DLOGE("SetParameter for enable failed ret %d", ret);
      return kErrorUndefined;
    }
    demuratn_enabled_ = false;
  }

  return kErrorNone;
}

DisplayError DisplayBuiltIn::RetrieveDemuraTnFiles() {
  int ret = 0;
  GenericPayload payload;

  if (!demuratn_ || !demuratn_enabled_) {
    DLOGE("demuratn_ %pK demuratn_enabled_ %d", demuratn_.get(), demuratn_enabled_);
    return kErrorUndefined;
  }

  ret = demuratn_->SetParameter(kDemuraTnCoreUvmParamRetrieveFiles, payload);
  if (ret) {
    DLOGE("SetParameter for RetrieveFiles failed ret %d", ret);
    return kErrorUndefined;
  }

  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetUpCommit(LayerStack *layer_stack) {
  DTRACE_SCOPED();
  last_panel_mode_ = hw_panel_info_.mode;
  PreCommit(layer_stack);

  return DisplayBase::SetUpCommit(layer_stack);
}

DisplayError DisplayBuiltIn::CommitLocked(LayerStack *layer_stack) {
  DTRACE_SCOPED();
  last_panel_mode_ = hw_panel_info_.mode;
  PreCommit(layer_stack);

  return DisplayBase::CommitLocked(layer_stack);
}

DisplayError DisplayBuiltIn::PostCommit(HWLayersInfo *hw_layers_info) {
  DisplayBase::PostCommit(hw_layers_info);
  // Mutex scope
  {
    lock_guard<recursive_mutex> obj(brightness_lock_);
    if (pending_brightness_) {
      Fence::Wait(retire_fence_);
      SetPanelBrightness(cached_brightness_);
      pending_brightness_ = false;
    }
  }

  if (commit_event_enabled_) {
    dpps_info_.DppsNotifyOps(kDppsCommitEvent, &display_type_, sizeof(display_type_));
  }

  deferred_config_.UpdateDeferCount();

  ReconfigureDisplay();

  if (deferred_config_.CanApplyDeferredState()) {
    validated_ = false;
    deferred_config_.Clear();
  }

  clock_gettime(CLOCK_MONOTONIC, &idle_timer_start_);
  int idle_time_ms = disp_layer_stack_.info.set_idle_time_ms;
  if (idle_time_ms >= 0) {
    hw_intf_->SetIdleTimeoutMs(UINT32(idle_time_ms));
    idle_time_ms_ = idle_time_ms;
  }

  if (switch_to_cmd_) {
    uint32_t pending;
    switch_to_cmd_ = false;
    ControlPartialUpdateLocked(true /* enable */, &pending);
  }

  if (last_panel_mode_ != hw_panel_info_.mode) {
    UpdateDisplayModeParams();
  }

  if (dpps_pu_nofiy_pending_) {
    dpps_pu_nofiy_pending_ = false;
    dpps_pu_lock_.Broadcast();
  }
  dpps_info_.Init(this, hw_panel_info_.panel_name, this);

  if (demuratn_)
    EnableDemuraTn(true);

  HandleQsyncPostCommit();

  handle_idle_timeout_ = false;

  pending_commit_ = false;
  lower_fps_ = false;

  return kErrorNone;
}

void DisplayBuiltIn::HandleQsyncPostCommit() {
  if (qsync_mode_ == kQsyncModeOneShot) {
    // Reset qsync mode.
    SetQSyncMode(kQSyncModeNone);
  } else if (qsync_mode_ == kQsyncModeOneShotContinuous) {
    // No action needed.
  } else if (qsync_mode_ == kQSyncModeContinuous) {
      if (!avoid_qsync_mode_change_) {
        needs_avr_update_ = false;
      } else if (needs_avr_update_) {
        validated_ = false;
        event_handler_->Refresh();
      }
  } else if (qsync_mode_ == kQSyncModeNone) {
    needs_avr_update_ = false;
  }

  avoid_qsync_mode_change_ = false;
  SetVsyncStatus(true /*Re-enable vsync.*/);

  bool notify_idle = enable_qsync_idle_ && (active_qsync_mode_ != kQSyncModeNone) &&
                     handle_idle_timeout_;
  if (notify_idle) {
    event_handler_->HandleEvent(kPostIdleTimeout);
  }

  bool qsync_enabled = (qsync_mode_ != kQSyncModeNone);
  if (qsync_enabled == qsync_enabled_) {
    return;
  }

  QsyncEventData event_data;
  event_data.enabled = qsync_enabled;
  event_data.refresh_rate = display_attributes_.fps;
  hw_intf_->GetQsyncFps(&event_data.qsync_refresh_rate);
  event_handler_->HandleQsyncState(event_data);

  qsync_enabled_ = qsync_enabled;
}

void DisplayBuiltIn::UpdateDisplayModeParams() {
  if (hw_panel_info_.mode == kModeVideo) {
    uint32_t pending = 0;
    ControlPartialUpdateLocked(false /* enable */, &pending);
  } else if (hw_panel_info_.mode == kModeCommand) {
    // Flush idle timeout value currently set.
    comp_manager_->SetIdleTimeoutMs(display_comp_ctx_, 0, 0);
    switch_to_cmd_ = true;
  }
}

DisplayError DisplayBuiltIn::SetDisplayState(DisplayState state, bool teardown,
                                             shared_ptr<Fence> *release_fence) {
  ClientLock lock(disp_mutex_);
  DisplayError error = kErrorNone;
  HWDisplayMode panel_mode = hw_panel_info_.mode;

  if ((state == kStateOn) && deferred_config_.IsDeferredState()) {
    SetDeferredFpsConfig();
  }

  // Must go in NullCommit
  if (demura_intended_ && demura_dynamic_enabled_ &&
      comp_manager_->GetDemuraStatusForDisplay(display_id_) && (state == kStateOff)) {
    comp_manager_->SetDemuraStatusForDisplay(display_id_, false);
    SetDemuraIntfStatus(false);
  }

  error = DisplayBase::SetDisplayState(state, teardown, release_fence);
  if (error != kErrorNone) {
    return error;
  }

  if (secure_event_ == kTUITransitionEnd && state == kStateOff) {
    SetPanelBrightness(cached_brightness_);
    pending_brightness_ = false;
  }

  if (hw_panel_info_.mode != panel_mode) {
    UpdateDisplayModeParams();
  }

  // Set vsync enable state to false, as driver disables vsync during display power off.
  if (state == kStateOff) {
    vsync_enable_ = false;
    if (qsync_mode_ != kQSyncModeNone) {
      needs_avr_update_ = true;
    }
  }

  if (pending_power_state_ != kPowerStateNone) {
    event_handler_->Refresh();
  }

  // Must only happen after NullCommit and get applied in next frame
  if (demura_intended_ && demura_dynamic_enabled_ &&
      !comp_manager_->GetDemuraStatusForDisplay(display_id_) &&
      (state == kStateOn || state == kStateDoze)) {
    comp_manager_->SetDemuraStatusForDisplay(display_id_, true);
    SetDemuraIntfStatus(true);
  }

  return kErrorNone;
}

void DisplayBuiltIn::SetIdleTimeoutMs(uint32_t active_ms, uint32_t inactive_ms) {
  ClientLock lock(disp_mutex_);
  comp_manager_->SetIdleTimeoutMs(display_comp_ctx_, active_ms, inactive_ms);
  validated_ = false;
  handle_idle_timeout_ = false;
}

DisplayError DisplayBuiltIn::SetDisplayMode(uint32_t mode) {
  DisplayError error = kErrorNone;

  // Limit scope of mutex to this block
  {
    ClientLock lock(disp_mutex_);
    HWDisplayMode hw_display_mode = static_cast<HWDisplayMode>(mode);
    uint32_t pending = 0;

    if (!active_) {
      DLOGW("Invalid display state = %d. Panel must be on.", state_);
      return kErrorNotSupported;
    }

    if (hw_display_mode != kModeCommand && hw_display_mode != kModeVideo) {
      DLOGW("Invalid panel mode parameters on display %d-%d. Requested = %d",
            display_id_, display_type_, hw_display_mode);
      return kErrorParameters;
    }

    if (hw_display_mode == hw_panel_info_.mode) {
      DLOGW("Same display mode requested on display %d-%d. Current = %d, Requested = %d",
            display_id_, display_type_, hw_panel_info_.mode, hw_display_mode);
      return kErrorNone;
    }

    error = hw_intf_->SetDisplayMode(hw_display_mode);
    if (error != kErrorNone) {
      DLOGW("Retaining current display mode on display %d-%d. Current = %d, Requested = %d",
            display_id_, display_type_, hw_panel_info_.mode, hw_display_mode);
      return error;
    }

    avoid_qsync_mode_change_ = true;
    DisplayBase::ReconfigureDisplay();

    if (mode == kModeVideo) {
      ControlPartialUpdateLocked(false /* enable */, &pending);
      uint32_t active_ms = 0;
      uint32_t inactive_ms = 0;
      Debug::GetIdleTimeoutMs(&active_ms, &inactive_ms);
      comp_manager_->SetIdleTimeoutMs(display_comp_ctx_, active_ms, inactive_ms);
    } else if (mode == kModeCommand) {
      // Flush idle timeout value currently set.
      comp_manager_->SetIdleTimeoutMs(display_comp_ctx_, 0, 0);
      switch_to_cmd_ = true;
    }
  }

  // Request for a new draw cycle. New display mode will get applied on next draw cycle.
  // New idle time will get configured as part of this.
  event_handler_->Refresh();

  return error;
}

DisplayError DisplayBuiltIn::SetPanelBrightness(float brightness) {
  lock_guard<recursive_mutex> obj(brightness_lock_);

  if (brightness != -1.0f && !(0.0f <= brightness && brightness <= 1.0f)) {
    DLOGE("Bad brightness value = %f", brightness);
    return kErrorParameters;
  }

  // -1.0f = off, 0.0f = min, 1.0f = max
  float level_remainder = 0.0f;
  int level = 0;
  if (brightness == -1.0f) {
    level = 0;
  } else {
    // Node only supports int level, so store the float remainder for accurate GetPanelBrightness
    float max = hw_panel_info_.panel_max_brightness;
    float min = hw_panel_info_.panel_min_brightness;
    if (min >= max) {
      DLOGE("Minimum brightness is greater than or equal to maximum brightness");
      return kErrorDriverData;
    }
    float t = (brightness * (max - min)) + min;
    level = static_cast<int>(t);
    level_remainder = t - level;
  }

  DisplayError err = hw_intf_->SetPanelBrightness(level);
  if (err == kErrorNone) {
    level_remainder_ = level_remainder;
    pending_brightness_ = false;
    comp_manager_->SetBacklightLevel(display_comp_ctx_, level);
    DLOGI_IF(kTagDisplay, "Setting brightness to level %d (%f percent)", level,
             brightness * 100);
  } else if (err == kErrorDeferred) {
    // TODO(user): I8508d64a55c3b30239c6ed2886df391407d22f25 causes mismatch between perceived
    // power state and actual panel power state. Requires a rework. Below check will set up
    // deferment of brightness operation if DAL reports defer use case.
    cached_brightness_ = brightness;
    pending_brightness_ = true;
    return kErrorNone;
  }

  return err;
}

DisplayError DisplayBuiltIn::GetRefreshRateRange(uint32_t *min_refresh_rate,
                                                 uint32_t *max_refresh_rate) {
  ClientLock lock(disp_mutex_);
  DisplayError error = kErrorNone;

  if (hw_panel_info_.min_fps && hw_panel_info_.max_fps) {
    *min_refresh_rate = hw_panel_info_.min_fps;
    *max_refresh_rate = hw_panel_info_.max_fps;
  } else {
    error = DisplayBase::GetRefreshRateRange(min_refresh_rate, max_refresh_rate);
  }

  return error;
}

DisplayError DisplayBuiltIn::SetRefreshRate(uint32_t refresh_rate, bool final_rate,
                                            bool idle_screen) {
  ClientLock lock(disp_mutex_);

  if (!active_ || !hw_panel_info_.dynamic_fps || qsync_mode_ != kQSyncModeNone ||
      disable_dyn_fps_) {
    return kErrorNotSupported;
  }

  if (refresh_rate < hw_panel_info_.min_fps || refresh_rate > hw_panel_info_.max_fps) {
    DLOGE("Invalid Fps = %d request", refresh_rate);
    return kErrorParameters;
  }

  if (CanLowerFps(idle_screen) && !final_rate && !enable_qsync_idle_) {
    refresh_rate = hw_panel_info_.min_fps;
  }

  if (current_refresh_rate_ != refresh_rate) {
    DisplayError error = hw_intf_->SetRefreshRate(refresh_rate);
    if (error != kErrorNone) {
      // Attempt to update refresh rate can fail if rf interfenence is detected.
      // Just drop min fps settting for now.
      handle_idle_timeout_ = false;
      return error;
    }

    error = comp_manager_->CheckEnforceSplit(display_comp_ctx_, refresh_rate);
    if (error != kErrorNone) {
      return error;
    }
  }

  // Set safe mode upon success.
  if (enhance_idle_time_ && handle_idle_timeout_ && (refresh_rate == hw_panel_info_.min_fps)) {
    comp_manager_->ProcessIdleTimeout(display_comp_ctx_);
  }

  // On success, set current refresh rate to new refresh rate
  current_refresh_rate_ = refresh_rate;
  deferred_config_.MarkDirty();

  return ReconfigureDisplay();
}

bool DisplayBuiltIn::CanLowerFps(bool idle_screen) {
  if (!enhance_idle_time_) {
    return handle_idle_timeout_;
  }

  if (!handle_idle_timeout_ || !idle_screen) {
    return false;
  }

  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  uint64_t elapsed_time_ms = GetTimeInMs(now) - GetTimeInMs(idle_timer_start_);
  bool can_lower = elapsed_time_ms >= UINT32(idle_time_ms_);
  DLOGV_IF(kTagDisplay, "lower fps: %d", can_lower);

  return can_lower;
}

DisplayError DisplayBuiltIn::VSync(int64_t timestamp) {
  DTRACE_SCOPED();
  bool qsync_enabled = enable_qsync_idle_ && (active_qsync_mode_ != kQSyncModeNone);
  // Client isn't aware of underlying qsync mode.
  // Disable vsync propagation as long as qsync is enabled.
  bool propagate_vsync = vsync_enable_ && !drop_hw_vsync_ && !qsync_enabled;
  if (!propagate_vsync) {
    // Re enable when display updates.
    SetVsyncStatus(false /*Disable vsync events.*/);
    return kErrorNone;
  }

  DisplayEventVSync vsync;
  vsync.timestamp = timestamp;
  event_handler_->VSync(vsync);

  return kErrorNone;
}

void DisplayBuiltIn::SetVsyncStatus(bool enable) {
  string trace_name = enable ? "enable" : "disable";
  DTRACE_BEGIN(trace_name.c_str());
  if (enable) {
    // Enable if vsync is still enabled.
    hw_events_intf_->SetEventState(HWEvent::VSYNC, vsync_enable_);
    pending_vsync_enable_ = false;
  } else {
    hw_events_intf_->SetEventState(HWEvent::VSYNC, false);
    pending_vsync_enable_ = true;
  }
  DTRACE_END();
}

void DisplayBuiltIn::IdleTimeout() {
  DTRACE_SCOPED();
  if (state_ == kStateOff || hw_panel_info_.mode != kModeVideo) {
    return;
  }

  if (pending_commit_) {
    return;
  }

  handle_idle_timeout_ = true;
  event_handler_->Refresh();
  if (!enhance_idle_time_) {
    comp_manager_->ProcessIdleTimeout(display_comp_ctx_);
  }
  validated_ = false;
}

void DisplayBuiltIn::PingPongTimeout() {
  ClientLock lock(disp_mutex_);
  hw_intf_->DumpDebugData();
}

void DisplayBuiltIn::IdlePowerCollapse() {
  if (hw_panel_info_.mode == kModeCommand) {
    ClientLock lock(disp_mutex_);
    validated_ = false;
    comp_manager_->ProcessIdlePowerCollapse(display_comp_ctx_);
    event_handler_->HandleEvent(kIdleTimeout);
  }
}

DisplayError DisplayBuiltIn::ClearLUTs() {
  validated_ = false;
  comp_manager_->ProcessIdlePowerCollapse(display_comp_ctx_);
  return kErrorNone;
}

void DisplayBuiltIn::MMRMEvent(uint32_t clk) {
  DTRACE_SCOPED();
  DisplayBase::MMRMEvent(clk);
}

void DisplayBuiltIn::PanelDead() {
  {
    ClientLock lock(disp_mutex_);
    reset_panel_ = true;
    validated_ = false;
  }
  event_handler_->HandleEvent(kPanelDeadEvent);
  event_handler_->Refresh();
}

// HWEventHandler overload, not DisplayBase
void DisplayBuiltIn::HwRecovery(const HWRecoveryEvent sdm_event_code) {
  DisplayBase::HwRecovery(sdm_event_code);
}

void DisplayBuiltIn::Histogram(int histogram_fd, uint32_t blob_id) {
  event_handler_->HistogramEvent(histogram_fd, blob_id);
}

void DisplayBuiltIn::HandleBacklightEvent(float brightness_level) {
  DLOGI("backlight event occurred %f ipc_intf %p", brightness_level, ipc_intf_.get());
  if (ipc_intf_) {
    GenericPayload in;
    IPCBacklightParams *backlight_params = nullptr;
    int ret = in.CreatePayload<IPCBacklightParams>(backlight_params);
    if (ret) {
      DLOGW("failed to create the payload. Error:%d", ret);
      return;
    }
    float brightness = 0.0f;
    if (GetPanelBrightnessFromLevel(brightness_level, &brightness) != kErrorNone) {
      return;
    }
    backlight_params->brightness = brightness;
    backlight_params->is_primary = IsPrimaryDisplayLocked();
    if ((ret = ipc_intf_->SetParameter(kIpcParamBacklight, in))) {
      DLOGW("Failed to set backlight, error = %d", ret);
    }
    lock_guard<recursive_mutex> obj(brightness_lock_);
    cached_brightness_ = brightness;
    pending_brightness_ = true;
  }
}

DisplayError DisplayBuiltIn::GetPanelBrightness(float *brightness) {
  lock_guard<recursive_mutex> obj(brightness_lock_);

  DisplayError err = kErrorNone;
  int level = 0;
  if ((err = hw_intf_->GetPanelBrightness(&level)) != kErrorNone) {
    return err;
  }
  return GetPanelBrightnessFromLevel(level, brightness);
}

DisplayError DisplayBuiltIn::GetPanelBrightnessFromLevel(float level, float *brightness) {
  // -1.0f = off, 0.0f = min, 1.0f = max
  float max = hw_panel_info_.panel_max_brightness;
  float min = hw_panel_info_.panel_min_brightness;
  if (level == 0) {
    *brightness = -1.0f;
  } else if ((max > min) && (min <= level && level <= max)) {
    *brightness = (static_cast<float>(level) + level_remainder_ - min) / (max - min);
  } else {
    min >= max ? DLOGE("Minimum brightness is greater than or equal to maximum brightness") :
                 DLOGE("Invalid brightness level %f", level);
    return kErrorDriverData;
  }

  DLOGI_IF(kTagDisplay, "Received level %f (%f percent)", level, *brightness * 100);

  return kErrorNone;
}

DisplayError DisplayBuiltIn::GetPanelMaxBrightness(uint32_t *max_brightness_level) {
  lock_guard<recursive_mutex> obj(brightness_lock_);

  if (!max_brightness_level) {
    DLOGE("Invalid input pointer is null");
    return kErrorParameters;
  }

  *max_brightness_level = static_cast<uint32_t>(hw_panel_info_.panel_max_brightness);

  DLOGI_IF(kTagDisplay, "Get panel max_brightness_level %u", *max_brightness_level);
  return kErrorNone;
}

DisplayError DisplayBuiltIn::ControlPartialUpdate(bool enable, uint32_t *pending) {
  ClientLock lock(disp_mutex_);
  return ControlPartialUpdateLocked(enable, pending);
}

DisplayError DisplayBuiltIn::ControlPartialUpdateLocked(bool enable, uint32_t *pending) {
  if (!pending) {
    return kErrorParameters;
  }

  if (dpps_info_.disable_pu_ && enable) {
    // Nothing to be done.
    DLOGI("partial update is disabled by DPPS for display %d-%d", display_id_, display_type_);
    return kErrorNotSupported;
  }

  *pending = 0;
  if (enable == partial_update_control_) {
    DLOGI("Same state transition is requested.");
    return kErrorNone;
  }
  validated_ = false;
  partial_update_control_ = enable;

  if (!enable) {
    // If the request is to turn off feature, new draw call is required to have
    // the new setting into effect.
    *pending = 1;
  }

  return kErrorNone;
}

DisplayError DisplayBuiltIn::DisablePartialUpdateOneFrame() {
  ClientLock lock(disp_mutex_);
  disable_pu_one_frame_ = true;
  validated_ = false;

  return kErrorNone;
}

DisplayError DisplayBuiltIn::DisablePartialUpdateOneFrameInternal() {
  disable_pu_one_frame_ = true;
  validated_ = false;

  return kErrorNone;
}

DisplayError DisplayBuiltIn::DppsProcessOps(enum DppsOps op, void *payload, size_t size) {
  DisplayError error = kErrorNone;
  uint32_t pending;
  bool enable = false;
  DppsDisplayInfo *info;

  switch (op) {
    case kDppsSetFeature:
      if (!payload) {
        DLOGE("Invalid payload parameter for op %d", op);
        error = kErrorParameters;
        break;
      }
      {
        ClientLock lock(disp_mutex_);
        error = SetDppsFeatureLocked(payload, size);
      }
      break;
    case kDppsGetFeatureInfo:
      if (!payload) {
        DLOGE("Invalid payload parameter for op %d", op);
        error = kErrorParameters;
        break;
      }
      error = hw_intf_->GetDppsFeatureInfo(payload, size);
      break;
    case kDppsScreenRefresh:
      event_handler_->Refresh();
      break;
    case kDppsPartialUpdate: {
      int ret;
      if (!payload) {
        DLOGE("Invalid payload parameter for op %d", op);
        error = kErrorParameters;
        break;
      }
      enable = *(reinterpret_cast<bool *>(payload));
      dpps_info_.disable_pu_ = !enable;
      ControlPartialUpdate(enable, &pending);
      event_handler_->Refresh();
      {
        ClientLock lock(disp_mutex_);
        validated_ = false;
        dpps_pu_nofiy_pending_ = true;
      }
      ret = dpps_pu_lock_.WaitFinite(kPuTimeOutMs);
      if (ret) {
        DLOGW("failed to %s partial update ret %d", ((enable) ? "enable" : "disable"), ret);
        error = kErrorTimeOut;
      }
      break;
    }
    case kDppsRequestCommit:
      if (!payload) {
        DLOGE("Invalid payload parameter for op %d", op);
        error = kErrorParameters;
        break;
      }
      {
        ClientLock lock(disp_mutex_);
        commit_event_enabled_ = *(reinterpret_cast<bool *>(payload));
      }
      break;
    case kDppsGetDisplayInfo:
      if (!payload) {
        DLOGE("Invalid payload parameter for op %d", op);
        error = kErrorParameters;
        break;
      }
      info = reinterpret_cast<DppsDisplayInfo *>(payload);
      info->width = display_attributes_.x_pixels;
      info->height = display_attributes_.y_pixels;
      info->is_primary = IsPrimaryDisplayLocked();
      info->display_id = display_id_;
      info->display_type = display_type_;
      info->fps = enable_dpps_dyn_fps_ ? display_attributes_.fps : 0;

      error = hw_intf_->GetPanelBrightnessBasePath(&(info->brightness_base_path));
      if (error != kErrorNone) {
        DLOGE("Failed to get brightness base path %d", error);
      }
      break;
    case kDppsSetPccConfig:
      error = color_mgr_->ColorMgrSetLtmPccConfig(payload, size);
      if (error != kErrorNone) {
        DLOGE("Failed to set PCC config to ColorManagerProxy, error %d", error);
      } else {
        ClientLock lock(disp_mutex_);
        validated_ = false;
        DisablePartialUpdateOneFrameInternal();
      }
      break;
    default:
      DLOGE("Invalid input op %d", op);
      error = kErrorParameters;
      break;
  }
  return error;
}

DisplayError DisplayBuiltIn::SetDisplayDppsAdROI(void *payload) {
  ClientLock lock(disp_mutex_);
  DisplayError err = kErrorNone;

  err = hw_intf_->SetDisplayDppsAdROI(payload);
  if (err != kErrorNone)
    DLOGE("Failed to set ad roi config, err %d", err);

  return err;
}

DisplayError DisplayBuiltIn::SetFrameTriggerMode(FrameTriggerMode mode) {
  ClientLock lock(disp_mutex_);
  validated_ = false;
  trigger_mode_debug_ = mode;
  return kErrorNone;
}

DisplayError DisplayBuiltIn::GetStcColorModes(snapdragoncolor::ColorModeList *mode_list) {
  ClientLock lock(disp_mutex_);
  if (!mode_list) {
    return kErrorParameters;
  }

  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  mode_list->list = stc_color_modes_.list;
  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetStcColorMode(const snapdragoncolor::ColorMode &color_mode) {
  ClientLock lock(disp_mutex_);
  if (!color_mgr_) {
    return kErrorNotSupported;
  }
  DisplayError ret = kErrorNone;
  PrimariesTransfer blend_space = {};
  blend_space = GetBlendSpaceFromStcColorMode(color_mode);
  ret = comp_manager_->SetBlendSpace(display_comp_ctx_, blend_space);
  if (ret != kErrorNone) {
    DLOGE("SetBlendSpace failed, ret = %d on display %d-%d", ret, display_id_, display_type_);
  }

  ret = hw_intf_->SetBlendSpace(blend_space);
  if (ret != kErrorNone) {
    DLOGE("Failed to pass blend space, ret = %d on display %d-%d", ret, display_id_,
          display_type_);
  }

  ret = color_mgr_->ColorMgrSetStcMode(color_mode);
  if (ret != kErrorNone) {
    DLOGE("Failed to set stc color mode, ret = %d on display %d-%d", ret,
          display_id_, display_type_);
    return ret;
  }

  current_color_mode_ = color_mode;

  DynamicRangeType dynamic_range = kSdrType;
  if (std::find(color_mode.hw_assets.begin(), color_mode.hw_assets.end(),
                snapdragoncolor::kPbHdrBlob) != color_mode.hw_assets.end()) {
    dynamic_range = kHdrType;
  }
  if ((color_mode.gamut == ColorPrimaries_BT2020 && color_mode.gamma == Transfer_SMPTE_ST2084) ||
      (color_mode.gamut == ColorPrimaries_BT2020 && color_mode.gamma == Transfer_HLG)) {
    dynamic_range = kHdrType;
  }
  comp_manager_->ControlDpps(dynamic_range != kHdrType);

  return ret;
}

DisplayError DisplayBuiltIn::NotifyDisplayCalibrationMode(bool in_calibration) {
  ClientLock lock(disp_mutex_);
  if (!color_mgr_) {
    return kErrorNotSupported;
  }
  DisplayError ret = kErrorNone;
  ret = color_mgr_->NotifyDisplayCalibrationMode(in_calibration);
  if (ret != kErrorNone) {
    DLOGE("Failed to notify QDCM Mode status, ret = %d state = %d", ret, in_calibration);
  }

  return ret;
}

std::string DisplayBuiltIn::Dump() {
  ClientLock lock(disp_mutex_);
  HWDisplayAttributes attrib;
  uint32_t active_index = 0;
  uint32_t num_modes = 0;
  std::ostringstream os;
  char capabilities[16];

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
  os << " Min TransferTime: " << hw_panel_info_.transfer_time_us_min << "us";
  os << " Max TransferTime: " << hw_panel_info_.transfer_time_us_max << "us";
  os << " AllowedModeSwitch: " << hw_panel_info_.allowed_mode_switch;
  os << " PanelModeCaps: ";
  snprintf(capabilities, sizeof(capabilities), "0x%x", hw_panel_info_.panel_mode_caps);
  os << capabilities;
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
  os << " Qsync mode: " << active_qsync_mode_;
  os << std::noboolalpha;

  DynamicRangeType curr_dynamic_range = kSdrType;
  if (std::find(current_color_mode_.hw_assets.begin(), current_color_mode_.hw_assets.end(),
                snapdragoncolor::kPbHdrBlob) != current_color_mode_.hw_assets.end()) {
    curr_dynamic_range = kHdrType;
  }
  os << "\nCurrent Color Mode: gamut " << current_color_mode_.gamut << " gamma "
     << current_color_mode_.gamma << " intent " << current_color_mode_.intent << " Dynamice_range"
     << (curr_dynamic_range == kSdrType ? " SDR" : " HDR");

  uint32_t num_hw_layers = UINT32(disp_layer_stack_.info.hw_layers.size());

  if (num_hw_layers == 0) {
    os << "\nNo hardware layers programmed";
    return os.str();
  }

  std::shared_ptr<LayerBuffer> out_buffer = disp_layer_stack_.info.output_buffer;
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

    const char *comp_type = GetCompositionName(hw_layer.composition);
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

  os << comp_manager_->Dump();
  os << newline << "\n";

  return os.str();
}


DppsInterface* DppsInfo::dpps_intf_ = NULL;
std::vector<int32_t> DppsInfo::display_id_ = {};

void DppsInfo::Init(DppsPropIntf *intf, const std::string &panel_name,
                    DisplayInterface *display_intf) {
  std::lock_guard<std::mutex> guard(lock_);
  int error = 0;

  if (!intf || !display_intf) {
    DLOGE("Invalid intf %pK display_intf %pK", intf, display_intf);
    return;
  }

  DppsDisplayInfo info_payload = {};
  DisplayError ret = intf->DppsProcessOps(kDppsGetDisplayInfo, &info_payload, sizeof(info_payload));
  if (ret != kErrorNone) {
    DLOGE("Get display information failed, ret %d", ret);
    return;
  }

  if (std::find(display_id_.begin(), display_id_.end(), info_payload.display_id)
    != display_id_.end()) {
    return;
  }
  DLOGI("Ready to register display %d-%d ", info_payload.display_id,
        info_payload.display_type);

  if (!dpps_intf_) {
    if (!dpps_impl_lib_.Open(kDppsLib_)) {
      DLOGW("Failed to load Dpps lib %s", kDppsLib_);
      goto exit;
    }

    if (!dpps_impl_lib_.Sym("GetDppsInterface", reinterpret_cast<void **>(&GetDppsInterface))) {
      DLOGE("GetDppsInterface not found!, err %s", dlerror());
      goto exit;
    }

    dpps_intf_ = GetDppsInterface();
    if (!dpps_intf_) {
      DLOGE("Failed to get Dpps Interface!");
      goto exit;
    }
  }
  error = dpps_intf_->Init(intf, panel_name, display_intf);
  if (error) {
    DLOGE("DPPS Interface init failure with err %d", error);
    goto exit;
  }

  display_id_.push_back(info_payload.display_id);
  DLOGI("Registered display %d-%d successfully", info_payload.display_id,
        info_payload.display_type);
  return;

exit:
  Deinit();
  dpps_intf_ = new DppsDummyImpl();
}

void DppsInfo::Deinit() {
  if (dpps_intf_) {
    dpps_intf_->Deinit();
    dpps_intf_ = NULL;
  }
  dpps_impl_lib_.~DynLib();
}

void DppsInfo::DppsNotifyOps(enum DppsNotifyOps op, void *payload, size_t size) {
  int ret = 0;
  if (!dpps_intf_) {
    DLOGW("Dpps intf nullptr");
    return;
  }
  ret = dpps_intf_->DppsNotifyOps(op, payload, size);
  if (ret)
    DLOGE("DppsNotifyOps op %d error %d", op, ret);
}

DisplayError DisplayBuiltIn::GetQSyncMode(QSyncMode *qsync_mode) {
  *qsync_mode = active_qsync_mode_;
  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetQSyncMode(QSyncMode qsync_mode) {
  ClientLock lock(disp_mutex_);

  if (!hw_panel_info_.qsync_support || first_cycle_) {
    DLOGW("Failed: qsync_support: %d first_cycle %d", hw_panel_info_.qsync_support,
          first_cycle_);
    return kErrorNotSupported;
  }

  // force clear qsync mode if set by idle timeout.
  if (qsync_mode_ ==  active_qsync_mode_ && qsync_mode_ == qsync_mode) {
    DLOGW("Qsync mode already set as requested mode: qsync_mode_=%d", qsync_mode_);
    return kErrorNone;
  }

  qsync_mode_ = qsync_mode;
  needs_avr_update_ = true;
  validated_ = false;
  event_handler_->Refresh();

  if (qsync_mode_ == kQSyncModeNone) {
    DLOGI("Qsync mode set to %d successfully, setting transfer time to min: %d", qsync_mode_,
          hw_panel_info_.transfer_time_us_min);
    UpdateTransferTime(hw_panel_info_.transfer_time_us_min);
  } else {
    DLOGI("Qsync mode set to %d successfully, setting transfer time to max: %d", qsync_mode_,
          hw_panel_info_.transfer_time_us_max);
    UpdateTransferTime(hw_panel_info_.transfer_time_us_max);
  }

  return kErrorNone;
}

DisplayError DisplayBuiltIn::ControlIdlePowerCollapse(bool enable, bool synchronous) {
  ClientLock lock(disp_mutex_);
  if (!active_) {
    DLOGW("Invalid display state = %d on display %d-%d. Panel must be on.", state_, display_id_,
          display_type_);
    return kErrorPermission;
  }
  if (hw_panel_info_.mode == kModeVideo) {
    DLOGW("Idle power collapse not supported for video mode panel.");
    return kErrorNotSupported;
  }
  validated_ = false;
  return hw_intf_->ControlIdlePowerCollapse(enable, synchronous);
}

DisplayError DisplayBuiltIn::GetSupportedDSIClock(std::vector<uint64_t> *bitclk_rates) {
  ClientLock lock(disp_mutex_);
  if (!hw_panel_info_.dyn_bitclk_support) {
    return kErrorNotSupported;
  }

  *bitclk_rates = hw_panel_info_.bitclk_rates;
  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetJitterConfig(uint32_t jitter_type, float value, uint32_t time) {
  ClientLock lock(disp_mutex_);
  if (!active_) {
    DLOGW("Invalid display state = %d. Panel must be on.", state_);
    return kErrorNone;
  }

  if (jitter_type > 2 || (value > 10.0f || value < 0.0f)) {
    return kErrorNotSupported;
  }

  DLOGV("Setting jitter configuration; jitter_type: %d, jitter_val: %lf, jitter_time: %d",
        jitter_type, value, time);
  return hw_intf_->SetJitterConfig(jitter_type, value, time);
}

DisplayError DisplayBuiltIn::SetDynamicDSIClock(uint64_t bit_clk_rate) {
  ClientLock lock(disp_mutex_);
  if (!active_) {
    DLOGW("Invalid display state = %d on display %d-%d. Panel must be on.", state_, display_id_,
          display_type_);
    return kErrorNone;
  }

  if (!hw_panel_info_.dyn_bitclk_support) {
    return kErrorNotSupported;
  }

  uint64_t current_clk = 0;
  std::vector<uint64_t> &clk_rates = hw_panel_info_.bitclk_rates;
  GetDynamicDSIClock(&current_clk);
  bool valid = std::find(clk_rates.begin(), clk_rates.end(), bit_clk_rate) != clk_rates.end();
  if (current_clk == bit_clk_rate || !valid) {
    DLOGI("Invalid setting %d, Clk. already set %d", !valid, (current_clk == bit_clk_rate));
    return kErrorNone;
  }

  validated_ = false;
  avoid_qsync_mode_change_ = true;
  DLOGV("Setting new dynamic bit clk value: %" PRIu64, bit_clk_rate);
  return hw_intf_->SetDynamicDSIClock(bit_clk_rate);
}

DisplayError DisplayBuiltIn::GetDynamicDSIClock(uint64_t *bit_clk_rate) {
  ClientLock lock(disp_mutex_);
  if (!hw_panel_info_.dyn_bitclk_support) {
    return kErrorNotSupported;
  }

  return hw_intf_->GetDynamicDSIClock(bit_clk_rate);
}

DisplayError DisplayBuiltIn::GetRefreshRate(uint32_t *refresh_rate) {
  *refresh_rate = current_refresh_rate_;
  return kErrorNone;
}

DisplayError DisplayBuiltIn::SetBLScale(uint32_t level) {
  ClientLock lock(disp_mutex_);

  DisplayError err = hw_intf_->SetBLScale(level);
  if (err) {
    DLOGE("Failed to set backlight scale to level %d", level);
  } else {
    DLOGI_IF(kTagDisplay, "Setting backlight scale on display %d-%d to level %d", display_id_,
             display_type_, level);
  }
  return err;
}

bool DisplayBuiltIn::CanCompareFrameROI(LayerStack *layer_stack) {
  // Check Display validation and safe-mode states.
  if (needs_validate_ || comp_manager_->IsSafeMode() || layer_stack->needs_validate) {
    return false;
  }

  // Check Panel and Layer Stack attributes.
  int8_t stack_fudge_factor = 1;  // GPU Target Layer always present in input
  if (layer_stack->flags.stitch_present)
    stack_fudge_factor++;
  if (layer_stack->flags.demura_present)
    stack_fudge_factor++;

  if (!hw_panel_info_.partial_update || (hw_panel_info_.left_roi_count != 1) ||
      layer_stack->flags.geometry_changed || layer_stack->flags.skip_present ||
      (layer_stack->layers.size() !=
       (disp_layer_stack_.info.app_layer_count + stack_fudge_factor))) {
    return false;
  }

  // Check for Partial Update disable requests/scenarios.
  if (color_mgr_ && color_mgr_->NeedsPartialUpdateDisable()) {
    DisablePartialUpdateOneFrameInternal();
  }

  if (!partial_update_control_ || disable_pu_one_frame_ || disable_pu_on_dest_scaler_) {
    return false;
  }

  bool surface_damage = false;
  uint32_t surface_damage_mask_value = (1 << kSurfaceDamage);
  for (uint32_t i = 0; i < layer_stack->layers.size(); i++) {
    Layer *layer = layer_stack->layers.at(i);
    if (layer->update_mask.none()) {
      continue;
    }
    // Only kSurfaceDamage bit should be set in layer's update-mask.
    if (layer->update_mask.to_ulong() == surface_damage_mask_value) {
      surface_damage = true;
    } else {
      return false;
    }
  }

  return surface_damage;
}

bool DisplayBuiltIn::CanSkipDisplayPrepare(LayerStack *layer_stack) {
  if (!CanCompareFrameROI(layer_stack)) {
    return false;
  }

  if (disp_layer_stack_.info.iwe_target_index != -1) {
    return false;
  }

  disp_layer_stack_.info.left_frame_roi.clear();
  disp_layer_stack_.info.right_frame_roi.clear();
  disp_layer_stack_.info.dest_scale_info_map.clear();
  comp_manager_->GenerateROI(display_comp_ctx_, &disp_layer_stack_);

  if (!disp_layer_stack_.info.left_frame_roi.size() ||
      !disp_layer_stack_.info.right_frame_roi.size()) {
    return false;
  }

  // Compare the cached and calculated Frame ROIs.
  bool same_roi = IsCongruent(left_frame_roi_, disp_layer_stack_.info.left_frame_roi.at(0)) &&
                  IsCongruent(right_frame_roi_, disp_layer_stack_.info.right_frame_roi.at(0));

  if (same_roi) {
    // Update Surface Damage rectangle(s) in HW layers.
    uint32_t hw_layer_count = UINT32(disp_layer_stack_.info.hw_layers.size());
    for (uint32_t j = 0; j < hw_layer_count; j++) {
      Layer &hw_layer = disp_layer_stack_.info.hw_layers.at(j);
      Layer *sdm_layer = layer_stack->layers.at(disp_layer_stack_.info.index.at(j));
      if (hw_layer.dirty_regions.size() != sdm_layer->dirty_regions.size()) {
        return false;
      }
      for (uint32_t k = 0; k < hw_layer.dirty_regions.size(); k++) {
        hw_layer.dirty_regions.at(k) = sdm_layer->dirty_regions.at(k);
      }
    }

    // Set the composition type for SDM layers.
    size_t size_ff = 1;  // GPU Target Layer always present in input
    if (layer_stack->flags.stitch_present)
      size_ff++;
    if (layer_stack->flags.demura_present)
      size_ff++;
    if (disp_layer_stack_.info.flags.noise_present)
      size_ff++;

    for (uint32_t i = 0; i < (layer_stack->layers.size() - size_ff); i++) {
      layer_stack->layers.at(i)->composition = kCompositionSDE;
    }
  }

  return same_roi;
}

DisplayError DisplayBuiltIn::HandleDemuraLayer(LayerStack *layer_stack) {
  if (!layer_stack) {
    DLOGE("layer_stack is null");
    return kErrorParameters;
  }
  std::vector<Layer *> &layers = layer_stack->layers;
  HWLayersInfo &hw_layers_info = disp_layer_stack_.info;

  if (comp_manager_->GetDemuraStatus() &&
      comp_manager_->GetDemuraStatusForDisplay(display_id_) &&
      demura_layer_.input_buffer.planes[0].fd > 0) {
    if (hw_layers_info.demura_target_index == -1) {
      // If demura layer added for first time, do not skip validate
      needs_validate_ = true;
    }
    layers.push_back(&demura_layer_);
    DLOGI_IF(kTagDisplay, "Demura layer added to layer stack on display %d-%d", display_id_,
             display_type_);
  } else if (hw_layers_info.demura_target_index != -1) {
    // Demura was present last frame but is now disabled
    needs_validate_ = true;
    hw_layers_info.demura_present = false;
    DLOGD_IF(kTagDisplay, "Demura layer to be removed on display %d-%d in this frame",
             display_id_, display_type_);
  }
  return kErrorNone;
}

DisplayError DisplayBuiltIn::UpdateTransferTime(uint32_t transfer_time) {
  DisplayError error = kErrorNone;
  {
    ClientLock lock(disp_mutex_);

    if (!active_) {
      DLOGW("Invalid display state = %d. Panel must be on.", state_);
      return kErrorNotSupported;
    }

    if (transfer_time == hw_panel_info_.transfer_time_us) {
      DLOGW("Same transfer time requested. Current = %d, Requested = %d",
            hw_panel_info_.transfer_time_us, transfer_time);
      return kErrorNone;
    } else if (transfer_time > hw_panel_info_.transfer_time_us_max ||
               transfer_time < hw_panel_info_.transfer_time_us_min) {
      DLOGW(
          "Invalid transfer time requested or panel info missing valid range. Min = %d, Max = %d, "
          "Requested = %d, Current = %d",
          hw_panel_info_.transfer_time_us_min, hw_panel_info_.transfer_time_us_max, transfer_time,
          hw_panel_info_.transfer_time_us);
      return kErrorParameters;
    }

    error = hw_intf_->UpdateTransferTime(transfer_time);
    if (error != kErrorNone) {
      DLOGW("Retaining the older transfer time.");
      return error;
    }

    DLOGV_IF(kTagDisplay, "Updated transfer time to %d", transfer_time);

    DisplayBase::ReconfigureDisplay();
  }

  event_handler_->Refresh();

  return error;
}

DisplayError DisplayBuiltIn::BuildLayerStackStats(LayerStack *layer_stack) {
  std::vector<Layer *> &layers = layer_stack->layers;
  HWLayersInfo &hw_layers_info = disp_layer_stack_.info;
  hw_layers_info.app_layer_count = 0;
  hw_layers_info.gpu_target_index = -1;
  hw_layers_info.stitch_target_index = -1;
  hw_layers_info.demura_target_index = -1;
  hw_layers_info.noise_layer_index = -1;
  hw_layers_info.cwb_target_index = -1;

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
      hw_layers_info.gpu_target_index = index;
    } else if (layer->composition == kCompositionStitchTarget) {
      hw_layers_info.stitch_target_index = index;
      disp_layer_stack_.stack->flags.stitch_present = true;
      hw_layers_info.stitch_present = true;
    } else if (layer->composition == kCompositionDemura) {
      hw_layers_info.demura_target_index = index;
      disp_layer_stack_.stack->flags.demura_present = true;
      hw_layers_info.demura_present = true;
      DLOGD_IF(kTagDisplay, "Display %d-%d shall request Demura in this frame", display_id_,
               display_type_);
    } else if (layer->flags.is_noise) {
      hw_layers_info.flags.noise_present = true;
      hw_layers_info.noise_layer_index = index;
      hw_layers_info.noise_layer_info = noise_layer_info_;
      DLOGV_IF(kTagDisplay, "Display %d-%d requested Noise at index = %d with zpos_n = %d",
               display_id_, display_type_, index, noise_layer_info_.zpos_noise);
    } else if (layer->composition == kCompositionCWBTarget) {
      hw_layers_info.cwb_target_index = index;
      hw_layers_info.cwb_present = true;
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

  DLOGI_IF(kTagDisplay, "LayerStack layer_count: %zu, app_layer_count: %d "
            "gpu_target_index: %d, stitch_index: %d demura_index: %d cwb_target_index: %d "
            "game_present: %d noise_present: %d display: %d-%d", layers.size(),
            hw_layers_info.app_layer_count, hw_layers_info.gpu_target_index,
            hw_layers_info.stitch_target_index, hw_layers_info.demura_target_index,
            hw_layers_info.cwb_target_index, hw_layers_info.game_present,
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

DisplayError DisplayBuiltIn::SetActiveConfig(uint32_t index) {
  deferred_config_.MarkDirty();
  return DisplayBase::SetActiveConfig(index);
}

DisplayError DisplayBuiltIn::ReconfigureDisplay() {
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

  const bool dirty = deferred_config_.IsDirty();
  if (deferred_config_.IsDeferredState()) {
    if (dirty) {
      SetDeferredFpsConfig();
    } else {
      // In Deferred state, use current config for comparison.
      GetFpsConfig(&display_attributes, &hw_panel_info);
    }
  }

  const bool display_unchanged = (display_attributes == display_attributes_);
  const bool mixer_unchanged = (mixer_attributes == mixer_attributes_);
  const bool panel_unchanged = (hw_panel_info == hw_panel_info_);
  if (!dirty && display_unchanged && mixer_unchanged && panel_unchanged) {
    return kErrorNone;
  }

  if (CanDeferFpsConfig(display_attributes.fps)) {
    deferred_config_.Init(display_attributes.fps, display_attributes.vsync_period_ns,
                          hw_panel_info.transfer_time_us);

    // Apply current config until new Fps is deferred.
    GetFpsConfig(&display_attributes, &hw_panel_info);
  }

  error = comp_manager_->ReconfigureDisplay(display_comp_ctx_, display_attributes, hw_panel_info,
                                            mixer_attributes, fb_config_,
                                            &cached_qos_data_);
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

  if (enable_dpps_dyn_fps_) {
    uint32_t dpps_fps = display_attributes_.fps;
    DppsNotifyPayload dpps_payload = {};
    dpps_payload.is_primary = IsPrimaryDisplayLocked();
    dpps_payload.payload = &dpps_fps;
    dpps_payload.payload_size = sizeof(dpps_fps);
    dpps_info_.DppsNotifyOps(kDppsUpdateFpsEvent, &dpps_payload, sizeof(dpps_payload));
  }

  return kErrorNone;
}

bool DisplayBuiltIn::CanDeferFpsConfig(uint32_t fps) {
  if (deferred_config_.CanApplyDeferredState()) {
    // Deferred Fps Config needs to be applied.
    return false;
  }

  // In case of higher to lower Fps transition on a Builtin display, defer the Fps
  // (Transfer time) configuration, for the number of frames based on frame_count.
  return ((deferred_config_.frame_count != 0) && (display_attributes_.fps > fps));
}

void DisplayBuiltIn::SetDeferredFpsConfig() {
  // Update with the deferred Fps Config.
  display_attributes_.fps = deferred_config_.fps;
  display_attributes_.vsync_period_ns = deferred_config_.vsync_period_ns;
  hw_panel_info_.transfer_time_us = deferred_config_.transfer_time_us;
  deferred_config_.Clear();
}

void DisplayBuiltIn::GetFpsConfig(HWDisplayAttributes *display_attr, HWPanelInfo *panel_info) {
  display_attr->fps = display_attributes_.fps;
  display_attr->vsync_period_ns = display_attributes_.vsync_period_ns;
  panel_info->transfer_time_us = hw_panel_info_.transfer_time_us;
}

PrimariesTransfer DisplayBuiltIn::GetBlendSpaceFromStcColorMode(
    const snapdragoncolor::ColorMode &color_mode) {
  PrimariesTransfer blend_space = {};
  if (!color_mgr_) {
    return blend_space;
  }

  // Set sRGB as default blend space.
  bool native_mode = (color_mode.intent == snapdragoncolor::kNative) ||
                     (color_mode.gamut == ColorPrimaries_Max && color_mode.gamma == Transfer_Max);
  if (stc_color_modes_.list.empty() || (native_mode && allow_tonemap_native_)) {
    return blend_space;
  }

  blend_space.primaries = color_mode.gamut;
  blend_space.transfer = color_mode.gamma;

  return blend_space;
}

DisplayError DisplayBuiltIn::GetConfig(DisplayConfigFixedInfo *fixed_info) {
  ClientLock lock(disp_mutex_);
  fixed_info->is_cmdmode = (hw_panel_info_.mode == kModeCommand);

  HWResourceInfo hw_resource_info = HWResourceInfo();
  hw_info_intf_->GetHWResourceInfo(&hw_resource_info);
  bool hdr_plus_supported = false;
  bool dolby_vision_supported = false;

  // Checking library support for HDR10+
  comp_manager_->GetHDRCapability(&hdr_plus_supported, &dolby_vision_supported);

  fixed_info->hdr_supported = hw_resource_info.has_hdr && hw_panel_info_.hdr_enabled;
  // Built-in displays always support HDR10+ when the target supports HDR
  fixed_info->hdr_plus_supported = fixed_info->hdr_supported && hdr_plus_supported;
  fixed_info->dolby_vision_supported = fixed_info->hdr_supported && dolby_vision_supported;
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

void DisplayBuiltIn::SendBacklight() {
  DisplayError err = kErrorNone;
  int level = 0;
  if ((err = hw_intf_->GetPanelBrightness(&level)) != kErrorNone) {
    return;
  }
  HandleBacklightEvent(level);
}

void DisplayBuiltIn::SendDisplayConfigs() {
  if (ipc_intf_) {
    GenericPayload in;
    uint32_t active_index = 0;
    IPCDisplayConfigParams *disp_configs = nullptr;
    int ret = in.CreatePayload<IPCDisplayConfigParams>(disp_configs);
    if (ret) {
      DLOGW("failed to create the payload. Error:%d", ret);
      return;
    }
    DisplayError error = hw_intf_->GetActiveConfig(&active_index);
    if (error != kErrorNone) {
      return;
    }
    disp_configs->h_total = display_attributes_.h_total;
    disp_configs->v_total = display_attributes_.v_total;
    disp_configs->fps = display_attributes_.fps;
    disp_configs->smart_panel = display_attributes_.smart_panel;
    disp_configs->is_primary = IsPrimaryDisplayLocked();
    if ((ret = ipc_intf_->SetParameter(kIpcParamDisplayConfigs, in))) {
      DLOGW("Failed to send display config, error = %d", ret);
    }
  }
}

int DisplayBuiltIn::SetDemuraIntfStatus(bool enable) {
  int ret = 0;
  bool *reconfig = nullptr;
  GenericPayload reconfig_pl;
  if (!demura_) {
    DLOGE("demura_ is nullptr");
    return -EINVAL;
  }

  if (enable) {
    if ((ret = reconfig_pl.CreatePayload<bool>(reconfig))) {
      DLOGE("Failed to create payload for reconfig, error = %d", ret);
      return ret;
    }

    ret = demura_->GetParameter(kDemuraFeatureParamPendingReconfig, &reconfig_pl);
    if (ret) {
      DLOGE("Failed to get reconfig, error %d", ret);
      return ret;
    }
    if (*reconfig) {
      DLOGI("SetDemuraLayer for DemuraTn reconfig");
      ret = SetupDemuraLayer();
      if (ret) {
        DLOGE("Failed to setup Demura layer, error %d", ret);
        return ret;
      }
    }
  }

  GenericPayload pl;
  bool* enable_ptr = nullptr;
  if ((ret = pl.CreatePayload<bool>(enable_ptr))) {
    DLOGE("Failed to create payload for enable, error = %d", ret);
    return ret;
  } else {
    *enable_ptr = enable;
    if ((ret = demura_->SetParameter(kDemuraFeatureParamActive, pl))) {
      DLOGE("Failed to set Active, error = %d", ret);
      return ret;
    }
  }

  if (enable && reconfig && (*reconfig)) {
    *reconfig = false;
    ret = demura_->SetParameter(kDemuraFeatureParamPendingReconfig, reconfig_pl);
    if (ret) {
      DLOGE("Failed to set reconfig, error %d", ret);
      return ret;
    }
  }
  DLOGI("Demura is now %s", enable ? "Enabled" : "Disabled");
  return ret;
}

DisplayError DisplayBuiltIn::SetDppsFeatureLocked(void *payload, size_t size) {
  return hw_intf_->SetDppsFeature(payload, size);
}

void DisplayBuiltIn::HandlePowerEvent() {
  return ProcessPowerEvent();
}

void DisplayBuiltIn::HandleVmReleaseEvent() {
  if (event_handler_)
    event_handler_->HandleEvent(kVmReleaseDone);
}

DisplayError DisplayBuiltIn::GetQsyncFps(uint32_t *qsync_fps) {
  ClientLock lock(disp_mutex_);
  return hw_intf_->GetQsyncFps(qsync_fps);
}

DisplayError DisplayBuiltIn::SetAlternateDisplayConfig(uint32_t *alt_config) {
  ClientLock lock(disp_mutex_);
  if (!alt_config) {
    return kErrorResources;
  }
  DisplayError error = hw_intf_->SetAlternateDisplayConfig(alt_config);

  if (error == kErrorNone) {
    ReconfigureDisplay();
    validated_ = false;
  }

  return error;
}


// LCOV_EXCL_START
DisplayError DisplayBuiltIn::HandleSecureEvent(SecureEvent secure_event, bool *needs_refresh) {
  DisplayError error = kErrorNone;

  error = DisplayBase::HandleSecureEvent(secure_event, needs_refresh);
  if (error) {
    DLOGE("Failed to handle secure event %d", secure_event);
    return error;
  }

  if (secure_event == kTUITransitionEnd) {
    // enable demura after TUI transition end
    if (demura_) {
      SetDemuraIntfStatus(true);
    }
  }

  return error;
}

DisplayError DisplayBuiltIn::PostHandleSecureEvent(SecureEvent secure_event) {
  ClientLock lock(disp_mutex_);
  if (secure_event == kTUITransitionStart) {
    if (vm_cb_intf_) {
      vm_cb_intf_->ExportHFCBuffer();
    }
    if (!pending_brightness_) {
      if (secure_event == kTUITransitionStart) {
        // Send the panel brightness event to secondary VM on TUI session start
        SendBacklight();
      }
    }
    if (secure_event == kTUITransitionStart) {
      // Send display config information to secondary VM on TUI session start
      SendDisplayConfigs();
    }

    if (secure_event == kTUITransitionStart) {
      //  disable demura before TUI transition start
      if (demura_) {
        SetDemuraIntfStatus(false);
      }
    }
  }
  if (secure_event == kTUITransitionEnd) {
    if (vm_cb_intf_) {
      vm_cb_intf_->FreeExportBuffer();
    }
  }
  return kErrorNone;
}

DisplayIPCVmCallbackImpl::DisplayIPCVmCallbackImpl(BufferAllocator *buffer_allocator,
                                                       std::shared_ptr<IPCIntf> ipc_intf,
                                                       uint64_t panel_id, uint32_t width,
                                                       uint32_t height)
  : buffer_allocator_(buffer_allocator),  ipc_intf_(ipc_intf), panel_id_(panel_id),
    hfc_buffer_width_(width), hfc_buffer_height_(height) {}

void DisplayIPCVmCallbackImpl::Init() {
  if (!ipc_intf_) {
    DLOGW("IPC interface is NULL");
    return;
  }
  GenericPayload in_reg;
  DisplayIPCVmCallbackImpl **cb_intf = nullptr;
  int ret = in_reg.CreatePayload<DisplayIPCVmCallbackImpl *>(cb_intf);
  if (ret) {
    DLOGE("failed to create the payload for in_reg. Error:%d", ret);
    return;
  }
  *cb_intf = this;
  GenericPayload out_reg;
  ret = out_reg.CreatePayload<int>(cb_hnd_out_);
  if (ret) {
    DLOGE("failed to create the payload for out_reg. Error:%d", ret);
    return;
  }
  if ((ret = ipc_intf_->ProcessOps(kIpcOpsRegisterVmCallback, in_reg, &out_reg))) {
    DLOGE("Failed to register vm callback, error = %d", ret);
    return;
  }
}
void DisplayIPCVmCallbackImpl::Deinit() {
  if (!ipc_intf_) {
    DLOGW("IPC interface is NULL");
    return;
  }
  GenericPayload in_unreg;
  int *cb_hnd_in = nullptr;
  int ret = in_unreg.CreatePayload<int>(cb_hnd_in);
  if (ret) {
    DLOGE("failed to create the payload for in_unreg. Error:%d", ret);
    return;
  }
  *cb_hnd_in = *cb_hnd_out_;
  if ((ret = ipc_intf_->ProcessOps(kIpcOpsUnRegisterVmCallback, in_unreg, nullptr))) {
    DLOGE("Failed to unregister vm callback, error = %d", ret);
    return;
  }
}
void DisplayIPCVmCallbackImpl::OnServerReady() {
  lock_guard<recursive_mutex> obj(cb_mutex_);
  server_ready_ = true;
}

void DisplayIPCVmCallbackImpl::ExportHFCBuffer() {
  lock_guard<recursive_mutex> obj(cb_mutex_);
  if (!server_ready_) {
    DLOGW("Server not ready, Failed to export HFC buffers");
    return;
  }

  if (!ipc_intf_ || !buffer_allocator_) {
    DLOGE("Invalid parameters ipc_intf_ %p, buffer_allocator_ %p", ipc_intf_.get(),
          buffer_allocator_);
    return;
  }

  buffer_info_hfc_.buffer_config.width = hfc_buffer_width_;
  buffer_info_hfc_.buffer_config.height = hfc_buffer_height_;
  buffer_info_hfc_.buffer_config.format = kFormatRGB888;
  buffer_info_hfc_.buffer_config.buffer_count = 1;
  std::bitset<kBufferPermMax> buf_perm;
  buf_perm.set(kBufferPermRead);
  buf_perm.set(kBufferPermWrite);
  buffer_info_hfc_.buffer_config.access_control.insert(
      std::make_pair(kBufferClientUnTrustedVM, buf_perm));
  buffer_info_hfc_.buffer_config.access_control.insert(
      std::make_pair(kBufferClientTrustedVM, buf_perm));

  int ret = buffer_allocator_->AllocateBuffer(&buffer_info_hfc_);
  if (ret != 0) {
    DLOGE("Fail to allocate hfc buffer");
    return;
  }

  GenericPayload in;
  IPCBufferInfo *export_buf_in_params = nullptr;
  ret = in.CreatePayload<IPCBufferInfo>(export_buf_in_params);
  if (ret) {
    DLOGE("failed to create IPCExportBufInParams payload. Error:%d", ret);
    buffer_allocator_->FreeBuffer(&buffer_info_hfc_);
    return;
  }

  export_buf_in_params->size = buffer_info_hfc_.alloc_buffer_info.size;
  export_buf_in_params->panel_id = panel_id_;
  export_buf_in_params->mem_handle = buffer_info_hfc_.alloc_buffer_info.mem_handle;

  DLOGI("Allocated hfc buffer mem_handle %d size %d panel id :%x", export_buf_in_params->mem_handle,
        export_buf_in_params->size, export_buf_in_params->panel_id);
  if ((ret = ipc_intf_->SetParameter(kIpcParamSetHFCBuffer, in))) {
    DLOGE("Failed to export demura buffers, error = %d", ret);
    buffer_allocator_->FreeBuffer(&buffer_info_hfc_);
    return;
  }
}

void DisplayIPCVmCallbackImpl::FreeExportBuffer() {
  lock_guard<recursive_mutex> obj(cb_mutex_);
  buffer_allocator_->FreeBuffer(&buffer_info_hfc_);
  DLOGI("Free hfc export buffer and fd");
}

void DisplayIPCVmCallbackImpl::OnServerExit() {
  lock_guard<recursive_mutex> obj(cb_mutex_);
  server_ready_ = false;
}
// LCOV_EXCL_STOP

void DisplayBuiltIn::InitCWBBuffer() {
  if (hw_panel_info_.mode != kModeVideo || !hw_resource_info_.has_concurrent_writeback
      || !hw_panel_info_.is_primary_panel) {
    return;
  }

  if (disable_cwb_idle_fallback_ || cwb_buffer_initialized_) {
    return;
  }

  // Initialize CWB buffer with display resolution to get full size buffer
  // as mixer or fb can init with custom values based on property
  output_buffer_info_.buffer_config.width = display_attributes_.x_pixels;
  output_buffer_info_.buffer_config.height = display_attributes_.y_pixels;

  output_buffer_info_.buffer_config.format = kFormatRGBX8888Ubwc;
  output_buffer_info_.buffer_config.buffer_count = 1;
  if (buffer_allocator_->AllocateBuffer(&output_buffer_info_) != 0) {
    DLOGE("Buffer allocation failed");
    return;
  }

  LayerBuffer buffer = {};
  buffer.planes[0].fd = output_buffer_info_.alloc_buffer_info.fd;
  buffer.planes[0].offset = 0;
  buffer.planes[0].stride = output_buffer_info_.alloc_buffer_info.stride;
  buffer.size = output_buffer_info_.alloc_buffer_info.size;
  buffer.handle_id = output_buffer_info_.alloc_buffer_info.id;
  buffer.width = output_buffer_info_.alloc_buffer_info.aligned_width;
  buffer.height = output_buffer_info_.alloc_buffer_info.aligned_height;
  buffer.format = output_buffer_info_.alloc_buffer_info.format;
  buffer.unaligned_width = output_buffer_info_.buffer_config.width;
  buffer.unaligned_height = output_buffer_info_.buffer_config.height;

  cwb_layer_.composition = kCompositionCWBTarget;
  cwb_layer_.input_buffer = buffer;
  cwb_layer_.input_buffer.buffer_id = reinterpret_cast<uint64_t>(output_buffer_info_.private_data);
  cwb_layer_.src_rect = {0, 0, FLOAT(cwb_layer_.input_buffer.unaligned_width),
                         FLOAT(cwb_layer_.input_buffer.unaligned_height)};
  cwb_layer_.dst_rect = {0, 0, FLOAT(cwb_layer_.input_buffer.unaligned_width),
                         FLOAT(cwb_layer_.input_buffer.unaligned_height)};

  cwb_layer_.flags.is_cwb = 1;
  cwb_buffer_initialized_ = true;
  return;
}

void DisplayBuiltIn::DeinitCWBBuffer() {
  if (!cwb_buffer_initialized_) {
    return;
  }

  buffer_allocator_->FreeBuffer(&output_buffer_info_);
  cwb_layer_ = {};
  cwb_buffer_initialized_ = false;
}

void DisplayBuiltIn::AppendCWBLayer(LayerStack *layer_stack) {
  if (cwb_buffer_initialized_ &&
      (cwb_layer_.input_buffer.unaligned_width < display_attributes_.x_pixels ||
      cwb_layer_.input_buffer.unaligned_height < display_attributes_.y_pixels)) {
    DLOGI("Resetting CWB layer due to insufficient buffer size(%dx%d) compare to output(%dx%d).",
          cwb_layer_.input_buffer.unaligned_width, cwb_layer_.input_buffer.unaligned_height,
          display_attributes_.x_pixels, display_attributes_.y_pixels);
    DeinitCWBBuffer();
  }

  if (!cwb_buffer_initialized_) {
    // If CWB buffer is not initialized, then it must be initialized for video mode
    InitCWBBuffer();
  }

  if (!hw_panel_info_.is_primary_panel || disable_cwb_idle_fallback_ ||
      !cwb_buffer_initialized_) {
    return;
  }

  uint32_t new_mixer_width = fb_config_.x_pixels;
  uint32_t new_mixer_height = fb_config_.y_pixels;
  NeedsMixerReconfiguration(layer_stack, &new_mixer_width, &new_mixer_height);
  // Set cwb src_rect same as mixer resolution since LM tappoint
  // and dest_rect equal to fb resolution as strategy scales HWLayer dest rect based on fb
  cwb_layer_.src_rect = {0, 0, FLOAT(new_mixer_width), FLOAT(new_mixer_height)};
  cwb_layer_.dst_rect = {0, 0, FLOAT(fb_config_.x_pixels), FLOAT(fb_config_.y_pixels)};
  cwb_layer_.composition = kCompositionCWBTarget;
  layer_stack->layers.push_back(&cwb_layer_);
}

uint32_t DisplayBuiltIn::GetUpdatingAppLayersCount(LayerStack *layer_stack) {
  uint32_t updating_count = 0;

  for (uint i = 0; i < layer_stack->layers.size(); i++) {
    auto layer = layer_stack->layers.at(i);
    if (layer->composition == kCompositionGPUTarget) {
      break;
    }
    if (layer->flags.updating) {
      updating_count++;
    }
  }

  return updating_count;
}

DisplayError DisplayBuiltIn::ChangeFps() {
  ClientLock lock(disp_mutex_);

  if (!active_ || !hw_panel_info_.dynamic_fps || qsync_mode_ != kQSyncModeNone ||
      disable_dyn_fps_) {
    return kErrorNotSupported;
  }

  uint32_t num_updating_layers = GetUpdatingLayersCount();
  bool one_updating_layer = (num_updating_layers == 1);
  uint32_t refresh_rate = GetOptimalRefreshRate(one_updating_layer);

  if (refresh_rate < hw_panel_info_.min_fps || refresh_rate > hw_panel_info_.max_fps) {
    DLOGE("Invalid Fps = %d request", refresh_rate);
    return kErrorParameters;
  }

  bool idle_screen = GetUpdatingAppLayersCount(disp_layer_stack_.stack) == 0;
  if (!disp_layer_stack_.stack->force_refresh_rate && IdleFallbackLowerFps(idle_screen)
      && !enable_qsync_idle_) {
    refresh_rate = hw_panel_info_.min_fps;
  }

  if (current_refresh_rate_ != refresh_rate) {
    DisplayError error = hw_intf_->SetRefreshRate(refresh_rate);
    if (error != kErrorNone) {
      // Attempt to update refresh rate can fail if rf interference settings is detected.
      // Just drop min fps settting for now.
      if (disp_layer_stack_.info.lower_fps) {
        disp_layer_stack_.info.lower_fps = false;
      }
      return error;
    }

    error = comp_manager_->CheckEnforceSplit(display_comp_ctx_, refresh_rate);
    if (error != kErrorNone) {
      return error;
    }
  }

  // Set safe mode upon success.
  if (enhance_idle_time_ && (refresh_rate == hw_panel_info_.min_fps) &&
      (disp_layer_stack_.info.lower_fps)) {
    comp_manager_->ProcessIdleTimeout(display_comp_ctx_);
  }

  // On success, set current refresh rate to new refresh rate
  current_refresh_rate_ = refresh_rate;
  deferred_config_.MarkDirty();

  return ReconfigureDisplay();
}

bool DisplayBuiltIn::IdleFallbackLowerFps(bool idle_screen) {
  if (!enhance_idle_time_) {
    return (disp_layer_stack_.info.lower_fps);
  }
  if (!idle_screen || !disp_layer_stack_.info.lower_fps) {
    return false;
  }

  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  uint64_t elapsed_time_ms = GetTimeInMs(now) - GetTimeInMs(idle_timer_start_);
  bool can_lower = elapsed_time_ms >= UINT32(idle_time_ms_);
  DLOGV_IF(kTagDisplay, "display %d-%d , lower fps: %d", display_id_, display_type_, can_lower);

  return can_lower;
}

uint32_t DisplayBuiltIn::GetUpdatingLayersCount() {
  uint32_t updating_count = 0;

  for (uint i = 0; i < disp_layer_stack_.stack->layers.size(); i++) {
    auto layer = disp_layer_stack_.stack->layers.at(i);
    if (layer->flags.updating) {
      updating_count++;
    }
  }
  return updating_count;
}

uint32_t DisplayBuiltIn::GetOptimalRefreshRate(bool one_updating_layer) {
  LayerStack *layer_stack = disp_layer_stack_.stack;
  if (layer_stack->force_refresh_rate) {
    return layer_stack->force_refresh_rate;
  }

  uint32_t metadata_refresh_rate = CalculateMetaDataRefreshRate();
  if (layer_stack->flags.use_metadata_refresh_rate && one_updating_layer &&
      metadata_refresh_rate) {
    return metadata_refresh_rate;
  }

  return active_refresh_rate_;
}

uint32_t DisplayBuiltIn::CalculateMetaDataRefreshRate() {
  LayerStack *layer_stack = disp_layer_stack_.stack;
  uint32_t metadata_refresh_rate = 0;
  if (!layer_stack->flags.use_metadata_refresh_rate) {
    return 0;
  }

  uint32_t max_refresh_rate = 0;
  uint32_t min_refresh_rate = 0;
  GetRefreshRateRange(&min_refresh_rate, &max_refresh_rate);

  for (uint i = 0; i < layer_stack->layers.size(); i++) {
    auto layer = layer_stack->layers.at(i);
    if (layer->flags.has_metadata_refresh_rate && layer->frame_rate > metadata_refresh_rate) {
      metadata_refresh_rate = SanitizeRefreshRate(layer->frame_rate, max_refresh_rate,
                                                  min_refresh_rate);
    }
  }
  return metadata_refresh_rate;
}

uint32_t DisplayBuiltIn::SanitizeRefreshRate(uint32_t req_refresh_rate, uint32_t max_refresh_rate,
                                             uint32_t min_refresh_rate) {
  uint32_t refresh_rate = req_refresh_rate;

  if (refresh_rate < min_refresh_rate) {
    // Pick the next multiple of request which is within the range
    refresh_rate = (((min_refresh_rate / refresh_rate) +
                     ((min_refresh_rate % refresh_rate) ? 1 : 0)) * refresh_rate);
  }

  if (refresh_rate > max_refresh_rate) {
    refresh_rate = max_refresh_rate;
  }

  return refresh_rate;
}

DisplayError DisplayBuiltIn::SetDemuraState(int state) {
  int ret = 0;

  if (!demura_intended_) {
    DLOGW("Demura has not enabled");
    return kErrorNone;
  }

  if (state && !comp_manager_->GetDemuraStatusForDisplay(display_id_)) {
    ret = SetDemuraIntfStatus(true);
    if (ret) {
      DLOGE("Failed to set demura status to true, ret = %d", ret);
      return kErrorUndefined;
    }
    comp_manager_->SetDemuraStatusForDisplay(display_id_, true);
    demura_dynamic_enabled_ = true;
  } else if (!state && comp_manager_->GetDemuraStatusForDisplay(display_id_)) {
    ret = SetDemuraIntfStatus(false);
    if (ret) {
      DLOGE("Failed to set demura status to false, ret = %d", ret);
      return kErrorUndefined;
    }
    comp_manager_->SetDemuraStatusForDisplay(display_id_, false);
    demura_dynamic_enabled_ = false;
  }

  // Disable Partial Update for one frame.
  DisablePartialUpdateOneFrameInternal();

  return kErrorNone;
}

}  // namespace sdm
