/*
Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <utils/debug.h>
#include <vector>

#include "hw_peripheral_drm.h"

#define __CLASS__ "HWPeripheralDRM"

using sde_drm::DRMDisplayType;
using sde_drm::DRMOps;
using sde_drm::DRMPowerMode;
using sde_drm::DppsFeaturePayload;
using sde_drm::DRMDppsFeatureInfo;
using sde_drm::DRMSecureMode;
using sde_drm::DRMCWbCaptureMode;

namespace sdm {

HWPeripheralDRM::HWPeripheralDRM(BufferSyncHandler *buffer_sync_handler,
                                 BufferAllocator *buffer_allocator,
                                 HWInfoInterface *hw_info_intf)
  : HWDeviceDRM(buffer_sync_handler, buffer_allocator, hw_info_intf) {
  disp_type_ = DRMDisplayType::PERIPHERAL;
  device_name_ = "Peripheral";
}

DisplayError HWPeripheralDRM::Init() {
  DisplayError ret = HWDeviceDRM::Init();
  if (ret != kErrorNone) {
    DLOGE("Init failed for %s", device_name_);
    return ret;
  }

  scalar_data_.resize(hw_resource_.hw_dest_scalar_info.count);

  return kErrorNone;
}

DisplayError HWPeripheralDRM::Validate(HWLayers *hw_layers) {
  HWLayersInfo &hw_layer_info = hw_layers->info;
  SetDestScalarData(hw_layer_info);
  SetupConcurrentWriteback(hw_layer_info, true);

  return HWDeviceDRM::Validate(hw_layers);
}

DisplayError HWPeripheralDRM::Commit(HWLayers *hw_layers) {
  HWLayersInfo &hw_layer_info = hw_layers->info;
  SetDestScalarData(hw_layer_info);
  SetupConcurrentWriteback(hw_layer_info, false);

  DisplayError error = HWDeviceDRM::Commit(hw_layers);

  if (cwb_config_.enabled && (error == kErrorNone)) {
    PostCommitConcurrentWriteback(hw_layer_info.stack->output_buffer);
  }

  return error;
}

void HWPeripheralDRM::ResetDisplayParams() {
  sde_dest_scalar_data_ = {};
  for (uint32_t j = 0; j < scalar_data_.size(); j++) {
    scalar_data_[j] = {};
  }
}

void HWPeripheralDRM::SetDestScalarData(HWLayersInfo hw_layer_info) {
  if (!hw_scale_ || !hw_resource_.hw_dest_scalar_info.count) {
    return;
  }

  uint32_t index = 0;
  for (uint32_t i = 0; i < hw_resource_.hw_dest_scalar_info.count; i++) {
    DestScaleInfoMap::iterator it = hw_layer_info.dest_scale_info_map.find(i);

    if (it == hw_layer_info.dest_scale_info_map.end()) {
      continue;
    }

    HWDestScaleInfo *dest_scale_info = it->second;
    SDEScaler *scale = &scalar_data_[index];
    hw_scale_->SetScaler(dest_scale_info->scale_data, scale);
    sde_drm_dest_scaler_cfg *dest_scalar_data = &sde_dest_scalar_data_.ds_cfg[index];
    dest_scalar_data->flags = 0;
    if (scale->scaler_v2.enable) {
      dest_scalar_data->flags |= SDE_DRM_DESTSCALER_ENABLE;
    }
    if (scale->scaler_v2.de.enable) {
      dest_scalar_data->flags |= SDE_DRM_DESTSCALER_ENHANCER_UPDATE;
    }
    if (dest_scale_info->scale_update) {
      dest_scalar_data->flags |= SDE_DRM_DESTSCALER_SCALE_UPDATE;
    }
    dest_scalar_data->index = i;
    dest_scalar_data->lm_width = dest_scale_info->mixer_width;
    dest_scalar_data->lm_height = dest_scale_info->mixer_height;
    dest_scalar_data->scaler_cfg = reinterpret_cast<uint64_t>(&scale->scaler_v2);
    if (hw_panel_info_.partial_update) {
      dest_scalar_data->flags |= SDE_DRM_DESTSCALER_PU_ENABLE;
    }
    index++;
  }
  sde_dest_scalar_data_.num_dest_scaler = UINT32(hw_layer_info.dest_scale_info_map.size());
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_DEST_SCALER_CONFIG, token_.crtc_id,
                            reinterpret_cast<uint64_t>(&sde_dest_scalar_data_));
}

DisplayError HWPeripheralDRM::Flush() {
  DisplayError err = HWDeviceDRM::Flush();
  if (err != kErrorNone) {
    return err;
  }

  ResetDisplayParams();
  return kErrorNone;
}

DisplayError HWPeripheralDRM::SetDppsFeature(void *payload, size_t size) {
  uint32_t obj_id = 0, object_type = 0, feature_id = 0;
  uint64_t value = 0;

  if (size != sizeof(DppsFeaturePayload)) {
    DLOGE("invalid payload size %d, expected %d", size, sizeof(DppsFeaturePayload));
    return kErrorParameters;
  }

  DppsFeaturePayload *feature_payload = reinterpret_cast<DppsFeaturePayload *>(payload);
  object_type = feature_payload->object_type;
  feature_id = feature_payload->feature_id;
  value = feature_payload->value;
  if (object_type == DRM_MODE_OBJECT_CRTC) {
    obj_id = token_.crtc_id;
  } else if (object_type == DRM_MODE_OBJECT_CONNECTOR) {
    obj_id = token_.conn_id;
  } else {
    DLOGE("invalid object type 0x%x", object_type);
    return kErrorUndefined;
  }

  drm_atomic_intf_->Perform(DRMOps::DPPS_CACHE_FEATURE, obj_id, feature_id, value);
  return kErrorNone;
}

DisplayError HWPeripheralDRM::GetDppsFeatureInfo(void *payload, size_t size) {
  if (size != sizeof(DRMDppsFeatureInfo)) {
    DLOGE("invalid payload size %d, expected %d", size, sizeof(DRMDppsFeatureInfo));
    return kErrorParameters;
  }
  DRMDppsFeatureInfo *feature_info = reinterpret_cast<DRMDppsFeatureInfo *>(payload);
  drm_mgr_intf_->GetDppsFeatureInfo(feature_info);
  return kErrorNone;
}

DisplayError HWPeripheralDRM::HandleSecureEvent(SecureEvent secure_event) {
  switch (secure_event) {
    case kSecureDisplayStart: {
      secure_display_active_ = true;
      if (hw_panel_info_.mode != kModeCommand) {
        DisplayError err = Flush();
        if (err != kErrorNone) {
          return err;
        }
      }
    }
    break;

    case kSecureDisplayEnd: {
      if (hw_panel_info_.mode != kModeCommand) {
        DisplayError err = Flush();
        if (err != kErrorNone) {
          return err;
        }
      }
      secure_display_active_ = false;
    }
    break;

    default:
      DLOGE("Invalid secure event %d", secure_event);
      return kErrorNotSupported;
  }

  return kErrorNone;
}

void HWPeripheralDRM::SetupConcurrentWriteback(const HWLayersInfo &hw_layer_info, bool validate) {
  bool enable = hw_resource_.has_concurrent_writeback && hw_layer_info.stack->output_buffer;
  if (!(enable || cwb_config_.enabled)) {
    return;
  }

  bool setup_modes = enable && !cwb_config_.enabled && validate;
  if (setup_modes && (SetupConcurrentWritebackModes() == kErrorNone)) {
    cwb_config_.enabled = true;
  }

  if (cwb_config_.enabled) {
    if (enable) {
      // Set DRM properties for Concurrent Writeback.
      ConfigureConcurrentWriteback(hw_layer_info.stack);
    } else {
      // Tear down the Concurrent Writeback topology.
      drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, cwb_config_.token.conn_id, 0);
    }
  }
}

DisplayError HWPeripheralDRM::SetupConcurrentWritebackModes() {
  // To setup Concurrent Writeback topology, get the Connector ID of Virtual display
  if (drm_mgr_intf_->RegisterDisplay(DRMDisplayType::VIRTUAL, &cwb_config_.token)) {
    DLOGE("RegisterDisplay failed for Concurrent Writeback");
    return kErrorResources;
  }

  // Set the modes based on Primary display.
  std::vector<drmModeModeInfo> modes;
  for (auto &item : connector_info_.modes) {
    modes.push_back(item.mode);
  }

  // Inform the mode list to driver.
  struct sde_drm_wb_cfg cwb_cfg = {};
  cwb_cfg.connector_id = cwb_config_.token.conn_id;
  cwb_cfg.flags = SDE_DRM_WB_CFG_FLAGS_CONNECTED;
  cwb_cfg.count_modes = UINT32(modes.size());
  cwb_cfg.modes = (uint64_t)modes.data();

  int ret = -EINVAL;
#ifdef DRM_IOCTL_SDE_WB_CONFIG
  ret = drmIoctl(dev_fd_, DRM_IOCTL_SDE_WB_CONFIG, &cwb_cfg);
#endif
  if (ret) {
    drm_mgr_intf_->UnregisterDisplay(cwb_config_.token);
    DLOGE("Dump CWBConfig: mode_count %d flags %x", cwb_cfg.count_modes, cwb_cfg.flags);
    DumpConnectorModeInfo();
    return kErrorHardware;
  }

  return kErrorNone;
}

void HWPeripheralDRM::ConfigureConcurrentWriteback(LayerStack *layer_stack) {
  LayerBuffer *output_buffer = layer_stack->output_buffer;
  registry_.MapOutputBufferToFbId(output_buffer);

  // Set the topology for Concurrent Writeback: [CRTC_PRIMARY_DISPLAY - CONNECTOR_VIRTUAL_DISPLAY].
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, cwb_config_.token.conn_id, token_.crtc_id);

  // Set CRTC Capture Mode
  DRMCWbCaptureMode capture_mode = layer_stack->flags.post_processed_output ?
                                   DRMCWbCaptureMode::DSPP_OUT : DRMCWbCaptureMode::MIXER_OUT;
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_CAPTURE_MODE, token_.crtc_id, capture_mode);

  // Set Connector Output FB
  uint32_t fb_id = registry_.GetOutputFbId(output_buffer->handle_id);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_OUTPUT_FB_ID, cwb_config_.token.conn_id, fb_id);

  // Set Connector Secure Mode
  bool secure = output_buffer->flags.secure;
  DRMSecureMode mode = secure ? DRMSecureMode::SECURE : DRMSecureMode::NON_SECURE;
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_FB_SECURE_MODE, cwb_config_.token.conn_id, mode);

  // Set Connector Output Rect
  sde_drm::DRMRect dst = {};
  dst.left = 0;
  dst.top = 0;
  dst.right = display_attributes_[current_mode_index_].x_pixels;
  dst.bottom = display_attributes_[current_mode_index_].y_pixels;
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_OUTPUT_RECT, cwb_config_.token.conn_id, dst);
}

void HWPeripheralDRM::PostCommitConcurrentWriteback(LayerBuffer *output_buffer) {
  bool enabled = hw_resource_.has_concurrent_writeback && output_buffer;

  if (enabled) {
    // Get Concurrent Writeback fence
    int *fence = &output_buffer->release_fence_fd;
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, cwb_config_.token.conn_id, fence);
  } else {
    drm_mgr_intf_->UnregisterDisplay(cwb_config_.token);
    cwb_config_.enabled = false;
  }
}

}  // namespace sdm
