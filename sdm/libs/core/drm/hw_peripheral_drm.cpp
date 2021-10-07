/*
Copyright (c) 2017-2018, 2020, The Linux Foundation. All rights reserved.

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

#include <fcntl.h>
#include <utils/debug.h>
#include <utils/sys.h>
#include <string>

#include "hw_peripheral_drm.h"

#define __CLASS__ "HWPeripheralDRM"

using sde_drm::DRMDisplayType;
using sde_drm::DRMOps;
using sde_drm::DRMPowerMode;

namespace sdm {

HWPeripheralDRM::HWPeripheralDRM(int32_t display_id, BufferSyncHandler *buffer_sync_handler,
                                 BufferAllocator *buffer_allocator,
                                 HWInfoInterface *hw_info_intf)
  : HWDeviceDRM(buffer_sync_handler, buffer_allocator, hw_info_intf) {
  disp_type_ = DRMDisplayType::PERIPHERAL;
  device_name_ = "Peripheral Display";
  display_id_ = display_id;
}

DisplayError HWPeripheralDRM::Init() {
  DisplayError ret = HWDeviceDRM::Init();
  if (ret != kErrorNone) {
    DLOGE("Init failed for %s", device_name_);
    return ret;
  }

  scalar_data_.resize(hw_resource_.hw_dest_scalar_info.count);
  PopulateBitClkRates();

  return kErrorNone;
}

void HWPeripheralDRM::PopulateBitClkRates() {
  if (!hw_panel_info_.dyn_bitclk_support) {
    return;
  }

  // Group all bit_clk_rates corresponding to DRM_PREFERRED mode.
  uint32_t width = connector_info_.modes[current_mode_index_].mode.hdisplay;
  uint32_t height = connector_info_.modes[current_mode_index_].mode.vdisplay;

  for (auto &mode_info : connector_info_.modes) {
    auto &mode = mode_info.mode;
    if (mode.hdisplay == width && mode.vdisplay == height) {
      bitclk_rates_.push_back(mode_info.bit_clk_rate);
      DLOGI("Possible bit_clk_rates %d", mode_info.bit_clk_rate);
    }
  }

  hw_panel_info_.bitclk_rates = bitclk_rates_;
  DLOGI("bit_clk_rates Size %d", bitclk_rates_.size());
}

DisplayError HWPeripheralDRM::SetDynamicDSIClock(uint64_t bit_clk_rate) {
  bit_clk_rate_ = bit_clk_rate;
  return kErrorNone;
}

DisplayError HWPeripheralDRM::GetDynamicDSIClock(uint64_t *bit_clk_rate) {
  // Update bit_rate corresponding to current refresh rate.
  *bit_clk_rate = (uint32_t)connector_info_.modes[current_mode_index_].bit_clk_rate;
  return kErrorNone;
}

DisplayError HWPeripheralDRM::Validate(HWLayers *hw_layers) {
  HWLayersInfo &hw_layer_info = hw_layers->info;
  SetDestScalarData(hw_layer_info);

  return HWDeviceDRM::Validate(hw_layers);
}

DisplayError HWPeripheralDRM::Commit(HWLayers *hw_layers) {
  HWLayersInfo &hw_layer_info = hw_layers->info;
  SetDestScalarData(hw_layer_info);

  return HWDeviceDRM::Commit(hw_layers);
}

void HWPeripheralDRM::ResetDisplayParams() {
  sde_dest_scalar_data_ = {};
  for (uint32_t j = 0; j < scalar_data_.size(); j++) {
    scalar_data_[j] = {};
  }
}

void HWPeripheralDRM::SetDestScalarData(HWLayersInfo hw_layer_info) {
  if (!hw_resource_.hw_dest_scalar_info.count) {
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

DisplayError HWPeripheralDRM::Flush(HWLayers *hw_layers) {
  DisplayError err = HWDeviceDRM::Flush(hw_layers);
  if (err != kErrorNone) {
    return err;
  }

  ResetDisplayParams();
  return kErrorNone;
}

DisplayError HWPeripheralDRM::SetPanelBrightness(int level) {
  char buffer[kMaxSysfsCommandLength] = {0};

  if (brightness_base_path_.empty()) {
    return kErrorHardware;
  }

  std::string brightness_node(brightness_base_path_ + "brightness");
  int fd = Sys::open_(brightness_node.c_str(), O_RDWR);
  if (fd < 0) {
    DLOGE("Failed to open node = %s, error = %s ", brightness_node.c_str(),
          strerror(errno));
    return kErrorFileDescriptor;
  }

  int32_t bytes = snprintf(buffer, kMaxSysfsCommandLength, "%d\n", level);
  ssize_t ret = Sys::pwrite_(fd, buffer, static_cast<size_t>(bytes), 0);
  if (ret <= 0) {
    DLOGE("Failed to write to node = %s, error = %s ", brightness_node.c_str(),
          strerror(errno));
    Sys::close_(fd);
    return kErrorHardware;
  }

  Sys::close_(fd);

  return kErrorNone;
}

DisplayError HWPeripheralDRM::GetPanelBrightness(int *level) {
  char value[kMaxStringLength] = {0};

  if (!level) {
    DLOGE("Invalid input, null pointer.");
    return kErrorParameters;
  }

  if (brightness_base_path_.empty()) {
    return kErrorHardware;
  }

  std::string brightness_node(brightness_base_path_ + "brightness");
  int fd = Sys::open_(brightness_node.c_str(), O_RDWR);
  if (fd < 0) {
    DLOGE("Failed to open brightness node = %s, error = %s", brightness_node.c_str(),
           strerror(errno));
    return kErrorFileDescriptor;
  }

  if (Sys::pread_(fd, value, sizeof(value), 0) > 0) {
    *level = atoi(value);
  } else {
    DLOGE("Failed to read panel brightness");
    Sys::close_(fd);
    return kErrorHardware;
  }

  Sys::close_(fd);

  return kErrorNone;
}

void HWPeripheralDRM::GetHWPanelMaxBrightness() {
  char value[kMaxStringLength] = {0};
  hw_panel_info_.panel_max_brightness = 255.0f;

  // Panel nodes, driver connector creation, and DSI probing all occur in sync, for each DSI. This
  // means that the connector_type_id - 1 will reflect the same # as the panel # for panel node.
  char s[kMaxStringLength] = {};
  snprintf(s, sizeof(s), "/sys/class/backlight/panel%d-backlight/",
           static_cast<int>(connector_info_.type_id - 1));
  brightness_base_path_.assign(s);

  std::string brightness_node(brightness_base_path_ + "max_brightness");
  int fd = Sys::open_(brightness_node.c_str(), O_RDONLY);
  if (fd < 0) {
    DLOGE("Failed to open max brightness node = %s, error = %s", brightness_node.c_str(),
          strerror(errno));
    return;
  }

  if (Sys::pread_(fd, value, sizeof(value), 0) > 0) {
    hw_panel_info_.panel_max_brightness = static_cast<float>(atof(value));
    DLOGI_IF(kTagDriverConfig, "Max brightness = %f", hw_panel_info_.panel_max_brightness);
  } else {
    DLOGE("Failed to read max brightness. error = %s", strerror(errno));
  }

  Sys::close_(fd);
  return;
}

}  // namespace sdm
