/*
* Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <algorithm>
#include "display_null.h"

#define __CLASS__ "DisplayNull"

namespace sdm {

DisplayError DisplayNull::Init() {
  default_variable_config_.vsync_period_ns = 16600000;
  default_variable_config_.x_pixels = 1080;
  default_variable_config_.y_pixels = 1920;
  default_variable_config_.x_dpi = 300;
  default_variable_config_.y_dpi = 300;
  default_variable_config_.fps = 60;
  default_variable_config_.is_yuv = false;

  return kErrorNone;
}

DisplayError DisplayNull::GetMixerResolution(uint32_t *width, uint32_t *height) {
  if (!width || !height) {
    return kErrorParameters;
  }

  *width = default_variable_config_.x_pixels;
  *height = default_variable_config_.y_pixels;
  return kErrorNone;
}

DisplayError DisplayNull::GetFrameBufferConfig(DisplayConfigVariableInfo *variable_info) {
  if (!variable_info) {
    return kErrorParameters;
  }

  *variable_info = default_variable_config_;
  return kErrorNone;
}

DisplayError DisplayNull::GetConfig(uint32_t index, DisplayConfigVariableInfo *disp_attr) {
  if (!disp_attr) {
    return kErrorParameters;
  }

  *disp_attr = default_variable_config_;
  return kErrorNone;
}

DisplayError DisplayNull::GetConfig(DisplayConfigFixedInfo *fixed_info) {
  if (!fixed_info) {
    return kErrorParameters;
  }

  *fixed_info = default_fixed_config_;
  return kErrorNone;
}

DisplayError DisplayNull::GetRefreshRateRange(uint32_t *min_refresh_rate,
                                              uint32_t *max_refresh_rate) {
  if (!min_refresh_rate || !max_refresh_rate) {
    return kErrorParameters;
  }

  *min_refresh_rate = 60;
  *max_refresh_rate = 60;
  return kErrorNone;
}

DisplayError DisplayNull::GetActiveConfig(uint32_t *config) {
  if (!config) {
    return kErrorParameters;
  }

  *config = 0;
  return kErrorNone;
}

DisplayError DisplayNull::GetNumVariableInfoConfigs(uint32_t *count) {
  if (!count) {
    return kErrorParameters;
  }

  *count = 1;
  return kErrorNone;
}

DisplayError DisplayNull::Prepare(LayerStack *layer_stack) {
  if (!layer_stack) {
    return kErrorParameters;
  }

  for (auto layer : layer_stack->layers) {
    if (layer->composition != kCompositionGPUTarget) {
      layer->composition = kCompositionGPU;
    }
  }
  return kErrorNone;
}

DisplayError DisplayNull::GetDisplayIdentificationData(uint8_t *out_port, uint32_t *out_data_size,
                                                       uint8_t *out_data) {
  *out_port = 1;  // DSI0 Encoder Index
  if (out_data == nullptr) {
    *out_data_size = (uint32_t)(edid_.size());
  } else {
    *out_data_size = std::min(*out_data_size, (uint32_t)(edid_.size()));
    memcpy(out_data, edid_.data(), *out_data_size);
  }

  return kErrorNone;
}

DisplayError DisplayNullExternal::Commit(LayerStack *layer_stack) {
  if (!layer_stack) {
    return kErrorParameters;
  }

  for (Layer *layer : layer_stack->layers) {
    if (layer->composition != kCompositionGPUTarget) {
      layer->composition = kCompositionSDE;
      layer->input_buffer.release_fence = nullptr;
    }
  }

  return kErrorNone;
}

DisplayError DisplayNullExternal::GetDisplayState(DisplayState *state) {
  if (!state) {
    return kErrorParameters;
  }

  *state = state_;
  return kErrorNone;
}

DisplayError DisplayNullExternal::SetDisplayState(DisplayState state, bool teardown,
                                                  shared_ptr<Fence> *release_fence) {
  state_ = state;
  return kErrorNone;
}

DisplayError DisplayNullExternal::SetFrameBufferConfig(const DisplayConfigVariableInfo
                                                       &variable_info) {
  fb_config_ = variable_info;
  return kErrorNone;
}

DisplayError DisplayNullExternal::GetFrameBufferConfig(DisplayConfigVariableInfo *variable_info) {
  if (!variable_info) {
    return kErrorParameters;
  }

  *variable_info = fb_config_;
  return kErrorNone;
}

DisplayError DisplayNullExternal::GetDisplayIdentificationData(uint8_t *out_port,
                                                               uint32_t *out_data_size,
                                                               uint8_t *out_data) {
  DisplayNull::GetDisplayIdentificationData(out_port, out_data_size, out_data);
  *out_port = 4;  // TMDS Encoder Index

  return kErrorNone;
}

}  // namespace sdm
