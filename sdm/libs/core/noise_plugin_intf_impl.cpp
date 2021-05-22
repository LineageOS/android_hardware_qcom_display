/*
 * Copyright (c) 2021 The Linux Foundation. All rights reserved.
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
 *
 */

#include "noise_plugin_intf_impl.h"
#include <private/noise_plugin_dbg.h>

#define __CLASS__ "NoisePlugInIntfImpl"

namespace sdm {

using std::lock_guard;
using std::mutex;

static NoisePlugInFactoryIntfImpl factory_instance;

NoisePlugInFactoryIntf *GetNoisePlugInFactoryIntf() {
  return &factory_instance;
}

std::unique_ptr<NoisePlugInIntf> NoisePlugInFactoryIntfImpl::CreateNoisePlugInIntf(
    uint32_t major_ver, uint32_t minor_ver) {
  lock_guard<mutex> lock(lock_);
  std::unique_ptr<NoisePlugInIntf> intf = nullptr;
  if (major_ver == NOISE_PLUGIN_REVISION_MAJOR && minor_ver == NOISE_PLUGIN_REVISION_MINOR) {
    intf = std::unique_ptr<NoisePlugInIntf>(new NoisePlugInIntfImpl());
    if (!intf) {
      DLOGW("Failed to allocate memory for NoisePlugInIntf\n");
    }
  } else {
    DLOGW("Unsupported NoisePlugIn Version\n");
  }
  return intf;
}

NoisePlugInIntfImpl::NoisePlugInIntfImpl() {
  set_param_func_[kNoisePlugInBackLightMax] = &NoisePlugInIntfImpl::SetBackLightMax;
  set_param_func_[kNoisePlugInMixerStages] = &NoisePlugInIntfImpl::SetMixerStages;
  set_param_func_[kNoisePlugInDisable] = &NoisePlugInIntfImpl::SetDisable;
  set_param_func_[static_cast<NoisePlugInParams>(kNoisePlugInDebugOverride)] =
                 &NoisePlugInIntfImpl::SetDebugOverride;
  set_param_func_[static_cast<NoisePlugInParams>(kNoisePlugInDebugAttn)] =
                 &NoisePlugInIntfImpl::SetDebugAttn;
  set_param_func_[static_cast<NoisePlugInParams>(kNoisePlugInDebugNoiseZpos)] =
                 &NoisePlugInIntfImpl::SetDebugNoiseZpos;
  set_param_func_[static_cast<NoisePlugInParams>(kNoisePlugInDebugBacklightThr)] =
                 &NoisePlugInIntfImpl::SetDebugBacklightThr;
  ops_func_[kOpsRunNoisePlugIn] = &NoisePlugInIntfImpl::RunNoisePlugIn;
}

int NoisePlugInIntfImpl::Init() {
  lock_guard<mutex> lock(lock_);

  enable_ = true;
  attn_ = NOISE_ATTN_DEFAULT;
  noise_zpos_override_ = -1;

  init_done_ = true;
  return 0;
}

int NoisePlugInIntfImpl::Deinit() {
  lock_guard<mutex> lock(lock_);

  if (!init_done_) {
    DLOGI("interface not initialized\n");
    return 0;
  }

  backlight_max_ = -1;
  init_done_ = false;
  return 0;
}

int NoisePlugInIntfImpl::SetParameter(NoisePlugInParams param, const GenericPayload &in) {
  lock_guard<mutex> lock(lock_);

  if (!init_done_) {
    DLOGE("interface not initialized\n");
    return -EINVAL;
  }

  auto it = set_param_func_.find(param);
  if (it == set_param_func_.end()) {
    DLOGE("set param %d is not supported", param);
    return -ENOTSUP;
  }
  SetParam func = it->second;
  if (!func) {
    DLOGE("invalid set param function for parameter %d", param);
    return -EINVAL;
  }

  return (this->*func)(in);
}

int NoisePlugInIntfImpl::GetParameter(NoisePlugInParams param, GenericPayload *out) {
  lock_guard<mutex> lock(lock_);
  (void)(param);
  (void)(out);
  return -ENOTSUP;
}

int NoisePlugInIntfImpl::ProcessOps(NoisePlugInOps op, const GenericPayload &in,
                                    GenericPayload *out) {
  lock_guard<mutex> lock(lock_);

  if (!init_done_) {
    DLOGE("interface not initialized\n");
    return -EINVAL;
  }

  if (!out) {
    DLOGE("invalid arguments for noise process op %d\n", op);
    return -EINVAL;
  }

  auto it = ops_func_.find(op);
  if (it == ops_func_.end()) {
    DLOGE("process op function not supported %d", op);
    return -ENOTSUP;
  }
  Ops func = it->second;
  if (!func) {
    DLOGE("invalid process op function %d", op);
    return -EINVAL;
  }

  return (this->*func)(in, out);
}

int NoisePlugInIntfImpl::SetBackLightMax(const GenericPayload &in) {
  int32_t err = 0;
  uint32_t payload_size = 0;
  int32_t *val = nullptr;

  err = in.GetPayload<int32_t>(val, &payload_size);
  if (err || payload_size != 1 || !val) {
    DLOGE("GetPayload for input failed %d\n", err);
    return -EINVAL;
  }

  // update maximum back light and back light threshold
  if (*val > 0) {
    backlight_max_ = *val;
    backlight_thr_ = (BLT_DEFAULT_PERCENT * backlight_max_) / BLT_MAX_PERCENT;
  } else {
    DLOGE("invalid max backlight\n");
    return -EINVAL;
  }

  return 0;
}

int NoisePlugInIntfImpl::SetMixerStages(const GenericPayload &in) {
  int32_t err = 0;
  uint32_t payload_size = 0;
  int32_t *val = nullptr;

  err = in.GetPayload<int32_t>(val, &payload_size);
  if (err || payload_size != 1 || !val) {
    DLOGE("GetPayload for input failed %d\n", err);
    return -EINVAL;
  }

  // update mixer blend stages.
  if (*val > 0) {
    blend_stages_max_ = *val;
  } else {
    DLOGE("invalid mixer stages\n");
    return -EINVAL;
  }

  return 0;
}

int NoisePlugInIntfImpl::SetDisable(const GenericPayload &in) {
  int32_t err = 0;
  uint32_t payload_size = 0;
  int32_t *val = nullptr;

  err = in.GetPayload<int32_t>(val, &payload_size);
  if (err || payload_size != 1 || !val) {
    DLOGE("GetPayload for input failed %d\n", err);
    return -EINVAL;
  }

  // update noise enable status.
  enable_ = !((*val) == 1);

  return 0;
}

int NoisePlugInIntfImpl::SetDebugOverride(const GenericPayload &in) {
  int32_t err = 0;
  uint32_t payload_size = 0;
  int32_t *val = nullptr;

  err = in.GetPayload<int32_t>(val, &payload_size);
  if (err || payload_size != 1 || !val) {
    DLOGE("GetPayload for input failed %d\n", err);
    return -EINVAL;
  }

  // Reset the override values at each override.
  // Each override paramter will be set seperately.
  override_ = ((*val) == 1);
  attn_ = NOISE_ATTN_DEFAULT;
  noise_zpos_override_ = -1;
  if (backlight_max_ != -1) {
    backlight_thr_ = (BLT_DEFAULT_PERCENT * backlight_max_) / BLT_MAX_PERCENT;
  }

  return 0;
}

int NoisePlugInIntfImpl::SetDebugAttn(const GenericPayload &in) {
  int32_t err = 0;
  uint32_t payload_size = 0;
  int32_t *val = nullptr;

  err = in.GetPayload<int32_t>(val, &payload_size);
  if (err || payload_size != 1 || !val) {
    DLOGE("GetPayload for input failed %d\n", err);
    return -EINVAL;
  }

  // update attenuation override.
  if ((*val > NOISE_ATTN_MIN) && (*val <= NOISE_ATTN_MAX) && override_) {
    attn_ = *val;
  } else {
    DLOGE("invalid attenuation override\n");
    return -EINVAL;
  }

  return 0;
}

int NoisePlugInIntfImpl::SetDebugNoiseZpos(const GenericPayload &in) {
  int32_t err = 0;
  uint32_t payload_size = 0;
  int32_t *val = nullptr;

  err = in.GetPayload<int32_t>(val, &payload_size);
  if (err || payload_size != 1 || !val) {
    DLOGE("GetPayload for input failed %d\n", err);
    return -EINVAL;
  }

  // update z-order override (should be greater than zero
  if ((*val > 0) && (*val < INT32_MAX) && override_) {
    noise_zpos_override_ = *val;
  } else {
    DLOGE("invalid z position override\n");
    return -EINVAL;
  }

  return 0;
}

int NoisePlugInIntfImpl::SetDebugBacklightThr(const GenericPayload &in) {
  int32_t err = 0;
  uint32_t payload_size = 0;
  int32_t *val = nullptr;

  err = in.GetPayload<int32_t>(val, &payload_size);
  if (err || payload_size != 1 || !val) {
    DLOGE("GetPayload for input failed %d\n", err);
    return -EINVAL;
  }

  // update backlight threshold
  if ((*val >= BLT_MIN_PERCENT) && (*val <= BLT_MAX_PERCENT) && (backlight_max_ != -1) &&
       override_) {
    backlight_thr_ = ((*val) * backlight_max_) / BLT_MAX_PERCENT;
  } else {
    DLOGW("invalid backlight threshold override\n");
    return -EINVAL;
  }

  return 0;
}

int NoisePlugInIntfImpl::RunNoisePlugIn(const GenericPayload &in, GenericPayload *out) {
  int32_t err = -1, fod_zpos = -1, mask_zpos = -1;
  uint32_t payload_size = 0;
  NoisePlugInInputParams *input_params = nullptr;
  NoisePlugInOutputParams *output_params = nullptr;

  // get input payload
  err = in.GetPayload<NoisePlugInInputParams>(input_params, &payload_size);
  if (err || (payload_size != 1) || !input_params) {
    DLOGE("GetPayload for input failed %d\n", err);
    return -EINVAL;;
  }

  // get output payload
  err = out->GetPayload<NoisePlugInOutputParams>(output_params, &payload_size);
  if (err || (payload_size != 1) || !output_params) {
    DLOGE("GetPayload for output failed %d\n", err);
    return -EINVAL;
  }

  // return if noise is not enabled
  if (!enable_) {
    output_params->enabled = false;
    return 0;
  }

  // return with overridden z position when zpos override is enabled
  if (override_ && (noise_zpos_override_ > 0)) {
    if ((noise_zpos_override_ >= input_params->layers.size()) ||
        (input_params->layers[noise_zpos_override_].layer_type == kMaskLayer)) {
      DLOGW("invalid z order override %d\n", noise_zpos_override_);
      return -EINVAL;
    }
    output_params->enabled = true;
    output_params->zpos[0] = noise_zpos_override_;
    output_params->zpos[1] = noise_zpos_override_ + 1;
    output_params->attn = attn_;
    return 0;
  }

  // calculate z position as follows:
  // FOD usecase : top most FOD layer when multiple FOD layers are present.
  // GD usecase : beneath mask layers.
  mask_zpos = input_params->layers.size();
  for (uint32_t i = 0; i < input_params->layers.size(); i++) {
    if (input_params->layers[i].layer_type == kFodLayer) {
      if ((int32_t)input_params->layers[i].zorder > fod_zpos) {
        fod_zpos = (int32_t)input_params->layers[i].zorder;
      }
    } else if (input_params->layers[i].layer_type == kMaskLayer) {
      if ((int32_t)input_params->layers[i].zorder < mask_zpos) {
        mask_zpos = (int32_t)input_params->layers[i].zorder;
      }
    }
  }

  // update output parameters
  if (fod_zpos > 0) {
    // FOD layer is present in layer stack
    output_params->enabled = true;
    output_params->zpos[0] = fod_zpos;
    output_params->zpos[1] = fod_zpos + 1;
    output_params->attn = attn_;
  } else if ((backlight_max_ != -1) && (input_params->backlight <= backlight_thr_)) {
    // GD use case is present (max backlight must be set for GD usecase)
    output_params->enabled = true;
    output_params->zpos[0] = mask_zpos;
    output_params->zpos[1] = mask_zpos + 1;
    output_params->attn = attn_;
  } else {
    // FOD, GD usecases are not present
    output_params->enabled = false;
  }

  return 0;
}

}  // namespace sdm
