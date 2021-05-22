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

#ifndef __NOISE_PLUGIN_INTF_IMPL_H__
#define __NOISE_PLUGIN_INTF_IMPL_H__

#include <private/noise_plugin_intf.h>
#include <mutex>
#include <map>

namespace sdm {

#define NOISE_PLUGIN_REVISION_MAJOR (1)  // Major version number
#define NOISE_PLUGIN_REVISION_MINOR (0)  // Minor version number
#define NOISE_ATTN_MIN 0                 // Minimum Noise attenuation value
#define NOISE_ATTN_MAX 255               // Maximum Noise attenuation value
#define NOISE_ATTN_DEFAULT 125           // Default Noise attenuation value
#define BLT_MIN_PERCENT 0                // Minimum backlight threshold %
#define BLT_MAX_PERCENT 100              // Maximum backlight threshold %
#define BLT_DEFAULT_PERCENT 10           // Default backlight threshold %

class NoisePlugInIntfImpl;
typedef int (NoisePlugInIntfImpl::*SetParam)(const GenericPayload &in);
typedef int (NoisePlugInIntfImpl::*Ops)(const GenericPayload& in, GenericPayload* out);

class NoisePlugInIntfImpl : public NoisePlugInIntf {
 public:
  NoisePlugInIntfImpl();
  virtual ~NoisePlugInIntfImpl() {}
  virtual int Init();
  virtual int Deinit();
  virtual int SetParameter(NoisePlugInParams param, const GenericPayload &in);
  virtual int GetParameter(NoisePlugInParams param, GenericPayload *out);
  virtual int ProcessOps(NoisePlugInOps op, const GenericPayload &in, GenericPayload *out);

 private:
  std::mutex lock_;
  bool init_done_ = false;
  bool override_ = false;             // flag to enable/disable override
  bool enable_ = false;               // flag to enable/disable noiselayer
  int32_t backlight_max_ = -1;        // maximum BL value (actual value, not percentage)
  int32_t blend_stages_max_ = -1;     // maximum number of blend stages
  int32_t attn_ = 0;                  // noise attenuation factor
  int32_t noise_zpos_override_ = -1;  // noise layer z position (overridden value)
  int32_t backlight_thr_ = 0;         // BL threshold value (actual value, not percentage)
  std::map<NoisePlugInParams, SetParam> set_param_func_;
  std::map<NoisePlugInOps, Ops> ops_func_;

  /*set param handlers */
  int SetBackLightMax(const GenericPayload &in);
  int SetMixerStages(const GenericPayload &in);
  int SetDisable(const GenericPayload &in);
  int SetDebugOverride(const GenericPayload &in);
  int SetDebugAttn(const GenericPayload &in);
  int SetDebugNoiseZpos(const GenericPayload &in);
  int SetDebugBacklightThr(const GenericPayload &in);

  /*Process ops handlers */
  int RunNoisePlugIn(const GenericPayload &in, GenericPayload *out);
};

class NoisePlugInFactoryIntfImpl : public NoisePlugInFactoryIntf {
 private:
  std::mutex lock_;

 public:
  virtual ~NoisePlugInFactoryIntfImpl() {}
  virtual std::unique_ptr<NoisePlugInIntf> CreateNoisePlugInIntf(uint32_t major_ver,
                                                                 uint32_t minor_ver);
};

}  // namespace sdm

#endif  // __NOISE_PLUGIN_INTF_IMPL_H__
