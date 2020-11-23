/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

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

#ifndef __HWC_COLOR_MODE_STC_H__
#define __HWC_COLOR_MODE_STC_H__

#include <mutex>
#include <string>
#include <vector>

#include "hwc_display.h"

namespace sdm {

class HWCColorModeStc : public HWCColorMode {
 public:
  explicit HWCColorModeStc(DisplayInterface *display_intf);
  ~HWCColorModeStc() {}
  HWC2::Error Init();
  HWC2::Error DeInit();
  void Dump(std::ostringstream *os);
  uint32_t GetColorModeCount();
  uint32_t GetRenderIntentCount(ColorMode mode);
  HWC2::Error GetColorModes(uint32_t *out_num_modes, ColorMode *out_modes);
  HWC2::Error GetRenderIntents(ColorMode mode, uint32_t *out_num_intents,
                               RenderIntent *out_intents);
  HWC2::Error SetColorTransform(const float *matrix, android_color_transform_t hint);
  HWC2::Error ApplyCurrentColorModeWithRenderIntent(bool hdr_present);
  HWC2::Error CacheColorModeWithRenderIntent(ColorMode mode, RenderIntent intent);
  ColorMode GetCurrentColorMode() { return current_color_mode_; }
  HWC2::Error NotifyDisplayCalibrationMode(bool in_calibration);

 private:
  snapdragoncolor::ColorModeList stc_mode_list_;
  typedef std::map<DynamicRangeType, snapdragoncolor::ColorMode> DynamicRangeMap;
  typedef std::map<RenderIntent, DynamicRangeMap> RenderIntentMap;
  std::map<ColorMode, RenderIntentMap> color_mode_map_ = {};

  void PopulateColorModes();
  int32_t GetStcColorModeFromMap(const ColorMode &mode, const RenderIntent &intent,
                                 const DynamicRangeType &dynamic_range,
                                 snapdragoncolor::ColorMode *out_mode);
};

}  // namespace sdm

#endif  // __HWC_COLOR_MODE_STC_H__
