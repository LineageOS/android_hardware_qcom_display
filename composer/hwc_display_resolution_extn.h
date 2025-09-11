/*
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#ifndef __HWC_DISPLAY_RESOLUTION_EXTN_H__
#define __HWC_DISPLAY_RESOLUTION_EXTN_H__

#include <core/sdm_types.h>

#include <utility>
#include <vector>

namespace sdm {

class HWCDisplayResolutionExtn {
 public:
  DisplayError GetExtendedDisplayResolutions(uint32_t panel_width, uint32_t panel_height,
                               std::vector<std::pair<uint32_t, uint32_t>> *extended_disp_res);
  float aspect_ratio_threshold_ = 1.0f;
};

}  // namespace sdm

#endif  // __HWC_DISPLAY_RESOLUTION_EXTN_H__
