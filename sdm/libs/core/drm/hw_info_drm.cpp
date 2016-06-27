/*
* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/sys.h>
#include <dlfcn.h>
#include <drm/drm_fourcc.h>

#include <algorithm>
#include <iostream>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hw_info_drm.h"

#define __CLASS__ "HWInfoDRM"

using std::vector;
using std::map;
using std::string;
using std::fstream;
using std::to_string;

namespace sdm {

int HWInfoDRM::ParseString(const char *input, char *tokens[], const uint32_t max_token,
                        const char *delim, uint32_t *count) {
  char *tmp_token = NULL;
  char *temp_ptr;
  uint32_t index = 0;
  if (!input) {
    return -1;
  }
  tmp_token = strtok_r(const_cast<char *>(input), delim, &temp_ptr);
  while (tmp_token && index < max_token) {
    tokens[index++] = tmp_token;
    tmp_token = strtok_r(NULL, delim, &temp_ptr);
  }
  *count = index;

  return 0;
}

DisplayError HWInfoDRM::GetDynamicBWLimits(HWResourceInfo *hw_resource) {
  HWDynBwLimitInfo* bw_info = &hw_resource->dyn_bw_info;
  for (int index = 0; index < kBwModeMax; index++) {
    bw_info->total_bw_limit[index] = UINT32(hw_resource->max_bandwidth_low);
    bw_info->pipe_bw_limit[index] = hw_resource->max_pipe_bw;
  }

  return kErrorNone;
}

DisplayError HWInfoDRM::GetHWResourceInfo(HWResourceInfo *hw_resource) {
  InitSupportedFormatMap(hw_resource);
  hw_resource->hw_version = kHWMdssVersion5;
  hw_resource->hw_revision = 268894210;  // HW Rev, v1/v2
  hw_resource->num_blending_stages = 7;
  hw_resource->max_scale_down = 4;
  hw_resource->max_scale_up = 20;
  hw_resource->max_bandwidth_low = 9600000;
  hw_resource->max_bandwidth_high = 9600000;
  hw_resource->max_mixer_width = 2560;
  hw_resource->max_pipe_width = 2560;
  hw_resource->max_cursor_size = 128;
  hw_resource->max_pipe_bw = 4500000;
  hw_resource->max_sde_clk = 412500000;
  hw_resource->clk_fudge_factor = FLOAT(105) / FLOAT(100);
  hw_resource->macrotile_nv12_factor = 8;
  hw_resource->macrotile_factor = 4;
  hw_resource->linear_factor = 1;
  hw_resource->scale_factor = 1;
  hw_resource->extra_fudge_factor = 2;
  hw_resource->amortizable_threshold = 0;
  hw_resource->system_overhead_lines = 0;
  hw_resource->writeback_index = 2;
  hw_resource->hw_dest_scalar_info.count = 0;
  hw_resource->hw_dest_scalar_info.max_scale_up = 0;
  hw_resource->hw_dest_scalar_info.max_input_width = 0;
  hw_resource->hw_dest_scalar_info.max_output_width = 0;
  hw_resource->has_bwc = false;
  hw_resource->has_ubwc = true;
  hw_resource->has_decimation = true;
  hw_resource->has_macrotile = true;
  hw_resource->is_src_split = true;
  hw_resource->has_non_scalar_rgb = false;
  hw_resource->perf_calc = false;
  hw_resource->has_dyn_bw_support = false;
  hw_resource->separate_rotator = true;
  hw_resource->has_qseed3 = false;
  hw_resource->has_concurrent_writeback = false;
  hw_resource->num_vig_pipe = 0;
  hw_resource->num_dma_pipe = 0;
  hw_resource->num_cursor_pipe = 0;
  uint32_t pipe_count = 2;
  for (uint32_t i = 0; i < pipe_count; i++) {
    HWPipeCaps pipe_caps;
    pipe_caps.type = kPipeTypeUnused;
    pipe_caps.type = kPipeTypeRGB;
    hw_resource->num_rgb_pipe++;
    pipe_caps.id = UINT32(0x8 << i);
    pipe_caps.max_rects = 1;
    hw_resource->hw_pipes.push_back(pipe_caps);
  }

  // Disable destination scalar count to 0 if extension library is not present
  DynLib extension_lib;
  if (!extension_lib.Open("libsdmextension.so")) {
    hw_resource->hw_dest_scalar_info.count = 0;
  }

  DLOGI("SDE Version = %d, SDE Revision = %x, RGB = %d, VIG = %d, DMA = %d, Cursor = %d",
        hw_resource->hw_version, hw_resource->hw_revision, hw_resource->num_rgb_pipe,
        hw_resource->num_vig_pipe, hw_resource->num_dma_pipe, hw_resource->num_cursor_pipe);
  DLOGI("Upscale Ratio = %d, Downscale Ratio = %d, Blending Stages = %d", hw_resource->max_scale_up,
        hw_resource->max_scale_down, hw_resource->num_blending_stages);
  DLOGI("SourceSplit = %d QSEED3 = %d", hw_resource->is_src_split, hw_resource->has_qseed3);
  DLOGI("BWC = %d, UBWC = %d, Decimation = %d, Tile Format = %d Concurrent Writeback = %d",
        hw_resource->has_bwc, hw_resource->has_ubwc, hw_resource->has_decimation,
        hw_resource->has_macrotile, hw_resource->has_concurrent_writeback);
  DLOGI("MaxLowBw = %" PRIu64 " , MaxHighBw = % " PRIu64 "", hw_resource->max_bandwidth_low,
        hw_resource->max_bandwidth_high);
  DLOGI("MaxPipeBw = %" PRIu64 " KBps, MaxSDEClock = % " PRIu64 " Hz, ClockFudgeFactor = %f",
        hw_resource->max_pipe_bw, hw_resource->max_sde_clk, hw_resource->clk_fudge_factor);
  DLOGI("Prefill factors: Tiled_NV12 = %d, Tiled = %d, Linear = %d, Scale = %d, Fudge_factor = %d",
        hw_resource->macrotile_nv12_factor, hw_resource->macrotile_factor,
        hw_resource->linear_factor, hw_resource->scale_factor, hw_resource->extra_fudge_factor);

  if (hw_resource->separate_rotator || hw_resource->num_dma_pipe) {
    GetHWRotatorInfo(hw_resource);
  }

  // If the driver doesn't spell out the wb index, assume it to be the number of rotators,
  // based on legacy implementation.
  if (hw_resource->writeback_index == kHWBlockMax) {
    hw_resource->writeback_index = hw_resource->hw_rot_info.num_rotator;
  }

  if (hw_resource->has_dyn_bw_support) {
    DisplayError ret = GetDynamicBWLimits(hw_resource);
    if (ret != kErrorNone) {
      DLOGE("Failed to read dynamic band width info");
      return ret;
    }

    DLOGI("Has Support for multiple bw limits shown below");
    for (int index = 0; index < kBwModeMax; index++) {
      DLOGI("Mode-index=%d  total_bw_limit=%d and pipe_bw_limit=%d",
            index, hw_resource->dyn_bw_info.total_bw_limit[index],
            hw_resource->dyn_bw_info.pipe_bw_limit[index]);
    }
  }

  return kErrorNone;
}

DisplayError HWInfoDRM::GetHWRotatorInfo(HWResourceInfo *hw_resource) {
  const uint32_t kMaxV4L2Nodes = 64;
  bool found = false;

  for (uint32_t i = 0; (i < kMaxV4L2Nodes) && (false == found); i++) {
    string path = "/sys/class/video4linux/video" + to_string(i) + "/name";
    Sys::fstream fs(path, fstream::in);
    if (!fs.is_open()) {
      continue;
    }

    string line;
    if (Sys::getline_(fs, line) &&
        (!strncmp(line.c_str(), "sde_rotator", strlen("sde_rotator")))) {
       hw_resource->hw_rot_info.device_path = string("/dev/video" + to_string(i));
       hw_resource->hw_rot_info.num_rotator++;
       hw_resource->hw_rot_info.type = HWRotatorInfo::ROT_TYPE_V4L2;
       hw_resource->hw_rot_info.has_downscale = true;
       // We support only 1 rotator
       found = true;
    }
  }

  DLOGI("V4L2 Rotator: Count = %d, Downscale = %d", hw_resource->hw_rot_info.num_rotator,
        hw_resource->hw_rot_info.has_downscale);

  return kErrorNone;
}

LayerBufferFormat HWInfoDRM::GetSDMFormat(uint32_t drm_format, uint32_t drm_format_modifier) {
  switch (drm_format) {
    case DRM_FORMAT_RGBA8888: return kFormatRGBA8888;
    default: return kFormatInvalid;
  }
}

void HWInfoDRM::InitSupportedFormatMap(HWResourceInfo *hw_resource) {
  hw_resource->supported_formats_map.clear();

  for (int sub_blk_type = INT(kHWVIGPipe); sub_blk_type < INT(kHWSubBlockMax); sub_blk_type++) {
    PopulateSupportedFormatMap((HWSubBlockType)sub_blk_type, hw_resource);
  }
}

void HWInfoDRM::PopulateSupportedFormatMap(HWSubBlockType sub_blk_type,
                                           HWResourceInfo *hw_resource) {
  vector <LayerBufferFormat> supported_sdm_formats;
  LayerBufferFormat sdm_format = kFormatRGBA8888;  // GetSDMFormat(INT(mdp_format));
  if (sdm_format != kFormatInvalid) {
    supported_sdm_formats.push_back(sdm_format);
  }

  hw_resource->supported_formats_map.erase(sub_blk_type);
  hw_resource->supported_formats_map.insert(make_pair(sub_blk_type, supported_sdm_formats));
}

DisplayError HWInfoDRM::GetFirstDisplayInterfaceType(HWDisplayInterfaceInfo *hw_disp_info) {
  hw_disp_info->type = kPrimary;
  hw_disp_info->is_connected = true;

  return kErrorNone;
}

}  // namespace sdm
