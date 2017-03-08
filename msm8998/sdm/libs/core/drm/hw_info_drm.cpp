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

#include <dlfcn.h>
#include <drm/drm_fourcc.h>
#include <drm_lib_loader.h>
#include <drm_master.h>
#include <drm_res_mgr.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/sys.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hw_info_drm.h"

#define __CLASS__ "HWInfoDRM"

using drm_utils::DRMMaster;
using drm_utils::DRMResMgr;
using drm_utils::DRMLogger;
using drm_utils::DRMLibLoader;
using sde_drm::GetDRMManager;
using sde_drm::DRMPlanesInfo;
using sde_drm::DRMCrtcInfo;
using sde_drm::DRMPlaneType;

using std::vector;
using std::map;
using std::string;
using std::fstream;
using std::to_string;

namespace sdm {

class DRMLoggerImpl : public DRMLogger {
 public:
#define PRINTLOG(method, format, buf)        \
  va_list list;                              \
  va_start(list, format);                    \
  vsnprintf(buf, sizeof(buf), format, list); \
  va_end(list);                              \
  Debug::Get()->method(kTagNone, "%s", buf);

  void Error(const char *format, ...) { PRINTLOG(Error, format, buf_); }
  void Warning(const char *format, ...) { PRINTLOG(Warning, format, buf_); }
  void Info(const char *format, ...) { PRINTLOG(Info, format, buf_); }
  void Debug(const char *format, ...) { PRINTLOG(Debug, format, buf_); }

 private:
  char buf_[1024] = {};
};

HWResourceInfo *HWInfoDRM::hw_resource_ = nullptr;

HWInfoDRM::HWInfoDRM() {
  DRMLogger::Set(new DRMLoggerImpl());
  default_mode_ = (DRMLibLoader::GetInstance()->IsLoaded() == false);
  if (!default_mode_) {
    DRMMaster *drm_master = {};
    int dev_fd = -1;
    DRMMaster::GetInstance(&drm_master);
    if (!drm_master) {
      DLOGE("Failed to acquire DRMMaster instance");
      return;
    }
    drm_master->GetHandle(&dev_fd);
    DRMLibLoader::GetInstance()->FuncGetDRMManager()(dev_fd, &drm_mgr_intf_);
  }
}

HWInfoDRM::~HWInfoDRM() {
  delete hw_resource_;
  hw_resource_ = nullptr;

  if (drm_mgr_intf_) {
    DRMLibLoader::GetInstance()->FuncDestroyDRMManager()();
    drm_mgr_intf_ = nullptr;
  }
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
  if (hw_resource_) {
    *hw_resource = *hw_resource_;
    return kErrorNone;
  }

  hw_resource->num_blending_stages = 1;
  hw_resource->max_pipe_width = 2560;
  hw_resource->max_cursor_size = 128;
  hw_resource->max_scale_down = 1;
  hw_resource->max_scale_up = 1;
  hw_resource->has_decimation = false;
  hw_resource->max_bandwidth_low = 9600000;
  hw_resource->max_bandwidth_high = 9600000;
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
  hw_resource->hw_dest_scalar_info.count = 0;
  hw_resource->hw_dest_scalar_info.max_scale_up = 0;
  hw_resource->hw_dest_scalar_info.max_input_width = 0;
  hw_resource->hw_dest_scalar_info.max_output_width = 0;
  hw_resource->is_src_split = true;
  hw_resource->perf_calc = false;
  hw_resource->has_dyn_bw_support = false;
  hw_resource->has_qseed3 = false;
  hw_resource->has_concurrent_writeback = false;

  // TODO(user): Deprecate
  hw_resource->hw_version = kHWMdssVersion5;
  hw_resource->hw_revision = 0;
  hw_resource->max_mixer_width = 0;
  hw_resource->writeback_index = 0;
  hw_resource->has_bwc = false;
  hw_resource->has_ubwc = true;
  hw_resource->has_macrotile = true;
  hw_resource->separate_rotator = true;
  hw_resource->has_non_scalar_rgb = false;

  GetSystemInfo(hw_resource);
  GetHWPlanesInfo(hw_resource);
  GetWBInfo(hw_resource);

  // Disable destination scalar count to 0 if extension library is not present
  DynLib extension_lib;
  if (!extension_lib.Open("libsdmextension.so")) {
    hw_resource->hw_dest_scalar_info.count = 0;
  }

  DLOGI("Max plane width = %d", hw_resource->max_pipe_width);
  DLOGI("Max cursor width = %d", hw_resource->max_cursor_size);
  DLOGI("Max plane upscale = %d", hw_resource->max_scale_up);
  DLOGI("Max plane downscale = %d", hw_resource->max_scale_down);
  DLOGI("Has Decimation = %d", hw_resource->has_decimation);
  DLOGI("Max Blending Stages = %d", hw_resource->num_blending_stages);
  DLOGI("Has Source Split = %d", hw_resource->is_src_split);
  DLOGI("Has QSEED3 = %d", hw_resource->has_qseed3);
  DLOGI("Has UBWC = %d", hw_resource->has_ubwc);
  DLOGI("Has Concurrent Writeback = %d", hw_resource->has_concurrent_writeback);
  DLOGI("Max Low Bw = %" PRIu64 "", hw_resource->max_bandwidth_low);
  DLOGI("Max High Bw = % " PRIu64 "", hw_resource->max_bandwidth_high);
  DLOGI("Max Pipe Bw = %" PRIu64 " KBps", hw_resource->max_pipe_bw);
  DLOGI("MaxSDEClock = % " PRIu64 " Hz", hw_resource->max_sde_clk);
  DLOGI("Clock Fudge Factor = %f", hw_resource->clk_fudge_factor);
  DLOGI("Prefill factors:");
  DLOGI("\tTiled_NV12 = %d", hw_resource->macrotile_nv12_factor);
  DLOGI("\tTiled = %d", hw_resource->macrotile_factor);
  DLOGI("\tLinear = %d", hw_resource->linear_factor);
  DLOGI("\tScale = %d", hw_resource->scale_factor);
  DLOGI("\tFudge_factor = %d", hw_resource->extra_fudge_factor);

  if (hw_resource->separate_rotator || hw_resource->num_dma_pipe) {
    GetHWRotatorInfo(hw_resource);
  }

  if (hw_resource->has_dyn_bw_support) {
    DisplayError ret = GetDynamicBWLimits(hw_resource);
    if (ret != kErrorNone) {
      DLOGE("Failed to read dynamic band width info");
      return ret;
    }

    DLOGI("Has Support for multiple bw limits shown below");
    for (int index = 0; index < kBwModeMax; index++) {
      DLOGI("Mode-index=%d  total_bw_limit=%d and pipe_bw_limit=%d", index,
            hw_resource->dyn_bw_info.total_bw_limit[index],
            hw_resource->dyn_bw_info.pipe_bw_limit[index]);
    }
  }

  if (!hw_resource_) {
    hw_resource_ = new HWResourceInfo();
    *hw_resource_ = *hw_resource;
  }

  return kErrorNone;
}

void HWInfoDRM::GetSystemInfo(HWResourceInfo *hw_resource) {
  DRMCrtcInfo info;
  drm_mgr_intf_->GetCrtcInfo(0 /* system_info */, &info);
  hw_resource->is_src_split = info.has_src_split;
  hw_resource->has_qseed3 = (info.qseed_version == sde_drm::QSEEDVersion::V3);
  hw_resource->num_blending_stages = info.max_blend_stages;
}

void HWInfoDRM::GetHWPlanesInfo(HWResourceInfo *hw_resource) {
  DRMPlanesInfo info;
  drm_mgr_intf_->GetPlanesInfo(&info);
  for (auto &pipe_obj : info.planes) {
    HWPipeCaps pipe_caps;
    string name = {};
    switch (pipe_obj.second) {
      case DRMPlaneType::RGB:
        pipe_caps.type = kPipeTypeRGB;
        hw_resource->num_rgb_pipe++;
        name = "RGB";
        break;
      case DRMPlaneType::VIG:
        pipe_caps.type = kPipeTypeVIG;
        hw_resource->num_vig_pipe++;
        name = "VIG";
        break;
      case DRMPlaneType::DMA:
        pipe_caps.type = kPipeTypeDMA;
        hw_resource->num_dma_pipe++;
        name = "DMA";
        break;
      case DRMPlaneType::CURSOR:
        pipe_caps.type = kPipeTypeCursor;
        hw_resource->num_cursor_pipe++;
        name = "CURSOR";
        break;
      default:
        break;
    }
    pipe_caps.id = pipe_obj.first;
    pipe_caps.max_rects = 1;
    DLOGI("%s Pipe : Id %d", name.c_str(), pipe_obj.first);
    hw_resource->hw_pipes.push_back(std::move(pipe_caps));
  }

  for (auto &pipe_type : info.types) {
    vector<LayerBufferFormat> supported_sdm_formats = {};
    for (auto &fmts : pipe_type.second.formats_supported) {
      GetSDMFormat(fmts.first, fmts.second, &supported_sdm_formats);
    }

    HWSubBlockType sub_blk_type = kHWSubBlockMax;
    switch (pipe_type.first) {
      case DRMPlaneType::RGB:
        sub_blk_type = kHWRGBPipe;
        // These properties are per plane but modeled in SDM as system-wide.
        hw_resource->max_pipe_width = pipe_type.second.max_linewidth;
        hw_resource->max_scale_down = pipe_type.second.max_downscale;
        hw_resource->max_scale_up = pipe_type.second.max_upscale;
        hw_resource->has_decimation =
            pipe_type.second.max_horizontal_deci > 1 && pipe_type.second.max_vertical_deci > 1;
        break;
      case DRMPlaneType::VIG:
        sub_blk_type = kHWVIGPipe;
        // These properties are per plane but modeled in SDM as system-wide.
        hw_resource->max_pipe_width = pipe_type.second.max_linewidth;
        hw_resource->max_scale_down = pipe_type.second.max_downscale;
        hw_resource->max_scale_up = pipe_type.second.max_upscale;
        hw_resource->has_decimation =
            pipe_type.second.max_horizontal_deci > 1 && pipe_type.second.max_vertical_deci > 1;
        break;
      case DRMPlaneType::DMA:
        sub_blk_type = kHWDMAPipe;
        break;
      case DRMPlaneType::CURSOR:
        sub_blk_type = kHWCursorPipe;
        hw_resource->max_cursor_size = pipe_type.second.max_linewidth;
        break;
      default:
        break;
    }

    if (sub_blk_type != kHWSubBlockMax) {
      hw_resource->supported_formats_map.erase(sub_blk_type);
      hw_resource->supported_formats_map.insert(make_pair(sub_blk_type, supported_sdm_formats));
    }
  }
}

void HWInfoDRM::GetWBInfo(HWResourceInfo *hw_resource) {
  HWSubBlockType sub_blk_type = kHWWBIntfOutput;
  vector<LayerBufferFormat> supported_sdm_formats = {};
  sde_drm::DRMDisplayToken token;

  // Fake register
  if (drm_mgr_intf_->RegisterDisplay(sde_drm::DRMDisplayType::VIRTUAL, &token)) {
    return;
  }

  sde_drm::DRMConnectorInfo connector_info;
  drm_mgr_intf_->GetConnectorInfo(token.conn_id, &connector_info);
  for (auto &fmts : connector_info.formats_supported) {
    GetSDMFormat(fmts.first, fmts.second, &supported_sdm_formats);
  }

  hw_resource->supported_formats_map.erase(sub_blk_type);
  hw_resource->supported_formats_map.insert(make_pair(sub_blk_type, supported_sdm_formats));

  drm_mgr_intf_->UnregisterDisplay(token);
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
    if (Sys::getline_(fs, line) && (!strncmp(line.c_str(), "sde_rotator", strlen("sde_rotator")))) {
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

void HWInfoDRM::GetSDMFormat(uint32_t drm_format, uint64_t drm_format_modifier,
                             vector<LayerBufferFormat> *sdm_formats) {
  vector<LayerBufferFormat> &fmts(*sdm_formats);
  switch (drm_format) {
    case DRM_FORMAT_ARGB8888:
      fmts.push_back(kFormatARGB8888);
      break;
    case DRM_FORMAT_RGBA8888:
      fmts.push_back(drm_format_modifier ? kFormatRGBA8888Ubwc : kFormatRGBA8888);
      break;
    case DRM_FORMAT_BGRA8888:
      fmts.push_back(kFormatBGRA8888);
      break;
    case DRM_FORMAT_XRGB8888:
      fmts.push_back(kFormatXRGB8888);
      break;
    case DRM_FORMAT_RGBX8888:
      fmts.push_back(drm_format_modifier ? kFormatRGBX8888Ubwc : kFormatRGBX8888);
      break;
    case DRM_FORMAT_BGRX8888:
      fmts.push_back(kFormatBGRX8888);
      break;
    case DRM_FORMAT_RGBA5551:
      fmts.push_back(kFormatRGBA5551);
      break;
    case DRM_FORMAT_RGBA4444:
      fmts.push_back(kFormatRGBA4444);
      break;
    case DRM_FORMAT_RGB888:
      fmts.push_back(kFormatRGB888);
      break;
    case DRM_FORMAT_BGR888:
      fmts.push_back(kFormatBGR888);
      break;
    case DRM_FORMAT_RGB565:
      fmts.push_back(drm_format_modifier ? kFormatBGR565Ubwc : kFormatBGR565);
      break;
    case DRM_FORMAT_BGR565:
      fmts.push_back(kFormatBGR565);
      break;
    case DRM_FORMAT_RGBA1010102:
      fmts.push_back(drm_format_modifier ? kFormatRGBA1010102Ubwc : kFormatRGBA1010102);
      break;
    case DRM_FORMAT_ARGB2101010:
      fmts.push_back(kFormatARGB2101010);
      break;
    case DRM_FORMAT_RGBX1010102:
      fmts.push_back(drm_format_modifier ? kFormatRGBX1010102Ubwc : kFormatRGBX1010102);
      break;
    case DRM_FORMAT_XRGB2101010:
      fmts.push_back(kFormatXRGB2101010);
      break;
    case DRM_FORMAT_BGRA1010102:
      fmts.push_back(kFormatBGRA1010102);
      break;
    case DRM_FORMAT_ABGR2101010:
      fmts.push_back(kFormatABGR2101010);
      break;
    case DRM_FORMAT_BGRX1010102:
      fmts.push_back(kFormatBGRX1010102);
      break;
    case DRM_FORMAT_XBGR2101010:
      fmts.push_back(kFormatXBGR2101010);
      break;
    /* case DRM_FORMAT_P010:
         fmts.push_back(drm_format_modifier == (DRM_FORMAT_MOD_QCOM_COMPRESSED |
       DRM_FORMAT_MOD_QCOM_TIGHT) ?
         kFormatYCbCr420TP10Ubwc : kFormatYCbCr420P010; */
    case DRM_FORMAT_YVU420:
      fmts.push_back(kFormatYCrCb420PlanarStride16);
      break;
    case DRM_FORMAT_NV12:
      if (drm_format_modifier) {
        fmts.push_back(kFormatYCbCr420SPVenusUbwc);
      } else {
        fmts.push_back(kFormatYCbCr420SemiPlanarVenus);
        fmts.push_back(kFormatYCbCr420SemiPlanar);
      }
      break;
    case DRM_FORMAT_NV21:
      fmts.push_back(kFormatYCrCb420SemiPlanarVenus);
      fmts.push_back(kFormatYCrCb420SemiPlanar);
      break;
    case DRM_FORMAT_NV16:
      fmts.push_back(kFormatYCbCr422H2V1SemiPlanar);
      break;
    default:
      break;
  }
}

DisplayError HWInfoDRM::GetFirstDisplayInterfaceType(HWDisplayInterfaceInfo *hw_disp_info) {
  hw_disp_info->type = kPrimary;
  hw_disp_info->is_connected = true;

  return kErrorNone;
}

}  // namespace sdm
