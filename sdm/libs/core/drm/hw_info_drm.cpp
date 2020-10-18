/*
* Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
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
#include <media/msm_sde_rotator.h>
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

#ifndef DRM_FORMAT_MOD_QCOM_COMPRESSED
#define DRM_FORMAT_MOD_QCOM_COMPRESSED fourcc_mod_code(QCOM, 1)
#endif
#ifndef DRM_FORMAT_MOD_QCOM_DX
#define DRM_FORMAT_MOD_QCOM_DX fourcc_mod_code(QCOM, 0x2)
#endif
#ifndef DRM_FORMAT_MOD_QCOM_TIGHT
#define DRM_FORMAT_MOD_QCOM_TIGHT fourcc_mod_code(QCOM, 0x4)
#endif

#define __CLASS__ "HWInfoDRM"

using drm_utils::DRMMaster;
using drm_utils::DRMResMgr;
using drm_utils::DRMLibLoader;
using sde_drm::GetDRMManager;
using sde_drm::DRMPlanesInfo;
using sde_drm::DRMCrtcInfo;
using sde_drm::DRMPlaneType;
using sde_drm::DRMTonemapLutType;

using std::vector;
using std::map;
using std::string;
using std::fstream;
using std::to_string;

namespace sdm {

static HWQseedStepVersion GetQseedStepVersion(sde_drm::QSEEDStepVersion drm_version) {
  HWQseedStepVersion sdm_version;
  switch (drm_version) {
    case sde_drm::QSEEDStepVersion::V2:
    default:
      sdm_version = kQseed3v2;
      break;
    case sde_drm::QSEEDStepVersion::V3:
      sdm_version = kQseed3v3;
      break;
    case sde_drm::QSEEDStepVersion::V4:
      sdm_version = kQseed3v4;
      break;
    case sde_drm::QSEEDStepVersion::V3LITE_V4:
      sdm_version = kQseed3litev4;
      break;
    case sde_drm::QSEEDStepVersion::V3LITE_V5:
      sdm_version = kQseed3litev5;
      break;
  }
  return sdm_version;
}

static InlineRotationVersion GetInRotVersion(sde_drm::InlineRotationVersion drm_version) {
  switch (drm_version) {
    case sde_drm::InlineRotationVersion::V1:
      return InlineRotationVersion::kInlineRotationV1;
    case sde_drm::InlineRotationVersion::V1p1:
      return InlineRotationVersion::kInlineRotationV1p1;
    default:
      return kInlineRotationNone;
  }
}

HWResourceInfo *HWInfoDRM::hw_resource_ = nullptr;

DisplayError HWInfoDRM::Init() {
  default_mode_ = (DRMLibLoader::GetInstance()->IsLoaded() == false);
  if (!default_mode_) {
    DRMMaster *drm_master = {};
    int dev_fd = -1;
    DRMMaster::GetInstance(&drm_master);
    if (!drm_master) {
      DLOGE("Failed to acquire DRMMaster instance");
      return kErrorCriticalResource;
    }
    drm_master->GetHandle(&dev_fd);
    DRMLibLoader::GetInstance()->FuncGetDRMManager()(dev_fd, &drm_mgr_intf_);
    if (!drm_mgr_intf_) {
      DRMLibLoader::Destroy();
      DRMMaster::DestroyInstance();
      DLOGE("Failed to get DRMManagerInterface");
      return kErrorCriticalResource;
    }
  }

  return kErrorNone;
}

void HWInfoDRM::Deinit() {
  delete hw_resource_;
  hw_resource_ = nullptr;

  if (drm_mgr_intf_) {
    DRMLibLoader::GetInstance()->FuncDestroyDRMManager()();
    drm_mgr_intf_ = nullptr;
  }

  DRMLibLoader::Destroy();
  DRMMaster::DestroyInstance();
}

HWInfoDRM::~HWInfoDRM() {
  Deinit();
}

DisplayError HWInfoDRM::GetDynamicBWLimits(HWResourceInfo *hw_resource) {
  HWDynBwLimitInfo* bw_info = &hw_resource->dyn_bw_info;
  for (int index = 0; index < kBwModeMax; index++) {
    bw_info->total_bw_limit[index] = hw_resource->max_bandwidth_low;
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
  hw_resource->amortizable_threshold = 25;
  hw_resource->system_overhead_lines = 0;
  hw_resource->hw_dest_scalar_info.count = 0;
  hw_resource->hw_dest_scalar_info.max_scale_up = 0;
  hw_resource->hw_dest_scalar_info.max_input_width = 0;
  hw_resource->hw_dest_scalar_info.max_output_width = 0;
  hw_resource->is_src_split = true;
  hw_resource->has_dyn_bw_support = false;
  hw_resource->has_qseed3 = false;
  hw_resource->has_concurrent_writeback = false;

  hw_resource->hw_version = SDEVERSION(4, 0, 1);
  // TODO(user): On FB driver hw_revision comprises of major version, minor version and hw_revision.
  // On DRM driver, hw_revision is deprecated and hw_version comprises major version, minor version
  // and hw_revision information. Since QDCM uses hw_revision variable populate hw_revision with
  // hw_version. Remove hw_revision variable when FB code is deperecated.
  hw_resource->hw_revision = SDEVERSION(4, 0, 1);

  // TODO(user): Deprecate
  hw_resource->max_mixer_width = 2560;
  hw_resource->writeback_index = 0;
  hw_resource->has_bwc = false;
  hw_resource->has_ubwc = true;
  hw_resource->has_macrotile = true;
  hw_resource->separate_rotator = true;
  hw_resource->has_non_scalar_rgb = false;

  GetSystemInfo(hw_resource);
  GetHWPlanesInfo(hw_resource);
  GetWBInfo(hw_resource);

  // Disable destination scalar count to 0 if extension library is not present or disabled
  // through property
  int value = 0;
  bool disable_dest_scalar = false;
  if (Debug::GetProperty(DISABLE_DESTINATION_SCALER_PROP, &value) == kErrorNone) {
    disable_dest_scalar = (value == 1);
  }
  DynLib extension_lib;
  if (!extension_lib.Open("libsdmextension.so") || disable_dest_scalar) {
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
  DLOGI("Has Src Tonemap = %d", hw_resource->src_tone_map);
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
  DLOGI("\tib_fudge_factor = %f", hw_resource->ib_fudge_factor);

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
  hw_resource->has_hdr = info.has_hdr;
  hw_resource->is_src_split = info.has_src_split;
  hw_resource->has_qseed3 = (info.qseed_version >= sde_drm::QSEEDVersion::V3);
  hw_resource->num_blending_stages = info.max_blend_stages;
  hw_resource->num_solidfill_stages = info.max_solidfill_stages;
  hw_resource->smart_dma_rev = (info.smart_dma_rev == sde_drm::SmartDMARevision::V2p5) ?
    SmartDMARevision::V2p5 : ((info.smart_dma_rev == sde_drm::SmartDMARevision::V2) ?
    SmartDMARevision::V2 : SmartDMARevision::V1);
  hw_resource->ib_fudge_factor = info.ib_fudge_factor;
  hw_resource->hw_dest_scalar_info.prefill_lines = info.dest_scale_prefill_lines;
  hw_resource->undersized_prefill_lines = info.undersized_prefill_lines;
  hw_resource->macrotile_factor = info.macrotile_prefill_lines;
  hw_resource->macrotile_nv12_factor = info.nv12_prefill_lines;
  hw_resource->linear_factor = info.linear_prefill_lines;
  hw_resource->scale_factor = info.downscale_prefill_lines;
  hw_resource->extra_fudge_factor = info.extra_prefill_lines;
  hw_resource->amortizable_threshold = info.amortized_threshold;
  hw_resource->max_bandwidth_low = info.max_bandwidth_low / kKiloUnit;
  hw_resource->max_bandwidth_high = info.max_bandwidth_high / kKiloUnit;
  hw_resource->max_sde_clk = info.max_sde_clk;
  hw_resource->hw_version = info.hw_version;
  // TODO(user): On FB driver hw_revision comprises of major version, minor version and hw_revision.
  // On DRM driver, hw_revision is deprecated and hw_version comprises major version, minor version
  // and hw_revision information. Since QDCM uses hw_revision variable populate hw_revision with
  // hw_version. Remove hw_revision variable when FB code is deperecated.
  hw_resource->hw_revision = info.hw_version;

  std::vector<LayerBufferFormat> sdm_format;
  for (auto &it : info.comp_ratio_rt_map) {
    std::pair<uint32_t, uint64_t> drm_format = it.first;
    GetSDMFormat(drm_format.first, drm_format.second, &sdm_format);
    hw_resource->comp_ratio_rt_map.insert(std::make_pair(sdm_format[0], it.second));
    sdm_format.clear();
  }

  for (auto &it : info.comp_ratio_nrt_map) {
    std::pair<uint32_t, uint64_t> drm_format = it.first;
    GetSDMFormat(drm_format.first, drm_format.second, &sdm_format);
    hw_resource->comp_ratio_rt_map.insert(std::make_pair(sdm_format[0], it.second));
    sdm_format.clear();
  }

  hw_resource->hw_dest_scalar_info.count = info.dest_scaler_count;
  hw_resource->hw_dest_scalar_info.max_scale_up = info.max_dest_scale_up;
  hw_resource->hw_dest_scalar_info.max_input_width = info.max_dest_scaler_input_width;
  hw_resource->hw_dest_scalar_info.max_output_width = info.max_dest_scaler_output_width;
  hw_resource->min_prefill_lines = info.min_prefill_lines;
  hw_resource->secure_disp_blend_stage = info.secure_disp_blend_stage;
  hw_resource->has_concurrent_writeback = info.concurrent_writeback;
  // In case driver doesn't report bus width default to 256 bit bus.
  hw_resource->num_mnocports = info.num_mnocports ? info.num_mnocports : 2;
  hw_resource->mnoc_bus_width = info.mnoc_bus_width ? info.mnoc_bus_width : 32;
  hw_resource->use_baselayer_for_stage = info.use_baselayer_for_stage;
  hw_resource->line_width_constraints_count = info.line_width_constraints_count;
  if (info.line_width_constraints_count) {
    auto &width_constraints = hw_resource->line_width_constraints;
    hw_resource->line_width_limits = std::move(info.line_width_limits);
    width_constraints.push_back(std::make_pair(kPipeVigLimit, info.vig_limit_index));
    width_constraints.push_back(std::make_pair(kPipeDmaLimit, info.dma_limit_index));
    width_constraints.push_back(std::make_pair(kPipeScalingLimit, info.scaling_limit_index));
    width_constraints.push_back(std::make_pair(kPipeRotationLimit, info.rotation_limit_index));
  }
  // Use fudge factor as 1.5 if not reported
  hw_resource->vbif_cmd_ff = (info.vbif_cmd_ff > 0.0f) ? info.vbif_cmd_ff : 1.5f;
}

void HWInfoDRM::GetHWPlanesInfo(HWResourceInfo *hw_resource) {
  DRMPlanesInfo planes;
  drm_mgr_intf_->GetPlanesInfo(&planes);

  // To simulate reduced config.
  uint32_t max_vig_pipes = 0;
  uint32_t max_dma_pipes = 0;
  Debug::GetReducedConfig(&max_vig_pipes, &max_dma_pipes);
  uint32_t max_virtual_pipes = max_vig_pipes + max_dma_pipes;
  uint32_t vig_pipe_count = 0;
  uint32_t dma_pipe_count = 0;
  uint32_t virtual_pipe_count = 0;
  int disable_src_tonemap = 0;
  Debug::Get()->GetProperty(DISABLE_SRC_TONEMAP_PROP, &disable_src_tonemap);

  for (auto &pipe_obj : planes) {
    if (max_vig_pipes && max_dma_pipes) {
      uint32_t master_plane_id = pipe_obj.second.master_plane_id;
      if ((pipe_obj.second.type == DRMPlaneType::DMA) && (dma_pipe_count < max_dma_pipes)
          && !master_plane_id) {
        dma_pipe_count++;
      } else if ((pipe_obj.second.type == DRMPlaneType::VIG) && (vig_pipe_count < max_vig_pipes)
          && !master_plane_id) {
        vig_pipe_count++;
      } else if ((master_plane_id) && (virtual_pipe_count < max_virtual_pipes)) {
        bool is_virtual = false;
        for (auto &pipe_caps : hw_resource->hw_pipes) {
          if (master_plane_id == pipe_caps.id) {
            is_virtual = true;
            virtual_pipe_count++;
            break;
          }
        }
        if (!is_virtual) {
          continue;
        }
      } else {
        continue;
      }
    }

    HWPipeCaps pipe_caps;
    string name = {};
    switch (pipe_obj.second.type) {
      case DRMPlaneType::DMA:
        name = "DMA";
        pipe_caps.type = kPipeTypeDMA;
        if (!hw_resource->num_dma_pipe) {
          PopulateSupportedFmts(kHWDMAPipe, pipe_obj.second, hw_resource);
        }
        hw_resource->num_dma_pipe++;
        break;
      case DRMPlaneType::VIG:
        name = "VIG";
        pipe_caps.type = kPipeTypeVIG;
        if (!hw_resource->num_vig_pipe) {
          PopulatePipeCaps(pipe_obj.second, hw_resource);
          PopulateSupportedFmts(kHWVIGPipe, pipe_obj.second, hw_resource);
        }
        hw_resource->num_vig_pipe++;
        break;
      case DRMPlaneType::CURSOR:
        name = "CURSOR";
        pipe_caps.type = kPipeTypeCursor;
        if (!hw_resource->num_cursor_pipe) {
          PopulateSupportedFmts(kHWCursorPipe, pipe_obj.second, hw_resource);
          hw_resource->max_cursor_size = pipe_obj.second.max_linewidth;
        }
        hw_resource->num_cursor_pipe++;
        break;
      default:
        continue;  // Not adding any other pipe type
    }
    pipe_caps.id = pipe_obj.first;
    pipe_caps.master_pipe_id = pipe_obj.second.master_plane_id;
    pipe_caps.block_sec_ui = pipe_obj.second.block_sec_ui;
    pipe_caps.hw_block_mask = pipe_obj.second.hw_block_mask;
    DLOGI("Adding %s Pipe : Id %d, master_pipe_id : Id %d block_sec_ui: %d hw_block_mask: 0x%x",
          name.c_str(), pipe_obj.first, pipe_obj.second.master_plane_id,
          pipe_obj.second.block_sec_ui, pipe_obj.second.hw_block_mask.to_ulong());
    pipe_caps.inverse_pma = pipe_obj.second.inverse_pma;
    pipe_caps.dgm_csc_version = pipe_obj.second.dgm_csc_version;
    // disable src tonemap feature if its disabled using property.
    if (!disable_src_tonemap) {
      for (auto &it : pipe_obj.second.tonemap_lut_version_map) {
        HWToneMapLut tonemap_lut = kLutNone;
        switch (it.first) {
          case DRMTonemapLutType::DMA_1D_IGC:
            tonemap_lut = kDma1dIgc;
            break;
          case DRMTonemapLutType::DMA_1D_GC:
            tonemap_lut = kDma1dGc;
            break;
          case DRMTonemapLutType::VIG_1D_IGC:
            tonemap_lut = kVig1dIgc;
            break;
          case DRMTonemapLutType::VIG_3D_GAMUT:
            tonemap_lut = kVig3dGamut;
            break;
          default:
            DLOGE("Unknown Tonemap Lut");
            break;
        }
        if (tonemap_lut != kLutNone) {
          pipe_caps.tm_lut_version_map[tonemap_lut] = it.second;
          if (pipe_caps.type == kPipeTypeVIG) {
            hw_resource->src_tone_map[kSrcTonemap3d] = 1;
          } else if (pipe_caps.type == kPipeTypeDMA) {
            hw_resource->src_tone_map[kSrcTonemap1d] = 1;
          }
        }
      }
    }
    hw_resource->hw_pipes.push_back(std::move(pipe_caps));
  }
  hw_resource->has_excl_rect = planes[0].second.has_excl_rect;
}

void HWInfoDRM::PopulatePipeCaps(const sde_drm::DRMPlaneTypeInfo &info,
                                    HWResourceInfo *hw_resource) {
  hw_resource->max_pipe_width = info.max_linewidth;
  hw_resource->max_scaler_pipe_width = info.max_scaler_linewidth;
  hw_resource->max_scale_down = info.max_downscale;
  hw_resource->max_scale_up = info.max_upscale;
  hw_resource->has_decimation = info.max_horizontal_deci > 1 && info.max_vertical_deci > 1;
  hw_resource->max_pipe_bw = info.max_pipe_bandwidth / kKiloUnit;
  hw_resource->cache_size = info.cache_size;
  hw_resource->pipe_qseed3_version = GetQseedStepVersion(info.qseed3_version);
  hw_resource->inrot_version = GetInRotVersion(info.inrot_version);
}

void HWInfoDRM::PopulateSupportedFmts(HWSubBlockType sub_blk_type,
                                      const sde_drm::DRMPlaneTypeInfo  &info,
                                      HWResourceInfo *hw_resource) {
  vector<LayerBufferFormat> sdm_formats;
  FormatsMap &fmts_map = hw_resource->supported_formats_map;

  if (fmts_map.find(sub_blk_type) == fmts_map.end()) {
    for (auto &fmts : info.formats_supported) {
      GetSDMFormat(fmts.first, fmts.second, &sdm_formats);
    }

    fmts_map.insert(make_pair(sub_blk_type, sdm_formats));
  }
}

void HWInfoDRM::GetWBInfo(HWResourceInfo *hw_resource) {
  HWSubBlockType sub_blk_type = kHWWBIntfOutput;
  vector<LayerBufferFormat> supported_sdm_formats;
  sde_drm::DRMDisplayToken token;
  int ret = 0;

  // Fake register
  ret = drm_mgr_intf_->RegisterDisplay(sde_drm::DRMDisplayType::VIRTUAL, &token);
  if (ret) {
    DLOGE("Failed registering display %d. Error: %d.", sde_drm::DRMDisplayType::VIRTUAL, ret);
    return;
  }

  sde_drm::DRMConnectorInfo connector_info;
  ret = drm_mgr_intf_->GetConnectorInfo(token.conn_id, &connector_info);
  if (ret) {
    DLOGE("Failed getting info for connector id %u. Error: %d.", token.conn_id, ret);
    drm_mgr_intf_->UnregisterDisplay(&token);
    return;
  }
  for (auto &fmts : connector_info.formats_supported) {
    GetSDMFormat(fmts.first, fmts.second, &supported_sdm_formats);
  }

  hw_resource->supported_formats_map.erase(sub_blk_type);
  hw_resource->supported_formats_map.insert(make_pair(sub_blk_type, supported_sdm_formats));

  drm_mgr_intf_->UnregisterDisplay(&token);
}

void HWInfoDRM::GetSDMFormat(uint32_t v4l2_format, LayerBufferFormat *sdm_format) {
  switch (v4l2_format) {
    case SDE_PIX_FMT_ARGB_8888:         *sdm_format = kFormatARGB8888;                 break;
    case SDE_PIX_FMT_RGBA_8888:         *sdm_format = kFormatRGBA8888;                 break;
    case SDE_PIX_FMT_BGRA_8888:         *sdm_format = kFormatBGRA8888;                 break;
    case SDE_PIX_FMT_RGBX_8888:         *sdm_format = kFormatRGBX8888;                 break;
    case SDE_PIX_FMT_BGRX_8888:         *sdm_format = kFormatBGRX8888;                 break;
    case SDE_PIX_FMT_RGBA_5551:         *sdm_format = kFormatRGBA5551;                 break;
    case SDE_PIX_FMT_RGBA_4444:         *sdm_format = kFormatRGBA4444;                 break;
    case SDE_PIX_FMT_RGB_888:           *sdm_format = kFormatRGB888;                   break;
    case SDE_PIX_FMT_BGR_888:           *sdm_format = kFormatBGR888;                   break;
    case SDE_PIX_FMT_RGB_565:           *sdm_format = kFormatRGB565;                   break;
    case SDE_PIX_FMT_BGR_565:           *sdm_format = kFormatBGR565;                   break;
    case SDE_PIX_FMT_Y_CB_CR_H2V2:      *sdm_format = kFormatYCbCr420Planar;           break;
    case SDE_PIX_FMT_Y_CR_CB_H2V2:      *sdm_format = kFormatYCrCb420Planar;           break;
    case SDE_PIX_FMT_Y_CR_CB_GH2V2:     *sdm_format = kFormatYCrCb420PlanarStride16;   break;
    case SDE_PIX_FMT_Y_CBCR_H2V2:       *sdm_format = kFormatYCbCr420SemiPlanar;       break;
    case SDE_PIX_FMT_Y_CRCB_H2V2:       *sdm_format = kFormatYCrCb420SemiPlanar;       break;
    case SDE_PIX_FMT_Y_CBCR_H1V2:       *sdm_format = kFormatYCbCr422H1V2SemiPlanar;   break;
    case SDE_PIX_FMT_Y_CRCB_H1V2:       *sdm_format = kFormatYCrCb422H1V2SemiPlanar;   break;
    case SDE_PIX_FMT_Y_CBCR_H2V1:       *sdm_format = kFormatYCbCr422H2V1SemiPlanar;   break;
    case SDE_PIX_FMT_Y_CRCB_H2V1:       *sdm_format = kFormatYCrCb422H2V1SemiPlanar;   break;
    case SDE_PIX_FMT_YCBYCR_H2V1:       *sdm_format = kFormatYCbCr422H2V1Packed;       break;
    case SDE_PIX_FMT_Y_CBCR_H2V2_VENUS: *sdm_format = kFormatYCbCr420SemiPlanarVenus;  break;
    case SDE_PIX_FMT_Y_CRCB_H2V2_VENUS: *sdm_format = kFormatYCrCb420SemiPlanarVenus;  break;
    case SDE_PIX_FMT_RGBA_8888_UBWC:    *sdm_format = kFormatRGBA8888Ubwc;             break;
    case SDE_PIX_FMT_RGBX_8888_UBWC:    *sdm_format = kFormatRGBX8888Ubwc;             break;
    case SDE_PIX_FMT_RGB_565_UBWC:      *sdm_format = kFormatBGR565Ubwc;               break;
    case SDE_PIX_FMT_Y_CBCR_H2V2_UBWC:  *sdm_format = kFormatYCbCr420SPVenusUbwc;      break;
    case SDE_PIX_FMT_RGBA_1010102:      *sdm_format = kFormatRGBA1010102;              break;
    case SDE_PIX_FMT_ARGB_2101010:      *sdm_format = kFormatARGB2101010;              break;
    case SDE_PIX_FMT_RGBX_1010102:      *sdm_format = kFormatRGBX1010102;              break;
    case SDE_PIX_FMT_XRGB_2101010:      *sdm_format = kFormatXRGB2101010;              break;
    case SDE_PIX_FMT_BGRA_1010102:      *sdm_format = kFormatBGRA1010102;              break;
    case SDE_PIX_FMT_ABGR_2101010:      *sdm_format = kFormatABGR2101010;              break;
    case SDE_PIX_FMT_BGRX_1010102:      *sdm_format = kFormatBGRX1010102;              break;
    case SDE_PIX_FMT_XBGR_2101010:      *sdm_format = kFormatXBGR2101010;              break;
    case SDE_PIX_FMT_RGBA_1010102_UBWC: *sdm_format = kFormatRGBA1010102Ubwc;          break;
    case SDE_PIX_FMT_RGBX_1010102_UBWC: *sdm_format = kFormatRGBX1010102Ubwc;          break;
    case SDE_PIX_FMT_Y_CBCR_H2V2_P010:  *sdm_format = kFormatYCbCr420P010;             break;
    case SDE_PIX_FMT_Y_CBCR_H2V2_TP10_UBWC:  *sdm_format = kFormatYCbCr420TP10Ubwc;     break;
    case SDE_PIX_FMT_Y_CBCR_H2V2_P010_UBWC:  *sdm_format = kFormatYCbCr420P010Ubwc;     break;
    case SDE_PIX_FMT_Y_CBCR_H2V2_P010_VENUS: *sdm_format = kFormatYCbCr420P010Venus;    break;
    default: *sdm_format = kFormatInvalid;
  }
}

void HWInfoDRM::GetRotatorFormatsForType(int fd, uint32_t type,
                                         vector<LayerBufferFormat> *supported_formats) {
  struct v4l2_fmtdesc fmtdesc = {};
  fmtdesc.type = type;
  while (!Sys::ioctl_(fd, static_cast<int>(VIDIOC_ENUM_FMT), &fmtdesc)) {
    LayerBufferFormat sdm_format = kFormatInvalid;
    GetSDMFormat(fmtdesc.pixelformat, &sdm_format);
    if (sdm_format != kFormatInvalid) {
      supported_formats->push_back(sdm_format);
    }
    fmtdesc.index++;
  }
}

DisplayError HWInfoDRM::GetRotatorSupportedFormats(uint32_t v4l2_index,
                                                   HWResourceInfo *hw_resource) {
  string path = "/dev/video" + to_string(v4l2_index);
  int fd = Sys::open_(path.c_str(), O_RDONLY);
  if (fd < 0) {
    DLOGE("Failed to open %s with error %d", path.c_str(), errno);
    return kErrorNotSupported;
  }

  vector<LayerBufferFormat> supported_formats = {};
  GetRotatorFormatsForType(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT, &supported_formats);
  hw_resource->supported_formats_map.erase(kHWRotatorInput);
  hw_resource->supported_formats_map.insert(make_pair(kHWRotatorInput, supported_formats));

  supported_formats = {};
  GetRotatorFormatsForType(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE, &supported_formats);
  hw_resource->supported_formats_map.erase(kHWRotatorOutput);
  hw_resource->supported_formats_map.insert(make_pair(kHWRotatorOutput, supported_formats));

  Sys::close_(fd);

  return kErrorNone;
}

DisplayError HWInfoDRM::GetHWRotatorInfo(HWResourceInfo *hw_resource) {
  string v4l2_path = "/sys/class/video4linux/video";
  const uint32_t kMaxV4L2Nodes = 64;

  for (uint32_t i = 0; i < kMaxV4L2Nodes; i++) {
    string path = v4l2_path + to_string(i) + "/name";
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
      GetRotatorSupportedFormats(i, hw_resource);

      string caps_path = v4l2_path + to_string(i) + "/device/caps";
      Sys::fstream caps_fs(caps_path, fstream::in);

      if (caps_fs.is_open()) {
        string caps;
        while (Sys::getline_(caps_fs, caps)) {
          const string downscale_compression = "downscale_compression=";
          const string min_downscale = "min_downscale=";
          const string max_line_width = "max_line_width=";
          if (caps.find(downscale_compression) != string::npos) {
            hw_resource->hw_rot_info.downscale_compression =
              std::stoi(string(caps, downscale_compression.length()));
          } else if (caps.find(min_downscale) != string::npos) {
            hw_resource->hw_rot_info.min_downscale =
              std::stof(string(caps, min_downscale.length()));
          } else if (caps.find(max_line_width) != string::npos) {
            hw_resource->hw_rot_info.max_line_width =
              std::stoul(string(caps, max_line_width.length()));
          }
        }
      }

      // We support only 1 rotator
      break;
    }
  }

  DLOGI("V4L2 Rotator: Count = %d, Downscale = %d, Min_downscale = %f," \
        "Downscale_compression = %d, Max_line_width = %d", hw_resource->hw_rot_info.num_rotator,
        hw_resource->hw_rot_info.has_downscale, hw_resource->hw_rot_info.min_downscale,
        hw_resource->hw_rot_info.downscale_compression, hw_resource->hw_rot_info.max_line_width);

  return kErrorNone;
}

void HWInfoDRM::GetSDMFormat(uint32_t drm_format, uint64_t drm_format_modifier,
                             vector<LayerBufferFormat> *sdm_formats) {
  vector<LayerBufferFormat> &fmts(*sdm_formats);
  switch (drm_format) {
    case DRM_FORMAT_BGRA8888:
      fmts.push_back(kFormatARGB8888);
      break;
    case DRM_FORMAT_ABGR8888:
      fmts.push_back(drm_format_modifier ? kFormatRGBA8888Ubwc : kFormatRGBA8888);
      break;
    case DRM_FORMAT_ARGB8888:
      fmts.push_back(kFormatBGRA8888);
      break;
    case DRM_FORMAT_BGRX8888:
      fmts.push_back(kFormatXRGB8888);
      break;
    case DRM_FORMAT_XBGR8888:
      fmts.push_back(drm_format_modifier ? kFormatRGBX8888Ubwc : kFormatRGBX8888);
      break;
    case DRM_FORMAT_XRGB8888:
      fmts.push_back(kFormatBGRX8888);
      break;
    case DRM_FORMAT_ABGR1555:
      fmts.push_back(kFormatRGBA5551);
      break;
    case DRM_FORMAT_ABGR4444:
      fmts.push_back(kFormatRGBA4444);
      break;
    case DRM_FORMAT_BGR888:
      fmts.push_back(kFormatRGB888);
      break;
    case DRM_FORMAT_RGB888:
      fmts.push_back(kFormatBGR888);
      break;
    case DRM_FORMAT_BGR565:
      fmts.push_back(drm_format_modifier ? kFormatBGR565Ubwc : kFormatRGB565);
      break;
    case DRM_FORMAT_RGB565:
      fmts.push_back(kFormatBGR565);
      break;
    case DRM_FORMAT_ABGR2101010:
      fmts.push_back(drm_format_modifier ? kFormatRGBA1010102Ubwc : kFormatRGBA1010102);
      break;
    case DRM_FORMAT_BGRA1010102:
      fmts.push_back(kFormatARGB2101010);
      break;
    case DRM_FORMAT_XBGR2101010:
      fmts.push_back(drm_format_modifier ? kFormatRGBX1010102Ubwc : kFormatRGBX1010102);
      break;
    case DRM_FORMAT_BGRX1010102:
      fmts.push_back(kFormatXRGB2101010);
      break;
    case DRM_FORMAT_ARGB2101010:
      fmts.push_back(kFormatBGRA1010102);
      break;
    case DRM_FORMAT_RGBA1010102:
      fmts.push_back(kFormatABGR2101010);
      break;
    case DRM_FORMAT_XRGB2101010:
      fmts.push_back(kFormatBGRX1010102);
      break;
    case DRM_FORMAT_RGBX1010102:
      fmts.push_back(kFormatXBGR2101010);
      break;
    case DRM_FORMAT_YVU420:
      fmts.push_back(kFormatYCrCb420PlanarStride16);
      break;
    case DRM_FORMAT_NV12:
      if (drm_format_modifier == (DRM_FORMAT_MOD_QCOM_COMPRESSED |
          DRM_FORMAT_MOD_QCOM_DX | DRM_FORMAT_MOD_QCOM_TIGHT)) {
          fmts.push_back(kFormatYCbCr420TP10Ubwc);
      } else if (drm_format_modifier == (DRM_FORMAT_MOD_QCOM_COMPRESSED |
                                         DRM_FORMAT_MOD_QCOM_DX)) {
        fmts.push_back(kFormatYCbCr420P010Ubwc);
      } else if (drm_format_modifier == DRM_FORMAT_MOD_QCOM_COMPRESSED) {
         fmts.push_back(kFormatYCbCr420SPVenusUbwc);
      } else if (drm_format_modifier == DRM_FORMAT_MOD_QCOM_DX) {
        fmts.push_back(kFormatYCbCr420P010);
        fmts.push_back(kFormatYCbCr420P010Venus);
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
  HWDisplaysInfo hw_displays_info;
  DisplayError error = kErrorNone;

  hw_disp_info->type = kBuiltIn;
  hw_disp_info->is_connected = true;

  error = GetDisplaysStatus(&hw_displays_info);
  if (error == kErrorNone) {
    for (auto &iter : hw_displays_info) {
      auto &info = iter.second;
      if (info.is_primary) {
        hw_disp_info->type = info.display_type;
        hw_disp_info->is_connected = info.is_connected;
        break;
      }
    }
  }

  return error;
}

DisplayError HWInfoDRM::GetDisplaysStatus(HWDisplaysInfo *hw_displays_info) {
  static DebugTag log_once = kTagNone;

  if (!hw_displays_info) {
    DLOGE("No output parameter provided!");
    return kErrorParameters;
  }

  if (!drm_mgr_intf_) {
    DLOGE("DRM Driver not initialized!");
    return kErrorCriticalResource;
  }

  hw_displays_info->clear();
  sde_drm::DRMConnectorsInfo conns_info = {};
  int drm_err = drm_mgr_intf_->GetConnectorsInfo(&conns_info);
  if (drm_err) {
    DLOGE("DRM Driver error %d while getting displays' status!", drm_err);
    return kErrorUndefined;
  }

  for (auto &iter : conns_info) {
    HWDisplayInfo hw_info = {};
    hw_info.display_id =
        ((0 == iter.first) || (iter.first > INT32_MAX)) ? -1 : (int32_t)(iter.first);
    switch (iter.second.type) {
      case DRM_MODE_CONNECTOR_DSI:
        hw_info.display_type = kBuiltIn;
        break;
      case DRM_MODE_CONNECTOR_TV:
      case DRM_MODE_CONNECTOR_HDMIA:
      case DRM_MODE_CONNECTOR_HDMIB:
      case DRM_MODE_CONNECTOR_DisplayPort:
      case DRM_MODE_CONNECTOR_VGA:
        hw_info.display_type = kPluggable;
        break;
      case DRM_MODE_CONNECTOR_VIRTUAL:
        hw_info.display_type = kVirtual;
        break;
      default:
        DLOGE("Unknown display type = %d on connector id %u.", iter.second.type,
              hw_info.display_id);
        break;
    }
    hw_info.is_connected = iter.second.is_connected ? 1 : 0;
    hw_info.is_primary = iter.second.is_primary ? 1 : 0;
    hw_info.is_wb_ubwc_supported = iter.second.is_wb_ubwc_supported;
    if (hw_info.display_id >= 0) {
      (*hw_displays_info)[hw_info.display_id] = hw_info;
    }

    DLOGI_IF(log_once, "display: %4d-%d, connected: %s, primary: %s", hw_info.display_id,
             hw_info.display_type, hw_info.is_connected ? "true" : "false",
             hw_info.is_primary ? "true" : "false");
  }

  log_once = kTagDisplay;

  return kErrorNone;
}

DisplayError HWInfoDRM::GetMaxDisplaysSupported(const DisplayType type, int32_t *max_displays) {
  static DebugTag log_once = kTagNone;

  if (!max_displays) {
    DLOGE("No output parameter provided!");
    return kErrorParameters;
  }

  if (!drm_mgr_intf_) {
    DLOGE("DRM Driver not initialized!");
    return kErrorCriticalResource;
  }

  sde_drm::DRMEncodersInfo encoders_info = {};
  int drm_err = drm_mgr_intf_->GetEncodersInfo(&encoders_info);
  if (drm_err) {
    DLOGE("DRM Driver error %d while getting max displays supported!", drm_err);
    return kErrorUndefined;
  }

  int32_t max_displays_builtin = 0;
  int32_t max_displays_tmds = 0;
  int32_t max_displays_virtual = 0;
  int32_t max_displays_dpmst = 0;
  for (auto &iter : encoders_info) {
    switch (iter.second.type) {
      case DRM_MODE_ENCODER_DSI:
        max_displays_builtin++;
        break;
      case DRM_MODE_ENCODER_TMDS:
        max_displays_tmds++;
        break;
      case DRM_MODE_ENCODER_VIRTUAL:
        max_displays_virtual++;
        break;
      case DRM_MODE_ENCODER_DPMST:
        max_displays_dpmst++;
        break;
      default:
        break;
    }
  }

  switch (type) {
    case kBuiltIn:
      *max_displays = max_displays_builtin;
      break;
    case kPluggable:
      *max_displays = std::max(max_displays_tmds, max_displays_dpmst);
      break;
    case kVirtual:
      *max_displays = max_displays_virtual;
      break;
    case kDisplayTypeMax:
      *max_displays = max_displays_builtin + std::max(max_displays_tmds, max_displays_dpmst) +
                      max_displays_virtual;
      break;
    default:
      DLOGE("Unknown display type %d.", type);
      return kErrorParameters;
  }

  DLOGI_IF(log_once, "Max %d concurrent displays.",
           max_displays_builtin + std::max(max_displays_tmds, max_displays_dpmst) +
               max_displays_virtual);
  DLOGI_IF(log_once, "Max %d concurrent displays of type %d (BuiltIn).", max_displays_builtin,
           kBuiltIn);
  DLOGI_IF(log_once, "Max %d concurrent displays of type %d (Pluggable).",
           std::max(max_displays_tmds, max_displays_dpmst), kPluggable);
  DLOGI_IF(log_once, "Max %d concurrent displays of type %d (Virtual).", max_displays_virtual,
           kVirtual);

  log_once = kTagDisplay;

  return kErrorNone;
}

}  // namespace sdm
