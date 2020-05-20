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

/*
Changes from Qualcomm Innovation Center are provided under the following license:
Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the
disclaimer below) provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of Qualcomm Innovation Center, Inc. nor the
      names of its contributors may be used to endorse or promote
      products derived from this software without specific prior
      written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define __STDC_FORMAT_MACROS

#include <ctype.h>
#include <time.h>
#include <drm/drm_fourcc.h>
#include <drm_lib_loader.h>
#include <drm_master.h>
#include <drm_res_mgr.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/fb.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/formats.h>
#include <utils/sys.h>
#include <display/drm/sde_drm.h>
#include <private/color_params.h>
#include <utils/rect.h>
#include <utils/utils.h>
#include <utils/fence.h>

#include <sstream>
#include <ctime>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <limits>

#include "hw_device_drm.h"
#include "hw_info_interface.h"

#define __CLASS__ "HWDeviceDRM"

#ifndef DRM_FORMAT_MOD_QCOM_COMPRESSED
#define DRM_FORMAT_MOD_QCOM_COMPRESSED fourcc_mod_code(QCOM, 1)
#endif
#ifndef DRM_FORMAT_MOD_QCOM_DX
#define DRM_FORMAT_MOD_QCOM_DX fourcc_mod_code(QCOM, 0x2)
#endif
#ifndef DRM_FORMAT_MOD_QCOM_TIGHT
#define DRM_FORMAT_MOD_QCOM_TIGHT fourcc_mod_code(QCOM, 0x4)
#endif

#define DEST_SCALAR_OVERFETCH_SIZE 5

using std::string;
using std::to_string;
using std::fstream;
using std::unordered_map;
using std::stringstream;
using std::ifstream;
using std::ofstream;
using drm_utils::DRMMaster;
using drm_utils::DRMResMgr;
using drm_utils::DRMLibLoader;
using drm_utils::DRMBuffer;
using sde_drm::GetDRMManager;
using sde_drm::DestroyDRMManager;
using sde_drm::DRMDisplayType;
using sde_drm::DRMDisplayToken;
using sde_drm::DRMConnectorInfo;
using sde_drm::DRMPPFeatureInfo;
using sde_drm::DRMRect;
using sde_drm::DRMRotation;
using sde_drm::DRMBlendType;
using sde_drm::DRMSrcConfig;
using sde_drm::DRMOps;
using sde_drm::DRMTopology;
using sde_drm::DRMPowerMode;
using sde_drm::DRMSecureMode;
using sde_drm::DRMSecurityLevel;
using sde_drm::DRMCscType;
using sde_drm::DRMMultiRectMode;
using sde_drm::DRMCrtcInfo;
using sde_drm::DRMCWbCaptureMode;

namespace sdm {

std::atomic<uint32_t> HWDeviceDRM::hw_dest_scaler_blocks_used_(0);
HWCwbConfig HWDeviceDRM::cwb_config_ = {};
std::mutex HWDeviceDRM::cwb_state_lock_;
int HWDeviceDRM::display_count_ = 0;

static PPBlock GetPPBlock(const HWToneMapLut &lut_type) {
  PPBlock pp_block = kPPBlockMax;
  switch (lut_type) {
    case kDma1dIgc:
    case kDma1dGc:
      pp_block = kDGM;
      break;
    case kVig1dIgc:
    case kVig3dGamut:
      pp_block = kVIG;
      break;
    default:
      DLOGE("Unknown PP Block");
      break;
  }
  return pp_block;
}

static void GetDRMFormat(LayerBufferFormat format, uint32_t *drm_format,
                         uint64_t *drm_format_modifier) {
  switch (format) {
    case kFormatARGB8888:
      *drm_format = DRM_FORMAT_BGRA8888;
      break;
    case kFormatRGBA8888:
      *drm_format = DRM_FORMAT_ABGR8888;
      break;
    case kFormatRGBA8888Ubwc:
      *drm_format = DRM_FORMAT_ABGR8888;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatRGBA5551:
      *drm_format = DRM_FORMAT_ABGR1555;
      break;
    case kFormatRGBA4444:
      *drm_format = DRM_FORMAT_ABGR4444;
      break;
    case kFormatBGRA8888:
      *drm_format = DRM_FORMAT_ARGB8888;
      break;
    case kFormatRGBX8888:
      *drm_format = DRM_FORMAT_XBGR8888;
      break;
    case kFormatRGBX8888Ubwc:
      *drm_format = DRM_FORMAT_XBGR8888;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatBGRX8888:
      *drm_format = DRM_FORMAT_XRGB8888;
      break;
    case kFormatRGB888:
      *drm_format = DRM_FORMAT_BGR888;
      break;
    case kFormatBGR888:
      *drm_format = DRM_FORMAT_RGB888;
      break;
    case kFormatRGB565:
      *drm_format = DRM_FORMAT_BGR565;
      break;
    case kFormatBGR565:
      *drm_format = DRM_FORMAT_RGB565;
      break;
    case kFormatBGR565Ubwc:
      *drm_format = DRM_FORMAT_BGR565;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatRGBA1010102:
      *drm_format = DRM_FORMAT_ABGR2101010;
      break;
    case kFormatRGBA1010102Ubwc:
      *drm_format = DRM_FORMAT_ABGR2101010;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatARGB2101010:
      *drm_format = DRM_FORMAT_BGRA1010102;
      break;
    case kFormatRGBX1010102:
      *drm_format = DRM_FORMAT_XBGR2101010;
      break;
    case kFormatRGBX1010102Ubwc:
      *drm_format = DRM_FORMAT_XBGR2101010;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatXRGB2101010:
      *drm_format = DRM_FORMAT_BGRX1010102;
      break;
    case kFormatBGRA1010102:
      *drm_format = DRM_FORMAT_ARGB2101010;
      break;
    case kFormatABGR2101010:
      *drm_format = DRM_FORMAT_RGBA1010102;
      break;
    case kFormatBGRX1010102:
      *drm_format = DRM_FORMAT_XRGB2101010;
      break;
    case kFormatXBGR2101010:
      *drm_format = DRM_FORMAT_RGBX1010102;
      break;
    case kFormatYCbCr420SemiPlanar:
      *drm_format = DRM_FORMAT_NV12;
      break;
    case kFormatYCbCr420SemiPlanarVenus:
      *drm_format = DRM_FORMAT_NV12;
      break;
    case kFormatYCbCr420SPVenusUbwc:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatYCbCr420SPVenusTile:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_TILE;
      break;
    case kFormatYCrCb420SemiPlanar:
      *drm_format = DRM_FORMAT_NV21;
      break;
    case kFormatYCrCb420SemiPlanarVenus:
      *drm_format = DRM_FORMAT_NV21;
      break;
    case kFormatYCbCr420P010:
    case kFormatYCbCr420P010Venus:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_DX;
      break;
    case kFormatYCbCr420P010Ubwc:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED |
        DRM_FORMAT_MOD_QCOM_DX;
      break;
    case kFormatYCbCr420P010Tile:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_TILE |
        DRM_FORMAT_MOD_QCOM_DX;
      break;
    case kFormatYCbCr420TP10Ubwc:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED |
        DRM_FORMAT_MOD_QCOM_DX | DRM_FORMAT_MOD_QCOM_TIGHT;
      break;
    case kFormatYCbCr420TP10Tile:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_TILE |
        DRM_FORMAT_MOD_QCOM_DX | DRM_FORMAT_MOD_QCOM_TIGHT;
      break;
    case kFormatYCbCr422H2V1SemiPlanar:
      *drm_format = DRM_FORMAT_NV16;
      break;
    case kFormatYCrCb422H2V1SemiPlanar:
      *drm_format = DRM_FORMAT_NV61;
      break;
    case kFormatYCrCb420PlanarStride16:
      *drm_format = DRM_FORMAT_YVU420;
      break;
    default:
      DLOGW("Unsupported format %s", GetFormatString(format));
  }
}

class FrameBufferObject : public LayerBufferObject {
 public:
  explicit FrameBufferObject(uint32_t fb_id, LayerBufferFormat format,
                             uint32_t width, uint32_t height)
    :fb_id_(fb_id), format_(format), width_(width), height_(height) {
  }

  ~FrameBufferObject() {
    DRMMaster *master;
    DRMMaster::GetInstance(&master);
    int ret = master->RemoveFbId(fb_id_);
    if (ret < 0) {
      DLOGE("Removing fb_id %d failed with error %d", fb_id_, errno);
    }
  }
  uint32_t GetFbId() { return fb_id_; }
  bool IsEqual(LayerBufferFormat format, uint32_t width, uint32_t height) {
    return (format == format_ && width == width_ && height == height_);
  }

 private:
  uint32_t fb_id_;
  LayerBufferFormat format_;
  uint32_t width_;
  uint32_t height_;
};

HWDeviceDRM::Registry::Registry(BufferAllocator *buffer_allocator) :
  buffer_allocator_(buffer_allocator) {
  int value = 0;
  if (Debug::GetProperty(DISABLE_FBID_CACHE, &value) == kErrorNone) {
    disable_fbid_cache_ = (value == 1);
  }
}

void HWDeviceDRM::Registry::Register(HWLayersInfo *hw_layers_info) {
  uint32_t hw_layer_count = UINT32(hw_layers_info->hw_layers.size());

  for (uint32_t i = 0; i < hw_layer_count; i++) {
    Layer &layer = hw_layers_info->hw_layers.at(i);
    LayerBuffer input_buffer = layer.input_buffer;
    HWRotatorSession *hw_rotator_session = &hw_layers_info->config[i].hw_rotator_session;
    HWRotateInfo *hw_rotate_info = &hw_rotator_session->hw_rotate_info[0];
    fbid_cache_limit_ = input_buffer.flags.video ? VIDEO_FBID_LIMIT : UI_FBID_LIMIT;

    if (hw_rotator_session->mode == kRotatorOffline && hw_rotate_info->valid) {
      input_buffer = hw_rotator_session->output_buffer;
      fbid_cache_limit_ = OFFLINE_ROTATOR_FBID_LIMIT;
    }

    if (input_buffer.flags.interlace) {
      input_buffer.width *= 2;
      input_buffer.height /= 2;
    }
    MapBufferToFbId(&layer, input_buffer);
  }
}

int HWDeviceDRM::Registry::CreateFbId(const LayerBuffer &buffer, uint32_t *fb_id) {
  DRMMaster *master = nullptr;
  DRMMaster::GetInstance(&master);
  int ret = -1;

  if (!master) {
    DLOGE("Failed to acquire DRM Master instance");
    return ret;
  }

  DRMBuffer layout{};
  AllocatedBufferInfo buf_info{};
  buf_info.fd = layout.fd = buffer.planes[0].fd;
  buf_info.aligned_width = layout.width = buffer.width;
  buf_info.aligned_height = layout.height = buffer.height;
  buf_info.format = buffer.format;
  buf_info.usage = buffer.usage;
  GetDRMFormat(buf_info.format, &layout.drm_format, &layout.drm_format_modifier);
  buffer_allocator_->GetBufferLayout(buf_info, layout.stride, layout.offset, &layout.num_planes);
  ret = master->CreateFbId(layout, fb_id);
  if (ret < 0) {
    DLOGE("CreateFbId failed. width %d, height %d, format: %s, stride %u, error %d",
        layout.width, layout.height, GetFormatString(buf_info.format), layout.stride[0], errno);
  }

  return ret;
}

void HWDeviceDRM::Registry::MapBufferToFbId(Layer* layer, const LayerBuffer &buffer) {
  if (buffer.planes[0].fd < 0) {
    return;
  }

  uint64_t handle_id = buffer.handle_id;
  if (!handle_id || disable_fbid_cache_) {
    // In legacy path, clear fb_id map in each frame.
    layer->buffer_map->buffer_map.clear();
  } else {
    auto it = layer->buffer_map->buffer_map.find(handle_id);
    if (it != layer->buffer_map->buffer_map.end()) {
      FrameBufferObject *fb_obj = static_cast<FrameBufferObject*>(it->second.get());
      if (fb_obj->IsEqual(buffer.format, buffer.width, buffer.height)) {
        // Found fb_id for given handle_id key
        return;
      } else {
        // Erase from fb_id map if format or size have been modified
        layer->buffer_map->buffer_map.erase(it);
      }
    }

    if (layer->buffer_map->buffer_map.size() >= fbid_cache_limit_) {
      // Clear fb_id map, if the size reaches cache limit.
      layer->buffer_map->buffer_map.clear();
    }
  }

  uint32_t fb_id = 0;
  if (CreateFbId(buffer, &fb_id) >= 0) {
    // Create and cache the fb_id in map
    layer->buffer_map->buffer_map[handle_id] = std::make_shared<FrameBufferObject>(fb_id,
        buffer.format, buffer.width, buffer.height);
  }
}

void HWDeviceDRM::Registry::MapOutputBufferToFbId(LayerBuffer *output_buffer) {
  if (output_buffer->planes[0].fd < 0) {
    return;
  }

  uint64_t handle_id = output_buffer->handle_id;
  if (!handle_id || disable_fbid_cache_) {
    // In legacy path, clear output buffer map in each frame.
    output_buffer_map_.clear();
  } else {
    auto it = output_buffer_map_.find(handle_id);
    if (it != output_buffer_map_.end()) {
      FrameBufferObject *fb_obj = static_cast<FrameBufferObject*>(it->second.get());
      if (fb_obj->IsEqual(output_buffer->format, output_buffer->width, output_buffer->height)) {
        return;
      } else {
        output_buffer_map_.erase(it);
      }
    }

    if (output_buffer_map_.size() >= UI_FBID_LIMIT) {
      // Clear output buffer map, if the size reaches cache limit.
      output_buffer_map_.clear();
    }
  }

  uint32_t fb_id = 0;
  if (CreateFbId(*output_buffer, &fb_id) >= 0) {
    output_buffer_map_[handle_id] = std::make_shared<FrameBufferObject>(fb_id,
        output_buffer->format, output_buffer->width, output_buffer->height);
  }
}

void HWDeviceDRM::Registry::Clear() {
  output_buffer_map_.clear();
}

uint32_t HWDeviceDRM::Registry::GetFbId(Layer *layer, uint64_t handle_id) {
  auto it = layer->buffer_map->buffer_map.find(handle_id);
  if (it != layer->buffer_map->buffer_map.end()) {
    FrameBufferObject *fb_obj = static_cast<FrameBufferObject*>(it->second.get());
    return fb_obj->GetFbId();
  }

  return 0;
}

uint32_t HWDeviceDRM::Registry::GetOutputFbId(uint64_t handle_id) {
  auto it = output_buffer_map_.find(handle_id);
  if (it != output_buffer_map_.end()) {
    FrameBufferObject *fb_obj = static_cast<FrameBufferObject*>(it->second.get());
    return fb_obj->GetFbId();
  }

  return 0;
}

HWDeviceDRM::HWDeviceDRM(BufferAllocator *buffer_allocator, HWInfoInterface *hw_info_intf)
    : hw_info_intf_(hw_info_intf), registry_(buffer_allocator) {
  hw_info_intf_ = hw_info_intf;
}

DisplayError HWDeviceDRM::Init() {
  int ret = 0;
  DRMMaster *drm_master = {};
  DRMMaster::GetInstance(&drm_master);
  drm_master->GetHandle(&dev_fd_);
  DRMLibLoader *drm_lib_loader = DRMLibLoader::GetInstance();

  if (!drm_lib_loader) {
    DLOGW("Failed to retrieve DRMLibLoader instance");
    return kErrorResources;
  }

  drm_lib_loader->FuncGetDRMManager()(dev_fd_, &drm_mgr_intf_);

  if (-1 == display_id_) {
    if (drm_mgr_intf_->RegisterDisplay(disp_type_, &token_)) {
      DLOGE("RegisterDisplay (by type) failed for %s", device_name_);
      return kErrorResources;
    }
  } else if (drm_mgr_intf_->RegisterDisplay(display_id_, &token_)) {
    DLOGE("RegisterDisplay (by id) failed for %s - %d", device_name_, display_id_);
    return kErrorResources;
  }

  if (token_.conn_id > INT32_MAX) {
    DLOGE("Connector id %u beyond supported range", token_.conn_id);
    drm_mgr_intf_->UnregisterDisplay(&token_);
    return kErrorNotSupported;
  }

  display_id_ = static_cast<int32_t>(token_.conn_id);

  ret = drm_mgr_intf_->CreateAtomicReq(token_, &drm_atomic_intf_);
  if (ret) {
    DLOGE("Failed creating atomic request for connector id %u. Error: %d.", token_.conn_id, ret);
    drm_mgr_intf_->UnregisterDisplay(&token_);
    return kErrorResources;
  }

  ret = drm_mgr_intf_->GetConnectorInfo(token_.conn_id, &connector_info_);
  if (ret) {
    DLOGE("Failed getting info for connector id %u. Error: %d.", token_.conn_id, ret);
    drm_mgr_intf_->DestroyAtomicReq(drm_atomic_intf_);
    drm_atomic_intf_ = {};
    drm_mgr_intf_->UnregisterDisplay(&token_);
    return kErrorHardware;
  }

  if (!connector_info_.is_connected || connector_info_.modes.empty()) {
    DLOGW("Device removal detected on connector id %u. Connector status %s and %zu modes.",
          token_.conn_id, connector_info_.is_connected ? "connected":"disconnected",
          connector_info_.modes.size());
    drm_mgr_intf_->DestroyAtomicReq(drm_atomic_intf_);
    drm_atomic_intf_ = {};
    drm_mgr_intf_->UnregisterDisplay(&token_);
    return kErrorDeviceRemoved;
  }

  hw_info_intf_->GetHWResourceInfo(&hw_resource_);

  InitializeConfigs();
  PopulateHWPanelInfo();
  UpdateMixerAttributes();

  // TODO(user): In future, remove has_qseed3 member, add version and pass version to constructor
  if (hw_resource_.has_qseed3) {
    hw_scale_ = new HWScaleDRM(HWScaleDRM::Version::V2);
  }

  std::unique_ptr<HWColorManagerDrm> hw_color_mgr(new HWColorManagerDrm());
  hw_color_mgr_ = std::move(hw_color_mgr);

  GetCWBCapabilities();
  display_count_ += 1;
  return kErrorNone;
}

DisplayError HWDeviceDRM::Deinit() {
  DisplayError err = kErrorNone;
  // first_null_cycle_ = false after the first power-mode NullCommit is completed, or the first
  // frame commit handled the power transition.
  // Power-on will set the CRTC_SET_MODE anytime power mode is not deferred to first commit.
  // Without first commit, if display is disconnected, CRTC_SET_MODE is not set to NULL,
  // this leads to a synchronization issue.
  // So because of previously successful NullCommit, set CRTC_SET_MODE to NULL here for proper sync.

  // first_cycle = false after the first frame commit is completed.
  // A null-commit here is also needed if the first commit has gone through. e.g., If a
  // display is connected and disconnected, HWDeviceDRM::Deinit() may be called
  // before any driver commit happened on the device. The driver may have removed any not-in-use
  // connector (i.e., any connector which did not have a display commit on it and a crtc path
  // setup), so token_.conn_id may have been removed if there was no commit, resulting in
  // drmModeAtomicCommit() failure with ENOENT, 'No such file or directory'.
  if (!first_cycle_ || !first_null_cycle_) {
    ClearSolidfillStages();
    ClearNoiseLayerConfig();
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, 0);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::OFF);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, nullptr);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 0);
#ifdef TRUSTED_VM
    drm_atomic_intf_->Perform(sde_drm::DRMOps::CRTC_SET_VM_REQ_STATE, token_.crtc_id,
                              sde_drm::DRMVMRequestState::RELEASE);
#endif
    int ret = NullCommit(true /* synchronous */, false /* retain_planes */);
    if (ret) {
      DLOGE("Commit failed with error: %d", ret);
      err = kErrorHardware;
    }
  }
  delete hw_scale_;
  registry_.Clear();
  display_attributes_ = {};
  drm_mgr_intf_->DestroyAtomicReq(drm_atomic_intf_);
  drm_atomic_intf_ = {};
  drm_mgr_intf_->UnregisterDisplay(&token_);
  hw_dest_scaler_blocks_used_ -= dest_scaler_blocks_used_;
  display_count_ -= 1;
  return err;
}

void HWDeviceDRM::GetCWBCapabilities() {
  sde_drm::DRMConnectorsInfo conns_info = {};
  int ret = drm_mgr_intf_->GetConnectorsInfo(&conns_info);
  if (ret) {
    DLOGW("DRM Driver error %d while getting Connectors info.", ret);
    return;
  }
  for (auto &iter : conns_info) {
    if (iter.second.type == DRM_MODE_CONNECTOR_VIRTUAL) {
      has_cwb_crop_ = static_cast<bool>(iter.second.modes[current_mode_index_].has_cwb_crop);
      has_dedicated_cwb_ =
          static_cast<bool>(iter.second.modes[current_mode_index_].has_dedicated_cwb);
      has_cwb_dither_ = static_cast<bool>(iter.second.has_cwb_dither);
      break;
    }
  }
}

void HWDeviceDRM::OverrideConnectorInfo() {
  // Overriding panel orientation
  int value = 0;
  if ((Debug::GetProperty(ENABLE_PANEL_INVERSE_MOUNT, &value) == kErrorNone) &&
      connector_info_.is_primary) {
    if (value == 1) {
      connector_info_.panel_orientation = DRMRotation::ROT_180;
      DLOGI("Panel_Orientation:%d -- %d-%d",
      connector_info_.panel_orientation, display_id_, disp_type_);
    }
  }
}

DisplayError HWDeviceDRM::GetDisplayId(int32_t *display_id) {
  *display_id = display_id_;
  return kErrorNone;
}

void HWDeviceDRM::InitializeConfigs() {
  current_mode_index_ = 0;
  uint32_t modes_count = connector_info_.modes.size();
  uint32_t panel_mode_pref = (connector_info_.panel_mode == sde_drm::DRMPanelMode::COMMAND)
                                 ? DRM_MODE_FLAG_CMD_MODE_PANEL
                                 : DRM_MODE_FLAG_VID_MODE_PANEL;

  // Set mode with preferred panel mode if supported, otherwise set based on capability
  for (uint32_t mode_index = 0; mode_index < modes_count; mode_index++) {
    uint32_t sub_mode_index = connector_info_.modes[mode_index].curr_submode_index;
    connector_info_.modes[mode_index].curr_compression_mode =
              connector_info_.modes[mode_index].sub_modes[sub_mode_index].panel_compression_mode;
    if (panel_mode_pref &
        connector_info_.modes[mode_index].sub_modes[sub_mode_index].panel_mode_caps) {
      connector_info_.modes[mode_index].cur_panel_mode = panel_mode_pref;
    } else if (panel_mode_pref == DRM_MODE_FLAG_VID_MODE_PANEL) {
      connector_info_.modes[mode_index].cur_panel_mode = DRM_MODE_FLAG_VID_MODE_PANEL;
    } else if (panel_mode_pref == DRM_MODE_FLAG_CMD_MODE_PANEL) {
      connector_info_.modes[mode_index].cur_panel_mode = DRM_MODE_FLAG_CMD_MODE_PANEL;
    }
    // Add mode variant if both panel modes are supported
    if (connector_info_.modes[mode_index].sub_modes[sub_mode_index].panel_mode_caps &
        DRM_MODE_FLAG_CMD_MODE_PANEL &&
        connector_info_.modes[mode_index].sub_modes[sub_mode_index].panel_mode_caps &
        DRM_MODE_FLAG_VID_MODE_PANEL) {
      sde_drm::DRMModeInfo mode_item = connector_info_.modes[mode_index];
      mode_item.cur_panel_mode =
          (connector_info_.modes[mode_index].cur_panel_mode == DRM_MODE_FLAG_CMD_MODE_PANEL)
              ? DRM_MODE_FLAG_VID_MODE_PANEL
              : DRM_MODE_FLAG_CMD_MODE_PANEL;
      connector_info_.modes.push_back(mode_item);
    }
  }
  // Update current mode with preferred mode
  for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if (connector_info_.modes[mode_index].mode.type & DRM_MODE_TYPE_PREFERRED) {
        DLOGI("Updating current display mode %d to preferred mode %d.", current_mode_index_,
              mode_index);
        current_mode_index_ = mode_index;
        break;
      }
  }

  display_attributes_.resize(connector_info_.modes.size());

  uint32_t width = connector_info_.modes[current_mode_index_].mode.hdisplay;
  uint32_t height = connector_info_.modes[current_mode_index_].mode.vdisplay;
  for (uint32_t i = 0; i < connector_info_.modes.size(); i++) {
    auto &mode = connector_info_.modes[i].mode;
    if (mode.hdisplay != width || mode.vdisplay != height) {
      resolution_switch_enabled_ = true;
    }
    PopulateDisplayAttributes(i);
  }
  SetDisplaySwitchMode(current_mode_index_);
}

DisplayError HWDeviceDRM::PopulateDisplayAttributes(uint32_t index) {
  drmModeModeInfo mode = {};
  sde_drm::DRMModeInfo conn_mode = {};
  uint32_t mm_width = 0;
  uint32_t mm_height = 0;
  DRMTopology topology = DRMTopology::SINGLE_LM;

  if (default_mode_) {
    DRMResMgr *res_mgr = nullptr;
    int ret = DRMResMgr::GetInstance(&res_mgr);
    if (ret < 0) {
      DLOGE("Failed to acquire DRMResMgr instance");
      return kErrorResources;
    }

    res_mgr->GetMode(&mode);
    res_mgr->GetDisplayDimInMM(&mm_width, &mm_height);
  } else {
    uint32_t submode_idx = connector_info_.modes[index].curr_submode_index;
    conn_mode = connector_info_.modes[index];
    mode = conn_mode.mode;
    mm_width = connector_info_.mmWidth;
    mm_height = connector_info_.mmHeight;
    topology = connector_info_.modes[index].sub_modes[submode_idx].topology;
    if (connector_info_.modes[index].sub_modes[submode_idx].panel_mode_caps &
        DRM_MODE_FLAG_CMD_MODE_PANEL) {
      display_attributes_[index].smart_panel = true;
    }
  }

  display_attributes_[index].x_pixels = mode.hdisplay;
  display_attributes_[index].y_pixels = mode.vdisplay;
  display_attributes_[index].fps = mode.vrefresh;
  display_attributes_[index].vsync_period_ns =
    UINT32(1000000000L / display_attributes_[index].fps);

  /*
              Active                 Front           Sync           Back
              Region                 Porch                          Porch
     <-----------------------><----------------><-------------><-------------->
     <----- [hv]display ----->
     <------------- [hv]sync_start ------------>
     <--------------------- [hv]sync_end --------------------->
     <-------------------------------- [hv]total ----------------------------->
   */

  display_attributes_[index].v_front_porch = mode.vsync_start - mode.vdisplay;
  bool adjusted = (connector_info_.dyn_bitclk_support &&
                   conn_mode.fp_type == sde_drm::DynamicFrontPorchType::VERTICAL &&
                   !conn_mode.dyn_fp_list.empty());
  if (adjusted) {
    display_attributes_[index].v_front_porch =
        *std::min_element(conn_mode.dyn_fp_list.begin(), conn_mode.dyn_fp_list.end());
  }
  display_attributes_[index].v_pulse_width = mode.vsync_end - mode.vsync_start;
  display_attributes_[index].v_back_porch = mode.vtotal - mode.vsync_end;
  display_attributes_[index].v_total = mode.vtotal;
  display_attributes_[index].h_total = mode.htotal;

  // TODO(user): This clock should no longer be used as mode's pixel clock for RFI connectors.
  // Driver can expose list of dynamic pixel clocks, userspace needs to support dynamic change.
  display_attributes_[index].clock_khz = mode.clock;

  // If driver doesn't return panel width/height information, default to 320 dpi
  if (INT(mm_width) <= 0 || INT(mm_height) <= 0) {
    mm_width  = UINT32(((FLOAT(mode.hdisplay) * 25.4f) / 320.0f) + 0.5f);
    mm_height = UINT32(((FLOAT(mode.vdisplay) * 25.4f) / 320.0f) + 0.5f);
    DLOGW("Driver doesn't report panel physical width and height - defaulting to 320dpi");
  }

  display_attributes_[index].x_dpi = (FLOAT(mode.hdisplay) * 25.4f) / FLOAT(mm_width);
  display_attributes_[index].y_dpi = (FLOAT(mode.vdisplay) * 25.4f) / FLOAT(mm_height);
  SetTopology(topology, &display_attributes_[index].topology);
  SetTopologySplit(display_attributes_[index].topology,
                   &display_attributes_[index].topology_num_split);
  SetTopologyMuxUsage(display_attributes_[index].topology,
                      &display_attributes_[index].is_3d_mux_used);
  display_attributes_[index].is_device_split = (display_attributes_[index].topology_num_split > 1);

  DLOGI(
      "Display attributes[%d]: WxH: %dx%d, DPI: %fx%f, FPS: %d, LM_SPLIT: %d, V_BACK_PORCH: %d,"
      " V_FRONT_PORCH: %d [RFI Adjusted : %s], V_PULSE_WIDTH: %d, V_TOTAL: %d, H_TOTAL: %d,"
      " CLK: %dKHZ, TOPOLOGY: %d [SPLIT NUMBER: %d], HW_SPLIT: %d",
      index, display_attributes_[index].x_pixels, display_attributes_[index].y_pixels,
      display_attributes_[index].x_dpi, display_attributes_[index].y_dpi,
      display_attributes_[index].fps, display_attributes_[index].is_device_split,
      display_attributes_[index].v_back_porch, display_attributes_[index].v_front_porch,
      adjusted ? "True" : "False", display_attributes_[index].v_pulse_width,
      display_attributes_[index].v_total, display_attributes_[index].h_total,
      display_attributes_[index].clock_khz, display_attributes_[index].topology,
      display_attributes_[index].topology_num_split, mixer_attributes_.split_type);

  return kErrorNone;
}

void HWDeviceDRM::PopulateHWPanelInfo() {
  hw_panel_info_ = {};

  snprintf(hw_panel_info_.panel_name, sizeof(hw_panel_info_.panel_name), "%s",
           connector_info_.panel_name.c_str());

  uint32_t index = current_mode_index_;
  uint32_t sub_mode_index = connector_info_.modes[index].curr_submode_index;
  hw_panel_info_.split_info.left_split = display_attributes_[index].x_pixels;
  if (display_attributes_[index].is_device_split) {
    hw_panel_info_.split_info.left_split = hw_panel_info_.split_info.right_split =
        display_attributes_[index].x_pixels / 2;
  }

  hw_panel_info_.partial_update = connector_info_.modes[index].num_roi;
  hw_panel_info_.left_roi_count = UINT32(connector_info_.modes[index].num_roi);
  hw_panel_info_.right_roi_count = UINT32(connector_info_.modes[index].num_roi);
  hw_panel_info_.left_align = connector_info_.modes[index].xstart;
  hw_panel_info_.top_align = connector_info_.modes[index].ystart;
  hw_panel_info_.width_align = connector_info_.modes[index].walign;
  hw_panel_info_.height_align = connector_info_.modes[index].halign;
  hw_panel_info_.min_roi_width = connector_info_.modes[index].wmin;
  hw_panel_info_.min_roi_height = connector_info_.modes[index].hmin;
  hw_panel_info_.needs_roi_merge = connector_info_.modes[index].roi_merge;
  hw_panel_info_.transfer_time_us = connector_info_.modes[index].transfer_time_us;
  hw_panel_info_.allowed_mode_switch = connector_info_.modes[index].allowed_mode_switch;
  hw_panel_info_.panel_mode_caps =
                 connector_info_.modes[index].sub_modes[sub_mode_index].panel_mode_caps;
  hw_panel_info_.dynamic_fps = connector_info_.dynamic_fps;
  hw_panel_info_.qsync_support = connector_info_.qsync_support;
  drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
  if (hw_panel_info_.dynamic_fps) {
    uint32_t min_fps = current_mode.vrefresh;
    uint32_t max_fps = current_mode.vrefresh;
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay)) {
        if (min_fps > connector_info_.modes[mode_index].mode.vrefresh)  {
          min_fps = connector_info_.modes[mode_index].mode.vrefresh;
        }
        if (max_fps < connector_info_.modes[mode_index].mode.vrefresh)  {
          max_fps = connector_info_.modes[mode_index].mode.vrefresh;
        }
      }
    }
    hw_panel_info_.min_fps = min_fps;
    hw_panel_info_.max_fps = max_fps;
  } else {
    hw_panel_info_.min_fps = current_mode.vrefresh;
    hw_panel_info_.max_fps = current_mode.vrefresh;
  }

  uint32_t min_transfer_time_us = hw_panel_info_.transfer_time_us;
  for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
    if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
        (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay)) {
      if (min_transfer_time_us > connector_info_.modes[mode_index].transfer_time_us)  {
        min_transfer_time_us = connector_info_.modes[mode_index].transfer_time_us;
      }
    }
  }
  hw_panel_info_.min_transfer_time_us = min_transfer_time_us;

  if (connector_info_.qsync_fps > 0) {
    // For command mode panel, driver will set connector property qsync_fps
    hw_panel_info_.qsync_fps = connector_info_.qsync_fps;
  } else {
    // if for video mode panel, qsync_fps is not set, take default min_fps value
    hw_panel_info_.qsync_fps = hw_panel_info_.min_fps;
  }

  hw_panel_info_.is_primary_panel = connector_info_.is_primary;
  hw_panel_info_.is_pluggable = 0;
  hw_panel_info_.hdr_enabled = connector_info_.panel_hdr_prop.hdr_enabled;
  // Convert the luminance values to cd/m^2 units.
  hw_panel_info_.peak_luminance = FLOAT(connector_info_.panel_hdr_prop.peak_brightness) / 10000.0f;
  hw_panel_info_.blackness_level = FLOAT(connector_info_.panel_hdr_prop.blackness_level) / 10000.0f;
  hw_panel_info_.average_luminance = FLOAT(connector_info_.panel_hdr_prop.peak_brightness +
                                           connector_info_.panel_hdr_prop.blackness_level) /
                                           (2 * 10000.0f);
  hw_panel_info_.primaries.white_point[0] = connector_info_.panel_hdr_prop.display_primaries[0];
  hw_panel_info_.primaries.white_point[1] = connector_info_.panel_hdr_prop.display_primaries[1];
  hw_panel_info_.primaries.red[0] = connector_info_.panel_hdr_prop.display_primaries[2];
  hw_panel_info_.primaries.red[1] = connector_info_.panel_hdr_prop.display_primaries[3];
  hw_panel_info_.primaries.green[0] = connector_info_.panel_hdr_prop.display_primaries[4];
  hw_panel_info_.primaries.green[1] = connector_info_.panel_hdr_prop.display_primaries[5];
  hw_panel_info_.primaries.blue[0] = connector_info_.panel_hdr_prop.display_primaries[6];
  hw_panel_info_.primaries.blue[1] = connector_info_.panel_hdr_prop.display_primaries[7];
  hw_panel_info_.dyn_bitclk_support = connector_info_.dyn_bitclk_support;

  OverrideConnectorInfo();

  // no supprt for 90 rotation only flips or 180 supported
  hw_panel_info_.panel_orientation.rotation = 0;
  hw_panel_info_.panel_orientation.flip_horizontal =
    (connector_info_.panel_orientation == DRMRotation::FLIP_H) ||
    (connector_info_.panel_orientation == DRMRotation::ROT_180);
  hw_panel_info_.panel_orientation.flip_vertical =
    (connector_info_.panel_orientation == DRMRotation::FLIP_V) ||
    (connector_info_.panel_orientation == DRMRotation::ROT_180);

  GetHWDisplayPortAndMode();

  if (connector_info_.modes[current_mode_index_].cur_panel_mode & DRM_MODE_FLAG_CMD_MODE_PANEL) {
    hw_panel_info_.mode = kModeCommand;
  }

  if (connector_info_.modes[current_mode_index_].cur_panel_mode &
             DRM_MODE_FLAG_VID_MODE_PANEL) {
    hw_panel_info_.mode = kModeVideo;
  }

  DLOGI_IF(kTagDriverConfig, "%s, Panel Interface = %s, Panel Mode = %s, Is Primary = %d",
           device_name_, interface_str_.c_str(),
           hw_panel_info_.mode == kModeVideo ? "Video" : "Command",
           hw_panel_info_.is_primary_panel);
  DLOGI_IF(kTagDriverConfig, "Partial Update = %d, Dynamic FPS = %d, HDR Panel = %d QSync = %d",
           hw_panel_info_.partial_update, hw_panel_info_.dynamic_fps, hw_panel_info_.hdr_enabled,
           hw_panel_info_.qsync_support);
  DLOGI_IF(kTagDriverConfig, "Align: left = %d, width = %d, top = %d, height = %d",
           hw_panel_info_.left_align, hw_panel_info_.width_align, hw_panel_info_.top_align,
           hw_panel_info_.height_align);
  DLOGI_IF(kTagDriverConfig, "ROI: min_width = %d, min_height = %d, need_merge = %d",
           hw_panel_info_.min_roi_width, hw_panel_info_.min_roi_height,
           hw_panel_info_.needs_roi_merge);
  DLOGI_IF(kTagDriverConfig, "FPS: min = %d, max = %d", hw_panel_info_.min_fps,
           hw_panel_info_.max_fps);
  DLOGI_IF(kTagDriverConfig, "Left Split = %d, Right Split = %d",
           hw_panel_info_.split_info.left_split, hw_panel_info_.split_info.right_split);
  DLOGI_IF(kTagDriverConfig, "Mode Transfer time = %d us", hw_panel_info_.transfer_time_us);
  DLOGI_IF(kTagDriverConfig, "Panel Minimum Transfer time = %d us",
    hw_panel_info_.min_transfer_time_us);
  DLOGI_IF(kTagDriverConfig, "Dynamic Bit Clk Support = %d", hw_panel_info_.dyn_bitclk_support);
}

DisplayError HWDeviceDRM::GetDisplayIdentificationData(uint8_t *out_port, uint32_t *out_data_size,
                                                       uint8_t *out_data) {
  *out_port = token_.hw_port;
  std::vector<uint8_t> &edid = connector_info_.edid;

  if (out_data == nullptr) {
    *out_data_size = (uint32_t)(edid.size());
    if (*out_data_size == 0) {
      DLOGE("EDID blob is empty, no data to return");
      return kErrorDriverData;
    }
  } else {
    *out_data_size = std::min(*out_data_size, (uint32_t)(edid.size()));
    memcpy(out_data, edid.data(), *out_data_size);
  }

  return kErrorNone;
}

void HWDeviceDRM::GetHWDisplayPortAndMode() {
  hw_panel_info_.port = kPortDefault;
  hw_panel_info_.mode =
      (connector_info_.panel_mode == sde_drm::DRMPanelMode::VIDEO) ? kModeVideo : kModeCommand;

  if (default_mode_) {
    return;
  }

  switch (connector_info_.type) {
    case DRM_MODE_CONNECTOR_DSI:
      hw_panel_info_.port = kPortDSI;
      interface_str_ = "DSI";
      break;
    case DRM_MODE_CONNECTOR_LVDS:
      hw_panel_info_.port = kPortLVDS;
      interface_str_ = "LVDS";
      break;
    case DRM_MODE_CONNECTOR_eDP:
      hw_panel_info_.port = kPortEDP;
      interface_str_ = "EDP";
      break;
    case DRM_MODE_CONNECTOR_TV:
    case DRM_MODE_CONNECTOR_HDMIA:
    case DRM_MODE_CONNECTOR_HDMIB:
      hw_panel_info_.port = kPortDTV;
      interface_str_ = "HDMI";
      break;
    case DRM_MODE_CONNECTOR_VIRTUAL:
      hw_panel_info_.port = kPortWriteBack;
      interface_str_ = "Virtual";
      break;
    case DRM_MODE_CONNECTOR_DisplayPort:
      hw_panel_info_.port = kPortDP;
      interface_str_ = "DisplayPort";
      break;
  }

  return;
}

DisplayError HWDeviceDRM::GetActiveConfig(uint32_t *active_config) {
  *active_config = current_mode_index_;
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetNumDisplayAttributes(uint32_t *count) {
  *count = UINT32(display_attributes_.size());
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetDisplayAttributes(uint32_t index,
                                               HWDisplayAttributes *display_attributes) {
  if (index >= display_attributes_.size()) {
    DLOGW("Index > display_attributes_.size(). Return.");
    return kErrorParameters;
  }
  *display_attributes = display_attributes_[index];
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetHWPanelInfo(HWPanelInfo *panel_info) {
  *panel_info = hw_panel_info_;
  return kErrorNone;
}

void HWDeviceDRM::SetDisplaySwitchMode(uint32_t index) {
  if (current_mode_index_ == index && !first_cycle_) {
    DLOGI("Mode %d already set", index);
    return;
  }
  uint32_t mode_flag = 0;
  uint32_t curr_mode_flag = 0, switch_mode_flag = 0;
  sde_drm::DRMModeInfo to_set = connector_info_.modes[index];
  sde_drm::DRMModeInfo current_mode = connector_info_.modes[current_mode_index_];
  uint64_t target_bit_clk = connector_info_.modes[current_mode_index_].curr_bit_clk_rate;
  uint32_t target_compression = connector_info_.modes[current_mode_index_].curr_compression_mode;
  uint32_t switch_index  = 0;

  if (to_set.cur_panel_mode & DRM_MODE_FLAG_CMD_MODE_PANEL) {
    mode_flag = DRM_MODE_FLAG_CMD_MODE_PANEL;
    switch_mode_flag = DRM_MODE_FLAG_VID_MODE_PANEL;
  } else if (to_set.cur_panel_mode & DRM_MODE_FLAG_VID_MODE_PANEL) {
    mode_flag = DRM_MODE_FLAG_VID_MODE_PANEL;
    switch_mode_flag = DRM_MODE_FLAG_CMD_MODE_PANEL;
  }

  if (current_mode.cur_panel_mode & DRM_MODE_FLAG_CMD_MODE_PANEL) {
    curr_mode_flag = DRM_MODE_FLAG_CMD_MODE_PANEL;
  } else if (current_mode.cur_panel_mode & DRM_MODE_FLAG_VID_MODE_PANEL) {
    curr_mode_flag = DRM_MODE_FLAG_VID_MODE_PANEL;
  }

  if (curr_mode_flag != mode_flag) {
    panel_mode_changed_ = mode_flag;
  }

  for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
    if ((to_set.mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
        (to_set.mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
        (to_set.mode.vrefresh == connector_info_.modes[mode_index].mode.vrefresh) &&
        (mode_flag & connector_info_.modes[mode_index].cur_panel_mode)) {
      for (uint32_t submode_idx = 0; submode_idx <
           connector_info_.modes[mode_index].sub_modes.size(); submode_idx++) {
        sde_drm::DRMSubModeInfo sub_mode = connector_info_.modes[mode_index].sub_modes[submode_idx];
        if (sub_mode.panel_compression_mode == target_compression) {
          connector_info_.modes[mode_index].curr_submode_index = submode_idx;
          index = mode_index;
          to_set.curr_bit_clk_rate = GetSupportedBitClkRate(index, target_bit_clk);
          break;
        }
      }
      break;
    }
  }

  current_mode_index_ = index;

  switch_mode_valid_ = false;
  for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
    if ((to_set.mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
        (to_set.mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
        (to_set.mode.vrefresh == connector_info_.modes[mode_index].mode.vrefresh) &&
        (switch_mode_flag & connector_info_.modes[mode_index].cur_panel_mode)) {
      for (uint32_t submode_idx = 0; submode_idx <
           connector_info_.modes[mode_index].sub_modes.size(); submode_idx++) {
        sde_drm::DRMSubModeInfo sub_mode = connector_info_.modes[mode_index].sub_modes[submode_idx];
        if (sub_mode.panel_compression_mode == target_compression) {
          connector_info_.modes[mode_index].curr_submode_index = submode_idx;
          switch_index = mode_index;
          switch_mode_valid_ = true;
          break;
        }
      }
      break;
    }
  }

  if (!switch_mode_valid_) {
    // in case there is no corresponding switch mode with same fps, try for a switch
    // mode with lowest fps. This is to handle cases where there are multiple video mode fps
    // but only one command mode for doze like 30 fps.
    uint32_t refresh_rate = 0;
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((to_set.mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (to_set.mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
          (switch_mode_flag & connector_info_.modes[mode_index].cur_panel_mode)) {
        if (!refresh_rate || (refresh_rate > connector_info_.modes[mode_index].mode.vrefresh)) {
          for (uint32_t submode_idx = 0; submode_idx <
               connector_info_.modes[mode_index].sub_modes.size(); submode_idx++) {
            sde_drm::DRMSubModeInfo sub_mode =
                     connector_info_.modes[mode_index].sub_modes[submode_idx];
           if (sub_mode.panel_compression_mode == target_compression) {
              switch_index = mode_index;
              switch_mode_valid_ = true;
              refresh_rate = connector_info_.modes[mode_index].mode.vrefresh;
              break;
            }
          }
        }
      }
    }
  }

  if (switch_mode_valid_) {
    if (mode_flag & DRM_MODE_FLAG_VID_MODE_PANEL) {
      video_mode_index_ = current_mode_index_;
      cmd_mode_index_ = switch_index;
    } else {
      video_mode_index_ = switch_index;
      cmd_mode_index_ = current_mode_index_;
    }
  }
  if (current_mode.mode.hdisplay == to_set.mode.hdisplay &&
    current_mode.mode.vdisplay == to_set.mode.vdisplay) {
    seamless_mode_switch_ = true;
  }
}

DisplayError HWDeviceDRM::SetDisplayAttributes(uint32_t index) {
  if (index >= display_attributes_.size()) {
    DLOGE("Invalid mode index %d mode size %d", index, UINT32(display_attributes_.size()));
    return kErrorParameters;
  }

  SetDisplaySwitchMode(index);
  PopulateHWPanelInfo();
  UpdateMixerAttributes();

  DLOGI_IF(kTagDriverConfig,
      "Display attributes[%d]: WxH: %dx%d, DPI: %fx%f, FPS: %d, LM_SPLIT: %d, V_BACK_PORCH: %d,"
      " V_FRONT_PORCH: %d, V_PULSE_WIDTH: %d, V_TOTAL: %d, H_TOTAL: %d, CLK: %dKHZ, "
      "TOPOLOGY: %d, PanelMode %s",
      index, display_attributes_[index].x_pixels, display_attributes_[index].y_pixels,
      display_attributes_[index].x_dpi, display_attributes_[index].y_dpi,
      display_attributes_[index].fps, display_attributes_[index].is_device_split,
      display_attributes_[index].v_back_porch, display_attributes_[index].v_front_porch,
      display_attributes_[index].v_pulse_width, display_attributes_[index].v_total,
      display_attributes_[index].h_total, display_attributes_[index].clock_khz,
      display_attributes_[index].topology,
      (connector_info_.modes[index].cur_panel_mode & DRM_MODE_FLAG_VID_MODE_PANEL) ?
      "Video" : "Command");

  return kErrorNone;
}

DisplayError HWDeviceDRM::SetDisplayAttributes(const HWDisplayAttributes &display_attributes) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetConfigIndex(char *mode, uint32_t *index) {
  return kErrorNone;
}

DisplayError HWDeviceDRM::PowerOn(const HWQosData &qos_data, SyncPoints *sync_points) {
  SetQOSData(qos_data);

  if (tui_state_ != kTUIStateNone) {
    DLOGI("Request deferred TUI state %d", tui_state_);
    pending_power_state_ = kPowerStateOn;
    return kErrorDeferred;
  }

  int64_t release_fence_fd = -1;
  int64_t retire_fence_fd = -1;

  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::ON);
  drm_atomic_intf_->Perform(DRMOps::CRTC_GET_RELEASE_FENCE, token_.crtc_id, &release_fence_fd);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, &retire_fence_fd);

  int ret = NullCommit(false /* synchronous */, true /* retain_planes */);
  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire_power_on");
  shared_ptr<Fence> release_fence = Fence::Create(INT(release_fence_fd), "release_power_on");
  if (ret) {
    DLOGE("Failed with error: %d", ret);
    return kErrorHardware;
  }

  sync_points->retire_fence = retire_fence;
  sync_points->release_fence = release_fence;
  DLOGD_IF(kTagDriverConfig, "RELEASE fence: fd: %d", INT(release_fence_fd));
  pending_power_state_ = kPowerStateNone;

  last_power_mode_ = DRMPowerMode::ON;

  return kErrorNone;
}

DisplayError HWDeviceDRM::PowerOff(bool teardown, SyncPoints *sync_points) {
  DTRACE_SCOPED();
  if (!drm_atomic_intf_) {
    DLOGE("DRM Atomic Interface is null!");
    return kErrorUndefined;
  }

  if (first_cycle_) {
    return kErrorNone;
  }

  if (tui_state_ != kTUIStateNone) {
    DLOGI("Request deferred TUI state %d", tui_state_);
    pending_power_state_ = kPowerStateOff;
    return kErrorDeferred;
  }

  ResetROI();
  int64_t retire_fence_fd = -1;
  drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
  if (!IsSeamlessTransition()) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode);
  }
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::OFF);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 0);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, &retire_fence_fd);

  if (cwb_config_.cwb_disp_id == display_id_ && cwb_config_.enabled) {
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, cwb_config_.token.conn_id, 0);
    DLOGI("Tearing down the CWB topology");
  }

  int ret = NullCommit(false /* synchronous */, false /* retain_planes */);
  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire_power_off");
  if (ret) {
    DLOGE("Failed with error: %d, dynamic_fps=%d, seamless_mode_switch_=%d, vrefresh_=%d,"
     "panel_mode_changed_=%d bit_clk_rate_=%d", ret, hw_panel_info_.dynamic_fps,
     seamless_mode_switch_, vrefresh_, panel_mode_changed_, bit_clk_rate_);
    return kErrorHardware;
  }

  if (cwb_config_.cwb_disp_id == display_id_) {  // Incase display power-off in cwb active/teardown
    // state, then reset cwb_display_id to un-block other displays from performing CWB.
    if (cwb_config_.enabled) {
      FlushConcurrentWriteback();
    } else {  // for CWB Post-teardown (the frame following teardown) frame
      cwb_config_.cwb_disp_id = -1;
    }
  }

  sync_points->retire_fence = retire_fence;
  pending_power_state_ = kPowerStateNone;

  last_power_mode_ = DRMPowerMode::OFF;

  return kErrorNone;
}

DisplayError HWDeviceDRM::Doze(const HWQosData &qos_data, SyncPoints *sync_points) {
  DTRACE_SCOPED();

  if (first_cycle_ || tui_state_ != kTUIStateNone || last_power_mode_ != DRMPowerMode::OFF) {
    pending_power_state_ = kPowerStateDoze;
    return kErrorDeferred;
  }

  SetQOSData(qos_data);

  int64_t release_fence_fd = -1;
  int64_t retire_fence_fd = -1;

  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, token_.crtc_id);
  drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode);

  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::DOZE);
  drm_atomic_intf_->Perform(DRMOps::CRTC_GET_RELEASE_FENCE, token_.crtc_id, &release_fence_fd);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, &retire_fence_fd);

  int ret = NullCommit(false /* synchronous */, true /* retain_planes */);
  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire_doze");
  shared_ptr<Fence> release_fence = Fence::Create(release_fence_fd, "release_doze");
  if (ret) {
    DLOGE("Failed with error: %d", ret);
    return kErrorHardware;
  }

  sync_points->retire_fence = retire_fence;
  sync_points->release_fence = release_fence;
  DLOGD_IF(kTagDriverConfig, "RELEASE fence: fd: %d", INT(release_fence_fd));

  last_power_mode_ = DRMPowerMode::DOZE;

  return kErrorNone;
}

DisplayError HWDeviceDRM::DozeSuspend(const HWQosData &qos_data, SyncPoints *sync_points) {
  DTRACE_SCOPED();

  if (tui_state_ != kTUIStateNone && tui_state_ != kTUIStateEnd) {
    pending_power_state_ = kPowerStateDozeSuspend;
    return kErrorDeferred;
  }

  SetQOSData(qos_data);

  int64_t release_fence_fd = -1;
  int64_t retire_fence_fd = -1;

  if (first_cycle_) {
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, token_.crtc_id);
    drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode);
  }
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id,
                            DRMPowerMode::DOZE_SUSPEND);
  drm_atomic_intf_->Perform(DRMOps::CRTC_GET_RELEASE_FENCE, token_.crtc_id, &release_fence_fd);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, &retire_fence_fd);

  int ret = NullCommit(false /* synchronous */, true /* retain_planes */);
  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire_doze_suspend");
  shared_ptr<Fence> release_fence = Fence::Create(release_fence_fd, "release_doze_suspend");
  if (ret) {
    DLOGE("Failed with error: %d", ret);
    return kErrorHardware;
  }

  sync_points->retire_fence = retire_fence;
  sync_points->release_fence = release_fence;
  DLOGD_IF(kTagDriverConfig, "RELEASE fence: fd: %d", INT(release_fence_fd));

  pending_power_state_ = kPowerStateNone;

  last_power_mode_ = DRMPowerMode::DOZE_SUSPEND;

  return kErrorNone;
}

void HWDeviceDRM::SetQOSData(const HWQosData &qos_data) {
  if (!qos_data.valid) {
    return;
  }
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_CORE_CLK, token_.crtc_id, qos_data.clock_hz);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_CORE_AB, token_.crtc_id, qos_data.core_ab_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_CORE_IB, token_.crtc_id, qos_data.core_ib_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_LLCC_AB, token_.crtc_id, qos_data.llcc_ab_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_LLCC_IB, token_.crtc_id, qos_data.llcc_ib_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_DRAM_AB, token_.crtc_id, qos_data.dram_ab_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_DRAM_IB, token_.crtc_id, qos_data.dram_ib_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ROT_PREFILL_BW, token_.crtc_id,
                            qos_data.rot_prefill_bw_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ROT_CLK, token_.crtc_id, qos_data.rot_clock_hz);
}

DisplayError HWDeviceDRM::Standby(SyncPoints *sync_points) {
  return kErrorNone;
}

void HWDeviceDRM::SetupAtomic(Fence::ScopedRef &scoped_ref, HWLayersInfo *hw_layers_info,
                              bool validate, int64_t *release_fence_fd, int64_t *retire_fence_fd) {
  if (default_mode_) {
    return;
  }

  DTRACE_SCOPED();
  uint32_t hw_layer_count = UINT32(hw_layers_info->hw_layers.size());
  HWQosData &qos_data = hw_layers_info->qos_data;
  DRMSecurityLevel crtc_security_level = DRMSecurityLevel::SECURE_NON_SECURE;
  uint32_t index = current_mode_index_;
  sde_drm::DRMModeInfo current_mode = connector_info_.modes[index];

  solid_fills_.clear();
  noise_cfg_ = {};
  bool resource_update = hw_layers_info->updates_mask.test(kUpdateResources);
  bool buffer_update = hw_layers_info->updates_mask.test(kSwapBuffers);
  bool update_config = resource_update || buffer_update || tui_state_ == kTUIStateEnd ||
                       hw_layers_info->flags.geometry_changed;
  bool update_luts = hw_layers_info->updates_mask.test(kUpdateLuts);

  if (hw_panel_info_.partial_update && update_config) {
    if (IsFullFrameUpdate(*hw_layers_info)) {
      ResetROI();
    } else {
      const int kNumMaxROIs = 4;
      DRMRect crtc_rects[kNumMaxROIs] = {{0, 0, mixer_attributes_.width, mixer_attributes_.height}};
      DRMRect conn_rects[kNumMaxROIs] = {{0, 0, display_attributes_[index].x_pixels,
                                          display_attributes_[index].y_pixels}};

      for (uint32_t i = 0; i < hw_layers_info->left_frame_roi.size(); i++) {
        auto &roi = hw_layers_info->left_frame_roi.at(i);
        // TODO(user): In multi PU, stitch ROIs vertically adjacent and upate plane destination
        crtc_rects[i].left = UINT32(roi.left);
        crtc_rects[i].right = UINT32(roi.right);
        crtc_rects[i].top = UINT32(roi.top);
        crtc_rects[i].bottom = UINT32(roi.bottom);
        conn_rects[i].left = UINT32(roi.left);
        conn_rects[i].right = UINT32(roi.right);
        conn_rects[i].top = UINT32(roi.top);
        conn_rects[i].bottom = UINT32(roi.bottom);
      }

      uint32_t num_rects = std::max(1u, UINT32(hw_layers_info->left_frame_roi.size()));
      drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ROI, token_.crtc_id, num_rects, crtc_rects);
      drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_ROI, token_.conn_id, num_rects, conn_rects);
    }
  } else if (!hw_panel_info_.partial_update &&
             (current_mode.cur_panel_mode & DRM_MODE_FLAG_CMD_MODE_PANEL) && update_config) {
    if (!IsFullFrameUpdate(*hw_layers_info)) {
      DLOGW("Expected full frame ROI");
    }
    ResetROI();
  }

#ifdef TRUSTED_VM
  if (first_cycle_) {
    drm_atomic_intf_->Perform(sde_drm::DRMOps::RESET_PANEL_FEATURES, 0 /* argument is not used */);
  }
#endif

  if (first_cycle_ && display_count_ == 1) {
    // Used in 2 cases:
    // 1. Since driver doesnt clear the SSPP luts during the adb shell stop/start, clear once
    // 2. On TUI start also, need to clear the SSPP luts.
    drm_atomic_intf_->Perform(sde_drm::DRMOps::PLANES_RESET_LUT, token_.crtc_id);
  }

  for (uint32_t i = 0; i < hw_layer_count; i++) {
    Layer &layer = hw_layers_info->hw_layers.at(i);
    LayerBuffer *input_buffer = &layer.input_buffer;
    HWPipeInfo *left_pipe = &hw_layers_info->config[i].left_pipe;
    HWPipeInfo *right_pipe = &hw_layers_info->config[i].right_pipe;
    HWLayerConfig &layer_config = hw_layers_info->config[i];
    HWRotatorSession *hw_rotator_session = &layer_config.hw_rotator_session;

    if (hw_layers_info->config[i].use_solidfill_stage) {
      hw_layers_info->config[i].hw_solidfill_stage.solid_fill_info = layer.solid_fill_info;
      AddSolidfillStage(hw_layers_info->config[i].hw_solidfill_stage, layer.plane_alpha);
      continue;
    }

    if (layer_config.hw_noise_layer_cfg.enable) {
      SetNoiseLayerConfig(layer_config.hw_noise_layer_cfg);
      continue;
    }

    for (uint32_t count = 0; count < 2; count++) {
      HWPipeInfo *pipe_info = (count == 0) ? left_pipe : right_pipe;
      HWRotateInfo *hw_rotate_info = &hw_rotator_session->hw_rotate_info[count];

      if (hw_rotator_session->mode == kRotatorOffline && hw_rotate_info->valid) {
        input_buffer = &hw_rotator_session->output_buffer;
      }

      uint32_t fb_id = registry_.GetFbId(&layer, input_buffer->handle_id);

      if (pipe_info->valid && fb_id) {
        uint32_t pipe_id = pipe_info->pipe_id;

        if (update_config) {
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_ALPHA, pipe_id, layer.plane_alpha);

#ifdef UDFPS_ZPOS
          uint32_t z_order = pipe_info->z_order;
          if (layer.flags.fod_pressed) {
            z_order |= FOD_PRESSED_LAYER_ZORDER;
          }
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_ZORDER, pipe_id, z_order);
#else
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_ZORDER, pipe_id, pipe_info->z_order);
#endif

          // Account for PMA block activation directly at translation time to preserve layer
          // blending definition and avoid issues when a layer structure is reused.
          DRMBlendType blending = DRMBlendType::UNDEFINED;
          LayerBlending layer_blend = layer.blending;
          if (layer_blend ==  kBlendingPremultiplied && pipe_info->inverse_pma_info.inverse_pma) {
            layer_blend = kBlendingCoverage;
            DLOGI_IF(kTagDriverConfig, "PMA handled by Inverse PMA block - Pipe id: %u", pipe_id);
          }
          SetBlending(layer_blend, &blending);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_BLEND_TYPE, pipe_id, blending);

          DRMRect src = {};
          SetRect(pipe_info->src_roi, &src);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_SRC_RECT, pipe_id, src);

          DRMRect dst = {};
          SetRect(pipe_info->dst_roi, &dst);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_DST_RECT, pipe_id, dst);

          DRMRect excl = {};
          SetRect(pipe_info->excl_rect, &excl);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_EXCL_RECT, pipe_id, excl);

          uint32_t rot_bit_mask = 0;
          SetRotation(layer.transform, layer_config, &rot_bit_mask);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_ROTATION, pipe_id, rot_bit_mask);

          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_H_DECIMATION, pipe_id,
                                    pipe_info->horizontal_decimation);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_V_DECIMATION, pipe_id,
                                    pipe_info->vertical_decimation);

          DRMSecureMode fb_secure_mode;
          DRMSecurityLevel security_level;
          SetSecureConfig(layer.input_buffer, &fb_secure_mode, &security_level);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_FB_SECURE_MODE, pipe_id, fb_secure_mode);
          if (security_level > crtc_security_level) {
            crtc_security_level = security_level;
          }

          uint32_t config = 0;
          SetSrcConfig(layer.input_buffer, hw_rotator_session->mode, &config);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_SRC_CONFIG, pipe_id, config);;

          if (hw_scale_) {
            SDEScaler scaler_output = {};
            hw_scale_->SetScaler(pipe_info->scale_data, &scaler_output);
            // TODO(user): Remove qseed3 and add version check, then send appropriate scaler object
            if (hw_resource_.has_qseed3) {
              drm_atomic_intf_->Perform(DRMOps::PLANE_SET_SCALER_CONFIG, pipe_id,
                                        reinterpret_cast<uint64_t>(&scaler_output.scaler_v2));
            }
          }

          DRMCscType csc_type = DRMCscType::kCscTypeMax;
          SelectCscType(layer.input_buffer, &csc_type);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_CSC_CONFIG, pipe_id, &csc_type);

          DRMMultiRectMode multirect_mode;
          SetMultiRectMode(pipe_info->flags, &multirect_mode);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_MULTIRECT_MODE, pipe_id, multirect_mode);

          SetSsppTonemapFeatures(pipe_info);
        } else if (update_luts) {
          SetSsppTonemapFeatures(pipe_info);
        }

        drm_atomic_intf_->Perform(DRMOps::PLANE_SET_FB_ID, pipe_id, fb_id);
        drm_atomic_intf_->Perform(DRMOps::PLANE_SET_CRTC, pipe_id, token_.crtc_id);

        if (!validate && input_buffer->acquire_fence) {
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_INPUT_FENCE, pipe_id,
                                    scoped_ref.Get(input_buffer->acquire_fence));
        }
      }
    }
  }

  if (update_config) {
    SetSolidfillStages();
    ApplyNoiseLayerConfig();
    SetQOSData(qos_data);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_SECURITY_LEVEL, token_.crtc_id, crtc_security_level);
  }

  if (hw_layers_info->hw_avr_info.update) {
    sde_drm::DRMQsyncMode mode = sde_drm::DRMQsyncMode::NONE;
    if (hw_layers_info->hw_avr_info.mode == kContinuousMode) {
      mode = sde_drm::DRMQsyncMode::CONTINUOUS;
    } else if (hw_layers_info->hw_avr_info.mode == kOneShotMode) {
      mode = sde_drm::DRMQsyncMode::ONESHOT;
    }
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_QSYNC_MODE, token_.conn_id, mode);
  }

  // dpps commit feature ops doesn't use the obj id, set it as -1
  drm_atomic_intf_->Perform(DRMOps::DPPS_COMMIT_FEATURE, -1, ((validate) ? 1 : 0));
  if (!validate) {
    drm_atomic_intf_->Perform(DRMOps::COMMIT_PANEL_FEATURES, 0 /* argument is not used */);
  }

  if (reset_output_fence_offset_ && !validate) {
    // Change back the fence_offset
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_OUTPUT_FENCE_OFFSET, token_.crtc_id, 0);
    reset_output_fence_offset_ = false;
  }

  // Set panel mode
  if (panel_mode_changed_ & DRM_MODE_FLAG_VID_MODE_PANEL) {
    if (!validate) {
      // Switch to video mode, corresponding change the fence_offset
      DLOGI("set property: switch to video mode");
      drm_atomic_intf_->Perform(DRMOps::CRTC_SET_OUTPUT_FENCE_OFFSET, token_.crtc_id, 1);
      drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_PANEL_MODE, token_.conn_id,
                                panel_mode_changed_);
    }
    ResetROI();
  }

  if (panel_mode_changed_ & DRM_MODE_FLAG_CMD_MODE_PANEL && !validate) {
    // Switch to command mode
    DLOGI("set property: switch to command mode");
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_PANEL_MODE, token_.conn_id,
                              panel_mode_changed_);
  }

  if (panel_compression_changed_ && !validate) {
    DLOGI("set property: change the compression mode");
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_DSC_MODE, token_.conn_id,
                              panel_compression_changed_);
  }

  if (!validate && release_fence_fd && retire_fence_fd) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_GET_RELEASE_FENCE, token_.crtc_id, release_fence_fd);
    // Set retire fence offset.
    uint32_t offset = hw_layers_info->retire_fence_offset;
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_RETIRE_FENCE_OFFSET, token_.conn_id, offset);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, retire_fence_fd);
  }

  DLOGI_IF(kTagDriverConfig, "%s::%s System Clock=%d Hz, Core: AB=%f KBps, IB=%f Bps, " \
           "LLCC: AB=%f Bps, IB=%f Bps, DRAM AB=%f Bps, IB=%f Bps, "\
           "Rot: Bw=%f Bps, Clock=%d Hz", validate ? "Validate" : "Commit", device_name_,
           qos_data.clock_hz, qos_data.core_ab_bps / 1000.f, qos_data.core_ib_bps / 1000.f,
           qos_data.llcc_ab_bps / 1000.f, qos_data.llcc_ib_bps / 1000.f,
           qos_data.dram_ab_bps / 1000.f, qos_data.dram_ib_bps / 1000.f,
           qos_data.rot_prefill_bw_bps / 1000.f, qos_data.rot_clock_hz);

  // Set refresh rate
  if (vrefresh_) {
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((current_mode.mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (current_mode.mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
          (current_mode.cur_panel_mode == connector_info_.modes[mode_index].cur_panel_mode) &&
          (vrefresh_ == connector_info_.modes[mode_index].mode.vrefresh)) {
        current_mode = connector_info_.modes[mode_index];
        break;
      }
    }
  }

  if (bit_clk_rate_) {
    // Set the new bit clk rate
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_DYN_BIT_CLK, token_.conn_id, bit_clk_rate_);
  }

  if (first_cycle_) {
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_TOPOLOGY_CONTROL, token_.conn_id,
                              topology_control_);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, token_.crtc_id);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::ON);
#ifdef TRUSTED_VM
    drm_atomic_intf_->Perform(sde_drm::DRMOps::CRTC_SET_VM_REQ_STATE, token_.crtc_id,
                              sde_drm::DRMVMRequestState::ACQUIRE);
#endif
    last_power_mode_ = DRMPowerMode::ON;
  } else if (pending_power_state_ != kPowerStateNone && !validate) {
    DRMPowerMode power_mode;
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
    if (GetDRMPowerMode(pending_power_state_, &power_mode) == kErrorNone) {
      drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, power_mode);
      last_power_mode_ = power_mode;
    }
  }

  // Set CRTC mode, only if display config changes
  if (first_cycle_ || vrefresh_ || update_mode_) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode.mode);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_DSC_MODE, token_.conn_id,
                              current_mode.curr_compression_mode);
  }

  if (!validate && (hw_layers_info->set_idle_time_ms >= 0)) {
    DLOGI_IF(kTagDriverConfig, "Setting idle timeout to = %d ms",
             hw_layers_info->set_idle_time_ms);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_IDLE_TIMEOUT, token_.crtc_id,
                              hw_layers_info->set_idle_time_ms);
  }

  if (hw_panel_info_.mode == kModeCommand) {
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_AUTOREFRESH, token_.conn_id, autorefresh_);
  }
}

void HWDeviceDRM::SetNoiseLayerConfig(const NoiseLayerConfig &noise_config) {
  noise_cfg_.enable = noise_config.enable;
  noise_cfg_.flags = noise_config.flags;
  noise_cfg_.zpos_noise = noise_config.zpos_noise;
  noise_cfg_.zpos_attn = noise_config.zpos_attn;
  noise_cfg_.attn_factor =  noise_config.attenuation_factor;
  noise_cfg_.noise_strength =  noise_config.noise_strength;
  noise_cfg_.alpha_noise = noise_config.alpha_noise;
  noise_cfg_.temporal_en = noise_config.temporal_en;
  DLOGV_IF(kTagDriverConfig, "Display_id = %d z_noise = %d z_attn = %d attn_f = %d noise_str = %d"
           "alpha noise = %d temporal_en = %d", display_id_,
           noise_cfg_.zpos_noise, noise_cfg_.zpos_attn, noise_cfg_.attn_factor,
           noise_cfg_.noise_strength, noise_cfg_.alpha_noise, noise_cfg_.temporal_en);
}

void HWDeviceDRM::ApplyNoiseLayerConfig() {
  if (hw_resource_.has_noise_layer) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_NOISELAYER_CONFIG, token_.crtc_id,
                              reinterpret_cast<uint64_t>(&noise_cfg_));
  }
}

void HWDeviceDRM::ClearNoiseLayerConfig() {
  noise_cfg_ = {};
  ApplyNoiseLayerConfig();
}

void HWDeviceDRM::AddSolidfillStage(const HWSolidfillStage &sf, uint32_t plane_alpha) {
  sde_drm::DRMSolidfillStage solidfill;
  solidfill.bounding_rect.left = UINT32(sf.roi.left);
  solidfill.bounding_rect.top = UINT32(sf.roi.top);
  solidfill.bounding_rect.right = UINT32(sf.roi.right);
  solidfill.bounding_rect.bottom = UINT32(sf.roi.bottom);
  solidfill.is_exclusion_rect  = sf.is_exclusion_rect;
  solidfill.plane_alpha = plane_alpha;
  solidfill.z_order = sf.z_order;
  if (!sf.solid_fill_info.bit_depth) {
    solidfill.color_bit_depth = 8;
    solidfill.alpha = (0xff000000 & sf.color) >> 24;
    solidfill.red = (0xff0000 & sf.color) >> 16;
    solidfill.green = (0xff00 & sf.color) >> 8;
    solidfill.blue = 0xff & sf.color;
  } else {
    solidfill.color_bit_depth = sf.solid_fill_info.bit_depth;
    solidfill.alpha = sf.solid_fill_info.alpha;
    solidfill.red = sf.solid_fill_info.red;
    solidfill.green = sf.solid_fill_info.green;
    solidfill.blue = sf.solid_fill_info.blue;
  }
  solid_fills_.push_back(solidfill);
  DLOGI_IF(kTagDriverConfig, "Add a solidfill stage at z_order:%d argb_color:%x plane_alpha:%x",
           solidfill.z_order, solidfill.color, solidfill.plane_alpha);
}

void HWDeviceDRM::SetSolidfillStages() {
  if (hw_resource_.num_solidfill_stages) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_SOLIDFILL_STAGES, token_.crtc_id,
                              reinterpret_cast<uint64_t> (&solid_fills_));
  }
}

void HWDeviceDRM::ClearSolidfillStages() {
  solid_fills_.clear();
  SetSolidfillStages();
}

DisplayError HWDeviceDRM::Validate(HWLayersInfo *hw_layers_info) {
  DTRACE_SCOPED();

  DisplayError err = kErrorNone;
  registry_.Register(hw_layers_info);

  Fence::ScopedRef scoped_ref;
  SetupAtomic(scoped_ref, hw_layers_info, true /* validate */, nullptr, nullptr);

  int ret = drm_atomic_intf_->Validate();
  if (ret) {
    DLOGE("failed with error %d for %s", ret, device_name_);
    DumpHWLayers(hw_layers_info);
    vrefresh_ = 0;
    panel_mode_changed_ = 0;
    seamless_mode_switch_ = false;
    panel_compression_changed_ = 0;
    err = kErrorHardware;
  }

  return err;
}

DisplayError HWDeviceDRM::Commit(HWLayersInfo *hw_layers_info) {
  DTRACE_SCOPED();

  DisplayError err = kErrorNone;
  registry_.Register(hw_layers_info);

  if (default_mode_) {
    err = DefaultCommit(hw_layers_info);
  } else {
    err = AtomicCommit(hw_layers_info);
  }

  return err;
}

DisplayError HWDeviceDRM::DefaultCommit(HWLayersInfo *hw_layers_info) {
  DTRACE_SCOPED();


  for (Layer &layer : hw_layers_info->hw_layers) {
    layer.input_buffer.release_fence = nullptr;
  }

  DRMMaster *master = nullptr;
  int ret = DRMMaster::GetInstance(&master);
  if (ret < 0) {
    DLOGE("Failed to acquire DRMMaster instance");
    return kErrorResources;
  }

  DRMResMgr *res_mgr = nullptr;
  ret = DRMResMgr::GetInstance(&res_mgr);
  if (ret < 0) {
    DLOGE("Failed to acquire DRMResMgr instance");
    return kErrorResources;
  }

  int dev_fd = -1;
  master->GetHandle(&dev_fd);

  uint32_t connector_id = 0;
  res_mgr->GetConnectorId(&connector_id);

  uint32_t crtc_id = 0;
  res_mgr->GetCrtcId(&crtc_id);

  drmModeModeInfo mode;
  res_mgr->GetMode(&mode);

  uint64_t handle_id = hw_layers_info->hw_layers.at(0).input_buffer.handle_id;
  uint32_t fb_id = registry_.GetFbId(&hw_layers_info->hw_layers.at(0), handle_id);
  ret = drmModeSetCrtc(dev_fd, crtc_id, fb_id, 0 /* x */, 0 /* y */, &connector_id,
                       1 /* num_connectors */, &mode);
  if (ret < 0) {
    DLOGE("drmModeSetCrtc failed dev fd %d, fb_id %d, crtc id %d, connector id %d, %s", dev_fd,
          fb_id, crtc_id, connector_id, strerror(errno));
    return kErrorHardware;
  }

  return kErrorNone;
}

DisplayError HWDeviceDRM::AtomicCommit(HWLayersInfo *hw_layers_info) {
  DTRACE_SCOPED();

  int64_t release_fence_fd = -1;
  int64_t retire_fence_fd = -1;

  // scoped fence fds will be automatically closed when function scope ends,
  // atomic commit will have these fds already set on kernel by then.
  Fence::ScopedRef scoped_ref;
  SetupAtomic(scoped_ref, hw_layers_info, false /* validate */,
                                   &release_fence_fd, &retire_fence_fd);

  bool sync_commit = synchronous_commit_ || first_cycle_;

  if (hw_layers_info->elapse_timestamp > 0) {
    struct timespec t = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t current_time = (UINT64(t.tv_sec) * 1000000000LL + t.tv_nsec);
    if (current_time < hw_layers_info->elapse_timestamp) {
      usleep(UINT32((hw_layers_info->elapse_timestamp - current_time) / 1000));
    }
  }

  int ret = drm_atomic_intf_->Commit(sync_commit, false /* retain_planes*/);
  shared_ptr<Fence> release_fence = Fence::Create(INT(release_fence_fd), "release");
  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire");
  if (ret) {
    DLOGE("%s failed with error %d crtc %d", __FUNCTION__, ret, token_.crtc_id);
    DumpHWLayers(hw_layers_info);
    vrefresh_ = 0;
    panel_mode_changed_ = 0;
    seamless_mode_switch_ = false;
    panel_compression_changed_ = 0;
    return kErrorHardware;
  }

  DLOGD_IF(kTagDriverConfig, "RELEASE fence: fd: %s", Fence::GetStr(release_fence).c_str());
  DLOGD_IF(kTagDriverConfig, "RETIRE fence: fd: %s", Fence::GetStr(retire_fence).c_str());

  hw_layers_info->retire_fence = retire_fence;

  for (uint32_t i = 0; i < hw_layers_info->hw_layers.size(); i++) {
    Layer &layer = hw_layers_info->hw_layers.at(i);
    HWRotatorSession *hw_rotator_session = &hw_layers_info->config[i].hw_rotator_session;
    if (hw_rotator_session->mode == kRotatorOffline) {
      hw_rotator_session->output_buffer.release_fence = release_fence;
    } else {
      layer.input_buffer.release_fence = release_fence;
    }
  }

  hw_layers_info->sync_handle = release_fence;

  if (vrefresh_) {
    // Update current mode index if refresh rate is changed
    sde_drm::DRMModeInfo current_mode = connector_info_.modes[current_mode_index_];
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((current_mode.mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (current_mode.mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
          (current_mode.cur_panel_mode == connector_info_.modes[mode_index].cur_panel_mode) &&
          (vrefresh_ == connector_info_.modes[mode_index].mode.vrefresh)) {
        SetDisplaySwitchMode(mode_index);
        break;
      }
    }
    vrefresh_ = 0;
  }


  if (bit_clk_rate_) {
    // Update current mode index if bit clk rate is changed.
    connector_info_.modes[current_mode_index_].curr_bit_clk_rate = bit_clk_rate_;

    bit_clk_rate_ = 0;
  }

  if (panel_mode_changed_ & DRM_MODE_FLAG_CMD_MODE_PANEL) {
    panel_mode_changed_ = 0;
    synchronous_commit_ = false;
  } else if (panel_mode_changed_ & DRM_MODE_FLAG_VID_MODE_PANEL) {
    panel_mode_changed_ = 0;
    synchronous_commit_ = false;
    reset_output_fence_offset_ = true;
  }

  panel_compression_changed_ = 0;
  first_cycle_ = false;
  update_mode_ = false;
  hw_layers_info->updates_mask = 0;
  pending_power_state_ = kPowerStateNone;
  // Inherently a real commit ensures null commit properties have happened, so update the member
  first_null_cycle_ = false;
  seamless_mode_switch_ = false;

  SetTUIState();

#ifdef TRUSTED_VM
  drm_atomic_intf_->Perform(sde_drm::DRMOps::CRTC_SET_VM_REQ_STATE, token_.crtc_id,
                            sde_drm::DRMVMRequestState::NONE);
#endif

  return kErrorNone;
}

DisplayError HWDeviceDRM::Flush(HWLayersInfo *hw_layers_info) {
  ClearSolidfillStages();
  ClearNoiseLayerConfig();
  ResetROI();
  bool sync_commit = (tui_state_ == kTUIStateStart || tui_state_ == kTUIStateEnd ||
                      secure_display_active_);

  // TODO(user): Handle via flush call
#ifdef TRUSTED_VM
  sync_commit = true;
#endif

  // dpps commit feature ops doesn't use the obj id, set it as -1
  drm_atomic_intf_->Perform(DRMOps::DPPS_COMMIT_FEATURE, -1);

  int ret = NullCommit(sync_commit /* synchronous */, false /* retain_planes*/);
  if (ret) {
    DLOGE("failed with error %d", ret);
    return kErrorHardware;
  }
  return kErrorNone;
}

void HWDeviceDRM::SetBlending(const LayerBlending &source, DRMBlendType *target) {
  switch (source) {
    case kBlendingPremultiplied:
      *target = DRMBlendType::PREMULTIPLIED;
      break;
    case kBlendingOpaque:
      *target = DRMBlendType::OPAQUE;
      break;
    case kBlendingCoverage:
      *target = DRMBlendType::COVERAGE;
      break;
    case kBlendingSkip:
      *target = DRMBlendType::SKIP_BLENDING;
      break;
    default:
      *target = DRMBlendType::UNDEFINED;
  }
}

void HWDeviceDRM::SetSrcConfig(const LayerBuffer &input_buffer, const HWRotatorMode &mode,
                               uint32_t *config) {
  // In offline rotation case, rotator will handle deinterlacing.
  if (mode == kRotatorInline) {
    if (input_buffer.flags.interlace) {
      *config |= (0x01 << UINT32(DRMSrcConfig::DEINTERLACE));
    }
  }
}

void HWDeviceDRM::SelectCscType(const LayerBuffer &input_buffer, DRMCscType *type) {
  if (type == NULL) {
    return;
  }

  *type = DRMCscType::kCscTypeMax;
  if (input_buffer.format < kFormatYCbCr420Planar) {
    return;
  }

  switch (input_buffer.color_metadata.colorPrimaries) {
    case ColorPrimaries_BT601_6_525:
    case ColorPrimaries_BT601_6_625:
      *type = ((input_buffer.color_metadata.range == Range_Full) ?
               DRMCscType::kCscYuv2Rgb601FR : DRMCscType::kCscYuv2Rgb601L);
      break;
    case ColorPrimaries_BT709_5:
      *type = DRMCscType::kCscYuv2Rgb709L;
      break;
    case ColorPrimaries_BT2020:
      *type = ((input_buffer.color_metadata.range == Range_Full) ?
                DRMCscType::kCscYuv2Rgb2020FR : DRMCscType::kCscYuv2Rgb2020L);
      break;
    default:
      break;
  }
}

void HWDeviceDRM::SetRect(const LayerRect &source, DRMRect *target) {
  target->left = UINT32(source.left);
  target->top = UINT32(source.top);
  target->right = UINT32(source.right);
  target->bottom = UINT32(source.bottom);
}

void HWDeviceDRM::SetRotation(LayerTransform transform, const HWLayerConfig &layer_config,
                              uint32_t* rot_bit_mask) {
  HWRotatorMode mode = layer_config.hw_rotator_session.mode;
  // In offline rotation case, rotator will handle flips set via offline rotator interface.
  if (mode == kRotatorOffline) {
    *rot_bit_mask = 0;
    return;
  }

  // In no rotation case or inline rotation case, plane will handle flips
  // In DRM framework rotation is applied in counter-clockwise direction.
  if (layer_config.use_inline_rot && transform.rotation == 90) {
    // a) rotate 90 clockwise = rotate 270 counter-clockwise in DRM
    // rotate 270 is translated as hflip + vflip + rotate90
    // b) rotate 270 clockwise = rotate 90 counter-clockwise in DRM
    // c) hflip + rotate 90 clockwise = vflip + rotate 90 counter-clockwise in DRM
    // d) vflip + rotate 90 clockwise = hflip + rotate 90 counter-clockwise in DRM
    *rot_bit_mask = UINT32(DRMRotation::ROT_90);
    transform.flip_horizontal = !transform.flip_horizontal;
    transform.flip_vertical = !transform.flip_vertical;
  }

  if (transform.flip_horizontal) {
    *rot_bit_mask |= UINT32(DRMRotation::FLIP_H);
  }

  if (transform.flip_vertical) {
    *rot_bit_mask |= UINT32(DRMRotation::FLIP_V);
  }
}

bool HWDeviceDRM::EnableHotPlugDetection(int enable) {
  return true;
}

DisplayError HWDeviceDRM::SetCursorPosition(HWLayersInfo *hw_layers_info, int x, int y) {
  DTRACE_SCOPED();
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetPPFeaturesVersion(PPFeatureVersion *vers) {
  struct DRMPPFeatureInfo info = {};

  if (!hw_color_mgr_)
    return kErrorNotSupported;

  for (uint32_t i = 0; i < kMaxNumPPFeatures; i++) {
    std::vector<DRMPPFeatureID> drm_id = {};
    memset(&info, 0, sizeof(struct DRMPPFeatureInfo));
    hw_color_mgr_->ToDrmFeatureId(kDSPP, i, &drm_id);
    if (drm_id.empty())
      continue;

    info.id = drm_id.at(0);

    drm_mgr_intf_->GetCrtcPPInfo(token_.crtc_id, &info);
    vers->version[i] = hw_color_mgr_->GetFeatureVersion(info);
  }
  return kErrorNone;
}

DisplayError HWDeviceDRM::SetPPFeatures(PPFeaturesConfig *feature_list) {
  int ret = 0;
  DRMCrtcInfo crtc_info = {};
  PPFeatureInfo *feature = NULL;

  if (!hw_color_mgr_)
    return kErrorNotSupported;

  while (true) {
    std::vector<DRMPPFeatureID> drm_id = {};
    DRMPPFeatureInfo kernel_params = {};
    bool crtc_feature = true;

    ret = feature_list->RetrieveNextFeature(&feature);
    if (ret || !feature)
      break;

    hw_color_mgr_->ToDrmFeatureId(kDSPP, feature->feature_id_, &drm_id);
    if (drm_id.empty()) {
      continue;
    } else if (drm_id.at(0) == DRMPPFeatureID::kFeatureDither) {
      drm_mgr_intf_->GetCrtcInfo(token_.crtc_id, &crtc_info);
      if (crtc_info.has_spr)
        drm_id.at(0) = DRMPPFeatureID::kFeatureSprDither;
    }

    kernel_params.id = drm_id.at(0);
    drm_mgr_intf_->GetCrtcPPInfo(token_.crtc_id, &kernel_params);
    if (kernel_params.version == std::numeric_limits<uint32_t>::max())
      crtc_feature = false;

    DLOGV_IF(kTagDriverConfig, "feature_id = %d", feature->feature_id_);
    for (DRMPPFeatureID id : drm_id) {
      if (id >= kPPFeaturesMax) {
        DLOGE("Invalid feature id %d", id);
        continue;
      }

      kernel_params.id = id;
      ret = hw_color_mgr_->GetDrmFeature(feature, &kernel_params);
      if (!ret && crtc_feature)
        drm_atomic_intf_->Perform(DRMOps::CRTC_SET_POST_PROC,
                                  token_.crtc_id, &kernel_params);
      else if (!ret && !crtc_feature)
        drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POST_PROC,
                                  token_.conn_id, &kernel_params);

      hw_color_mgr_->FreeDrmFeatureData(&kernel_params);
    }
  }

  // Once all features were consumed, then destroy all feature instance from feature_list,
  feature_list->Reset();

  return kErrorNone;
}

DisplayError HWDeviceDRM::SetVSyncState(bool enable) {
  return kErrorNotSupported;
}

void HWDeviceDRM::SetIdleTimeoutMs(uint32_t timeout_ms) {
  // TODO(user): This function can be removed after fb is deprecated
}

DisplayError HWDeviceDRM::SetDisplayMode(const HWDisplayMode hw_display_mode) {
  if (!switch_mode_valid_) {
    return kErrorNotSupported;
  }

  uint32_t mode_flag = 0;
  sde_drm::DRMModeInfo current_mode = connector_info_.modes[current_mode_index_];

  // Refresh rate change needed if new panel mode differs from current
  if (hw_display_mode == kModeCommand) {
    mode_flag = DRM_MODE_FLAG_CMD_MODE_PANEL;
    if (connector_info_.modes[cmd_mode_index_].mode.vrefresh != current_mode.mode.vrefresh) {
      vrefresh_ = connector_info_.modes[cmd_mode_index_].mode.vrefresh;
    }
    current_mode_index_ = cmd_mode_index_;
    connector_info_.modes[current_mode_index_].cur_panel_mode = mode_flag;
    DLOGI_IF(kTagDriverConfig, "switch panel mode to command");
  } else if (hw_display_mode == kModeVideo) {
    mode_flag = DRM_MODE_FLAG_VID_MODE_PANEL;
    if (connector_info_.modes[video_mode_index_].mode.vrefresh != current_mode.mode.vrefresh) {
      vrefresh_ = connector_info_.modes[video_mode_index_].mode.vrefresh;
    }
    current_mode_index_ = video_mode_index_;
    connector_info_.modes[current_mode_index_].cur_panel_mode = mode_flag;
    DLOGI_IF(kTagDriverConfig, "switch panel mode to video");
  }
  PopulateHWPanelInfo();
  panel_mode_changed_ = mode_flag;
  synchronous_commit_ = true;
  return kErrorNone;
}

DisplayError HWDeviceDRM::SetRefreshRate(uint32_t refresh_rate) {
  if (bit_clk_rate_) {
    // bit rate update pending.
    // Defer any refresh rate setting.
    return kErrorNotSupported;
  }

  // Check if requested refresh rate is valid
  sde_drm::DRMModeInfo current_mode = connector_info_.modes[current_mode_index_];
  for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
    if ((current_mode.mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
        (current_mode.mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
        (current_mode.cur_panel_mode == connector_info_.modes[mode_index].cur_panel_mode) &&
        (refresh_rate == connector_info_.modes[mode_index].mode.vrefresh)) {
      for (uint32_t submode_idx = 0; submode_idx <
           connector_info_.modes[mode_index].sub_modes.size(); submode_idx++) {
        sde_drm::DRMSubModeInfo sub_mode = connector_info_.modes[mode_index].sub_modes[submode_idx];
        if (sub_mode.panel_compression_mode == current_mode.curr_compression_mode) {
          connector_info_.modes[mode_index].curr_submode_index = submode_idx;
          vrefresh_ = refresh_rate;
          DLOGV_IF(kTagDriverConfig, "Set refresh rate to %d", refresh_rate);
          return kErrorNone;
        }
      }
    }
  }
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetConfigIndexForFps(uint32_t refresh_rate, uint32_t *config) {
  // Check if requested refresh rate is valid
  drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
  for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
    if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
        (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
        (current_mode.flags == connector_info_.modes[mode_index].mode.flags) &&
        (refresh_rate == connector_info_.modes[mode_index].mode.vrefresh)) {
      *config = mode_index;
      DLOGV_IF(kTagDriverConfig, "Config index for fps %d is %d", refresh_rate, *config);
      return kErrorNone;
    }
  }
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetHWScanInfo(HWScanInfo *scan_info) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetVideoFormat(uint32_t config_index, uint32_t *video_format) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetMaxCEAFormat(uint32_t *max_cea_format) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::OnMinHdcpEncryptionLevelChange(uint32_t min_enc_level) {
  DisplayError error = kErrorNone;
  int fd = -1;
  char data[kMaxStringLength] = {'\0'};

  snprintf(data, sizeof(data), "/sys/devices/virtual/hdcp/msm_hdcp/min_level_change");

  fd = Sys::open_(data, O_WRONLY);
  if (fd < 0) {
    DLOGE("File '%s' could not be opened. errno = %d, desc = %s", data, errno, strerror(errno));
    return kErrorHardware;
  }

  snprintf(data, sizeof(data), "%d", min_enc_level);

  ssize_t err = Sys::pwrite_(fd, data, strlen(data), 0);
  if (err <= 0) {
    DLOGE("Write failed, Error = %s", strerror(errno));
    error = kErrorHardware;
  }

  Sys::close_(fd);

  return error;
}

DisplayError HWDeviceDRM::SetScaleLutConfig(HWScaleLutInfo *lut_info) {
  sde_drm::DRMScalerLUTInfo drm_lut_info = {};
  drm_lut_info.cir_lut = lut_info->cir_lut;
  drm_lut_info.dir_lut = lut_info->dir_lut;
  drm_lut_info.sep_lut = lut_info->sep_lut;
  drm_lut_info.cir_lut_size = lut_info->cir_lut_size;
  drm_lut_info.dir_lut_size = lut_info->dir_lut_size;
  drm_lut_info.sep_lut_size = lut_info->sep_lut_size;
  drm_mgr_intf_->SetScalerLUT(drm_lut_info);

  return kErrorNone;
}

DisplayError HWDeviceDRM::UnsetScaleLutConfig() {
  drm_mgr_intf_->UnsetScalerLUT();

  return kErrorNone;
}

DisplayError HWDeviceDRM::SetMixerAttributes(const HWMixerAttributes &mixer_attributes) {
  if (IsResolutionSwitchEnabled()) {
    return kErrorNotSupported;
  }

  if (!dest_scaler_blocks_used_) {
    return kErrorNotSupported;
  }

  uint32_t index = current_mode_index_;

  if (mixer_attributes.width > display_attributes_[index].x_pixels ||
      mixer_attributes.height > display_attributes_[index].y_pixels) {
    DLOGW("Input resolution exceeds display resolution! input: res %dx%d display: res %dx%d",
          mixer_attributes.width, mixer_attributes.height, display_attributes_[index].x_pixels,
          display_attributes_[index].y_pixels);
    return kErrorNotSupported;
  }

  uint32_t topology_num_split = display_attributes_[index].topology_num_split;
  if (mixer_attributes.width % topology_num_split != 0) {
    DLOGW("Uneven LM split: topology:%d supported_split:%d mixer_width:%d",
          display_attributes_[index].topology, topology_num_split, mixer_attributes.width);
    return kErrorNotSupported;
  }

  uint32_t max_input_width = hw_resource_.hw_dest_scalar_info.max_input_width;
  uint32_t split_max_input_width = max_input_width - DEST_SCALAR_OVERFETCH_SIZE;
  uint32_t lm_split_width = mixer_attributes.width / topology_num_split;
  if (topology_num_split > 1 && lm_split_width > split_max_input_width) {
    DLOGW("Input width exceeds width limit in split LM mode. input_width %d width_limit %d",
          lm_split_width, split_max_input_width);
    return kErrorNotSupported;
  }

  if (mixer_attributes.width > max_input_width * topology_num_split) {
    DLOGW("Input width exceeds width limit! input_width %d width_limit %d", mixer_attributes.width,
          max_input_width * topology_num_split);
    return kErrorNotSupported;
  }

  float mixer_aspect_ratio = FLOAT(mixer_attributes.width) / FLOAT(mixer_attributes.height);
  float display_aspect_ratio =
      FLOAT(display_attributes_[index].x_pixels) / FLOAT(display_attributes_[index].y_pixels);

  if (display_aspect_ratio != mixer_aspect_ratio) {
    DLOGW("Aspect ratio mismatch! input: res %dx%d display: res %dx%d", mixer_attributes.width,
          mixer_attributes.height, display_attributes_[index].x_pixels,
          display_attributes_[index].y_pixels);
    return kErrorNotSupported;
  }

  float scale_x = FLOAT(display_attributes_[index].x_pixels) / FLOAT(mixer_attributes.width);
  float scale_y = FLOAT(display_attributes_[index].y_pixels) / FLOAT(mixer_attributes.height);
  float max_scale_up = hw_resource_.hw_dest_scalar_info.max_scale_up;
  if (scale_x > max_scale_up || scale_y > max_scale_up) {
    DLOGW(
        "Up scaling ratio exceeds for destination scalar upscale limit scale_x %f scale_y %f "
        "max_scale_up %f",
        scale_x, scale_y, max_scale_up);
    return kErrorNotSupported;
  }

  float mixer_split_ratio = FLOAT(mixer_attributes_.split_left) / FLOAT(mixer_attributes_.width);

  mixer_attributes_ = mixer_attributes;
  mixer_attributes_.split_left = mixer_attributes_.width;
  mixer_attributes_.split_type = kNoSplit;
  mixer_attributes_.dest_scaler_blocks_used = dest_scaler_blocks_used_;  // No change.
  if (display_attributes_[index].is_device_split) {
    mixer_attributes_.split_left = UINT32(FLOAT(mixer_attributes.width) * mixer_split_ratio);
    mixer_attributes_.split_type = kDualSplit;
    if (display_attributes_[index].topology == kQuadLMMerge ||
        display_attributes_[index].topology == kQuadLMDSCMerge ||
        display_attributes_[index].topology == kQuadLMMergeDSC ||
        display_attributes_[index].topology == kQuadLMDSC4HSMerge) {
      mixer_attributes_.split_type = kQuadSplit;
    }
  }

  return kErrorNone;
}

DisplayError HWDeviceDRM::GetMixerAttributes(HWMixerAttributes *mixer_attributes) {
  if (!mixer_attributes) {
    DLOGW("mixer_attributes invalid. Return.");
    return kErrorParameters;
  }

  *mixer_attributes = mixer_attributes_;

  return kErrorNone;
}

DisplayError HWDeviceDRM::DumpDebugData() {
  string dir_path = "/data/vendor/display/hw_recovery/";
  string device_str = device_name_;

  // Attempt to make hw_recovery dir, it may exist
  if (mkdir(dir_path.c_str(), 0777) != 0 && errno != EEXIST) {
    DLOGW("Failed to create %s directory errno = %d, desc = %s", dir_path.c_str(), errno,
          strerror(errno));
    return kErrorPermission;
  }
  // If it does exist, ensure permissions are fine
  if (errno == EEXIST && chmod(dir_path.c_str(), 0777) != 0) {
    DLOGW("Failed to change permissions on %s directory", dir_path.c_str());
    return kErrorPermission;
  }

  string filename = dir_path+device_str+"_HWR_"+to_string(debug_dump_count_);
  ofstream dst(filename);
  debug_dump_count_++;

  {
    ifstream src;
    src.open("/sys/kernel/debug/dri/0/debug/dump");
    if (src.fail()) {
      DLOGW("Unable to open dump debugfs node");
     } else {
      dst << "---- Event Logs ----" << std::endl;
      dst << src.rdbuf() << std::endl;
      if (src.fail()) {
        DLOGW("Unable to read dump debugfs node");
      }
      src.close();
    }
  }

  {
    ifstream src;
    src.open("/sys/kernel/debug/dri/0/debug/recovery_reg");
    if (src.fail()) {
      DLOGW("Unable to open recovery_reg debugfs node");
    } else {
      dst << "---- All Registers ----" << std::endl;
      dst << src.rdbuf() << std::endl;
      if (src.fail()) {
        DLOGW("Unable to read recovery_reg debugfs node");
      }
      src.close();
    }
  }

  {
    ifstream src;
    src.open("/sys/kernel/debug/dri/0/debug/recovery_dbgbus");
    if (src.fail()) {
      DLOGW("Unable to open recovery_dbgbus debugfs node");
    } else {
      dst << "---- Debug Bus ----" << std::endl;
      dst << src.rdbuf() << std::endl;
      if (src.fail()) {
        DLOGW("Unable to read recovery_dbgbus debugfs node");
      }
      src.close();
    }
  }

  {
    ifstream src;
    src.open("/sys/kernel/debug/dri/0/debug/recovery_vbif_dbgbus");
    if (src.fail()) {
      DLOGW("Unable to open recovery_vbif_dbgbus debugfs node");
    } else {
      dst << "---- VBIF Debug Bus ----" << std::endl;
      dst << src.rdbuf() << std::endl;
      if (src.fail()) {
        DLOGW("Unable to read recovery_vbif_dbgbus debugfs node");
      }
      src.close();
    }
  }

  {
    ifstream src;
    src.open("/sys/kernel/debug/dri/0/debug/recovery_dsi_dbgbus");
    if (src.fail()) {
      DLOGW("Unable to open recovery_dsi_dbgbus debugfs node");
    } else {
      dst << "---- DSI Debug Bus ----" << std::endl;
      dst << src.rdbuf() << std::endl;
      if (src.fail()) {
        DLOGW("Unable to read recovery_dsi_dbgbus debugfs node");
      }
    src.close();
    }
  }

  dst.close();
  DLOGI("Wrote hw_recovery file %s", filename.c_str());

  return kErrorNone;
}

void HWDeviceDRM::GetDRMDisplayToken(sde_drm::DRMDisplayToken *token) const {
  *token = token_;
}

void HWDeviceDRM::UpdateMixerAttributes() {
  uint32_t index = current_mode_index_;

  mixer_attributes_.width = display_attributes_[index].x_pixels;
  mixer_attributes_.height = display_attributes_[index].y_pixels;
  mixer_attributes_.split_left = display_attributes_[index].is_device_split
                                     ? hw_panel_info_.split_info.left_split
                                     : mixer_attributes_.width;
  mixer_attributes_.split_type = kNoSplit;
  if (display_attributes_[index].is_device_split) {
    mixer_attributes_.split_type = kDualSplit;
    if (display_attributes_[index].topology == kQuadLMMerge ||
        display_attributes_[index].topology == kQuadLMDSCMerge ||
        display_attributes_[index].topology == kQuadLMMergeDSC ||
        display_attributes_[index].topology == kQuadLMDSC4HSMerge) {
      mixer_attributes_.split_type = kQuadSplit;
    }
  }

  DLOGI("Mixer WxH %dx%d-%d for %s", mixer_attributes_.width, mixer_attributes_.height,
        mixer_attributes_.split_type, device_name_);
  update_mode_ = true;
}

void HWDeviceDRM::SetSecureConfig(const LayerBuffer &input_buffer, DRMSecureMode *fb_secure_mode,
                                  DRMSecurityLevel *security_level) {
  *fb_secure_mode = DRMSecureMode::NON_SECURE;
  *security_level = DRMSecurityLevel::SECURE_NON_SECURE;

  if (input_buffer.flags.secure) {
    if (input_buffer.flags.secure_camera) {
      // IOMMU configuration for this framebuffer mode is secure domain & requires
      // only stage II translation, when this buffer is accessed by Display H/W.
      // Secure and non-secure planes can be attached to this CRTC.
      *fb_secure_mode = DRMSecureMode::SECURE_DIR_TRANSLATION;
    } else if (input_buffer.flags.secure_display) {
      // IOMMU configuration for this framebuffer mode is secure domain & requires
      // only stage II translation, when this buffer is accessed by Display H/W.
      // Only secure planes can be attached to this CRTC.
      *fb_secure_mode = DRMSecureMode::SECURE_DIR_TRANSLATION;
      *security_level = DRMSecurityLevel::SECURE_ONLY;
    } else {
      // IOMMU configuration for this framebuffer mode is secure domain & requires both
      // stage I and stage II translations, when this buffer is accessed by Display H/W.
      // Secure and non-secure planes can be attached to this CRTC.
      *fb_secure_mode = DRMSecureMode::SECURE;
    }
  }
}

void HWDeviceDRM::SetTopology(sde_drm::DRMTopology drm_topology, HWTopology *hw_topology) {
  switch (drm_topology) {
    case DRMTopology::SINGLE_LM:            *hw_topology = kSingleLM;           break;
    case DRMTopology::SINGLE_LM_DSC:        *hw_topology = kSingleLMDSC;        break;
    case DRMTopology::DUAL_LM:              *hw_topology = kDualLM;             break;
    case DRMTopology::DUAL_LM_DSC:          *hw_topology = kDualLMDSC;          break;
    case DRMTopology::DUAL_LM_MERGE:        *hw_topology = kDualLMMerge;        break;
    case DRMTopology::DUAL_LM_MERGE_DSC:    *hw_topology = kDualLMMergeDSC;     break;
    case DRMTopology::DUAL_LM_DSCMERGE:     *hw_topology = kDualLMDSCMerge;     break;
    case DRMTopology::QUAD_LM_MERGE:        *hw_topology = kQuadLMMerge;        break;
    case DRMTopology::QUAD_LM_DSCMERGE:     *hw_topology = kQuadLMDSCMerge;     break;
    case DRMTopology::QUAD_LM_MERGE_DSC:    *hw_topology = kQuadLMMergeDSC;     break;
    case DRMTopology::QUAD_LM_DSC4HSMERGE:  *hw_topology = kQuadLMDSC4HSMerge;  break;
    case DRMTopology::PPSPLIT:              *hw_topology = kPPSplit;            break;
    default:                                *hw_topology = kUnknown;            break;
  }
}

void HWDeviceDRM::SetMultiRectMode(const uint32_t flags, DRMMultiRectMode *target) {
  *target = DRMMultiRectMode::NONE;
  if (flags & kMultiRect) {
    *target = DRMMultiRectMode::SERIAL;
    if (flags & kMultiRectParallelMode) {
      *target = DRMMultiRectMode::PARALLEL;
    }
  }
}

void HWDeviceDRM::SetSsppTonemapFeatures(HWPipeInfo *pipe_info) {
  if (pipe_info->dgm_csc_info.op != kNoOp) {
    SDECsc csc = {};
    SetDGMCsc(pipe_info->dgm_csc_info, &csc);
    DLOGV_IF(kTagDriverConfig, "Call Perform DGM CSC Op = %s",
            (pipe_info->dgm_csc_info.op == kSet) ? "Set" : "Reset");
    drm_atomic_intf_->Perform(DRMOps::PLANE_SET_DGM_CSC_CONFIG, pipe_info->pipe_id,
                              reinterpret_cast<uint64_t>(&csc.csc_v1));
  }
  if (pipe_info->inverse_pma_info.op != kNoOp) {
    DLOGV_IF(kTagDriverConfig, "Call Perform Inverse PMA Op = %s",
            (pipe_info->inverse_pma_info.op == kSet) ? "Set" : "Reset");
    drm_atomic_intf_->Perform(DRMOps::PLANE_SET_INVERSE_PMA, pipe_info->pipe_id,
                             (pipe_info->inverse_pma_info.inverse_pma) ? 1: 0);
  }
  SetSsppLutFeatures(pipe_info);
}

void HWDeviceDRM::SetDGMCsc(const HWPipeCscInfo &dgm_csc_info, SDECsc *csc) {
  SetDGMCscV1(dgm_csc_info.csc, &csc->csc_v1);
}

void HWDeviceDRM::SetDGMCscV1(const HWCsc &dgm_csc, sde_drm_csc_v1 *csc_v1) {
  uint32_t i = 0;
  for (i = 0; i < MAX_CSC_MATRIX_COEFF_SIZE; i++) {
    csc_v1->ctm_coeff[i] = dgm_csc.ctm_coeff[i];
    DLOGV_IF(kTagDriverConfig, " DGM csc_v1[%d] = %lld", i, csc_v1->ctm_coeff[i]);
  }
  for (i = 0; i < MAX_CSC_BIAS_SIZE; i++) {
    csc_v1->pre_bias[i] = dgm_csc.pre_bias[i];
    csc_v1->post_bias[i] = dgm_csc.post_bias[i];
  }
  for (i = 0; i < MAX_CSC_CLAMP_SIZE; i++) {
    csc_v1->pre_clamp[i] = dgm_csc.pre_clamp[i];
    csc_v1->post_clamp[i] = dgm_csc.post_clamp[i];
  }
}

void HWDeviceDRM::SetSsppLutFeatures(HWPipeInfo *pipe_info) {
  for (HWPipeTonemapLutInfo &lut_info : pipe_info->lut_info) {
    if (lut_info.op != kNoOp) {
      std::shared_ptr<PPFeatureInfo> feature = lut_info.pay_load;
      if (feature == nullptr) {
        DLOGE("Null Pointer for Op = %d lut type = %d", lut_info.op, lut_info.type);
        continue;
      }
      DRMPPFeatureInfo kernel_params = {};
      std::vector<DRMPPFeatureID> drm_id = {};
      PPBlock pp_block = GetPPBlock(lut_info.type);
      hw_color_mgr_->ToDrmFeatureId(pp_block, feature->feature_id_, &drm_id);
      for (DRMPPFeatureID id : drm_id) {
        if (id >= kPPFeaturesMax) {
          DLOGE("Invalid feature id %d", id);
          continue;
        }
        kernel_params.id = id;
        bool disable = (lut_info.op == kReset);
        DLOGV_IF(kTagDriverConfig, "Lut Type = %d PPBlock = %d Op = %s Disable = %d Feature = %p",
                 lut_info.type, pp_block, (lut_info.op ==kSet) ? "Set" : "Reset", disable,
                 feature.get());
        int ret = hw_color_mgr_->GetDrmFeature(feature.get(), &kernel_params, disable);
        if (!ret) {
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_POST_PROC, pipe_info->pipe_id,
                                    &kernel_params);
          hw_color_mgr_->FreeDrmFeatureData(&kernel_params);
        } else {
          DLOGE("GetDrmFeature failed for Lut type = %d", lut_info.type);
        }
      }
      drm_id.clear();
    }
  }
}

void HWDeviceDRM::AddDimLayerIfNeeded() {
  if (secure_display_active_ && hw_resource_.secure_disp_blend_stage >= 0) {
    HWSolidfillStage sf = {};
    sf.z_order = UINT32(hw_resource_.secure_disp_blend_stage);
    sf.roi = { 0.0, 0.0, FLOAT(mixer_attributes_.width), FLOAT(mixer_attributes_.height) };
    solid_fills_.clear();
    AddSolidfillStage(sf, 0xFF);
    SetSolidfillStages();
  }

  if (!secure_display_active_) {
    DRMSecurityLevel crtc_security_level = DRMSecurityLevel::SECURE_NON_SECURE;
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_SECURITY_LEVEL, token_.crtc_id, crtc_security_level);
  }
}

DisplayError HWDeviceDRM::NullCommit(bool synchronous, bool retain_planes) {
  DTRACE_SCOPED();
  AddDimLayerIfNeeded();
  drm_atomic_intf_->Perform(DRMOps::NULL_COMMIT_PANEL_FEATURES, 0 /* argument is not used */);
  int ret = drm_atomic_intf_->Commit(synchronous , retain_planes);
  if (ret) {
    DLOGE("failed with error %d, crtc=%u", ret, token_.crtc_id);
    return kErrorHardware;
  }
  DLOGI("Null commit succeeded crtc=%u", token_.crtc_id);

  if (first_null_cycle_)
    first_null_cycle_ = false;

  return kErrorNone;
}

void HWDeviceDRM::DumpConnectorModeInfo() {
  for (uint32_t i = 0; i < (uint32_t)connector_info_.modes.size(); i++) {
    DLOGI("Mode[%d] Name:%s vref:%d hdisp:%d hsync_s:%d hsync_e:%d htotal:%d " \
          "vdisp:%d vsync_s:%d vsync_e:%d vtotal:%d\n", i, connector_info_.modes[i].mode.name,
          connector_info_.modes[i].mode.vrefresh, connector_info_.modes[i].mode.hdisplay,
          connector_info_.modes[i].mode.hsync_start, connector_info_.modes[i].mode.hsync_end,
          connector_info_.modes[i].mode.htotal, connector_info_.modes[i].mode.vdisplay,
          connector_info_.modes[i].mode.vsync_start, connector_info_.modes[i].mode.vsync_end,
          connector_info_.modes[i].mode.vtotal);
  }
}

void HWDeviceDRM::ResetROI() {
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ROI, token_.crtc_id, 0, nullptr);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_ROI, token_.conn_id, 0, nullptr);
}

bool HWDeviceDRM::IsFullFrameUpdate(const HWLayersInfo &hw_layer_info) {
  LayerRect full_frame = {0, 0, FLOAT(mixer_attributes_.width), FLOAT(mixer_attributes_.height)};

  const LayerRect &frame_roi = hw_layer_info.left_frame_roi.at(0);
  // If multiple ROIs are present, then it's not fullscreen update.
  if (hw_layer_info.left_frame_roi.size() > 1 ||
      (IsValid(frame_roi) && !IsCongruent(full_frame, frame_roi))) {
    return false;
  }

  return true;
}

DisplayError HWDeviceDRM::SetDynamicDSIClock(uint64_t bit_clk_rate) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetDynamicDSIClock(uint64_t *bit_clk_rate) {
  return kErrorNotSupported;
}

void HWDeviceDRM::DumpHWLayers(HWLayersInfo *hw_layers_info) {
  DestScaleInfoMap &dest_scale_info_map = hw_layers_info->dest_scale_info_map;
  uint32_t hw_layer_count = UINT32(hw_layers_info->hw_layers.size());
  std::vector<LayerRect> &left_frame_roi = hw_layers_info->left_frame_roi;
  std::vector<LayerRect> &right_frame_roi = hw_layers_info->right_frame_roi;
  DLOGI("HWLayers Stack: layer_count: %d, app_layer_count: %d, gpu_target_index: %d",
         hw_layer_count, hw_layers_info->app_layer_count, hw_layers_info->gpu_target_index);
  DLOGI("LayerStackFlags = 0x%" PRIu32 ",  blend_cs = {primaries = %d, transfer = %d}",
         UINT32(hw_layers_info->flags.flags), UINT32(hw_layers_info->blend_cs.primaries),
         UINT32(hw_layers_info->blend_cs.transfer));
  for (uint32_t i = 0; i < left_frame_roi.size(); i++) {
    DLOGI("left_frame_roi: x = %d, y = %d, w = %d, h = %d", INT(left_frame_roi[i].left),
        INT(left_frame_roi[i].top), INT(left_frame_roi[i].right), INT(left_frame_roi[i].bottom));
  }
  for (uint32_t i = 0; i < right_frame_roi.size(); i++) {
  DLOGI("right_frame_roi: x = %d, y = %d, w = %d h = %d", INT(right_frame_roi[i].left),
        INT(right_frame_roi[i].top), INT(right_frame_roi[i].right),
        INT(right_frame_roi[i].bottom));
  }

  for (uint32_t i = 0; i < dest_scale_info_map.size(); i++) {
    HWDestScaleInfo *dest_scalar_data = dest_scale_info_map[i];
    if (dest_scalar_data->scale_data.enable.scale) {
      HWScaleData &scale_data = dest_scalar_data->scale_data;
      DLOGI("Dest scalar index %d Mixer WxH %dx%d", i,
            dest_scalar_data->mixer_width, dest_scalar_data->mixer_height);
      DLOGI("Panel ROI [%d, %d, %d, %d]", INT(dest_scalar_data->panel_roi.left),
           INT(dest_scalar_data->panel_roi.top), INT(dest_scalar_data->panel_roi.right),
           INT(dest_scalar_data->panel_roi.bottom));
      DLOGI("Dest scalar Dst WxH %dx%d", scale_data.dst_width, scale_data.dst_height);
    }
  }

  for (uint32_t i = 0; i < hw_layer_count; i++) {
    HWLayerConfig &hw_config = hw_layers_info->config[i];
    HWRotatorSession &hw_rotator_session = hw_config.hw_rotator_session;
    HWSessionConfig &hw_session_config = hw_rotator_session.hw_session_config;
    DLOGI("========================= HW_layer: %d =========================", i);
    DLOGI("src_width = %d, src_height = %d, src_format = %d, src_LayerBufferFlags = 0x%" PRIx32 ,
             hw_layers_info->hw_layers[i].input_buffer.width,
             hw_layers_info->hw_layers[i].input_buffer.height,
             hw_layers_info->hw_layers[i].input_buffer.format,
             hw_layers_info->hw_layers[i].input_buffer.flags.flags);

    if (hw_config.use_inline_rot) {
      DLOGI("rotator = %s, rotation = %d, flip_horizontal = %s, flip_vertical = %s",
            "inline rotator", INT(hw_session_config.transform.rotation),
            hw_session_config.transform.flip_horizontal ? "true" : "false",
            hw_session_config.transform.flip_vertical ? "true" : "false");
    } else if (hw_rotator_session.mode == kRotatorOffline) {
      DLOGI("rotator = %s, rotation = %d, flip_horizontal = %s, flip_vertical = %s",
            "offline rotator", INT(hw_session_config.transform.rotation),
            hw_session_config.transform.flip_horizontal ? "true" : "false",
            hw_session_config.transform.flip_vertical ? "true" : "false");
    }
    if (hw_config.use_solidfill_stage) {
      HWSolidfillStage &hw_solidfill_stage = hw_config.hw_solidfill_stage;
      LayerSolidFill &solid_fill_info = hw_solidfill_stage.solid_fill_info;
      DLOGI("HW Solid fill info: z_order = %d, color = %d", hw_solidfill_stage.z_order,
            hw_solidfill_stage.color);
      DLOGI("bit_depth = %d, red = %d, green = %d, blue = %d, alpha = %d",
            solid_fill_info.bit_depth, solid_fill_info.red, solid_fill_info.green,
            solid_fill_info.blue, solid_fill_info.alpha);
    }
    for (uint32_t count = 0; count < 2; count++) {
      HWPipeInfo &left_pipe = hw_config.left_pipe;
      HWPipeInfo &right_pipe = hw_config.right_pipe;
      HWPipeInfo &pipe_info = (count == 0) ? left_pipe : right_pipe;
      HWScaleData &scale_data = pipe_info.scale_data;
      if (!pipe_info.valid) {
        continue;
      }
      std::string pipe = (count == 0) ? "left_pipe" : "right_pipe";
      DLOGI("pipe = %s, pipe_id = %d, z_order = %d, flags = 0x%X",
           pipe.c_str(), pipe_info.pipe_id, pipe_info.z_order, pipe_info.flags);
      DLOGI("src_rect: x = %d, y = %d, w = %d, h = %d", INT(pipe_info.src_roi.left),
            INT(pipe_info.src_roi.top), INT(pipe_info.src_roi.right - pipe_info.src_roi.left),
            INT(pipe_info.src_roi.bottom - pipe_info.src_roi.top));
      DLOGI("dst_rect: x = %d, y = %d, w = %d, h = %d", INT(pipe_info.dst_roi.left),
            INT(pipe_info.dst_roi.top), INT(pipe_info.dst_roi.right - pipe_info.dst_roi.left),
            INT(pipe_info.dst_roi.bottom - pipe_info.dst_roi.top));
      DLOGI("excl_rect: left = %d, top = %d, right = %d, bottom = %d",
            INT(pipe_info.excl_rect.left), INT(pipe_info.excl_rect.top),
            INT(pipe_info.excl_rect.right), INT(pipe_info.excl_rect.bottom));
      if (scale_data.enable.scale) {
      DLOGI("HWScaleData enable flags: scale = %s, direction_detection = %s, detail_enhance = %s,"
            " dyn_exp_disable = %s", scale_data.enable.scale ? "true" : "false",
            scale_data.enable.direction_detection ? "true" : "false",
            scale_data.enable.detail_enhance ? "true" : "false",
            scale_data.enable.dyn_exp_disable ? "true" : "false");
      DLOGI("lut_flags: lut_swap = 0x%X, lut_dir_wr = 0x%X, lut_y_cir_wr = 0x%X, "
            "lut_uv_cir_wr = 0x%X, lut_y_sep_wr = 0x%X, lut_uv_sep_wr = 0x%X",
            scale_data.lut_flag.lut_swap, scale_data.lut_flag.lut_dir_wr,
            scale_data.lut_flag.lut_y_cir_wr, scale_data.lut_flag.lut_uv_cir_wr,
            scale_data.lut_flag.lut_y_sep_wr, scale_data.lut_flag.lut_uv_sep_wr);
      DLOGI("dir_lut_idx = %d, y_rgb_cir_lut_idx = %d, uv_cir_lut_idx = %d, "
            "y_rgb_sep_lut_idx = %d, uv_sep_lut_idx = %d", scale_data.dir_lut_idx,
            scale_data.y_rgb_cir_lut_idx, scale_data.uv_cir_lut_idx,
            scale_data.y_rgb_sep_lut_idx, scale_data.uv_sep_lut_idx);
      }
    }
  }
}

DisplayError HWDeviceDRM::SetBlendSpace(const PrimariesTransfer &blend_space) {
  blend_space_ = blend_space;
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetDRMPowerMode(const HWPowerState &power_state,
                                          DRMPowerMode *drm_power_mode) {
  if (!drm_power_mode) {
    return kErrorParameters;
  }

  switch (power_state) {
    case kPowerStateOn:
      *drm_power_mode = DRMPowerMode::ON;
      break;
    case kPowerStateOff:
      *drm_power_mode = DRMPowerMode::OFF;
      break;
    case kPowerStateDoze:
      *drm_power_mode = DRMPowerMode::DOZE;
      break;
    case kPowerStateDozeSuspend:
      *drm_power_mode = DRMPowerMode::DOZE_SUSPEND;
      break;
    default:
      return kErrorParameters;
  }
  return kErrorNone;
}

void HWDeviceDRM::SetTUIState() {
  if (tui_state_ == kTUIStateStart) {
    tui_state_ = kTUIStateInProgress;
  } else if (tui_state_ == kTUIStateEnd) {
    tui_state_ = kTUIStateNone;
  } else if (tui_state_ == kTUIStateInProgress) {
    tui_state_ = kTUIStateNone;
  }
}

void HWDeviceDRM::SetTopologySplit(HWTopology hw_topology, uint32_t *split_number) {
  switch (hw_topology) {
    case kDualLM:
    case kDualLMDSC:
    case kDualLMMerge:
    case kDualLMMergeDSC:
    case kDualLMDSCMerge:
      *split_number = 2;
      break;
    case kQuadLMMerge:
    case kQuadLMDSCMerge:
    case kQuadLMMergeDSC:
    case kQuadLMDSC4HSMerge:
      *split_number = 4;
      break;
    case kPPSplit:
    default:
      *split_number = 1;
  }
}

void HWDeviceDRM::SetTopologyMuxUsage(HWTopology hw_topology, bool *is_3d_mux_used) {
  switch (hw_topology) {
    case kDualLMMerge:
    case kDualLMMergeDSC:
    case kQuadLMMerge:
    case kQuadLMMergeDSC:
      *is_3d_mux_used = true;
      break;
    default:
      *is_3d_mux_used = false;
  }
}

uint64_t HWDeviceDRM::GetSupportedBitClkRate(uint32_t new_mode_index,
                                uint64_t bit_clk_rate_request) {
  if (current_mode_index_ == new_mode_index) {
    uint32_t submode_idx = connector_info_.modes[current_mode_index_].curr_submode_index;
    sde_drm::DRMSubModeInfo curr_sub_mode =
                         connector_info_.modes[current_mode_index_].sub_modes[submode_idx];
    if ((std::find(curr_sub_mode.dyn_bitclk_list.begin(), curr_sub_mode.dyn_bitclk_list.end(),
         bit_clk_rate_request) != curr_sub_mode.dyn_bitclk_list.end())) {
      return bit_clk_rate_request;
    } else {
      DLOGW("Requested rate not supported: %" PRIu64, bit_clk_rate_request);
      return connector_info_.modes[current_mode_index_].curr_bit_clk_rate;
    }
  }

  uint32_t submode_idx = connector_info_.modes[new_mode_index].curr_submode_index;
  sde_drm::DRMSubModeInfo curr_sub_mode =
                       connector_info_.modes[new_mode_index].sub_modes[submode_idx];
  if ((std::find(curr_sub_mode.dyn_bitclk_list.begin(), curr_sub_mode.dyn_bitclk_list.end(),
       bit_clk_rate_request) != curr_sub_mode.dyn_bitclk_list.end())) {
    return bit_clk_rate_request;
  } else {
    DLOGW("Requested rate not supported: %" PRIu64, bit_clk_rate_request);
    return connector_info_.modes[new_mode_index].default_bit_clk_rate;
  }
}

bool HWDeviceDRM::SetupConcurrentWriteback(const HWLayersInfo &hw_layer_info, bool validate,
                                           int64_t *release_fence_fd) {
  std::lock_guard<std::mutex> lock(cwb_state_lock_);
  bool enable = hw_resource_.has_concurrent_writeback && hw_layer_info.output_buffer;
  if (!(enable || cwb_config_.enabled)) {  // the frame is neither cwb setup nor cwb teardown frame
    cwb_config_.cwb_disp_id = -1;
    return false;
  }

  if (cwb_config_.cwb_disp_id != -1 && cwb_config_.cwb_disp_id != display_id_) {
    // Either cwb is currently active or tearing down on display cwb_config_.cwb_disp_id
    DLOGW("CWB already busy with display : %d", cwb_config_.cwb_disp_id);
    return false;
  } else {
    cwb_config_.cwb_disp_id = display_id_;
  }

  bool setup_modes = enable && !cwb_config_.enabled;
  // Modes can be setup in prepare or commit path.
  if (setup_modes && (SetupConcurrentWritebackModes() == kErrorNone)) {
    cwb_config_.enabled = true;
  }

  if (cwb_config_.enabled) {
    if (enable) {
      // Set DRM properties for Concurrent Writeback.
      ConfigureConcurrentWriteback(hw_layer_info);

      if (!validate && release_fence_fd) {
        // Set GET_RETIRE_FENCE property to get Concurrent Writeback fence.
        drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, cwb_config_.token.conn_id,
                                  release_fence_fd);
        return true;
      }
    } else {
      // Tear down the Concurrent Writeback topology.
      drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, cwb_config_.token.conn_id, 0);
      DLOGI("Tear down the Concurrent Writeback topology");
    }
  }

  return false;
}

DisplayError HWDeviceDRM::SetupConcurrentWritebackModes() {
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
    drm_mgr_intf_->UnregisterDisplay(&(cwb_config_.token));
    DLOGE("Dump CWBConfig: mode_count %d flags %x", cwb_cfg.count_modes, cwb_cfg.flags);
    DumpConnectorModeInfo();
    return kErrorHardware;
  }

  return kErrorNone;
}

void HWDeviceDRM::ConfigureConcurrentWriteback(const HWLayersInfo &hw_layer_info) {
  CwbConfig *cwb_config = hw_layer_info.hw_cwb_config;
  LayerBuffer *output_buffer = hw_layer_info.output_buffer;
  registry_.MapOutputBufferToFbId(output_buffer);
  uint32_t &vitual_conn_id = cwb_config_.token.conn_id;

  // Set the topology for Concurrent Writeback: [CRTC_PRIMARY_DISPLAY - CONNECTOR_VIRTUAL_DISPLAY].
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, vitual_conn_id, token_.crtc_id);

  // Set CRTC Capture Mode
  DRMCWbCaptureMode capture_mode = DRMCWbCaptureMode::MIXER_OUT;
  if (cwb_config->tap_point == CwbTapPoint::kDsppTapPoint) {
    capture_mode = DRMCWbCaptureMode::DSPP_OUT;
  } else if (cwb_config->tap_point == CwbTapPoint::kDemuraTapPoint) {
    capture_mode = DRMCWbCaptureMode::DEMURA_OUT;
  }

  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_CAPTURE_MODE, token_.crtc_id, capture_mode);

  // Set Connector Output FB
  uint32_t fb_id = registry_.GetOutputFbId(output_buffer->handle_id);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_OUTPUT_FB_ID, vitual_conn_id, fb_id);

  // Set Connector Secure Mode
  bool secure = output_buffer->flags.secure;
  DRMSecureMode mode = secure ? DRMSecureMode::SECURE : DRMSecureMode::NON_SECURE;
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_FB_SECURE_MODE, vitual_conn_id, mode);

  // Set Connector Output Rect
  sde_drm::DRMRect full_frame = {};
  full_frame.left = 0;
  full_frame.top = 0;

  if (capture_mode == DRMCWbCaptureMode::MIXER_OUT) {
    full_frame.right = mixer_attributes_.width;
    full_frame.bottom = mixer_attributes_.height;
  } else {
    full_frame.right = display_attributes_[current_mode_index_].x_pixels;
    full_frame.bottom = display_attributes_[current_mode_index_].y_pixels;
  }

  sde_drm::DRMRect cwb_dst = full_frame;
  LayerRect cwb_roi = cwb_config->cwb_roi;

  if (has_cwb_crop_) {  // If CWB ROI feature is supported, then set WB connector's roi_v1 property
    // to PU ROI and DST_* properties to CWB ROI. Else, set DST_* properties to full frame ROI.
    bool is_full_frame_update = IsFullFrameUpdate(hw_layer_info);
    // Set WB connector's roi_v1 property to PU_ROI.
    if (is_full_frame_update) {
      drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_ROI, vitual_conn_id, 0, nullptr);
      DLOGV_IF(kTagDriverConfig, "roi_v1 of virtual connector is set NULL (Full Frame update).");
    } else {
      const int kNumMaxROIs = 4;
      sde_drm::DRMRect conn_rects[kNumMaxROIs] = {full_frame};
      for (uint32_t i = 0; i < hw_layer_info.left_frame_roi.size(); i++) {
        auto &roi = hw_layer_info.left_frame_roi.at(i);
        conn_rects[i].left = UINT32(roi.left);
        conn_rects[i].right = UINT32(roi.right);
        conn_rects[i].top = UINT32(roi.top);
        conn_rects[i].bottom = UINT32(roi.bottom);
      }
      uint32_t num_rects = std::max(1u, UINT32(hw_layer_info.left_frame_roi.size()));
      drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_ROI, vitual_conn_id, num_rects, conn_rects);
    }

    // Set WB connector's DST_* property to CWB_ROI.
    if (!is_full_frame_update || !cwb_config->pu_as_cwb_roi) {
      // Either incase of partial update or when client requests only its specified ROI to be set
      // as CWB ROI without including PU ROI in it (by setting pu_as_cwb_roi flag as false), then
      // set DST_* properties to aligned client specified ROI. This if condition is an optimization
      // check. Either we can keep this if with both the conditions or we can remove the if at all
      // with direct setting of the properties. This if condition prevents reseting cwb_dst to full
      // frame incase of full frame PU ROI with client requesting PU ROI to be included in CWB ROI.
      cwb_dst.left = UINT32(cwb_roi.left);
      cwb_dst.right = UINT32(cwb_roi.right);
      cwb_dst.top = UINT32(cwb_roi.top);
      cwb_dst.bottom = UINT32(cwb_roi.bottom);
    }
  }

  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_OUTPUT_RECT, vitual_conn_id, cwb_dst);
  ConfigureCWBDither(cwb_config->dither_info, vitual_conn_id, capture_mode);

  DLOGV_IF(kTagDriverConfig, "CWB Mode:%d roi.left:%u roi.top:%u roi.right:%u roi.bottom:%u",
           capture_mode, cwb_dst.left, cwb_dst.top, cwb_dst.right, cwb_dst.bottom);
}

DisplayError HWDeviceDRM::TeardownConcurrentWriteback(void) {
  if (cwb_config_.enabled) {
    drm_mgr_intf_->UnregisterDisplay(&(cwb_config_.token));
    cwb_config_.enabled = false;
    registry_.Clear();
  }

  return kErrorNone;
}

void HWDeviceDRM::PostCommitConcurrentWriteback(LayerBuffer *output_buffer) {
  if (hw_resource_.has_concurrent_writeback && output_buffer) {
    return;
  }

  std::lock_guard<std::mutex> lock(cwb_state_lock_);
  if (cwb_config_.cwb_disp_id == display_id_) {
    TeardownConcurrentWriteback();
  }
}

DisplayError HWDeviceDRM::GetFeatureSupportStatus(const HWFeature feature, uint32_t *status) {
  DisplayError error = kErrorNone;

  if (!status) {
    return kErrorParameters;
  }

  switch (feature) {
    case kAllowedModeSwitch:
      *status = connector_info_.modes[current_mode_index_].allowed_mode_switch;
      break;
    case kHasCwbCrop:
      *status = UINT32(has_cwb_crop_);
      break;
    case kHasDedicatedCwb:
      *status = UINT32(has_dedicated_cwb_);
      break;
    default:
      DLOGW("Unable to get status of feature : %d", feature);
      error = kErrorParameters;
      break;
  }

  return error;
}

void HWDeviceDRM::FlushConcurrentWriteback() {
  std::lock_guard<std::mutex> lock(cwb_state_lock_);
  TeardownConcurrentWriteback();
  cwb_config_.cwb_disp_id = -1;
  DLOGI("Flushing out CWB Config. cwb_enabled = %d , cwb_disp_id : %d", cwb_config_.enabled,
        cwb_config_.cwb_disp_id);
}

DisplayError HWDeviceDRM::ConfigureCWBDither(void *payload, uint32_t conn_id,
                sde_drm::DRMCWbCaptureMode mode) {
  int ret = 0;
  DRMPPFeatureInfo kernel_params = {};

  if (has_cwb_dither_) {
    kernel_params.id = DRMPPFeatureID::kFeatureCWBDither;
  } else {
    kernel_params.id = DRMPPFeatureID::kFeatureDither;
    DLOGI_IF(kTagDriverConfig, "pp cwb blocks not supported, to config pp-dither hw");
  }

  if (payload && (mode == sde_drm::DRMCWbCaptureMode::DSPP_OUT ||
                  mode == sde_drm::DRMCWbCaptureMode::DEMURA_OUT)) {
    PPFeatureInfo * feature_wrapper = reinterpret_cast<PPFeatureInfo *>(payload);
    ret = hw_color_mgr_->GetDrmFeature(feature_wrapper, &kernel_params);
    if (ret)
      DLOGE("Failed to get drm feature %d params, ret %d", kernel_params.id, ret);
  } else {
    kernel_params.type = sde_drm::kPropBlob;
    kernel_params.payload = NULL;
  }

  DLOGI_IF(kTagDriverConfig, "CWB dither %s case", kernel_params.payload ? "enable" : "disable");
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POST_PROC, conn_id, &kernel_params);
  hw_color_mgr_->FreeDrmFeatureData(&kernel_params);
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetPanelBlMaxLvl(uint32_t *bl_max) {
  if (!bl_max) {
    DLOGE("Invalid input param");
    return kErrorParameters;
  }

  *bl_max = connector_info_.max_panel_backlight;
  return kErrorNone;
}

DisplayError HWDeviceDRM::SetPPConfig(void *payload, size_t size) {
  if (!payload || size != sizeof(DRMPPFeatureInfo)) {
    DLOGE("Invalid input params payload %pK, size %zd expect size %zd", payload, size,
        sizeof(DRMPPFeatureInfo));
      return kErrorParameters;
  }

  struct DRMPPFeatureInfo *info = reinterpret_cast<struct DRMPPFeatureInfo *> (payload);

  if (info->object_type == DRM_MODE_OBJECT_CONNECTOR && token_.conn_id) {
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POST_PROC, token_.conn_id, payload);
  } else if (info->object_type == DRM_MODE_OBJECT_CRTC && token_.crtc_id) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_POST_PROC, token_.crtc_id, payload);
  } else {
    DLOGE("Invalid feature input, obj_type: 0x%x , feature_id: %d, event_type: 0x%x",
          info->object_type, info->id, info->event_type);
    return kErrorParameters;
  }

  return kErrorNone;
}

DisplayError HWDeviceDRM::CancelDeferredPowerMode() {
  DLOGI("Pending state reset %d on CRTC: %u", pending_power_state_, token_.crtc_id);
  pending_power_state_ = kPowerStateNone;

  return kErrorNone;
}

}  // namespace sdm
