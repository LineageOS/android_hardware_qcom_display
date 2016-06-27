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

#define __STDC_FORMAT_MACROS

#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/fb.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/sys.h>
#include <drm_master.h>
#include <drm_res_mgr.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "hw_device_drm.h"
#include "hw_info_interface.h"

#define __CLASS__ "HWDeviceDRM"

using std::string;
using std::to_string;
using std::fstream;
using drm_utils::DRMMaster;
using drm_utils::DRMResMgr;
using drm_utils::DRMLogger;

namespace sdm {

class DRMLoggerDAL : public DRMLogger {
 public:
#define PRIV_LOG(suffix) { \
  char buf[1024]; \
  va_list list; \
  va_start(list, str); \
  vsnprintf(buf, sizeof(buf), str, list); \
  va_end(list); \
  DLOG##suffix("%s", buf); \
}
  void Error(const char *str, ...) {
    PRIV_LOG(E);
  }
  void Info(const char *str, ...) {
    PRIV_LOG(I);
  }
  void Debug(const char *str, ...) {
    PRIV_LOG(D);
  }
};

HWDeviceDRM::HWDeviceDRM(BufferSyncHandler *buffer_sync_handler, HWInfoInterface *hw_info_intf)
  : hw_info_intf_(hw_info_intf), buffer_sync_handler_(buffer_sync_handler) {
  DRMLogger::Set(new DRMLoggerDAL());
}

DisplayError HWDeviceDRM::Init() {
  // Populate Panel Info (Used for Partial Update)
  PopulateHWPanelInfo();
  // Populate HW Capabilities
  hw_resource_ = HWResourceInfo();
  hw_info_intf_->GetHWResourceInfo(&hw_resource_);

  DRMResMgr *res_mgr = nullptr;
  int ret = DRMResMgr::GetInstance(&res_mgr);
  if (ret < 0) {
    DLOGE("Failed to acquire DRMResMgr instance");
    return kErrorResources;
  }

  // TODO(user): check if default mode enabled
  drmModeModeInfo mode;
  res_mgr->GetMode(&mode);

  uint32_t mm_width = 0;
  uint32_t mm_height = 0;
  res_mgr->GetDisplayDimInMM(&mm_width, &mm_height);

  display_attributes_.x_pixels = mode.hdisplay;
  display_attributes_.y_pixels = mode.vdisplay;
  display_attributes_.fps = mode.vrefresh;
  display_attributes_.vsync_period_ns = UINT32(1000000000L / display_attributes_.fps);

  /*
              Active                 Front           Sync           Back
              Region                 Porch                          Porch
     <-----------------------><----------------><-------------><-------------->
     <----- [hv]display ----->
     <------------- [hv]sync_start ------------>
     <--------------------- [hv]sync_end --------------------->
     <-------------------------------- [hv]total ----------------------------->
   */

  display_attributes_.v_front_porch = mode.vsync_start - mode.vdisplay;
  display_attributes_.v_pulse_width = mode.vsync_end - mode.vsync_start;
  display_attributes_.v_back_porch = mode.vtotal - mode.vsync_end;
  display_attributes_.v_total = mode.vtotal;

  display_attributes_.h_total = mode.htotal;
  uint32_t h_blanking = mode.htotal - mode.hdisplay;
  display_attributes_.is_device_split = true;
  display_attributes_.h_total += display_attributes_.is_device_split ? h_blanking : 0;

  display_attributes_.x_dpi =
      (FLOAT(mode.hdisplay) * 25.4f) / FLOAT(mm_width);
  display_attributes_.y_dpi =
      (FLOAT(mode.vdisplay) * 25.4f) / FLOAT(mm_height);

  return kErrorNone;
}

DisplayError HWDeviceDRM::Deinit() {
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetActiveConfig(uint32_t *active_config) {
  *active_config = 0;
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetNumDisplayAttributes(uint32_t *count) {
  *count = 1;
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetDisplayAttributes(uint32_t index,
                                            HWDisplayAttributes *display_attributes) {
  *display_attributes = display_attributes_;
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetHWPanelInfo(HWPanelInfo *panel_info) {
  *panel_info = hw_panel_info_;
  return kErrorNone;
}

DisplayError HWDeviceDRM::SetDisplayAttributes(uint32_t index) {
  return kErrorNone;
}

DisplayError HWDeviceDRM::SetDisplayAttributes(const HWDisplayAttributes &display_attributes) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetConfigIndex(uint32_t mode, uint32_t *index) {
  return kErrorNone;
}

DisplayError HWDeviceDRM::PowerOn() {
  DTRACE_SCOPED();
  return kErrorNone;
}

DisplayError HWDeviceDRM::PowerOff() {
  return kErrorNone;
}

DisplayError HWDeviceDRM::Doze() {
  return kErrorNone;
}

DisplayError HWDeviceDRM::DozeSuspend() {
  return kErrorNone;
}

DisplayError HWDeviceDRM::Standby() {
  return kErrorNone;
}

DisplayError HWDeviceDRM::Validate(HWLayers *hw_layers) {
  DTRACE_SCOPED();
  return kErrorNone;
}

DisplayError HWDeviceDRM::Commit(HWLayers *hw_layers) {
  DTRACE_SCOPED();

  HWLayersInfo &hw_layer_info = hw_layers->info;
  LayerStack *stack = hw_layer_info.stack;

  stack->retire_fence_fd = -1;
  for (Layer &layer : hw_layer_info.hw_layers) {
    layer.input_buffer.release_fence_fd = -1;
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

  LayerBuffer &input_buffer = hw_layer_info.hw_layers.at(0).input_buffer;
  ret = drmModeSetCrtc(dev_fd, crtc_id, input_buffer.fb_id, 0 /* x */, 0 /* y */, &connector_id,
                       1 /* num_connectors */, &mode);
  if (ret < 0) {
    DLOGE("drmModeSetCrtc failed dev fd %d, fb_id %d, crtc id %d, connector id %d, %s", dev_fd,
          input_buffer.fb_id, crtc_id, connector_id, strerror(errno));
    return kErrorHardware;
  }

  return kErrorNone;
}

DisplayError HWDeviceDRM::Flush() {
  return kErrorNone;
}

void HWDeviceDRM::PopulateHWPanelInfo() {
  hw_panel_info_ = HWPanelInfo();
  GetHWPanelInfoByNode(0 /* Primary */, &hw_panel_info_);
  DLOGI("Device type = %d, Display Port = %d, Display Mode = %d, Device Node = %d, Is Primary = %d",
        device_type_, hw_panel_info_.port, hw_panel_info_.mode, 0 /* primary */,
        hw_panel_info_.is_primary_panel);
  DLOGI("Partial Update = %d, Dynamic FPS = %d",
        hw_panel_info_.partial_update, hw_panel_info_.dynamic_fps);
  DLOGI("Align: left = %d, width = %d, top = %d, height = %d",
        hw_panel_info_.left_align, hw_panel_info_.width_align,
        hw_panel_info_.top_align, hw_panel_info_.height_align);
  DLOGI("ROI: min_width = %d, min_height = %d, need_merge = %d",
        hw_panel_info_.min_roi_width, hw_panel_info_.min_roi_height,
        hw_panel_info_.needs_roi_merge);
  DLOGI("FPS: min = %d, max =%d", hw_panel_info_.min_fps, hw_panel_info_.max_fps);
  DLOGI("Left Split = %d, Right Split = %d", hw_panel_info_.split_info.left_split,
        hw_panel_info_.split_info.right_split);
}

void HWDeviceDRM::GetHWPanelNameByNode(int device_node, HWPanelInfo *panel_info) {
  snprintf(panel_info->panel_name, sizeof(panel_info->panel_name), "%s",
           "Dual SHARP video mode dsi panel");
}

void HWDeviceDRM::GetHWPanelInfoByNode(int device_node, HWPanelInfo *panel_info) {
  panel_info->partial_update = 0;
  panel_info->left_align = 0;
  panel_info->width_align = 0;
  panel_info->top_align = 0;
  panel_info->height_align = 0;
  panel_info->min_roi_width = 0;
  panel_info->min_roi_height = 0;
  panel_info->needs_roi_merge = 0;
  panel_info->dynamic_fps = 0;
  panel_info->min_fps = 60;
  panel_info->max_fps = 60;
  panel_info->is_primary_panel = 1;
  panel_info->is_pluggable = 0;

  GetHWDisplayPortAndMode(device_node, panel_info);
  GetSplitInfo(device_node, panel_info);
  GetHWPanelNameByNode(device_node, panel_info);
  GetHWPanelMaxBrightnessFromNode(panel_info);
}

void HWDeviceDRM::GetHWDisplayPortAndMode(int device_node, HWPanelInfo *panel_info) {
  DisplayPort *port = &panel_info->port;
  HWDisplayMode *mode = &panel_info->mode;

  *port = kPortDefault;
  *mode = kModeDefault;

  string line = "mipi dsi video panel";

  if ((strncmp(line.c_str(), "mipi dsi cmd panel", strlen("mipi dsi cmd panel")) == 0)) {
    *port = kPortDSI;
    *mode = kModeCommand;
  } else if ((strncmp(line.c_str(), "mipi dsi video panel", strlen("mipi dsi video panel")) == 0)) {
    *port = kPortDSI;
    *mode = kModeVideo;
  } else if ((strncmp(line.c_str(), "lvds panel", strlen("lvds panel")) == 0)) {
    *port = kPortLVDS;
    *mode = kModeVideo;
  } else if ((strncmp(line.c_str(), "edp panel", strlen("edp panel")) == 0)) {
    *port = kPortEDP;
    *mode = kModeVideo;
  } else if ((strncmp(line.c_str(), "dtv panel", strlen("dtv panel")) == 0)) {
    *port = kPortDTV;
    *mode = kModeVideo;
  } else if ((strncmp(line.c_str(), "writeback panel", strlen("writeback panel")) == 0)) {
    *port = kPortWriteBack;
    *mode = kModeCommand;
  }

  return;
}

void HWDeviceDRM::GetSplitInfo(int device_node, HWPanelInfo *panel_info) {
  if (display_attributes_.is_device_split) {
    panel_info->split_info.left_split = panel_info->split_info.right_split =
      display_attributes_.x_pixels / 2;
  }
}

void HWDeviceDRM::GetHWPanelMaxBrightnessFromNode(HWPanelInfo *panel_info) {
  char brightness[kMaxStringLength] = { 0 };
  char kMaxBrightnessNode[64] = { 0 };

  snprintf(kMaxBrightnessNode, sizeof(kMaxBrightnessNode), "%s",
           "/sys/class/leds/lcd-backlight/max_brightness");

  panel_info->panel_max_brightness = 0;
  int fd = Sys::open_(kMaxBrightnessNode, O_RDONLY);
  if (fd < 0) {
    DLOGW("Failed to open max brightness node = %s, error = %s", kMaxBrightnessNode,
          strerror(errno));
    return;
  }

  if (Sys::pread_(fd, brightness, sizeof(brightness), 0) > 0) {
    panel_info->panel_max_brightness = atoi(brightness);
    DLOGI("Max brightness level = %d", panel_info->panel_max_brightness);
  } else {
    DLOGW("Failed to read max brightness level. error = %s", strerror(errno));
  }
  Sys::close_(fd);

  panel_info->panel_max_brightness = 255;
}

bool HWDeviceDRM::EnableHotPlugDetection(int enable) {
  return true;
}

void HWDeviceDRM::ResetDisplayParams() {
}

DisplayError HWDeviceDRM::SetCursorPosition(HWLayers *hw_layers, int x, int y) {
  DTRACE_SCOPED();
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetPPFeaturesVersion(PPFeatureVersion *vers) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::SetPPFeatures(PPFeaturesConfig *feature_list) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::SetVSyncState(bool enable) {
  return kErrorNone;
}

void HWDeviceDRM::SetIdleTimeoutMs(uint32_t timeout_ms) {
}

DisplayError HWDeviceDRM::SetDisplayMode(const HWDisplayMode hw_display_mode) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::SetRefreshRate(uint32_t refresh_rate) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::SetPanelBrightness(int level) {
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
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetPanelBrightness(int *level) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::SetS3DMode(HWS3DMode s3d_mode) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::SetScaleLutConfig(HWScaleLutInfo *lut_info) {
  return kErrorNone;
}

DisplayError HWDeviceDRM::SetMixerAttributes(const HWMixerAttributes &mixer_attributes) {
  if (!hw_resource_.hw_dest_scalar_info.count) {
    return kErrorNotSupported;
  }

  if (mixer_attributes.width > display_attributes_.x_pixels ||
      mixer_attributes.height > display_attributes_.y_pixels) {
    DLOGW("Input resolution exceeds display resolution! input: res %dx%d display: res %dx%d",
          mixer_attributes.width, mixer_attributes.height, display_attributes_.x_pixels,
          display_attributes_.y_pixels);
    return kErrorNotSupported;
  }

  uint32_t max_input_width = hw_resource_.hw_dest_scalar_info.max_input_width;
  if (display_attributes_.is_device_split) {
    max_input_width *= 2;
  }

  if (mixer_attributes.width > max_input_width) {
    DLOGW("Input width exceeds width limit! input_width %d width_limit %d", mixer_attributes.width,
          max_input_width);
    return kErrorNotSupported;
  }

  float mixer_aspect_ratio = FLOAT(mixer_attributes.width) / FLOAT(mixer_attributes.height);
  float display_aspect_ratio =
    FLOAT(display_attributes_.x_pixels) / FLOAT(display_attributes_.y_pixels);

  if (display_aspect_ratio != mixer_aspect_ratio) {
    DLOGW("Aspect ratio mismatch! input: res %dx%d display: res %dx%d", mixer_attributes.width,
          mixer_attributes.height, display_attributes_.x_pixels, display_attributes_.y_pixels);
    return kErrorNotSupported;
  }

  float scale_x = FLOAT(display_attributes_.x_pixels) / FLOAT(mixer_attributes.width);
  float scale_y = FLOAT(display_attributes_.y_pixels) / FLOAT(mixer_attributes.height);
  float max_scale_up = hw_resource_.hw_dest_scalar_info.max_scale_up;
  if (scale_x > max_scale_up || scale_y > max_scale_up) {
    DLOGW("Up scaling ratio exceeds for destination scalar upscale limit scale_x %f scale_y %f " \
          "max_scale_up %f", scale_x, scale_y, max_scale_up);
    return kErrorNotSupported;
  }

  float mixer_split_ratio = FLOAT(mixer_attributes_.split_left) / FLOAT(mixer_attributes_.width);

  mixer_attributes_ = mixer_attributes;
  mixer_attributes_.split_left = mixer_attributes_.width;
  if (display_attributes_.is_device_split) {
    mixer_attributes_.split_left = UINT32(FLOAT(mixer_attributes.width) * mixer_split_ratio);
  }

  return kErrorNone;
}

DisplayError HWDeviceDRM::GetMixerAttributes(HWMixerAttributes *mixer_attributes) {
  if (!mixer_attributes) {
    return kErrorParameters;
  }

  mixer_attributes_.width = display_attributes_.x_pixels;
  mixer_attributes_.height = display_attributes_.y_pixels;
  mixer_attributes_.split_left = display_attributes_.is_device_split ?
      hw_panel_info_.split_info.left_split : mixer_attributes_.width;
  *mixer_attributes = mixer_attributes_;

  return kErrorNone;
}

}  // namespace sdm
