/*
* Copyright (c) 2019, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

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

#include <errno.h>
#include <drm_logger.h>
#include "drm_dpps_mgr_imp.h"

#define __CLASS__ "DRMDppsManagerImp"
namespace sde_drm {

static DRMDppsManagerImp dpps_mgr;
static DRMDppsManagerDummyImp dpps_dummy_mgr;

DRMDppsManagerIntf* GetDppsManagerIntf()
{
#if (defined(__ANDROID__))
    return &dpps_mgr;
#else
    return &dpps_dummy_mgr;
#endif
}

int DRMDppsManagerImp::GetDrmResources(drmModeRes* res) {
  int enc_id = -1;
  drmModeConnector *conn = NULL;
  drmModeEncoder *enc = NULL;
  drmModeCrtc *crtc = NULL;

  if (drm_fd_ < 0 ) {
    DRM_LOGE("Invalid drm_fd_ %d", drm_fd_);
    return -EINVAL;
  }

  for (auto i = 0; i < res->count_connectors; i++) {
    conn = drmModeGetConnector(drm_fd_, res->connectors[i]);
    if (conn && conn->connector_type == DRM_MODE_CONNECTOR_DSI &&
        conn->count_modes && conn->connection == DRM_MODE_CONNECTED) {
      DRM_LOGI("Found connector %d", conn->connector_id);
      conn_id_ = conn->connector_id;
      break;
    }
    drmModeFreeConnector(conn);
    conn = NULL;
  }
  if (conn_id_ < 0 || !conn) {
    DRM_LOGE("Cannot find valid connector");
    conn_id_ = -1;
    return -EINVAL;
  }

  for (auto i = 0; i < conn->count_encoders; i++) {
    enc = drmModeGetEncoder(drm_fd_, conn->encoders[i]);
    if (enc && enc->encoder_type == DRM_MODE_ENCODER_DSI) {
      DRM_LOGI("Found encoder %d", enc->encoder_id);
      enc_id = enc->encoder_id;
      break;
    }
    drmModeFreeEncoder(enc);
    enc = NULL;
  }
  if (enc_id < 0 || !enc) {
    DRM_LOGE("Cannot find valid encoder");
    drmModeFreeConnector(conn);
    conn = NULL;
    res = NULL;
    conn_id_ = -1;
    return -EINVAL;
  }

  for (auto i = 0; i < res->count_crtcs; i++) {
    if (enc->possible_crtcs & (1 << i)) {
      crtc = drmModeGetCrtc(drm_fd_, res->crtcs[i]);
      if (crtc) {
        DRM_LOGI("Found crtc %d", crtc->crtc_id);
        crtc_id_ = crtc->crtc_id;
        break;
      }
      drmModeFreeCrtc(crtc);
      crtc = NULL;
    }
  }
  if (crtc_id_ < 0 || !crtc) {
    DRM_LOGE("Cannot find valid crtc");
    drmModeFreeEncoder(enc);
    drmModeFreeConnector(conn);
    enc = NULL;
    conn = NULL;
    conn_id_ = -1;
    crtc_id_ = -1;
    return -EINVAL;
  }
  return 0;
}

int DRMDppsManagerImp::InitCrtcProps() {
  if (drm_fd_ < 0 || crtc_id_ < 0) {
    DRM_LOGE("Invalid drm_fd_ %d or crtc_id_ %d", drm_fd_, crtc_id_);
    return -EINVAL;
  }

  drmModeObjectProperties *props =
    drmModeObjectGetProperties(drm_fd_, crtc_id_, DRM_MODE_OBJECT_CRTC);
  if (!props || !props->props || !props->prop_values) {
    drmModeFreeObjectProperties(props);
    return -EINVAL;
  }

  for (uint32_t j = 0; j < props->count_props; j++) {
    drmModePropertyRes *info = drmModeGetProperty(drm_fd_, props->props[j]);
    if (!info) {
      continue;
    }

    std::string property_name(info->name);
    DRMProperty prop_enum = prop_mgr_.GetPropertyEnum(property_name);
    if (prop_enum == DRMProperty::INVALID) {
      DRM_LOGD("DRMProperty %s missing from global property mapping", info->name);
      drmModeFreeProperty(info);
      continue;
    }

    prop_mgr_.SetPropertyId(prop_enum, info->prop_id);
    drmModeFreeProperty(info);
  }

  drmModeFreeObjectProperties(props);
  return 0;
}

int DRMDppsManagerImp::InitConnProps()
{
  if (drm_fd_ < 0 || conn_id_ < 0) {
    DRM_LOGE("Invalid drm_fd_ %d or conn_id_ %d", drm_fd_, conn_id_);
    return -EINVAL;
  }

  drmModeObjectProperties *props =
      drmModeObjectGetProperties(drm_fd_, conn_id_, DRM_MODE_OBJECT_CONNECTOR);
  if (!props || !props->props || !props->prop_values) {
    drmModeFreeObjectProperties(props);
    return -EINVAL;
  }

  for (uint32_t j = 0; j < props->count_props; j++) {
    drmModePropertyRes *info = drmModeGetProperty(drm_fd_, props->props[j]);
    if (!info) {
      continue;
    }

    std::string property_name(info->name);
    DRMProperty prop_enum = prop_mgr_.GetPropertyEnum(property_name);
    if (prop_enum == DRMProperty::INVALID) {
      DRM_LOGD("DRMProperty %s missing from global property mapping", info->name);
      drmModeFreeProperty(info);
      continue;
    }

    prop_mgr_.SetPropertyId(prop_enum, info->prop_id);
    drmModeFreeProperty(info);
  }

  drmModeFreeObjectProperties(props);
  return 0;
}

void DRMDppsManagerImp::Init(int fd, drmModeRes* res) {
  int ret = 0;

  if (fd < 0 || !res) {
    DRM_LOGE("Invalid drm fd %d or res %p", fd, res);
    return;
  }

  drm_fd_ = fd;
  ret = GetDrmResources(res);
  if (ret) {
    DRM_LOGE("Failed to get DRM resources %d", ret);
    return;
  } else {
    ret = InitCrtcProps();
    if (ret) {
      DRM_LOGE("Failed to initialize crtc properties %d", ret);
      return;
    }
    ret = InitConnProps();
    if (ret) {
      DRM_LOGE("Failed to initialize conn properties %d", ret);
      return;
    }
  }

  dpps_feature_[kFeatureAd4Mode] = DRMDppsPropInfo {
    (prop_mgr_.GetPropertyId(DRMProperty::SDE_DSPP_AD4_MODE) == 0 ? 0U : 4U) /* version */,
    DRMProperty::SDE_DSPP_AD4_MODE, prop_mgr_.GetPropertyId(DRMProperty::SDE_DSPP_AD4_MODE),
    false /* is_event */};
  dpps_feature_[kFeatureAd4Init] = DRMDppsPropInfo {4 /* version */,
    DRMProperty::SDE_DSPP_AD4_INIT, prop_mgr_.GetPropertyId(DRMProperty::SDE_DSPP_AD4_INIT),
    false /* is_event */};
  dpps_feature_[kFeatureAd4Cfg] = DRMDppsPropInfo { 4 /* version */,
    DRMProperty::SDE_DSPP_AD4_CFG, prop_mgr_.GetPropertyId(DRMProperty::SDE_DSPP_AD4_CFG),
    false /* is_event */};
  dpps_feature_[kFeatureAd4Input] = DRMDppsPropInfo {4 /* version */,
    DRMProperty::SDE_DSPP_AD4_INPUT, prop_mgr_.GetPropertyId(DRMProperty::SDE_DSPP_AD4_INPUT),
    false /* is_event */};
  dpps_feature_[kFeatureAd4Backlight] = DRMDppsPropInfo {4 /* version */,
    DRMProperty::SDE_DSPP_AD4_BACKLIGHT, prop_mgr_.GetPropertyId(DRMProperty::SDE_DSPP_AD4_BACKLIGHT),
    false /* is_event */};
  dpps_feature_[kFeatureAd4Assertiveness] = DRMDppsPropInfo {4 /* version */,
    DRMProperty::SDE_DSPP_AD4_ASSERTIVENESS, prop_mgr_.GetPropertyId(DRMProperty::SDE_DSPP_AD4_ASSERTIVENESS),
    false /* is_event */};
  dpps_feature_[kFeatureAd4Roi] = DRMDppsPropInfo {4 /* version */,
    DRMProperty::SDE_DSPP_AD4_ROI, prop_mgr_.GetPropertyId(DRMProperty::SDE_DSPP_AD4_ROI),
    false /* is_event */};
  dpps_feature_[kFeatureAd4ManualStrength] = DRMDppsPropInfo {4 /* version */,
    DRMProperty::SDE_DSPP_AD4_STRENGTH, prop_mgr_.GetPropertyId(DRMProperty::SDE_DSPP_AD4_STRENGTH),
    false /* is_event */};
  dpps_feature_[kFeatureAbaHistCtrl] = DRMDppsPropInfo {1 /* version */,
    DRMProperty::SDE_DSPP_ABA_HIST_CTRL, prop_mgr_.GetPropertyId(DRMProperty::SDE_DSPP_ABA_HIST_CTRL),
    false /* is_event */};
  dpps_feature_[kFeatureAbaHistIRQ] = DRMDppsPropInfo {1 /* version */,
    DRMProperty::SDE_DSPP_ABA_HIST_IRQ, prop_mgr_.GetPropertyId(DRMProperty::SDE_DSPP_ABA_HIST_IRQ),
    false /* is_event */};
  dpps_feature_[kFeatureAbaLut] = DRMDppsPropInfo {1 /* version */,
    DRMProperty::SDE_DSPP_ABA_LUT, prop_mgr_.GetPropertyId(DRMProperty::SDE_DSPP_ABA_LUT),
    false /* is_event */};
  dpps_feature_[kFeatureAd4BlScale] = DRMDppsPropInfo {1 /* version */,
    DRMProperty::SDE_DSPP_AD4_BL_SCALE, prop_mgr_.GetPropertyId(DRMProperty::SDE_DSPP_AD4_BL_SCALE),
    false /* is_event */};
  dpps_feature_[kFeatureBacklightScale] = DRMDppsPropInfo {1 /* version */,
    DRMProperty::SDE_DSPP_BL_SCALE, prop_mgr_.GetPropertyId(DRMProperty::SDE_DSPP_BL_SCALE),
    false /* is_event */};

  dpps_feature_[kFeaturePowerEvent] = DRMDppsPropInfo{1, DRMProperty::INVALID, 0, true /* is_event */};
  dpps_feature_[kFeatureAbaHistEvent] = DRMDppsPropInfo{1, DRMProperty::INVALID, 0, true /* is_event */};
  dpps_feature_[kFeatureBackLightEvent] = DRMDppsPropInfo{1, DRMProperty::INVALID, 0, true /* is_event */};
  dpps_feature_[kFeatureAdAttBlEvent] = DRMDppsPropInfo{1, DRMProperty::INVALID, 0, true /* is_event */};
}

void DRMDppsManagerImp::CacheDppsFeature(uint32_t obj_id, va_list args) {
  uint32_t feature_id = va_arg(args, uint32_t);
  uint64_t value = va_arg(args, uint64_t);
  struct DRMDppsPropInfo* info;

  if (feature_id >= kDppsFeaturesMax) {
    DRM_LOGE("Invalid feature id %d for obj_id 0x%x", feature_id, obj_id);
    return;
  }

  info = &dpps_feature_[feature_id];
  info->obj_id = obj_id;
  info->value = value;
  if (info->is_event) {
    dpps_dirty_event_.push_back(*info);
  } else {
    for (auto &it : dpps_dirty_prop_) {
      if ((it.obj_id == info->obj_id) && (it.prop_id == info->prop_id)) {
        it.value = info->value;
        return;
      }
    }
    dpps_dirty_prop_.push_back(*info);
  }
}

void DRMDppsManagerImp::CommitDppsFeatures(drmModeAtomicReq *req, const DRMDisplayToken &tok) {
  if (!req)
    return;

  // Set Dpps properties
  if (!dpps_dirty_prop_.empty()) {
    for (auto it = dpps_dirty_prop_.begin(); it != dpps_dirty_prop_.end();) {
      if (it->obj_id == tok.crtc_id || it->obj_id == tok.conn_id) {
        drmModeAtomicAddProperty(req, it->obj_id, it->prop_id, it->value);
        it = dpps_dirty_prop_.erase(it);
      } else {
        it++;
      }
    }
  }

  // Set Dpps events
  if (!dpps_dirty_event_.empty()) {
    for (auto it = dpps_dirty_event_.begin(); it != dpps_dirty_event_.end();) {
      if (!it->value)
        continue;

      struct DRMDppsEventInfo info = *(struct DRMDppsEventInfo*)it->value;
      struct drm_msm_event_req event_req = {};
      int ret;
      if (it->obj_id == tok.crtc_id || it->obj_id == tok.conn_id) {
        event_req.object_id = it->obj_id;
        event_req.object_type = info.object_type;
        event_req.event = info.event_type;
        if (info.enable)
          ret = drmIoctl(info.drm_fd, DRM_IOCTL_MSM_REGISTER_EVENT, &event_req);
        else
          ret = drmIoctl(info.drm_fd, DRM_IOCTL_MSM_DEREGISTER_EVENT, &event_req);
        if (ret)
          DRM_LOGE("Failed to set event 0x%x, object_id %u, object_type 0x%x, enable %d",
              event_req.event, event_req.object_id, info.object_type, info.enable);
        it = dpps_dirty_event_.erase(it);
      } else {
        it++;
      }
    }
  }
}

void DRMDppsManagerImp::GetDppsFeatureInfo(DRMDppsFeatureInfo *info)
{
  if (!info) {
    DRM_LOGE("Invalid info NULL");
    return;
  }

  DRMDPPSFeatureID id = info->id;
  if (id >= kDppsFeaturesMax) {
    DRM_LOGE("Invalid feature id %d", id);
    return;
  }
  info->version = dpps_feature_[id].version;
}

}  // namespace sde_drm
