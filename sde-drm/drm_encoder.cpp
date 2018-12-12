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

#include <stdint.h>
#include <stdlib.h>
#include <drm.h>
#include <drm/sde_drm.h>
#include <drm_logger.h>
#include <errno.h>
#include <stdlib.h>
#include <algorithm>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "drm_encoder.h"
#include "drm_utils.h"

namespace sde_drm {

using std::unique_ptr;
using std::map;

#define __CLASS__ "DRMEncoderManager"

DRMEncoderManager::~DRMEncoderManager() {}

void DRMEncoderManager::Init(drmModeRes *resource) {
  std::set<uint32_t> tmds_encoders;
  std::set<uint32_t> dpmst_encoders;
  for (int i = 0; i < resource->count_encoders; i++) {
    unique_ptr<DRMEncoder> encoder(new DRMEncoder(fd_));
    uint32_t encoder_type;
    drmModeEncoder *libdrm_encoder = drmModeGetEncoder(fd_, resource->encoders[i]);
    if (!libdrm_encoder) {
      DRM_LOGE("Critical error: drmModeGetEncoder() failed for encoder %d.", resource->encoders[i]);
      continue;
    }
    encoder->InitAndParse(libdrm_encoder);
    encoder_pool_[resource->encoders[i]] = std::move(encoder);
    encoder_pool_[resource->encoders[i]]->GetType(&encoder_type);
    switch (encoder_type) {
      case DRM_MODE_ENCODER_TMDS:
        tmds_encoders.insert(resource->encoders[i]);
        break;
      case DRM_MODE_ENCODER_DPMST:
        dpmst_encoders.insert(resource->encoders[i]);
        break;
      default:
        break;
    }
  }
  DRM_LOGI("Found %d TMDS encoders and %d DPMST encoders.", tmds_encoders.size(),
           dpmst_encoders.size());
  // DRM_MODE_ENCODER_TMDS type is for DVI, HDMI and (embedded) DisplayPort.
  // DRM_MODE_ENCODER_DPMST type is for special fake encoders used to allow mutliple DP MST streams
  // to share one physical encoder.
  // Maximum number of DRMDisplayType::TV displays supported is maximum of TMDS and DPMST encoders.
  // DRMEncoderManager is used only for discovering number of display interfaces supported and for
  // keeping track of display interfaces used/available. Reserving DRMDisplayType::TV does not
  // distinguish between TMDS and DPMST encoders. So remove TMDS/DPMST encoders of the type with
  // the least encoders. This will ensure HWInfoDRM::GetMaxDisplaysSupported() still works right.
  if (tmds_encoders.size() < dpmst_encoders.size()) {
    for (auto iter : tmds_encoders) {
      encoder_pool_.erase(iter);
    }
  } else {
    for (auto iter : dpmst_encoders) {
      encoder_pool_.erase(iter);
    }
  }
}

void DRMEncoderManager::DumpByID(uint32_t id) {
  encoder_pool_.at(id)->Dump();
}

void DRMEncoderManager::DumpAll() {
  for (auto &encoder : encoder_pool_) {
    encoder.second->Dump();
  }
}

int DRMEncoderManager::GetEncoderInfo(uint32_t encoder_id, DRMEncoderInfo *info) {
  int ret = -ENODEV;
  auto iter = encoder_pool_.find(encoder_id);

  if (iter != encoder_pool_.end()) {
    encoder_pool_[encoder_id]->GetInfo(info);
    ret = 0;
  }
  return ret;
}

int DRMEncoderManager::GetEncoderList(std::vector<uint32_t> *encoder_ids) {
  if (!encoder_ids) {
    return -EINVAL;
  }
  encoder_ids->clear();
  for (auto &encoder : encoder_pool_) {
    encoder_ids->push_back(encoder.first);
  }
  return 0;
}

static bool IsTVEncoder(uint32_t type) {
  return (type == DRM_MODE_ENCODER_TMDS || type == DRM_MODE_ENCODER_DPMST);
}

int DRMEncoderManager::Reserve(DRMDisplayType disp_type, DRMDisplayToken *token) {
  int ret = -ENODEV;
  for (auto &encoder : encoder_pool_) {
    if (encoder.second->GetStatus() == DRMStatus::FREE) {
      uint32_t encoder_type;
      encoder.second->GetType(&encoder_type);
      if ((disp_type == DRMDisplayType::PERIPHERAL && encoder_type == DRM_MODE_ENCODER_DSI) ||
          (disp_type == DRMDisplayType::VIRTUAL && encoder_type == DRM_MODE_ENCODER_VIRTUAL) ||
          (disp_type == DRMDisplayType::TV && IsTVEncoder(encoder_type))) {
        encoder.second->Lock();
        token->encoder_id = encoder.first;
        ret = 0;
        break;
      }
    }
  }
  return ret;
}

int DRMEncoderManager::Reserve(int32_t display_id, DRMDisplayToken *token) {
  int ret = -ENODEV;
  return ret;
}

void DRMEncoderManager::Free(const DRMDisplayToken &token) {
  auto iter = encoder_pool_.find(token.encoder_id);
  if (iter != encoder_pool_.end()) {
    iter->second->Unlock();
  } else {
    DRM_LOGW("Failed! encoder_id %u not found!", token.encoder_id);
  }
}

// ==============================================================================================//

#undef __CLASS__
#define __CLASS__ "DRMEncoder"

DRMEncoder::~DRMEncoder() {
  if (drm_encoder_) {
    drmModeFreeEncoder(drm_encoder_);
  }
}

void DRMEncoder::GetInfo(DRMEncoderInfo *info) {
  *info = encoder_info_;
}

void DRMEncoder::Lock() {
  status_ = DRMStatus::BUSY;
}

void DRMEncoder::Unlock() {
  status_ = DRMStatus::FREE;
}

void DRMEncoder::InitAndParse(drmModeEncoder *encoder) {
  drm_encoder_ = encoder;
  encoder_info_.type = drm_encoder_->encoder_type;
}

void DRMEncoder::Dump() {
  DRM_LOGI("id: %d\tencoder_type: %d fd = %d\n", drm_encoder_->encoder_id,
           drm_encoder_->encoder_type, fd_);
}

}  // namespace sde_drm
