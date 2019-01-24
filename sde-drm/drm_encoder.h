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

#ifndef __DRM_ENCODER_H__
#define __DRM_ENCODER_H__

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <map>
#include <memory>
#include <vector>

#include "drm_interface.h"
#include "drm_utils.h"

namespace sde_drm {

class DRMEncoder {
 public:
  explicit DRMEncoder(int fd) : fd_(fd) {}
  void InitAndParse(drmModeEncoder *encoder);
  DRMStatus GetStatus() { return status_; }
  void GetInfo(DRMEncoderInfo *info);
  void GetType(uint32_t *encoder_type) { *encoder_type = drm_encoder_->encoder_type; }
  void Dump();
  void Lock();
  void Unlock();
  ~DRMEncoder();

 private:
  int fd_ = -1;
  drmModeEncoder *drm_encoder_ = {};
  DRMStatus status_ = DRMStatus::FREE;
  DRMEncoderInfo encoder_info_ = {};
};

class DRMEncoderManager {
 public:
  explicit DRMEncoderManager(int fd) : fd_(fd) {}
  ~DRMEncoderManager();
  void Init(drmModeRes *res);
  void DeInit() {}
  void DumpAll();
  void DumpByID(uint32_t id);
  int Reserve(DRMDisplayType disp_type, DRMDisplayToken *token);
  int Reserve(int32_t display_id, DRMDisplayToken *token);
  void Free(const DRMDisplayToken &token);
  int GetEncoderInfo(uint32_t encoder_id, DRMEncoderInfo *info);
  int GetEncoderList(std::vector<uint32_t> *encoder_ids);

 private:
  int fd_ = -1;
  std::map<uint32_t, std::unique_ptr<DRMEncoder>> encoder_pool_{};
};

}  // namespace sde_drm

#endif  // __DRM_ENCODER_H__
