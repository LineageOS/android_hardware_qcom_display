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

#include <drm_lib_loader.h>
#include <drm_master.h>
#include <drm_res_mgr.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <utils/debug.h>
#include <utils/sys.h>
#include <utils/formats.h>

#include <vector>
#include <map>
#include <utility>

#include "hw_hdmi_drm.h"

#define __CLASS__ "HWHDMIDRM"

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
using sde_drm::DRMOps;
using sde_drm::DRMTopology;

namespace sdm {

HWHDMIDRM::HWHDMIDRM(BufferSyncHandler *buffer_sync_handler, BufferAllocator *buffer_allocator,
                     HWInfoInterface *hw_info_intf)
  : HWDeviceDRM(buffer_sync_handler, buffer_allocator, hw_info_intf),
  active_config_index_(0) {
  HWDeviceDRM::device_type_ = kDeviceHDMI;
  HWDeviceDRM::device_name_ = "HDMI Display Device";
}

// TODO(user) : split function in base class and avoid code duplicacy
// by using base implementation for this basic stuff
DisplayError HWHDMIDRM::Init() {
  DisplayError error = kErrorNone;

  default_mode_ = (DRMLibLoader::GetInstance()->IsLoaded() == false);

  if (!default_mode_) {
    DRMMaster *drm_master = {};
    int dev_fd = -1;
    DRMMaster::GetInstance(&drm_master);
    drm_master->GetHandle(&dev_fd);
    DRMLibLoader::GetInstance()->FuncGetDRMManager()(dev_fd, &drm_mgr_intf_);
    if (drm_mgr_intf_->RegisterDisplay(DRMDisplayType::TV, &token_)) {
      DLOGE("RegisterDisplay failed");
      return kErrorResources;
    }

    drm_mgr_intf_->CreateAtomicReq(token_, &drm_atomic_intf_);
    drm_mgr_intf_->GetConnectorInfo(token_.conn_id, &connector_info_);
    InitializeConfigs();
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode_);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);

    if (drm_atomic_intf_->Commit(true /* synchronous */)) {
      DLOGE("Setting up CRTC %d, Connector %d for %s failed", token_.crtc_id, token_.conn_id,
            device_name_);
      return kErrorResources;
    }

    // Reload connector info for updated info after 1st commit
    drm_mgr_intf_->GetConnectorInfo(token_.conn_id, &connector_info_);
    DLOGI("Setup CRTC %d, Connector %d for %s", token_.crtc_id, token_.conn_id, device_name_);
  }

  PopulateDisplayAttributes();
  PopulateHWPanelInfo();
  UpdateMixerAttributes();

  return error;
}

DisplayError HWHDMIDRM::GetNumDisplayAttributes(uint32_t *count) {
  *count = connector_info_.num_modes;
  if (*count <= 0) {
    return kErrorHardware;
  }

  return kErrorNone;
}

DisplayError HWHDMIDRM::GetActiveConfig(uint32_t *active_config_index) {
  *active_config_index = active_config_index_;
  return kErrorNone;
}

DisplayError HWHDMIDRM::SetDisplayAttributes(uint32_t index) {
  DTRACE_SCOPED();

  if (index >= connector_info_.num_modes) {
    return kErrorNotSupported;
  }

  active_config_index_ = index;

  // TODO(user): fix this hard coding
  frame_rate_ = 60;

  // Get the display attributes for current active config index
  GetDisplayAttributes(active_config_index_, &display_attributes_);
  UpdateMixerAttributes();

  return kErrorNone;
}

DisplayError HWHDMIDRM::GetConfigIndex(uint32_t mode, uint32_t *index) {
  *index = mode;

  return kErrorNone;
}

DisplayError HWHDMIDRM::Validate(HWLayers *hw_layers) {
  HWDeviceDRM::ResetDisplayParams();

  return HWDeviceDRM::Validate(hw_layers);
}

DisplayError HWHDMIDRM::Commit(HWLayers *hw_layers) {
  return HWDeviceDRM::Commit(hw_layers);
}

}  // namespace sdm

