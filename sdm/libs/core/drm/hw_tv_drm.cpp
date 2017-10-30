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

#include "hw_tv_drm.h"
#include <utils/debug.h>
#include <utils/sys.h>
#include <utils/formats.h>
#include <drm_lib_loader.h>
#include <drm_master.h>
#include <drm_res_mgr.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <map>
#include <utility>


#define __CLASS__ "HWTVDRM"

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
using sde_drm::DRMPowerMode;

namespace sdm {

HWTVDRM::HWTVDRM(BufferSyncHandler *buffer_sync_handler, BufferAllocator *buffer_allocator,
                     HWInfoInterface *hw_info_intf)
  : HWDeviceDRM(buffer_sync_handler, buffer_allocator, hw_info_intf) {
  disp_type_ = DRMDisplayType::TV;
  device_name_ = "TV Display Device";
}

DisplayError HWTVDRM::Init() {
  DisplayError error = HWDeviceDRM::Init();
  if (error != kErrorNone) {
    DLOGE("Init failed for %s", device_name_);
    return error;
  }
  drm_mgr_intf_->GetConnectorInfo(token_.conn_id, &connector_info_);

  InitializeConfigs();

  return error;
}

DisplayError HWTVDRM::SetDisplayAttributes(uint32_t index) {
  if (index >= connector_info_.modes.size()) {
    DLOGE("Invalid mode index %d mode size %d", index, UINT32(connector_info_.modes.size()));
    return kErrorNotSupported;
  }

  if (first_cycle_) {
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, token_.crtc_id);
  }

  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &connector_info_.modes[index]);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);

  // Commit to setup pipeline with mode, which then tells us the topology etc
  if (drm_atomic_intf_->Commit(true /* synchronous */, false /* retain_planes*/)) {
    DLOGE("Setting up CRTC %d, Connector %d for %s failed", token_.crtc_id,
          token_.conn_id, device_name_);
    return kErrorResources;
  }

  DLOGI("Setup CRTC %d, Connector %d for %s", token_.crtc_id, token_.conn_id, device_name_);
  first_cycle_ = false;

  // Reload connector info for updated info after 1st commit
  drm_mgr_intf_->GetConnectorInfo(token_.conn_id, &connector_info_);
  if (index >= connector_info_.modes.size()) {
    DLOGE("Invalid mode index %d mode size %d", index, UINT32(connector_info_.modes.size()));
    return kErrorNotSupported;
  }

  current_mode_index_ = index;
  PopulateDisplayAttributes(index);
  PopulateHWPanelInfo();
  UpdateMixerAttributes();

  return kErrorNone;
}

DisplayError HWTVDRM::GetConfigIndex(char *mode, uint32_t *index) {
  uint32_t width = 0, height = 0, fps = 0, format = 0;
  std::string str(mode);

  // mode should be in width:height:fps:format
  // TODO(user): it is not fully robust, User needs to provide in above format only
  if (str.length() != 0) {
    width = UINT32(stoi(str));
    height = UINT32(stoi(str.substr(str.find(':') + 1)));
    std::string str3 = str.substr(str.find(':') + 1);
    fps = UINT32(stoi(str3.substr(str3.find(':')  + 1)));
    std::string str4 = str3.substr(str3.find(':') + 1);
    format = UINT32(stoi(str4.substr(str4.find(':') + 1)));
  }

  for (size_t idex = 0; idex < connector_info_.modes.size(); idex ++) {
    if ((height == connector_info_.modes[idex].vdisplay) &&
        (width == connector_info_.modes[idex].hdisplay) &&
        (fps == connector_info_.modes[idex].vrefresh)) {
      if ((format >> 1) & (connector_info_.modes[idex].flags >> kBitYUV)) {
        *index = UINT32(idex);
        break;
      }

      if (format & (connector_info_.modes[idex].flags >> kBitRGB)) {
        *index = UINT32(idex);
        break;
      }
    }
  }

  return kErrorNone;
}

/* overriding display state funcs to have special or NO OP implementation for TVs */
DisplayError HWTVDRM::Deinit() {
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 0);

  return HWDeviceDRM::Deinit();
}

DisplayError HWTVDRM::PowerOff() {
  DTRACE_SCOPED();

  int ret = drm_atomic_intf_->Commit(true /* synchronous */, false /* retain_planes*/);
  if (ret) {
    DLOGE("%s failed with error %d", __FUNCTION__, ret);
    return kErrorHardware;
  }

  return kErrorNone;
}

DisplayError HWTVDRM::Doze() {
  return kErrorNone;
}

DisplayError HWTVDRM::DozeSuspend() {
  return kErrorNone;
}

DisplayError HWTVDRM::Standby() {
  return kErrorNone;
}

}  // namespace sdm

