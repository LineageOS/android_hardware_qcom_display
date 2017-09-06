/*
Copyright (c) 2017, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <utils/debug.h>

#include "hw_peripheral_drm.h"

#define __CLASS__ "HWPeripheralDRM"

using sde_drm::DRMDisplayType;
using sde_drm::DRMOps;
using sde_drm::DRMTopology;
using sde_drm::DRMPowerMode;

namespace sdm {

HWPeripheralDRM::HWPeripheralDRM(BufferSyncHandler *buffer_sync_handler,
                                 BufferAllocator *buffer_allocator,
                                 HWInfoInterface *hw_info_intf)
  : HWDeviceDRM(buffer_sync_handler, buffer_allocator, hw_info_intf) {
  disp_type_ = DRMDisplayType::PERIPHERAL;
  device_name_ = "Peripheral Display";
}

DisplayError HWPeripheralDRM::Init() {
  DisplayError ret = HWDeviceDRM::Init();
  if (ret != kErrorNone) {
    DLOGE("Init failed for %s", device_name_);
    return ret;
  }

  if (connector_info_.topology == DRMTopology::UNKNOWN) {
    connector_info_.topology = DRMTopology::DUAL_LM;
  }

  InitializeConfigs();
  PopulateHWPanelInfo();
  UpdateMixerAttributes();

  return kErrorNone;
}

DisplayError HWPeripheralDRM::Validate(HWLayers *hw_layers) {
  // Hijack the first validate to setup pipeline. This is a stopgap solution
  if (first_cycle_) {
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, token_.crtc_id);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::ON);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id,
                              &connector_info_.modes[current_mode_index_]);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
    if (drm_atomic_intf_->Commit(true /* synchronous */, false /* retain pipes*/)) {
      DLOGE("Setting up CRTC %d, Connector %d for %s failed",
            token_.crtc_id, token_.conn_id, device_name_);
      return kErrorResources;
    }
    // Reload connector info for updated info after 1st commit
    drm_mgr_intf_->GetConnectorInfo(token_.conn_id, &connector_info_);
    PopulateDisplayAttributes(current_mode_index_);
    UpdatePanelSplitInfo();
    first_cycle_ = false;
  }

  return HWDeviceDRM::Validate(hw_layers);
}

DisplayError HWPeripheralDRM::PowerOn() {
  if (first_cycle_) {
    return kErrorNone;
  }

  return HWDeviceDRM::PowerOn();
}

}  // namespace sdm
