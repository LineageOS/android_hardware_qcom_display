/*
*Copyright (c) 2020, The Linux Foundation. All rights reserved.
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
*
*THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
*WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
*MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
*ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
*BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
*BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
*WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
*OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
*IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <inttypes.h>
#include <log/log.h>

#include "ipc_impl.h"
#include <utils/sys.h>

#define __CLASS__ "IPCImpl"

namespace sdm {

DynLib IPCImpl::qrtr_client_lib_ = {};
CreateQrtrClientIntf IPCImpl::create_qrtr_client_intf_ = nullptr;
DestroyQrtrClientIntf IPCImpl::destroy_qrtr_client_intf_ = nullptr;
QRTRClientInterface *IPCImpl::qrtr_client_intf_ = nullptr;

int IPCImpl::Init() {
  // Try to load extension library & get handle to its interface.
  if (qrtr_client_lib_.Open(QRTR_CLIENT_LIB_NAME)) {
    if (!qrtr_client_lib_.Sym(CREATE_QRTR_CLIENT_INTERFACE_NAME,
                            reinterpret_cast<void **>(&create_qrtr_client_intf_)) ||
        !qrtr_client_lib_.Sym(DESTROY_QRTR_CLIENT_INTERFACE_NAME,
                            reinterpret_cast<void **>(&destroy_qrtr_client_intf_))) {
      DLOGW("Unable to load symbols, error = %s", qrtr_client_lib_.Error());
    }
    if (!create_qrtr_client_intf_ || !destroy_qrtr_client_intf_) {
      return -1;
    }

    if (create_qrtr_client_intf_) {
      QRTRConfig qrtr_config = {};
      qrtr_config.server_id = SDM_COMP_SERVICE_ID;
      qrtr_config.server_version = SDM_COMP_SERVICE_VERSION;
      qrtr_config.server_instance = SDM_COMP_SERVICE_INSTANCE;
      int error = create_qrtr_client_intf_(qrtr_config, this, &qrtr_client_intf_);
      if (error != 0) {
        DLOGW("Unable to create interface");
      }
    }
  } else {
    DLOGW("Unable to load = %s, error = %s", QRTR_CLIENT_LIB_NAME, qrtr_client_lib_.Error());
  }
  return 0;
}

int IPCImpl::Deinit() {
  if (destroy_qrtr_client_intf_) {
    return destroy_qrtr_client_intf_(qrtr_client_intf_);
  }
  return 0;
}

int IPCImpl::SetParameter(IPCParams param, const GenericPayload &in) {
  int ret = 0;
  switch(param) {
  case kIpcParamSetBacklight: {
    if (qrtr_client_intf_) {
      IPCBacklightParams *backlight_params = nullptr;
      uint32_t sz = 0;
      Command cmd = {};
      if ((ret = in.GetPayload(backlight_params, &sz))) {
        DLOGE("Failed to get input payload error = %d", ret);
        return ret;
      }
      CmdSetBacklight &cmd_bl = cmd.cmd_set_backlight;
      cmd.id = kCmdSetBacklight;
      cmd_bl.brightness = backlight_params->brightness;
      cmd_bl.disp_type = backlight_params->is_primary ? kDisplayTypePrimary :
                         kDisplayTypeSecondary1;
      DLOGI("Send brightness level %f, disp_type %d to SVM", cmd_bl.brightness, cmd_bl.disp_type);
      return qrtr_client_intf_->SendCommand(cmd);
    }
  } break;
  case kIpcParamSetDisplayConfigs: {
    if (qrtr_client_intf_) {
      IPCDisplayConfigParams *disp_configs = nullptr;
      uint32_t sz = 0;
      Command cmd = {};
      if ((ret = in.GetPayload(disp_configs, &sz))) {
        DLOGE("Failed to get input payload error = %d", ret);
        return ret;
      }
      CmdSetDisplayConfigs &cmd_disp_configs = cmd.cmd_set_disp_configs;
      cmd.id = kCmdSetDisplayConfig;
      cmd_disp_configs.x_pixels= disp_configs->x_pixels;
      cmd_disp_configs.y_pixels= disp_configs->y_pixels;
      cmd_disp_configs.fps= disp_configs->fps;
      cmd_disp_configs.config_idx= disp_configs->config_idx;
      cmd_disp_configs.smart_panel= disp_configs->smart_panel;
      cmd_disp_configs.disp_type = disp_configs->is_primary ? kDisplayTypePrimary :
                         kDisplayTypeSecondary1;
      DLOGI("Send display configs: WxH %dx%d, fps %d, config_idx %d, %s panel, disp_type %d to SVM",
             cmd_disp_configs.x_pixels, cmd_disp_configs.y_pixels, cmd_disp_configs.fps,
             cmd_disp_configs.config_idx, cmd_disp_configs.smart_panel ? "cmdmode" : "videomode",
             cmd_disp_configs.disp_type);
      return qrtr_client_intf_->SendCommand(cmd);
    }
  } break;
  default:
    break;
  }
  return 0;
}

int IPCImpl::GetParameter(IPCParams param, GenericPayload *out) {
    (void)param;
    (void)out;
    DLOGE("GetParameter on param %d is not supported", param);
    return -ENOTSUP;
}

int IPCImpl::ProcessOps(IPCOps op, const GenericPayload &in, GenericPayload *out) {
  (void)op;
  (void)in;
  (void)out;
  DLOGE("ProcessOps on op %d is not supported", op);
  return -ENOTSUP;
}

int IPCImpl::OnResponse(Response *rsp) {
  switch(rsp->id) {
  case kCmdSetBacklight: {
    if (rsp->status != 0) {
      DLOGW("Response for set backlight level failed with status %d", rsp->status);
      return rsp->status;
    }
    DLOGI("Response for set backlight level received successfully");
  } break;
  default:
    break;
  }
  return 0;
}

void IPCImpl::OnServerReady() {
  DLOGI("LE server is ready");
}

void IPCImpl::OnServerExit() {
  DLOGI("LE server is exited");
}


}  // namespace sdm
