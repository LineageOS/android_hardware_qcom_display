/*
* Copyright (c) 2020, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted
* provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright notice, this list of
*      conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright notice, this list of
*      conditions and the following disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation nor the names of its contributors may be used to
*      endorse or promote products derived from this software without specific prior written
*      permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __IPC_INTERFACE_H__
#define __IPC_INTERFACE_H__

#include "private/generic_intf.h"
#include "private/generic_payload.h"

namespace sdm {

enum IPCParams {
  kIpcParamSetBacklight,            //!< Send backlight params to SVM
  kIpcParamSetDisplayConfigs,       //!< Send display config information to SVM
  kIPCParamMax,
};

enum IPCOps {
  kIPCOpMax
};

struct IPCBacklightParams {
  float brightness = 0.0f;        //!< Specifies the brightness level to be passed to SVM
  bool is_primary = false;        //!< Flag specifies primary/secondary
};

struct IPCDisplayConfigParams {
  uint32_t x_pixels = 0;          //!< Total number of pixels in X-direction on the display panel.
  uint32_t y_pixels = 0;          //!< Total number of pixels in Y-direction on the display panel.
  uint32_t fps = 0;               //!< Frame rate per second.
  int config_idx = -1;            //!< Specifies the config index of the display resolution mode.
  bool is_primary = false;        //!< Flag specifies primary/secondary
  bool smart_panel = false;       //!< If the display config has smart panel.
};

using IPCIntf = sdm::GenericIntf<IPCParams, IPCOps, GenericPayload>;

}  // namespace sdm

#endif  // __IPC_INTERFACE_H__
