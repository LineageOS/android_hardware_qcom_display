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

#ifndef __IPC_IMPL_H__
#define __IPC_IMPL_H__

#include <core/ipc_interface.h>
#include "qrtr_client_interface.h"
#include "vm_interface.h"
#include "utils/sys.h"

namespace sdm {

class IPCImpl: public IPCIntf, QRTRCallbackInterface {
 public:
  virtual ~IPCImpl() {};
  int Init();
  int Deinit();
  int SetParameter(IPCParams param, const GenericPayload &in);
  int GetParameter(IPCParams param, GenericPayload *out);
  int ProcessOps(IPCOps op, const GenericPayload &in, GenericPayload *out);
  void OnServerReady();
  void OnServerExit();
  int OnResponse(Response *rsp);

 private:
  static DynLib qrtr_client_lib_;
  static CreateQrtrClientIntf create_qrtr_client_intf_;
  static DestroyQrtrClientIntf destroy_qrtr_client_intf_;
  static QRTRClientInterface *qrtr_client_intf_;
};
}  // namespace sdm

#endif
