/*
* Copyright (c) 2014 - 2016, 2018, 2020, 2021 The Linux Foundation. All rights reserved.
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

/*
* Changes from Qualcomm Innovation Center are provided under the following license:
*
* Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the
* disclaimer below) provided that the following conditions are met:
*
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*
*    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __CORE_IMPL_H__
#define __CORE_IMPL_H__

#include <core/core_interface.h>
#include <private/extension_interface.h>
#include <private/color_interface.h>
#include <private/panel_feature_factory_intf.h>
#include <private/utils_factory_intf.h>
#include <utils/locker.h>
#include <utils/sys.h>

#include <memory>
#include <vector>
#include <utility>

#include "hw_interface.h"
#include "comp_manager.h"

#define SET_REVISION(major, minor) ((major << 8) | minor)
#define GET_PANEL_FEATURE_FACTORY "GetPanelFeatureFactoryIntf"

namespace sdm {

typedef PanelFeatureFactoryIntf* (*GetPanelFeatureFactory)();
typedef UtilsFactoryIntf* (*GetUtilsFactory)();

class CoreIPCVmCallbackImpl : public IPCVmCallbackIntf {
 public:
  CoreIPCVmCallbackImpl(std::shared_ptr<IPCIntf> ipc_intf,
                        std::shared_ptr<DemuraParserManagerIntf> pm_intf_,
                        HWInfoInterface *hw_info_intf);
  void Init();
  void OnServerReady();
  void OnServerExit();
  void Deinit();
  static void OnServerReadyThread(CoreIPCVmCallbackImpl *obj);
  virtual ~CoreIPCVmCallbackImpl() {}

 private:
  int SendProperties();
  int ExportDemuraCalibBuffer();
  int SendPanelBootParams();

  IPCExportBufOutParams export_buf_out_params_ = {};
  int cb_hnd_out_ = 0;
  std::shared_ptr<IPCIntf> ipc_intf_ = nullptr;
  std::shared_ptr<DemuraParserManagerIntf> pm_intf_ = nullptr;
  HWInfoInterface *hw_info_intf_ = nullptr;
  bool server_ready_ = false;
};

class CoreImpl : public CoreInterface {
 public:
  // This class implements display core interface revision 1.0.
  static const uint16_t kRevision = SET_REVISION(1, 0);
  CoreImpl(BufferAllocator *buffer_allocator, SocketHandler *socket_handler,
           std::shared_ptr<IPCIntf> ipc_intf);
  virtual ~CoreImpl() { }

  // This method returns the interface revision for the current display core object.
  // Future revisions will override this method and return the appropriate revision upon query.
  virtual uint16_t GetRevision() { return kRevision; }
  virtual DisplayError Init();
  virtual DisplayError Deinit();

  // Methods from core interface
  virtual DisplayError ReserveDisplay(DisplayType type);
  virtual DisplayError CreateDisplay(DisplayType type, DisplayEventHandler *event_handler,
                                     DisplayInterface **intf);
  virtual DisplayError CreateDisplay(int32_t display_id, DisplayEventHandler *event_handler,
                                     DisplayInterface **intf);
  virtual DisplayError DestroyDisplay(DisplayInterface *intf);
  virtual DisplayError SetMaxBandwidthMode(HWBwModes mode);
  virtual DisplayError GetFirstDisplayInterfaceType(HWDisplayInterfaceInfo *hw_disp_info);
  virtual DisplayError GetDisplaysStatus(HWDisplaysInfo *hw_displays_info);
  virtual DisplayError GetMaxDisplaysSupported(DisplayType type, int32_t *max_displays);
  virtual bool IsRotatorSupportedFormat(LayerBufferFormat format);
  virtual DisplayError ReserveDemuraResources();

 protected:
  void InitializeSDMUtils();
  void ReleaseDemuraResources();
  void OverRideDemuraPanelIds(std::vector<uint64_t> *panel_ids);

  Locker locker_;
  BufferAllocator *buffer_allocator_ = NULL;
  HWResourceInfo hw_resource_;
  CompManager comp_mgr_;
  HWInfoInterface *hw_info_intf_ = NULL;
  DynLib extension_lib_;
  ExtensionInterface *extension_intf_ = NULL;
  CreateExtensionInterface create_extension_intf_ = NULL;
  DestroyExtensionInterface destroy_extension_intf_ = NULL;
  PanelFeatureFactoryIntf *panel_feature_factory_intf_ = NULL;
  UtilsFactoryIntf *sdm_utils_factory_intf_ = NULL;
  SocketHandler *socket_handler_ = NULL;
  HWDisplaysInfo hw_displays_info_ = {};
  std::shared_ptr<IPCIntf> ipc_intf_ = nullptr;
  CoreIPCVmCallbackImpl* vm_cb_intf_ = nullptr;
  std::vector<uint64_t> *panel_ids_;
  std::shared_ptr<DemuraParserManagerIntf> pm_intf_ = nullptr;
  bool reserve_done_  = false;
  char *raw_mapped_buffer_ = nullptr;
  std::vector<uint32_t> demura_display_ids_;
};

}  // namespace sdm

#endif  // __CORE_IMPL_H__

