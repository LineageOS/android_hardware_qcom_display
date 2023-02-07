/*
* Copyright (c) 2014 - 2016, 2018, 2020-2021 The Linux Foundation. All rights reserved.
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
* Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <dlfcn.h>
#include <signal.h>
#include <malloc.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/locker.h>
#include <utils/utils.h>
#include <sys/mman.h>
#include <map>
#include <vector>
#include <thread>

#include "color_manager.h"
#include "core_impl.h"
#include "display_builtin.h"
#include "display_pluggable.h"
#include "display_virtual.h"
#include "display_null.h"
#include "hw_info_default.h"

#define __CLASS__ "CoreImpl"

namespace sdm {

CoreImpl::CoreImpl(BufferAllocator *buffer_allocator,
                   SocketHandler *socket_handler, std::shared_ptr<IPCIntf> ipc_intf)
  : buffer_allocator_(buffer_allocator), socket_handler_(socket_handler), ipc_intf_(ipc_intf) {
}

DisplayError CoreImpl::Init() {
  SCOPE_LOCK(locker_);
  DisplayError error = kErrorNone;

  // Try to load extension library & get handle to its interface.
  if (extension_lib_.Open(EXTENSION_LIBRARY_NAME)) {
    if (!extension_lib_.Sym(CREATE_EXTENSION_INTERFACE_NAME,
                            reinterpret_cast<void **>(&create_extension_intf_)) ||
        !extension_lib_.Sym(DESTROY_EXTENSION_INTERFACE_NAME,
                            reinterpret_cast<void **>(&destroy_extension_intf_))) {
      DLOGE("Unable to load symbols, error = %s", extension_lib_.Error());
      return kErrorUndefined;
    }

    error = create_extension_intf_(EXTENSION_VERSION_TAG, &extension_intf_);
    if (error != kErrorNone) {
      DLOGE("Unable to create interface");
      return error;
    }
  } else {
#ifdef TRUSTED_VM
    // Any library linked to libsdmextension is not present for LE, LE wont be able to load the
    // libsdmextension library due to undefined reference. To avoid it mark it as fatal on LE
    DLOGE("Unable to load = %s, error = %s", EXTENSION_LIBRARY_NAME, extension_lib_.Error());
#else
    DLOGW("Unable to load = %s, error = %s", EXTENSION_LIBRARY_NAME, extension_lib_.Error());
#endif
  }

  int value = 0;
  Debug::Get()->GetProperty(ENABLE_NULL_DISPLAY_PROP, &value);
  enable_null_display_ = (value == 1);
  DLOGI("property: enable_null_display_ = %d", enable_null_display_);
  if (enable_null_display_) {
    hw_info_intf_ = new HWInfoDefault();
    return kErrorNone;
  }

  error = HWInfoInterface::Create(&hw_info_intf_);
  if (error != kErrorNone) {
    DisplayError err = HandleNullDisplay();

    if ((err != kErrorNone) || !enable_null_display_) {
      goto CleanupOnError;
    }
    hw_info_intf_ = new HWInfoDefault();
    return kErrorNone;
  }

  error = hw_info_intf_->GetHWResourceInfo(&hw_resource_);
  if (error != kErrorNone) {
    goto CleanupOnError;
  }

  InitializeSDMUtils();

  error = comp_mgr_.Init(hw_resource_, extension_intf_, buffer_allocator_, socket_handler_);

  if (error != kErrorNone) {
    goto CleanupOnError;
  }

  enable_null_display_ = !comp_mgr_.IsDisplayHWAvailable();
  if (enable_null_display_) {
    if (hw_info_intf_) {
      HWInfoInterface::Destroy(hw_info_intf_);
    }
    hw_info_intf_ = new HWInfoDefault();
    return kErrorNone;
  }

  error = ColorManagerProxy::Init(hw_resource_);
  // if failed, doesn't affect display core functionalities.
  if (error != kErrorNone) {
    DLOGW("Unable creating color manager and continue without it.");
  }

  // Populate hw_displays_info_ once.
  error = hw_info_intf_->GetDisplaysStatus(&hw_displays_info_);
  if (error != kErrorNone) {
    DLOGW("Failed getting displays status. Error = %d", error);
  }

  // Must only call after GetDisplaysStatus
#ifndef TRUSTED_VM
  if (ReserveDemuraResources() != kErrorNone) {
    comp_mgr_.SetDemuraStatus(false);
  }
  vm_cb_intf_ = new CoreIPCVmCallbackImpl(ipc_intf_, pm_intf_, hw_info_intf_);
  if (vm_cb_intf_) {
    vm_cb_intf_->Init();
  }
#endif
  signal(SIGPIPE, SIG_IGN);
  return kErrorNone;

CleanupOnError:
  if (hw_info_intf_) {
    HWInfoInterface::Destroy(hw_info_intf_);
  }

  return error;
}

void CoreImpl::ReleaseDemuraResources() {
  GenericPayload dummy;
  if (pm_intf_) {
    int ret = pm_intf_->SetParameter(kDemuraParserManagerParamReleaseParsers, dummy);
    if (ret < 0) {
        DLOGW("Failed to release demura parsers");
    }
  }

  for (auto &it : demura_display_ids_)
    comp_mgr_.FreeDemuraFetchResources(it);
}

DisplayError CoreImpl::Deinit() {
  SCOPE_LOCK(locker_);
  if (vm_cb_intf_) {
    vm_cb_intf_->Deinit();
    delete vm_cb_intf_;
  }

  ReleaseDemuraResources();
  ColorManagerProxy::Deinit();

  comp_mgr_.Deinit();

  if (enable_null_display_) {
    delete static_cast<HWInfoDefault *>(hw_info_intf_);
    hw_info_intf_ = nullptr;
  } else {
    HWInfoInterface::Destroy(hw_info_intf_);
  }
#ifdef TRUSTED_VM
  // release free memory from the heap, needed for Trusted_VM due to the limited
  // carveout size
  malloc_trim(0);
#endif
  return kErrorNone;
}

DisplayError CoreImpl::ReserveDisplay(DisplayType type) {
  SCOPE_LOCK(locker_);

  return comp_mgr_.ReserveDisplay(type);
}

DisplayError CoreImpl::CreateDisplay(DisplayType type, DisplayEventHandler *event_handler,
                                     DisplayInterface **intf) {
  SCOPE_LOCK(locker_);

  if (!event_handler || !intf) {
    return kErrorParameters;
  }

  if (enable_null_display_) {
    return CreateNullDisplayLocked(intf);
  }

  DisplayBase *display_base = NULL;

  switch (type) {
    case kBuiltIn:
      display_base = new DisplayBuiltIn(event_handler, hw_info_intf_, buffer_allocator_,
                                        &comp_mgr_, ipc_intf_);
      break;
    case kPluggable:
      display_base = new DisplayPluggable(event_handler, hw_info_intf_, buffer_allocator_,
                                          &comp_mgr_);
      break;
    case kVirtual:
      display_base = new DisplayVirtual(event_handler, hw_info_intf_, buffer_allocator_,
                                        &comp_mgr_);
      break;
    default:
      DLOGE("Spurious display type %d", type);
      return kErrorParameters;
  }

  if (!display_base) {
    return kErrorMemory;
  }

  DisplayError error = display_base->Init();
  if (error != kErrorNone) {
    delete display_base;
    return error;
  }

  *intf = display_base;
  return kErrorNone;
}

DisplayError CoreImpl::CreateDisplay(int32_t display_id, DisplayEventHandler *event_handler,
                                     DisplayInterface **intf) {
  SCOPE_LOCK(locker_);

  if (!event_handler || !intf) {
    return kErrorParameters;
  }

  if (enable_null_display_) {
    return CreateNullDisplayLocked(intf);
  }

  auto iter = hw_displays_info_.find(display_id);

  if (iter == hw_displays_info_.end()) {
    DLOGE("Spurious display id %d", display_id);
    return kErrorParameters;
  }

  DisplayBase *display_base = NULL;
  DisplayType display_type = iter->second.display_type;

  switch (display_type) {
    case kBuiltIn:
      display_base = new DisplayBuiltIn(display_id, event_handler, hw_info_intf_,
                                        buffer_allocator_, &comp_mgr_, ipc_intf_);
      break;
    case kPluggable:
      display_base = new DisplayPluggable(display_id, event_handler, hw_info_intf_,
                                          buffer_allocator_, &comp_mgr_);
      break;
    case kVirtual:
      display_base = new DisplayVirtual(display_id, event_handler, hw_info_intf_,
                                        buffer_allocator_, &comp_mgr_);
      break;
    default:
      DLOGE("Spurious display type %d", display_type);
      return kErrorParameters;
  }

  if (!display_base) {
    return kErrorMemory;
  }

  DisplayError error = display_base->Init();
  if (error != kErrorNone) {
    delete display_base;
    return error;
  }

  *intf = display_base;

  return kErrorNone;
}

DisplayError CoreImpl::CreateNullDisplay(DisplayInterface **intf) {
  SCOPE_LOCK(locker_);

  if (!intf) {
    return kErrorParameters;
  }

  return CreateNullDisplayLocked(intf);
}

DisplayError CoreImpl::CreateNullDisplayLocked(DisplayInterface **intf) {
  DisplayNull *display_null = new DisplayNull();

  if (!display_null) {
    return kErrorMemory;
  }

  DisplayError error = display_null->Init();
  if (error != kErrorNone) {
    delete display_null;
    return error;
  }

  *intf = display_null;

  return kErrorNone;
}

DisplayError CoreImpl::DestroyDisplay(DisplayInterface *intf) {
  SCOPE_LOCK(locker_);

  if (!intf) {
    return kErrorParameters;
  }

  if (enable_null_display_) {
    delete static_cast<DisplayNull *>(intf);
    return kErrorNone;
  }

  DisplayBase *display_base = static_cast<DisplayBase *>(intf);
  display_base->Deinit();
  delete display_base;

  return kErrorNone;
}

DisplayError CoreImpl::DestroyNullDisplay(DisplayInterface *intf) {
  SCOPE_LOCK(locker_);

  if (!intf) {
    return kErrorParameters;
  }

  delete static_cast<DisplayNull *>(intf);

  return kErrorNone;
}

DisplayError CoreImpl::HandleNullDisplay() {
  // Initializing comp_mgr with default hw resource
  DisplayError error = comp_mgr_.Init(hw_resource_, extension_intf_, buffer_allocator_,
                                      socket_handler_);
  if (error != kErrorNone) {
    DLOGW("comp manager initialization failed");
    return error;
  }
  DLOGI("comp manager successfully initialized with default hw resources");
  enable_null_display_ = !comp_mgr_.IsDisplayHWAvailable();
  return kErrorNone;
}

DisplayError CoreImpl::SetMaxBandwidthMode(HWBwModes mode) {
  SCOPE_LOCK(locker_);

  return comp_mgr_.SetMaxBandwidthMode(mode);
}

DisplayError CoreImpl::GetFirstDisplayInterfaceType(HWDisplayInterfaceInfo *hw_disp_info) {
  SCOPE_LOCK(locker_);
  return hw_info_intf_->GetFirstDisplayInterfaceType(hw_disp_info);
}

DisplayError CoreImpl::GetDisplaysStatus(HWDisplaysInfo *hw_displays_info) {
  SCOPE_LOCK(locker_);
  DisplayError error = kErrorNone;
  error = hw_info_intf_->GetDisplaysStatus(hw_displays_info);
  if (kErrorNone == error) {
    // Needed for error-checking in CreateDisplay(int32_t display_id, ...) and getting display-type.
    hw_displays_info_ = *hw_displays_info;
  }
  return error;
}

DisplayError CoreImpl::GetMaxDisplaysSupported(DisplayType type, int32_t *max_displays) {
  SCOPE_LOCK(locker_);
  return hw_info_intf_->GetMaxDisplaysSupported(type, max_displays);
}

bool CoreImpl::IsRotatorSupportedFormat(LayerBufferFormat format) {
  SCOPE_LOCK(locker_);
  return comp_mgr_.IsRotatorSupportedFormat(format);
}

void CoreImpl::InitializeSDMUtils() {
  GetUtilsFactory get_sdm_utils_f_ptr = nullptr;
  if (!extension_lib_.Sym(GET_SDM_UTILS_FACTORY,
                          reinterpret_cast<void **>(&get_sdm_utils_f_ptr))) {
    DLOGE("Unable to load symbols, error = %s", extension_lib_.Error());
    return;
  }

  sdm_utils_factory_intf_ = get_sdm_utils_f_ptr();
  sdm_utils_factory_intf_->CreateSDMPropUtils(hw_resource_);
}

void CoreImpl::OverRideDemuraPanelIds(std::vector<uint64_t> *panel_ids) {
  uint64_t panel_id_prim = 0, panel_id_sec = 0;
  int panel_id_w = 0;
  uint32_t count;

  if (!panel_ids)
    return;

  panel_id_w = 0;
  // primary panel id
  Debug::Get()->GetProperty(DEMURA_PRIMARY_PANEL_OVERRIDE_LOW, &panel_id_w);
  panel_id_prim = static_cast<uint32_t>(panel_id_w);
  Debug::Get()->GetProperty(DEMURA_PRIMARY_PANEL_OVERRIDE_HIGH, &panel_id_w);
  panel_id_prim |=  ((static_cast<uint64_t>(panel_id_w)) << 32);

  panel_id_w = 0;
  // secondary panel id
  Debug::Get()->GetProperty(DEMURA_SECONDARY_PANEL_OVERRIDE_LOW, &panel_id_w);
  panel_id_sec = static_cast<uint32_t>(panel_id_w);
  Debug::Get()->GetProperty(DEMURA_SECONDARY_PANEL_OVERRIDE_HIGH, &panel_id_w);
  panel_id_sec |=  ((static_cast<uint64_t>(panel_id_w)) << 32);

  count = panel_ids->size();

  if (count >= 2 && (!panel_id_prim || !panel_id_sec)) {
    DLOGI("skip panel override count 2 panel_id_prim %lx panel_id_sec %lx\n",
      panel_id_prim, panel_id_sec);
    return;
  }

  if (count == 1 && !panel_id_prim && !panel_id_sec) {
    DLOGI("skip panel override count 1 panel_id_prim %lx panel_id_sec %lx\n",
      panel_id_prim, panel_id_sec);
    return;
  }

  panel_ids->clear();
  if (panel_id_prim) {
    DLOGI("override primary panel id %lx\n", panel_id_prim);
    panel_ids->push_back(panel_id_prim);
  }
  if (panel_id_sec) {
    DLOGI("override secondary panel id %lx\n", panel_id_sec);
    panel_ids->push_back(panel_id_sec);
  }
}

DisplayError CoreImpl::ReserveDemuraResources() {
  DisplayError err = kErrorNone;
  int enable = 0;
  if (reserve_done_)
    return kErrorNone;

  Debug::Get()->GetProperty(ENABLE_DEMURA, &enable);
  if (!enable) {
    comp_mgr_.SetDemuraStatus(false);
    DLOGI("Demura is disabled");
    return kErrorNone;
  } else {
    DLOGI("Demura is enabled");
    comp_mgr_.SetDemuraStatus(true);
  }
  std::map<uint32_t, uint8_t> required_demura_fetch_cnt;  // display_id, count
  if ((err = hw_info_intf_->GetRequiredDemuraFetchResourceCount(&required_demura_fetch_cnt)) !=
      kErrorNone) {
    DLOGE("Unable to get required demura pipes count");
    return err;
  }

  if (!required_demura_fetch_cnt.size()) {
    DLOGW("Demura is enabled but no panels support it. Disabling..");
    comp_mgr_.SetDemuraStatus(false);
    return kErrorNone;
  }

  int primary_off = 0;
  int secondary_off = 0;
  Debug::Get()->GetProperty(DISABLE_DEMURA_PRIMARY, &primary_off);
  Debug::Get()->GetProperty(DISABLE_DEMURA_SECONDARY, &secondary_off);

  int available_blocks = hw_resource_.demura_count;
  for (auto r = required_demura_fetch_cnt.begin(); r != required_demura_fetch_cnt.end();) {
    HWDisplayInfo &info = hw_displays_info_[r->first];
    DLOGI("[%d] is_primary = %d, p_off = %d, s_off = %d", r->first, info.is_primary, primary_off,
          secondary_off);
    if (info.is_primary && primary_off) {
      r = required_demura_fetch_cnt.erase(r);
      continue;
    } else if (!info.is_primary && secondary_off) {
      r = required_demura_fetch_cnt.erase(r);
      continue;
    }

    available_blocks -= r->second;
    if (available_blocks < 0) {
      DLOGE("Not enough Demura blocks (%u)", hw_resource_.demura_count);
      return kErrorResources;
    }
    ++r;
  }

  std::map<uint32_t, uint8_t> fetch_resource_cnt;  // display id, count
  comp_mgr_.GetDemuraFetchResourceCount(&fetch_resource_cnt);
  for (auto &req : required_demura_fetch_cnt) {
    uint8_t cnt = 0;
    auto it = fetch_resource_cnt.find(req.first);
    if (it != fetch_resource_cnt.end()) {
      cnt = it->second;
    }
    uint8_t req_cnt = req.second;
    if (req_cnt != cnt && cnt != 0) {
      DLOGE("Cont Splash only allocated %u pipes for Demura, but %u is needed", cnt, req_cnt);
      return kErrorDriverData;
    }
    if (req_cnt != 0 && cnt == 0) {
      DLOGI("[%u] Needs Demura resources %u", req.first, req_cnt);
      // Reserving demura resources requires knowledge of which rect to reserve when the req_cnt
      // is 1. As the HW pipeline for any display is not known yet, we shall assume primary display
      // takes 0 and non-primary takes 1. When req_cnt > 1, pass in -1
      int8_t preferred_rect = -1;
      if (req_cnt == 1) {
        HWDisplayInfo &info = hw_displays_info_[req.first];
        preferred_rect = info.is_primary ? 0 : 1;
        DLOGI("[%u] is single LM. Requesting Demura rect %d", req.first, preferred_rect);
      }
      if ((err = comp_mgr_.ReserveDemuraFetchResources(req.first, preferred_rect)) !=
          kErrorNone) {
        DLOGE("Failed to reserve resources error = %d", err);
        return err;
      }
      demura_display_ids_.push_back(req.first);
    }
  }

  GetPanelFeatureFactory get_factory_f_ptr = nullptr;
  if (!extension_lib_.Sym(GET_PANEL_FEATURE_FACTORY,
                          reinterpret_cast<void **>(&get_factory_f_ptr))) {
    DLOGE("Unable to load symbols, error = %s", extension_lib_.Error());
    return kErrorUndefined;
  }

  panel_feature_factory_intf_ = get_factory_f_ptr();
  pm_intf_ = panel_feature_factory_intf_->CreateDemuraParserManager(ipc_intf_,
                                buffer_allocator_);
  if (!pm_intf_) {
    DLOGE("Failed to get Parser Manager intf");
    return kErrorResources;
  }
  if (pm_intf_->Init() != 0) {
    DLOGE("Failed to init Parser Manager intf");
    return kErrorResources;
  }

  std::vector<uint64_t> *panel_ids;
  GenericPayload in;
  int ret = in.CreatePayload<std::vector<uint64_t>>(panel_ids);
  if (ret) {
    DLOGE("Failed to create payload for panel ids, error = %d", ret);
    return kErrorResources;
  }

  if ((err = hw_info_intf_->GetDemuraPanelIds(panel_ids)) != kErrorNone) {
    DLOGE("Unable to get demura panel ids");
    return err;
  }

  for (auto &id : *panel_ids) {
    DLOGI("Detected panel_id = %" PRIu64 " (0x%" PRIx64 ")", id, id);
  }
  OverRideDemuraPanelIds(panel_ids);


  if ((ret = pm_intf_->SetParameter(kDemuraParserManagerParamPanelIds, in))) {
    DLOGE("Failed to set the panel ids to the parser manager");
    return kErrorResources;
  }

  reserve_done_ = true;
  return err;
}

// LCOV_EXCL_START
CoreIPCVmCallbackImpl::CoreIPCVmCallbackImpl(std::shared_ptr<IPCIntf> ipc_intf,
                                        std::shared_ptr<DemuraParserManagerIntf> pm_intf,
                                        HWInfoInterface *hw_info_intf)
  : ipc_intf_(ipc_intf), pm_intf_(pm_intf), hw_info_intf_(hw_info_intf) {
}

void CoreIPCVmCallbackImpl::Init() {
  if (!ipc_intf_) {
    DLOGW("IPC interface is NULL");
    return;
  }
  GenericPayload in_reg;
  int *cb_hnd_out = nullptr;
  CoreIPCVmCallbackImpl **cb_intf = nullptr;
  int ret = in_reg.CreatePayload<CoreIPCVmCallbackImpl *>(cb_intf);
  if (ret) {
    DLOGE("failed to create the payload for in_reg. Error:%d", ret);
    return;
  }
  *cb_intf = this;
  GenericPayload out_reg;
  ret = out_reg.CreatePayload<int>(cb_hnd_out);
  if (ret) {
    DLOGE("failed to create the payload for out_reg. Error:%d", ret);
    return;
  }
  if ((ret = ipc_intf_->ProcessOps(kIpcOpsRegisterVmCallback, in_reg, &out_reg))) {
    DLOGE("Failed to register vm callback, error = %d", ret);
    return;
  }
  cb_hnd_out_ = *cb_hnd_out;
}

void CoreIPCVmCallbackImpl::Deinit() {
  if (!ipc_intf_) {
    DLOGW("IPC interface is NULL");
    return;
  }
  GenericPayload in_unreg;
  int *cb_hnd_in = nullptr;
  int ret = in_unreg.CreatePayload<int>(cb_hnd_in);
  if (ret) {
    DLOGE("failed to create the payload for in_unreg. Error:%d", ret);
    return;
  }
  *cb_hnd_in = cb_hnd_out_;
  if ((ret = ipc_intf_->ProcessOps(kIpcOpsUnRegisterVmCallback, in_unreg, nullptr))) {
    DLOGE("Failed to unregister vm callback, error = %d", ret);
    return;
  }
  server_ready_ = false;
}

void CoreIPCVmCallbackImpl::OnServerReady() {
  if (server_ready_) {
    DLOGE("Server is ready");
    return;
  }
  server_ready_ = true;
  std::thread(CoreIPCVmCallbackImpl::OnServerReadyThread, this).detach();
}

void CoreIPCVmCallbackImpl::OnServerReadyThread(CoreIPCVmCallbackImpl *obj) {
  if (obj->SendProperties()) {
    DLOGE("Failed to send Properties");
  }
  if (obj->ExportDemuraCalibBuffer()) {
    DLOGE("Failed to export Calibration buffer");
  }
  if (obj->SendPanelBootParams()) {
    DLOGW("Failed to Send SendPanelBootParams");
  }
}

int CoreIPCVmCallbackImpl::SendProperties() {
  if (!ipc_intf_) {
    DLOGW("IPC interface is NULL");
    return -EINVAL;
  }

  GenericPayload in;
  int ret = 0, enable, count = 0;
  IPCSetPropertyParams *prop_configs = nullptr;
  ret = in.CreatePayload<IPCSetPropertyParams>(prop_configs);
  if (ret) {
    DLOGW("failed to create the payload. Error:%d", ret);
    return ret;
  }

  Debug::Get()->GetProperty(ENABLE_DEMURA, &enable);
  if (enable) {
    std::snprintf(prop_configs->props.property_list[count].prop_name,
                  sizeof prop_configs->props.property_list[count].prop_name,
                  "%s", ENABLE_DEMURA);
    std::snprintf(prop_configs->props.property_list[count].value,
                  sizeof prop_configs->props.property_list[count].value,
                  "%d", enable);
    count++;
  }
  Debug::Get()->GetProperty(ENABLE_SPR, &enable);
  if (enable) {
    std::snprintf(prop_configs->props.property_list[count].prop_name,
                  sizeof prop_configs->props.property_list[count].prop_name,
                  "%s", ENABLE_SPR);
    std::snprintf(prop_configs->props.property_list[count].value,
                  sizeof prop_configs->props.property_list[count].value,
                  "%d", enable);
    count++;
  }

  if (count) {
    prop_configs->props.count = count;
    if ((ret = ipc_intf_->SetParameter(kIpcParamProperties, in))) {
      DLOGW("Failed to set properties, error = %d", ret);
      return ret;
    }
    DLOGI("Sent Properties successful");
  }

  return 0;
}

int CoreIPCVmCallbackImpl::SendPanelBootParams() {
  if (!ipc_intf_) {
    DLOGW("IPC interface is NULL");
    return -EINVAL;
  }

  GenericPayload in;
  IPCPanelBootParams *panel_boot_params = nullptr;
  int ret = in.CreatePayload<IPCPanelBootParams>(panel_boot_params);
  if (ret) {
    DLOGW("failed to create the payload for panel boot params Error:%d", ret);
    return ret;
  }

  DisplayError err = hw_info_intf_->GetPanelBootParamString(&panel_boot_params->panel_boot_string);
  if (err != kErrorNone) {
    return -EINVAL;
  }
  if ((ret = ipc_intf_->SetParameter(kIpcParamPanelBoot, in))) {
    DLOGW("Failed to set panel boot params, error = %d", ret);
    return ret;
  }

  return 0;
}

int CoreIPCVmCallbackImpl::ExportDemuraCalibBuffer() {
  int enable = 0;
  Debug::Get()->GetProperty(ENABLE_DEMURA, &enable);
  if (enable && pm_intf_) {
    GenericPayload dummy;
    int ret = 0;
    if ((ret = pm_intf_->SetParameter(kDemuraParserManagerParamExportCalibBuffers, dummy))) {
      DLOGE("Failed to export calibration buffers");
      return ret;
    }
    DLOGI("Export buffer successful");
  }
  return 0;
}

void CoreIPCVmCallbackImpl::OnServerExit() {
  GenericPayload dummy;
  int ret = 0;

  server_ready_ = false;
  if (!pm_intf_)
    return;
  if ((ret = pm_intf_->SetParameter(kDemuraParserManagerParamFreeCalibBuffers, dummy))) {
    DLOGE("Failed to export calibration buffers");
    return;
  }
  DLOGI("Free Export buffers successful");
}
// LCOV_EXCL_STOP

}  // namespace sdm
