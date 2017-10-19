/*
* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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
*    * Neither the name of The Linux Foundation. nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
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

#include <core/buffer_allocator.h>
#include <utils/debug.h>
#include <sync/sync.h>
#include <profiler.h>

#include "hwc_buffer_sync_handler.h"
#include "hwc_session.h"

#define __CLASS__ "HWCSession"

namespace sdm {

using ::android::hardware::Void;

void HWCSession::StartServices() {
  status_t status = IDisplayConfig::registerAsService();
  if (status != OK) {
    DLOGW("Could not register IDisplayConfig as service (%d).", status);
  } else {
    DLOGI("IDisplayConfig service registration completed.");
  }
}

int MapDisplayType(IDisplayConfig::DisplayType dpy) {
  switch (dpy) {
    case IDisplayConfig::DisplayType::DISPLAY_PRIMARY:
      return HWC_DISPLAY_PRIMARY;

    case IDisplayConfig::DisplayType::DISPLAY_EXTERNAL:
      return HWC_DISPLAY_EXTERNAL;

    case IDisplayConfig::DisplayType::DISPLAY_VIRTUAL:
      return HWC_DISPLAY_VIRTUAL;

    default:
      break;
  }

  return -EINVAL;
}

HWCDisplay::DisplayStatus MapExternalStatus(IDisplayConfig::DisplayExternalStatus status) {
  switch (status) {
    case IDisplayConfig::DisplayExternalStatus::EXTERNAL_OFFLINE:
      return HWCDisplay::kDisplayStatusOffline;

    case IDisplayConfig::DisplayExternalStatus::EXTERNAL_ONLINE:
      return HWCDisplay::kDisplayStatusOnline;

    case IDisplayConfig::DisplayExternalStatus::EXTERNAL_PAUSE:
      return HWCDisplay::kDisplayStatusPause;

    case IDisplayConfig::DisplayExternalStatus::EXTERNAL_RESUME:
      return HWCDisplay::kDisplayStatusResume;

    default:
      break;
  }

  return HWCDisplay::kDisplayStatusInvalid;
}

// Methods from ::vendor::hardware::display::config::V1_0::IDisplayConfig follow.
Return<void> HWCSession::isDisplayConnected(IDisplayConfig::DisplayType dpy,
                                            isDisplayConnected_cb _hidl_cb) {
  int32_t error = -EINVAL;
  bool connected = false;

  int disp_id = MapDisplayType(dpy);
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);

  if (disp_id >= 0) {
    connected = hwc_display_[disp_id];
    error = 0;
  }

  _hidl_cb(error, connected);

  return Void();
}

int32_t HWCSession::SetSecondaryDisplayStatus(int disp_id, HWCDisplay::DisplayStatus status) {
  if (disp_id < 0) {
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);
  DLOGI("Display = %d, Status = %d", disp_id, status);

  if (disp_id == HWC_DISPLAY_PRIMARY) {
    DLOGE("Not supported for this display");
  } else if (!hwc_display_[disp_id]) {
    DLOGW("Display is not connected");
  } else {
    return hwc_display_[disp_id]->SetDisplayStatus(status);
  }

  return -EINVAL;
}

Return<int32_t> HWCSession::setSecondayDisplayStatus(IDisplayConfig::DisplayType dpy,
                                                  IDisplayConfig::DisplayExternalStatus status) {
  return SetSecondaryDisplayStatus(MapDisplayType(dpy), MapExternalStatus(status));
}

Return<int32_t> HWCSession::configureDynRefeshRate(IDisplayConfig::DisplayDynRefreshRateOp op,
                                                   uint32_t refreshRate) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);
  HWCDisplay *hwc_display = hwc_display_[HWC_DISPLAY_PRIMARY];

  switch (op) {
    case IDisplayConfig::DisplayDynRefreshRateOp::DISABLE_METADATA_DYN_REFRESH_RATE:
      return hwc_display->Perform(HWCDisplayPrimary::SET_METADATA_DYN_REFRESH_RATE, false);

    case IDisplayConfig::DisplayDynRefreshRateOp::ENABLE_METADATA_DYN_REFRESH_RATE:
      return hwc_display->Perform(HWCDisplayPrimary::SET_METADATA_DYN_REFRESH_RATE, true);

    case IDisplayConfig::DisplayDynRefreshRateOp::SET_BINDER_DYN_REFRESH_RATE:
      return hwc_display->Perform(HWCDisplayPrimary::SET_BINDER_DYN_REFRESH_RATE, refreshRate);

    default:
      DLOGW("Invalid operation %d", op);
      return -EINVAL;
  }

  return 0;
}

int32_t HWCSession::GetConfigCount(int disp_id, uint32_t *count) {
  if (disp_id < 0) {
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);

  if (hwc_display_[disp_id]) {
    return hwc_display_[disp_id]->GetDisplayConfigCount(count);
  }

  return -EINVAL;
}

Return<void> HWCSession::getConfigCount(IDisplayConfig::DisplayType dpy,
                                        getConfigCount_cb _hidl_cb) {
  uint32_t count = 0;
  int32_t error = GetConfigCount(MapDisplayType(dpy), &count);

  _hidl_cb(error, count);

  return Void();
}

int32_t HWCSession::GetActiveConfigIndex(int disp_id, uint32_t *config) {
  if (disp_id < 0) {
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);

  if (hwc_display_[disp_id]) {
    return hwc_display_[disp_id]->GetActiveDisplayConfig(config);
  }

  return -EINVAL;
}

Return<void> HWCSession::getActiveConfig(IDisplayConfig::DisplayType dpy,
                                         getActiveConfig_cb _hidl_cb) {
  uint32_t config = 0;
  int32_t error = GetActiveConfigIndex(MapDisplayType(dpy), &config);

  _hidl_cb(error, config);

  return Void();
}

int32_t HWCSession::SetActiveConfigIndex(int disp_id, uint32_t config) {
  if (disp_id < 0) {
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);
  int32_t error = -EINVAL;
  if (hwc_display_[disp_id]) {
    error = hwc_display_[disp_id]->SetActiveDisplayConfig(config);
    if (!error) {
      Refresh(0);
    }
  }

  return error;
}

Return<int32_t> HWCSession::setActiveConfig(IDisplayConfig::DisplayType dpy, uint32_t config) {
  return SetActiveConfigIndex(MapDisplayType(dpy), config);
}

Return<void> HWCSession::getDisplayAttributes(uint32_t configIndex,
                                              IDisplayConfig::DisplayType dpy,
                                              getDisplayAttributes_cb _hidl_cb) {
  int32_t error = -EINVAL;
  IDisplayConfig::DisplayAttributes display_attributes = {};
  int disp_id = MapDisplayType(dpy);

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);
  if (disp_id >= 0 && hwc_display_[disp_id]) {
    DisplayConfigVariableInfo hwc_display_attributes;
    error = hwc_display_[disp_id]->GetDisplayAttributesForConfig(static_cast<int>(configIndex),
                                                                 &hwc_display_attributes);
    if (!error) {
      display_attributes.vsyncPeriod = hwc_display_attributes.vsync_period_ns;
      display_attributes.xRes = hwc_display_attributes.x_pixels;
      display_attributes.yRes = hwc_display_attributes.y_pixels;
      display_attributes.xDpi = hwc_display_attributes.x_dpi;
      display_attributes.yDpi = hwc_display_attributes.y_dpi;
      display_attributes.panelType = IDisplayConfig::DisplayPortType::DISPLAY_PORT_DEFAULT;
      display_attributes.isYuv = hwc_display_attributes.is_yuv;
    }
  }

  return Void();
}

Return<int32_t> HWCSession::setPanelBrightness(uint32_t level) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);
  int32_t error = -EINVAL;

  if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
    error = hwc_display_[HWC_DISPLAY_PRIMARY]->SetPanelBrightness(static_cast<int>(level));
    if (error) {
      DLOGE("Failed to set the panel brightness = %d. Error = %d", level, error);
    }
  }

  return error;
}

int32_t HWCSession::GetPanelBrightness(int *level) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);
  int32_t error = -EINVAL;

  if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
    error = hwc_display_[HWC_DISPLAY_PRIMARY]->GetPanelBrightness(level);
    if (error) {
      DLOGE("Failed to get the panel brightness. Error = %d", error);
    }
  }

  return error;
}

Return<void> HWCSession::getPanelBrightness(getPanelBrightness_cb _hidl_cb) {
  int level = 0;
  int32_t error = GetPanelBrightness(&level);

  _hidl_cb(error, static_cast<uint32_t>(level));

  return Void();
}

int32_t HWCSession::MinHdcpEncryptionLevelChanged(int disp_id, uint32_t min_enc_level) {
  DLOGI("Display %d", disp_id);

  if (disp_id < 0) {
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);
  if (disp_id != HWC_DISPLAY_EXTERNAL) {
    DLOGE("Not supported for display");
  } else if (!hwc_display_[disp_id]) {
    DLOGW("Display is not connected");
  } else {
    return hwc_display_[disp_id]->OnMinHdcpEncryptionLevelChange(min_enc_level);
  }

  return -EINVAL;
}

Return<int32_t> HWCSession::minHdcpEncryptionLevelChanged(IDisplayConfig::DisplayType dpy,
                                                          uint32_t min_enc_level) {
  return MinHdcpEncryptionLevelChanged(MapDisplayType(dpy), min_enc_level);
}

Return<int32_t> HWCSession::refreshScreen() {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);
  Refresh(HWC_DISPLAY_PRIMARY);

  return 0;
}

int32_t HWCSession::ControlPartialUpdate(int disp_id, bool enable) {
  if (disp_id < 0) {
    return -EINVAL;
  }

  if (disp_id != HWC_DISPLAY_PRIMARY) {
    DLOGW("CONTROL_PARTIAL_UPDATE is not applicable for display = %d", disp_id);
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);
  HWCDisplay *hwc_display = hwc_display_[HWC_DISPLAY_PRIMARY];
  if (!hwc_display) {
    DLOGE("primary display object is not instantiated");
    return -EINVAL;
  }

  uint32_t pending = 0;
  DisplayError hwc_error = hwc_display->ControlPartialUpdate(enable, &pending);

  if (hwc_error == kErrorNone) {
    if (!pending) {
      return 0;
    }
  } else if (hwc_error == kErrorNotSupported) {
    return 0;
  } else {
    return -EINVAL;
  }

  // Todo(user): Unlock it before sending events to client. It may cause deadlocks in future.
  Refresh(HWC_DISPLAY_PRIMARY);

  // Wait until partial update control is complete
  int32_t error = locker_[disp_id].WaitFinite(kPartialUpdateControlTimeoutMs);

  return error;
}

Return<int32_t> HWCSession::controlPartialUpdate(IDisplayConfig::DisplayType dpy, bool enable) {
  return ControlPartialUpdate(MapDisplayType(dpy), enable);
}

Return<int32_t> HWCSession::toggleScreenUpdate(bool on) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  int32_t error = -EINVAL;
  if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
    error = hwc_display_[HWC_DISPLAY_PRIMARY]->ToggleScreenUpdates(on);
    if (error) {
      DLOGE("Failed to toggle screen updates = %d. Error = %d", on, error);
    }
  }

  return error;
}

Return<int32_t> HWCSession::setIdleTimeout(uint32_t value) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
    hwc_display_[HWC_DISPLAY_PRIMARY]->SetIdleTimeoutMs(value);
    return 0;
  }

  return -EINVAL;
}

Return<void> HWCSession::getHDRCapabilities(IDisplayConfig::DisplayType dpy,
                                            getHDRCapabilities_cb _hidl_cb) {
  int32_t error = -EINVAL;
  IDisplayConfig::DisplayHDRCapabilities hdr_caps = {};

  do {
    int disp_id = MapDisplayType(dpy);
    if (disp_id < 0) {
      DLOGE("Invalid display id = %d", disp_id);
      break;
    }

    SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);
    HWCDisplay *hwc_display = hwc_display_[disp_id];
    if (!hwc_display) {
      DLOGE("Display = %d is not connected.", disp_id);
      break;
    }

    // query number of hdr types
    uint32_t out_num_types = 0;
    if (hwc_display->GetHdrCapabilities(&out_num_types, nullptr, nullptr, nullptr, nullptr)
        != HWC2::Error::None) {
      break;
    }

    if (!out_num_types) {
      error = 0;
      break;
    }

    // query hdr caps
    hdr_caps.supportedHdrTypes.resize(out_num_types);

    float out_max_luminance = 0.0f;
    float out_max_average_luminance = 0.0f;
    float out_min_luminance = 0.0f;
    if (hwc_display->GetHdrCapabilities(&out_num_types, hdr_caps.supportedHdrTypes.data(),
                                        &out_max_luminance, &out_max_average_luminance,
                                        &out_min_luminance)
        == HWC2::Error::None) {
      error = 0;
    }
  } while (false);

  _hidl_cb(error, hdr_caps);

  return Void();
}

Return<int32_t> HWCSession::setCameraLaunchStatus(uint32_t on) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  HWBwModes mode = on > 0 ? kBwCamera : kBwDefault;

  // trigger invalidate to apply new bw caps.
  Refresh(HWC_DISPLAY_PRIMARY);

  if (core_intf_->SetMaxBandwidthMode(mode) != kErrorNone) {
    return -EINVAL;
  }

  new_bw_mode_ = true;
  need_invalidate_ = true;
  hwc_display_[HWC_DISPLAY_PRIMARY]->ResetValidation();

  return 0;
}

int32_t HWCSession::DisplayBWTransactionPending(bool *status) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
    if (sync_wait(bw_mode_release_fd_, 0) < 0) {
      DLOGI("bw_transaction_release_fd is not yet signaled: err= %s", strerror(errno));
      *status = false;
    }

    return 0;
  }

  return -EINVAL;
}

Return<void> HWCSession::displayBWTransactionPending(displayBWTransactionPending_cb _hidl_cb) {
  bool status = true;

  int32_t error = DisplayBWTransactionPending(&status);

  _hidl_cb(error, status);

  return Void();
}

}  // namespace sdm
