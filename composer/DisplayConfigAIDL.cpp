/*
* Copyright (c) 2021 The Linux Foundation. All rights reserved.
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

/*
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#include <QtiGralloc.h>
#include <aidl/vendor/qti/hardware/display/demura/IDemuraFileFinder.h>
#include <android/binder_manager.h>
#include <utils/Timers.h>

#include "DisplayConfigAIDL.h"

#include "sdm_interface_factory.h"
#include "display_properties.h"

using sdm::SDMInterfaceFactory;

using ::aidl::android::hardware::common::NativeHandle;
using sdm::Locker;

namespace aidl {
namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace config {

DisplayConfigAIDL::DisplayConfigAIDL() {
  SDMInterfaceFactory *sdm_factory = nullptr;
  sdm_factory = sdm::GetSDMInterfaceFactory();

  caps_ = sdm_factory->CreateCapsIntf();
  settings_ = sdm_factory->CreateSettingsIntf();
  lifecycle_ = sdm_factory->CreateLifeCycleIntf();
  lifecycle_->RegisterSideBandCallback(this, true);
  drawcycle_ = sdm_factory->CreateDrawCycleIntf();
  sideband_ = sdm_factory->CreateSideBandIntf();
  layer_builder_ = sdm_factory->CreateLayerBuilderIntf();
}

int MapDisplayType(DisplayType dpy) {
  switch (dpy) {
    case DisplayType::PRIMARY:
      return qdutils::DISPLAY_PRIMARY;

    case DisplayType::EXTERNAL:
      return qdutils::DISPLAY_EXTERNAL;

    case DisplayType::VIRTUAL:
      return qdutils::DISPLAY_VIRTUAL;

    case DisplayType::BUILTIN2:
      return qdutils::DISPLAY_BUILTIN_2;

    default:
      break;
  }

  return -EINVAL;
}

sdm::SDMDisplayStatus MapExternalStatus(ExternalStatus status) {
  switch (status) {
    case ExternalStatus::OFFLINE:
      return sdm::SDMDisplayStatus::kDisplayStatusOffline;

    case ExternalStatus::ONLINE:
      return sdm::SDMDisplayStatus::kDisplayStatusOnline;

    case ExternalStatus::PAUSE:
      return sdm::SDMDisplayStatus::kDisplayStatusPause;

    case ExternalStatus::RESUME:
      return sdm::SDMDisplayStatus::kDisplayStatusResume;

    default:
      break;
  }

  return sdm::SDMDisplayStatus::kDisplayStatusInvalid;
}

bool WaitForResourceNeeded(sdm::SDMPowerMode prev_mode, sdm::SDMPowerMode new_mode) {
  return ((prev_mode == sdm::SDMPowerMode::POWER_MODE_OFF) &&
          (new_mode == sdm::SDMPowerMode::POWER_MODE_ON ||
           new_mode == sdm::SDMPowerMode::POWER_MODE_DOZE));
}

ScopedAStatus DisplayConfigAIDL::isDisplayConnected(DisplayType dpy, bool *connected) {
  int disp_id = MapDisplayType(dpy);

  *connected = lifecycle_->IsDisplayConnected(disp_id);

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::setDisplayStatus(DisplayType dpy, ExternalStatus status) {
  int disp_id = MapDisplayType(dpy);
  sdm::SDMDisplayStatus external_status = MapExternalStatus(status);

  if (lifecycle_->SetDisplayStatus(disp_id, external_status) != 0) {
    ALOGW("%s: Setting status:%d to display:%d failed", __FUNCTION__, status, disp_id);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::configureDynRefreshRate(DynRefreshRateOp op, int refresh_rate) {
  sdm::SDMBuiltInDisplayOps ops;
  int val = 0;

  switch (op) {
    case DynRefreshRateOp::DISABLE_METADATA:
      ops = sdm::SDMBuiltInDisplayOps::SET_METADATA_DYN_REFRESH_RATE;
      val = false;
      break;

    case DynRefreshRateOp::ENABLE_METADATA:
      ops = sdm::SDMBuiltInDisplayOps::SET_METADATA_DYN_REFRESH_RATE;
      val = true;
      break;

    case DynRefreshRateOp::SET_BINDER:
      ops = sdm::SDMBuiltInDisplayOps::SET_BINDER_DYN_REFRESH_RATE;
      val = refresh_rate;
      break;

    default:
      ALOGW("%s: Invalid operation %d", __FUNCTION__, op);
      return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  auto ret = settings_->ConfigureDynRefreshRate(ops, val);
  if (ret != sdm::kErrorNone) {
    ALOGW("%s: Display = %d is not connected.", __FUNCTION__, sdm::HWC_DISPLAY_PRIMARY);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::getConfigCount(DisplayType dpy, int *count) {
  auto error = lifecycle_->GetConfigCount(MapDisplayType(dpy), (uint32_t *)count);
  if (error != sdm::kErrorNone) {
    ALOGW("%s: Failed to retrieve config count for display:%d", __FUNCTION__, dpy);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::getActiveConfig(DisplayType dpy, int *config) {
  int disp_id = MapDisplayType(dpy);

  int error = drawcycle_->GetActiveConfigIndex(disp_id, (uint32_t *)config);
  if (error != sdm::kErrorNone) {
    ALOGW("%s: Failed to retrieve the active config index for display:%d", __FUNCTION__, dpy);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::setActiveConfig(DisplayType dpy, int config) {
  int disp_id = MapDisplayType(dpy);

  if (drawcycle_->SetActiveConfigIndex(disp_id, (uint32_t)config) != sdm::kErrorNone) {
    ALOGW("%s: Failed to set active config index to display:%d", __FUNCTION__, dpy);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::getDisplayAttributes(int config_index, DisplayType dpy,
                                                      Attributes *attributes) {
  auto error = sdm::kErrorNone;
  int disp_id = MapDisplayType(dpy);

  sdm::DisplayConfigVariableInfo var_info{};
  error = settings_->GetDisplayAttributes(disp_id, config_index, &var_info);

  if (error != sdm::kErrorNone) {
    ALOGW("%s: Invalid display = %d", __FUNCTION__, disp_id);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  attributes->vsyncPeriod = var_info.vsync_period_ns;
  attributes->xRes = var_info.x_pixels;
  attributes->yRes = var_info.y_pixels;
  attributes->xDpi = var_info.x_dpi;
  attributes->yDpi = var_info.y_dpi;
  attributes->panelType = DisplayPortType::DEFAULT;
  attributes->isYuv = var_info.is_yuv;

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::setPanelBrightness(int level) {
  if (!(0 <= level && level <= 255)) {
    ALOGW("%s: Invalid panel brightness level :%d", __FUNCTION__, level);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  if (level == 0) {
    settings_->SetDisplayBrightness(sdm::HWC_DISPLAY_PRIMARY, -1.0f, false);
  } else {
    settings_->SetDisplayBrightness(sdm::HWC_DISPLAY_PRIMARY, (level - 1) / 254.0f, false);
  }
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::getPanelBrightness(int *level) {
  float brightness = -1.0f;

  settings_->GetDisplayBrightness(sdm::HWC_DISPLAY_PRIMARY, &brightness);
  if (brightness == -1.0f) {
    *level = 0;
  } else {
    *level = static_cast<uint32_t>(254.0f * brightness + 1);
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::minHdcpEncryptionLevelChanged(DisplayType dpy, int min_enc_level) {
  drawcycle_->MinHdcpEncryptionLevelChanged(MapDisplayType(dpy), min_enc_level);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::refreshScreen() {
  drawcycle_->Refresh(sdm::HWC_DISPLAY_PRIMARY);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::controlPartialUpdate(DisplayType dpy, bool enable) {
  settings_->ControlPartialUpdate(MapDisplayType(dpy), enable);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::toggleScreenUpdate(bool on) {
  settings_->ToggleScreenUpdate(on);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::setIdleTimeout(int value) {
  settings_->SetIdleTimeout(value);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::getHDRCapabilities(DisplayType dpy, HDRCapsParams *caps) {
  int error = -EINVAL;

  do {
    int disp_id = MapDisplayType(dpy);

    // query number of hdr types
    uint32_t out_num_types = 0;
    float out_max_luminance = 0.0f;
    float out_max_average_luminance = 0.0f;
    float out_min_luminance = 0.0f;
    if (caps_->GetHdrCapabilities(disp_id, &out_num_types, nullptr, &out_max_luminance,
                                  &out_max_average_luminance,
                                  &out_min_luminance) != sdm::kErrorNone) {
      break;
    }
    if (!out_num_types) {
      error = 0;
      break;
    }

    // query hdr caps
    caps->supportedHdrTypes.resize(out_num_types);

    if (caps_->GetHdrCapabilities(disp_id, &out_num_types, caps->supportedHdrTypes.data(),
                                  &out_max_luminance, &out_max_average_luminance,
                                  &out_min_luminance) == sdm::kErrorNone) {
      error = 0;
    }
  } while (false);

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::setCameraLaunchStatus(int on) {
  sideband_->SetCameraLaunchStatus(on);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::displayBWTransactionPending(bool *status) {
  sideband_->DisplayBWTransactionPending(status);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::setDisplayAnimating(long display_id, bool animating) {
  sideband_->SetDisplayAnimating(display_id, animating);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::controlIdlePowerCollapse(bool enable, bool synchronous) {
  sideband_->ControlIdlePowerCollapse(enable, synchronous);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::getWriteBackCapabilities(bool *is_wb_ubwc_supported) {
  caps_->IsWbUbwcSupported(is_wb_ubwc_supported);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::setDisplayDppsAdROI(int display_id, int h_start, int h_end,
                                                     int v_start, int v_end, int factor_in,
                                                     int factor_out) {
  settings_->SetDisplayDppsAdROI(display_id, h_start, h_end, v_start, v_end, factor_in, factor_out);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::updateVSyncSourceOnPowerModeOff() {
  settings_->UpdateVSyncSourceOnPowerModeOff();
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::updateVSyncSourceOnPowerModeDoze() {
  settings_->UpdateVSyncSourceOnPowerModeDoze();
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::setPowerMode(int disp_id, PowerMode power_mode) {
  // This API is deprecated
  return ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ScopedAStatus DisplayConfigAIDL::isPowerModeOverrideSupported(int disp_id, bool *supported) {
  *supported = false;
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::isHDRSupported(int disp_id, bool *supported) {
  if (disp_id < 0 || disp_id >= sdm::kNumDisplays) {
    ALOGW("%s: Not valid display", __FUNCTION__);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  *supported = settings_->IsHDRDisplay(disp_id);

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::isWCGSupported(int disp_id, bool *supported) {
  isHDRSupported(disp_id, supported);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::setLayerAsMask(int disp_id, long layer_id) {
  auto err = layer_builder_->SetLayerAsMask(disp_id, layer_id);
  if (err != sdm::kErrorNone) {
    return ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::getDebugProperty(const std::string &prop_name,
                                                  std::string *value) {
  std::string vendor_prop_name = DISP_PROP_PREFIX;
  int error = -EINVAL;
  char val[64] = {};

  vendor_prop_name += prop_name.c_str();
  if (!sideband_->GetProperty(vendor_prop_name.c_str(), val)) {
    *value = val;
    error = 0;
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::setClientUp() {
  sideband_->SetClientUp();

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::getActiveBuiltinDisplayAttributes(Attributes *attr) {
  uint64_t disp_id = sdm::kNumDisplays;
  auto error = settings_->GetActiveBuiltinDisplay(&disp_id);
  if (error != sdm::kErrorNone) {
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  sdm::Config config = -1;
  error = settings_->GetActiveConfig(disp_id, &config);
  if (error != sdm::kErrorNone) {
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  sdm::DisplayConfigVariableInfo var_info{};
  error = settings_->GetDisplayAttributes(disp_id, config, &var_info);

  if (error != sdm::kErrorNone) {
    ALOGW("%s: Invalid display = %ld", __FUNCTION__, disp_id);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  attr->vsyncPeriod = var_info.vsync_period_ns;
  attr->xRes = var_info.x_pixels;
  attr->yRes = var_info.y_pixels;
  attr->xDpi = var_info.x_dpi;
  attr->yDpi = var_info.y_dpi;
  attr->panelType = DisplayPortType::DEFAULT;
  attr->isYuv = var_info.is_yuv;

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::setPanelLuminanceAttributes(int disp_id, float min_lum,
                                                             float max_lum) {
  // currently doing only for virtual display
  if (disp_id != static_cast<int>(DisplayType::VIRTUAL)) {
    ALOGW("%s: Setting panel luminance on non virtual display is not supported", __FUNCTION__);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  // check for out of range luminance values
  if (min_lum <= 0.0f || min_lum >= 1.0f || max_lum <= 100.0f || max_lum >= 1000.0f) {
    ALOGW("%s: Luminance values are out of range : minimum_luminance:%f maximum_luminance:%f",
          __FUNCTION__, min_lum, max_lum);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  settings_->SetPanelLuminanceAttributes(disp_id, min_lum, max_lum);

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::isBuiltInDisplay(int disp_id, bool *is_built_in) {
  *is_built_in = sideband_->IsBuiltInDisplay(disp_id);

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::isAsyncVDSCreationSupported(bool *supported) {
  *supported = sideband_->IsAsyncVDSCreationSupported();

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::createVirtualDisplay(int width, int height, int format) {
  auto ret = sideband_->CreateVirtualDisplay(width, height, format);
  if (ret != sdm::kErrorNone) {
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::getSupportedDSIBitClks(int disp_id, std::vector<long> *bit_clks) {
  auto ret = caps_->GetSupportedDSIClock(disp_id, bit_clks);
  if (ret == sdm::kErrorResources) {
    ALOGW("%s: Display: %d is not connected", __FUNCTION__, disp_id);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::getDSIClk(int disp_id, long *bit_clk) {
  auto ret = settings_->GetDSIClk(disp_id, (uint64_t *)bit_clk);
  if (ret == sdm::kErrorResources) {
    ALOGW("%s: Invalid display: %d", __FUNCTION__, disp_id);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::setDSIClk(int disp_id, long bit_clk) {
  auto ret = settings_->SetDSIClk(disp_id, (uint64_t)bit_clk);
  if (ret == sdm::kErrorResources) {
    ALOGW("%s: Invalid display: %d", __FUNCTION__, disp_id);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::setQsyncMode(int disp_id, QsyncMode mode) {
  sdm::QSyncMode qsync_mode = sdm::kQSyncModeNone;
  switch (mode) {
    case QsyncMode::NONE:
      qsync_mode = sdm::kQSyncModeNone;
      break;

    case QsyncMode::WAIT_FOR_FENCES_ONE_FRAME:
      qsync_mode = sdm::kQsyncModeOneShot;
      break;

    case QsyncMode::WAIT_FOR_FENCES_EACH_FRAME:
      qsync_mode = sdm::kQsyncModeOneShotContinuous;
      break;

    case QsyncMode::WAIT_FOR_COMMIT_EACH_FRAME:
      qsync_mode = sdm::kQSyncModeContinuous;
      break;
  }

  auto ret = settings_->SetQsyncMode(disp_id, qsync_mode);
  if (ret == sdm::kErrorResources) {
    ALOGW("%s: failed: %d", __FUNCTION__, ret);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::isSmartPanelConfig(int disp_id, int config_id, bool *is_smart) {
  auto ret = caps_->IsSmartPanelConfig(disp_id, config_id, is_smart);
  if (ret != sdm::kErrorNone) {
    ALOGW("%s: failed: %d", __FUNCTION__, ret);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::isRotatorSupportedFormat(int hal_format, bool ubwc,
                                                          bool *supported) {
  int flag = ubwc ? IS_UBWC : 0;
  sdm::LayerBufferFormat sdm_format = layer_builder_->GetSDMFormat(hal_format, flag, 0);

  *supported = caps_->IsRotatorSupportedFormat(sdm_format);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::controlQsyncCallback(bool enable) {
  if (enable) {
    qsync_callback_ = callback_;
  } else {
    qsync_callback_.reset();
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::sendTUIEvent(DisplayType dpy, TUIEventType event_type) {
  int disp_id = MapDisplayType(dpy);
  auto ret = sideband_->TUIEventHandler(disp_id, static_cast<sdm::SDMTUIEventType>(event_type));
  if (ret != sdm::kErrorNone) {
    ALOGW("TUIEventHandler failed with %d", ret);
    return ScopedAStatus(AStatus_fromServiceSpecificError(EX_ILLEGAL_ARGUMENT));
  }
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::getDisplayHwId(int disp_id, int *display_hw_id) {
  auto ret = caps_->GetDisplayHwId(disp_id, display_hw_id);
  if (ret != sdm::kErrorNone) {
    ALOGW("getDisplayHwId failed with %d", ret);
    return ScopedAStatus(AStatus_fromServiceSpecificError(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::getSupportedDisplayRefreshRates(
    DisplayType dpy, std::vector<int> *supported_refresh_rates) {
  caps_->GetSupportedDisplayRefreshRates(MapDisplayType(dpy),
                                         (std::vector<uint32_t> *)supported_refresh_rates);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::isRCSupported(int disp_id, bool *supported) {
  // Mask layers can potentially be shown on any display so report RC supported on all displays if
  // the property enables the feature for use.
  int val = false;  // Default value.
  sideband_->GetProperty(ENABLE_ROUNDED_CORNER, &val);
  *supported = val ? true : false;

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::controlIdleStatusCallback(bool enable) {
  if (enable) {
    enable_aidl_idle_notification_ = true;
  } else {
    enable_aidl_idle_notification_ = false;
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::isSupportedConfigSwitch(int disp_id, int config, bool *supported) {
  *supported = caps_->IsModeSwitchAllowed(disp_id, config);
  return ScopedAStatus::ok();
}

DisplayType GetDisplayConfigDisplayType(int qdutils_disp_type) {
  switch (qdutils_disp_type) {
    case qdutils::DISPLAY_PRIMARY:
      return DisplayType::PRIMARY;

    case qdutils::DISPLAY_EXTERNAL:
      return DisplayType::EXTERNAL;

    case qdutils::DISPLAY_VIRTUAL:
      return DisplayType::VIRTUAL;

    case qdutils::DISPLAY_BUILTIN_2:
      return DisplayType::BUILTIN2;

    default:
      return DisplayType::INVALID;
  }
}

int DisplayConfigAIDL::GetDispTypeFromPhysicalId(uint64_t physical_disp_id,
                                                 DisplayType *disp_type) {
  // TODO(user): Least significant 8 bit is port id based on the SF current implementaion. Need to
  // revisit this if there is a change in logic to create physical display id in SF.
  int port_id = (physical_disp_id & 0xFF);
  int out_port = 0;
  for (int dpy = qdutils::DISPLAY_PRIMARY; dpy <= qdutils::DISPLAY_EXTERNAL_2; dpy++) {
    auto ret = caps_->GetDisplayPortId(dpy, &out_port);
    if (ret != sdm::kErrorNone) {
      return ret;
    }
    if (port_id == out_port) {
      *disp_type = GetDisplayConfigDisplayType(dpy);
      return 0;
    }
  }

  return -ENODEV;
}

ScopedAStatus DisplayConfigAIDL::getDisplayType(long physical_disp_id, DisplayType *display_type) {
  if (!display_type) {
    ALOGW("%s: Display type provided is invalid.", __FUNCTION__);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  GetDispTypeFromPhysicalId(physical_disp_id, display_type);
  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::setCWBOutputBuffer(
    const std::shared_ptr<IDisplayConfigCallback> &callback, int32_t disp_id, const Rect &rect,
    bool post_processed, const NativeHandle &buffer) {
  if (!callback) {
    ALOGE("%s: Callback provided is invalid.", __FUNCTION__);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

// TODO (user): Need to extend display type enumeration in DisplayConfig AIDL interface
// for display external-2 (MST) to replace corresponding macro.
#define DISPLAY_TYPE_EXTERNAL_2 (UINT32(DisplayType::BUILTIN2) + 1)
  std::unordered_map<int32_t, int32_t> disp_type_map = {
      {static_cast<int32_t>(DisplayType::PRIMARY), static_cast<int32_t>(qdutils::DISPLAY_PRIMARY)},
      {static_cast<int32_t>(DisplayType::EXTERNAL),
       static_cast<int32_t>(qdutils::DISPLAY_EXTERNAL)},
      {static_cast<int32_t>(DisplayType::BUILTIN2),
       static_cast<int32_t>(qdutils::DISPLAY_BUILTIN_2)},
      {static_cast<int32_t>(DISPLAY_TYPE_EXTERNAL_2),
       static_cast<int32_t>(qdutils::DISPLAY_EXTERNAL_2)},
  };

  if (disp_id <= static_cast<int32_t>(DisplayType::INVALID) ||
      disp_id > static_cast<int32_t>(DISPLAY_TYPE_EXTERNAL_2)) {
    ALOGE("%s: CWB is supported on 2 builtin as well as 2 exernal displays only at present.",
          __FUNCTION__);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  int32_t display_type = disp_type_map[disp_id];

  sdm::CwbConfig cwb_config = {};
  cwb_config.tap_point = static_cast<sdm::CwbTapPoint>(post_processed);
  sdm::LayerRect &roi = cwb_config.cwb_roi;
  roi.left = FLOAT(rect.left);
  roi.top = FLOAT(rect.top);
  roi.right = FLOAT(rect.right);
  roi.bottom = FLOAT(rect.bottom);

  ALOGI("CWB config passed by cwb_client : tappoint %d  CWB_ROI : (%f %f %f %f) for display-%d",
        cwb_config.tap_point, roi.left, roi.top, roi.right, roi.bottom, display_type);

  auto ret_status = EX_NONE;

  void *hdl = sdm::ConvertToSnapHandle(buffer);

  if (!hdl || !handle_importer_.importBuffer(static_cast<const SnapHandle *>(hdl))) {
    ALOGE("%s: Either retrieving snaphandle or importing buffer failed.", __FUNCTION__);
    ret_status = EX_ILLEGAL_ARGUMENT;
  }

  bool hdl_exists = false;
  if (ret_status == EX_NONE) {
    std::lock_guard<decltype(cwb_callbacks_lock_)> lock_guard(cwb_callbacks_lock_);
    hdl_exists = cwb_callbacks_.find(hdl) != cwb_callbacks_.end();
    ret_status = (hdl_exists) ? EX_ILLEGAL_ARGUMENT : EX_NONE;
  }

  if (ret_status == EX_NONE) {
    sdm::DisplayError ret = sideband_->PostBuffer(cwb_config, hdl, display_type);
    if (ret != sdm::kErrorNone) {
      ret_status = EX_TRANSACTION_FAILED;
    } else {
      std::lock_guard<decltype(cwb_callbacks_lock_)> lock_guard(cwb_callbacks_lock_);
      cwb_callbacks_.insert({hdl, {display_type, callback}});
    }
  } else if (hdl_exists) {
      ALOGE("%s: buffer(0x%p) already being handled by display-%d", __FUNCTION__, hdl, display_type);
  }

  if (ret_status != EX_NONE) {
    // Need to unregister and delete snap handle on CWB request rejection/failure
    handle_importer_.freeBuffer(static_cast<const SnapHandle *>(hdl));
  }

  return (ret_status == EX_NONE) ? ScopedAStatus::ok()
                                 : ScopedAStatus(AStatus_fromExceptionCode(ret_status));
}

void DisplayConfigAIDL::NotifyCWBStatus(int32_t status, void *hdl) {
  std::shared_ptr<IDisplayConfigCallback> callback = nullptr;
  int32_t display_type = 0;

  if (!hdl) {
    ALOGE("%s: Null buffer handle is detected to notify!", __FUNCTION__);
    return;
  } else {
    std::lock_guard<decltype(cwb_callbacks_lock_)> lock_guard(cwb_callbacks_lock_);
    auto hdl_exists = cwb_callbacks_.find(hdl) != cwb_callbacks_.end();
    if (hdl_exists) {
      std::tie(display_type, callback) = cwb_callbacks_[hdl];
      cwb_callbacks_.erase(hdl);
    }
  }

  if (!callback) {
    ALOGE("%s: buffer handle(0x%p) not found", __FUNCTION__, hdl);
  } else {
    NativeHandle buffer =
        sdm::AIDLNativeHandleFromSnapHandle(reinterpret_cast<SnapHandle *>(hdl), false);
    ALOGI("%s: Notify the client about buffer (0x%p) status %d for display-%d.", __FUNCTION__, hdl,
          status, display_type);

    callback->notifyCWBBufferDone(status, buffer);
  }

  handle_importer_.freeBuffer(static_cast<const SnapHandle *>(hdl));
}

ScopedAStatus DisplayConfigAIDL::setCameraSmoothInfo(CameraSmoothOp op, int32_t fps) {
  int ret = -1;

  if (fps < 0) {
    return ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
  }

  ret = sideband_->SetCameraSmoothInfo(static_cast<sdm::SDMCameraSmoothOp>(op), fps);

  return ret == sdm::kErrorNone ? ScopedAStatus::ok()
                                : ScopedAStatus::fromExceptionCode(EX_TRANSACTION_FAILED);
}

ScopedAStatus DisplayConfigAIDL::setContentFps(const std::string &name, int32_t fps) {
  int ret = -1;

  if (fps <= 0) {
    return ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
  }

  ret = sideband_->SetContentFps(name, fps);

  return ret == sdm::kErrorNone ? ScopedAStatus::ok()
                                : ScopedAStatus::fromExceptionCode(EX_TRANSACTION_FAILED);
}

ScopedAStatus DisplayConfigAIDL::registerCallback(
    const std::shared_ptr<IDisplayConfigCallback> &callback, int64_t *client_handle) {
  int ret = -1;

  if (callback == nullptr) {
    return ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
  }

  std::lock_guard<decltype(callbacks_lock_)> lock_guard(callbacks_lock_);
  callback_clients_.emplace(callback_client_id_, callback);
  *client_handle = callback_client_id_;
  callback_client_id_++;

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::unRegisterCallback(int64_t client_handle) {
  int ret = -1;
  bool removed = false;

  if (client_handle < 0) {
    return ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
  }

  std::lock_guard<decltype(callbacks_lock_)> lock_guard(callbacks_lock_);
  for (auto it = callback_clients_.begin(); it != callback_clients_.end();) {
    if (it->first == client_handle) {
      it = callback_clients_.erase(it);
      removed = true;
    } else {
      it++;
    }
  }

  return removed ? ScopedAStatus::ok() : ScopedAStatus::fromExceptionCode(EX_TRANSACTION_FAILED);
}

ScopedAStatus DisplayConfigAIDL::getDisplayPortId(int32_t disp_id, int32_t *port_id) {
  int ret = -1;

  if (!port_id) {
    return ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
  }

  ret = caps_->GetDisplayPortId(disp_id, port_id);

  return ret == 0 ? ScopedAStatus::ok() : ScopedAStatus::fromExceptionCode(EX_TRANSACTION_FAILED);
}

ScopedAStatus DisplayConfigAIDL::isCacV2Supported(int disp_id, bool *supported) {
  if (disp_id < 0 || disp_id >= sdm::kNumDisplays) {
    ALOGW("%s: Not valid display", __FUNCTION__);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  if (!supported) {
    return ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
  }

  auto ret = caps_->IsCacV2Supported(disp_id, supported);

  return ret == 0 ? ScopedAStatus::ok() : ScopedAStatus::fromExceptionCode(EX_TRANSACTION_FAILED);
}

ScopedAStatus DisplayConfigAIDL::configureCacV2(int32_t disp_id, const CacV2Config &config,
                                                bool enable) {
  if (disp_id < 0 || disp_id >= sdm::kNumDisplays) {
    ALOGW("%s: Not valid display", __FUNCTION__);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  sdm::CacConfig cac_config = {};
  cac_config.k0r = config.k0r;
  cac_config.k1r = config.k1r;
  cac_config.k0b = config.k0b;
  cac_config.k1b = config.k1b;
  cac_config.pixel_pitch = config.pixel_pitch;
  cac_config.normalization = config.normalization;

  auto ret = settings_->PerformCacConfig(disp_id, cac_config, enable);
  if (ret != sdm::kErrorNone) {
    ALOGW("%s: Failed to configure CAC = %d", __FUNCTION__, enable);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::configureCacV2PerEye(int32_t disp_id,
                                                      const CacV2Config &leftConfig,
                                                      const CacV2Config &rightConfig, bool enable) {
  if (disp_id < 0 || disp_id >= sdm::kNumDisplays) {
    ALOGW("%s: Not valid display", __FUNCTION__);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  // TODO(user): add support for CAC configuration per eye
  sdm::CacConfig cac_config = {};
  cac_config.k0r = leftConfig.k0r;
  cac_config.k1r = leftConfig.k1r;
  cac_config.k0b = leftConfig.k0b;
  cac_config.k1b = leftConfig.k1b;
  cac_config.pixel_pitch = leftConfig.pixel_pitch;
  cac_config.normalization = leftConfig.normalization;

  auto ret = settings_->PerformCacConfig(disp_id, cac_config, enable);
  if (ret != sdm::kErrorNone) {
    ALOGW("%s: Failed to configure CAC = %d", __FUNCTION__, enable);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

ScopedAStatus DisplayConfigAIDL::configureCacV2ExtPerEye(int32_t disp_id,
                                                         const CacV2ConfigExt &leftConfig,
                                                         const CacV2ConfigExt &rightConfig,
                                                         bool enable) {
  if (disp_id < 0 || disp_id >= sdm::kNumDisplays) {
    ALOGW("%s: Not valid display", __FUNCTION__);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  // TODO(user): add support for CAC configuration per eye
  sdm::CacConfig cac_config = {};
  cac_config.k0r = leftConfig.redCenterPhaseStep;
  cac_config.k1r = leftConfig.redSecondOrderPhaseStep;
  cac_config.k0b = leftConfig.blueCenterPhaseStep;
  cac_config.k1b = leftConfig.blueSecondOrderPhaseStep;
  cac_config.pixel_pitch = leftConfig.pixelPitch;
  cac_config.normalization = leftConfig.normalization;
  cac_config.mid_le_y_offset = leftConfig.verticalCenter;
  cac_config.mid_le_x_offset = leftConfig.horizontalCenter;
  cac_config.mid_re_y_offset = rightConfig.verticalCenter;
  cac_config.mid_re_x_offset = rightConfig.horizontalCenter;

  auto ret = settings_->PerformCacConfig(disp_id, cac_config, enable);
  if (ret != sdm::kErrorNone) {
    ALOGW("%s: Failed to configure CAC = %d", __FUNCTION__, enable);
    return ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
  }

  return ScopedAStatus::ok();
}

void DisplayConfigAIDL::NotifyQsyncChange(uint64_t display_id, bool qsync_enabled,
                                          uint32_t refresh_rate, uint32_t qsync_refresh_rate) {
  // AIDL callback
  if (!callback_clients_.empty()) {
    std::lock_guard<decltype(callbacks_lock_)> lock_guard(callbacks_lock_);
    for (auto const &[id, callback] : callback_clients_) {
      if (callback) {
        callback->notifyQsyncChange(qsync_enabled, refresh_rate, qsync_refresh_rate);
      }
    }
  }

  // HIDL callback
  std::shared_ptr<DisplayConfig::ConfigCallback> callback = qsync_callback_.lock();
  if (!callback) {
    return;
  }

  callback->NotifyQsyncChange(qsync_enabled, refresh_rate, qsync_refresh_rate);
}

void DisplayConfigAIDL::NotifyCameraSmoothInfo(sdm::SDMCameraSmoothOp op, int32_t fps) {
  std::lock_guard<decltype(callbacks_lock_)> lock_guard(callbacks_lock_);

  for (auto const &[id, callback] : callback_clients_) {
    if (callback) {
      callback->notifyCameraSmoothInfo(static_cast<CameraSmoothOp>(op), fps);
    }
  }
}

void DisplayConfigAIDL::NotifyResolutionChange(uint64_t display_id,
                                               sdm::SDMConfigAttributes &attr) {
  std::lock_guard<decltype(callbacks_lock_)> lock_guard(callbacks_lock_);

  Attributes attributes{};
  attributes.vsyncPeriod = attr.vsyncPeriod;
  attributes.xRes = attr.xRes;
  attributes.yRes = attr.yRes;
  attributes.xDpi = attr.xDpi;
  attributes.yDpi = attr.yDpi;
  attributes.panelType = static_cast<DisplayPortType>(attr.panelType);

  for (auto const &[id, callback] : callback_clients_) {
    if (callback) {
      callback->notifyResolutionChange(display_id, attributes);
    }
  }
}

typedef ::aidl::vendor::qti::hardware::display::config::DisplayType AIDLDisplayType;
AIDLDisplayType MapDisplayId(int disp_id) {
  switch (disp_id) {
    case qdutils::DISPLAY_PRIMARY:
      return AIDLDisplayType::PRIMARY;

    case qdutils::DISPLAY_EXTERNAL:
      return AIDLDisplayType::EXTERNAL;

    case qdutils::DISPLAY_VIRTUAL:
      return AIDLDisplayType::VIRTUAL;

    case qdutils::DISPLAY_BUILTIN_2:
      return AIDLDisplayType::BUILTIN2;

    default:
      break;
  }

  return AIDLDisplayType::INVALID;
}

void DisplayConfigAIDL::NotifyTUIEventDone(uint32_t ret, uint32_t disp_id,
                                           sdm::SDMTUIEventType event_type) {
  std::lock_guard<decltype(callbacks_lock_)> lock_guard(callbacks_lock_);

  AIDLDisplayType disp_type = MapDisplayId(disp_id);
  for (auto const &[id, callback] : callback_clients_) {
    if (callback) {
      callback->notifyTUIEventDone(ret, disp_type, static_cast<TUIEventType>(event_type));
    }
  }
}

void DisplayConfigAIDL::NotifyIdleStatus(bool status) {
  if (!enable_aidl_idle_notification_) {
    return;
  }

  std::lock_guard<decltype(callbacks_lock_)> lock_guard(callbacks_lock_);
  for (auto const &[id, callback] : callback_clients_) {
    if (callback) {
      callback->notifyIdleStatus(status);
    }
  }
}

void DisplayConfigAIDL::NotifyContentFps(const std::string &name, int32_t fps) {
  std::lock_guard<decltype(callbacks_lock_)> lock_guard(callbacks_lock_);

  for (auto const &[id, callback] : callback_clients_) {
    if (callback) {
      callback->notifyContentFps(name, fps);
    }
  }
}

void DisplayConfigAIDL::OnHdmiHotplug(bool connected) {
  if (qservice_ == nullptr) {
    qservice_ = sdm::QServiceBackend::GetInstance();
  }

  qservice_->OnHdmiHotplug(connected);
}

using ::aidl::vendor::qti::hardware::display::demura::IDemuraFileFinder;
using DemuraFilePaths = IDemuraFileFinder::DemuraFilePaths;
using sdm::DemuraPaths;

int DisplayConfigAIDL::GetDemuraFilePaths(const GenericPayload &in, GenericPayload *out) {
  int ret = 0;
  std::shared_ptr<IDemuraFileFinder> demuraAidl = nullptr;
  const std::string instance = std::string() + IDemuraFileFinder::descriptor + "/default";
  if (!AServiceManager_isDeclared(instance.c_str())) {
    ALOGE("demura hal service is not declared");
    return -ENODEV;
  }
  auto demuraBinder = ::ndk::SpAIBinder(AServiceManager_waitForService(instance.c_str()));
  if (demuraBinder.get() == nullptr) {
    ALOGE("demura hal service doesn't exist");
    return -EINVAL;
  }
  demuraAidl = IDemuraFileFinder::fromBinder(demuraBinder);
  if (demuraAidl == nullptr) {
    ALOGE("Could not get IDemuraFileFinder");
    return -ENODEV;
  }
  uint32_t sz = 0;
  uint64_t *panel_id = nullptr;
  DemuraPaths *file_paths = nullptr;
  if ((ret = in.GetPayload(panel_id, &sz))) {
    ALOGE("Failed to get input payload error = %d", ret);
    return ret;
  }
  ALOGI("panel_id %" PRIu64, *panel_id);
  if ((ret = out->GetPayload(file_paths, &sz))) {
    ALOGE("Failed to get output payload error = %d", ret);
    return ret;
  }
  DemuraFilePaths paths = {};
  auto status = demuraAidl->getDemuraFilePaths(*panel_id, &paths);
  if (!status.isOk()) {
    ALOGE("getDemuraFilePaths failed, status: %d: %s", status.getStatus(), status.getMessage());
    return -EINVAL;
  }
  file_paths->configPath = paths.configFilePath;
  file_paths->signaturePath = paths.signatureFilePath;
  file_paths->publickeyPath = paths.publickeyFilePath;

  return 0;
}

GLRect SdmRectToGlRect(sdm::SDMRect &r) {
  GLRect rect = {FLOAT(r.left), FLOAT(r.top), FLOAT(r.right), FLOAT(r.bottom)};
  return rect;
}

void DisplayConfigAIDL::StitchLayers(uint64_t display, sdm::LayerStitchContext *ctx) {
  if (layer_stitch_map_.find(display) == layer_stitch_map_.end()) {
    ALOGW("GL Layer stitch not initialized for display %lu!", display);
    return;
  }

  // convert from sdmclient defs to gl defs
  std::vector<sdm::StitchParams> gl_params;
  for (auto p : ctx->stitch_params) {
    sdm::StitchParams param;
    param.src_hnd = sdm::SnapHandleToLegacyHandle(reinterpret_cast<SnapHandle *>(p.src_hnd));
    param.dst_hnd = sdm::SnapHandleToLegacyHandle(reinterpret_cast<SnapHandle *>(p.dst_hnd));
    param.src_rect = SdmRectToGlRect(p.src_rect);
    param.dst_rect = SdmRectToGlRect(p.dst_rect);
    param.scissor_rect = SdmRectToGlRect(p.scissor_rect);
    param.src_acquire_fence = p.src_acquire_fence;
    param.dst_acquire_fence = p.dst_acquire_fence;
    gl_params.push_back(param);
  }

  layer_stitch_map_.at(display)->Blit(gl_params, &(ctx->release_fence));
}

void DisplayConfigAIDL::InitLayerStitch(uint64_t display) {
  if (layer_stitch_map_.find(display) == layer_stitch_map_.end()) {
    layer_stitch_map_.insert({display, nullptr});
  }

  layer_stitch_map_.at(display) = GLLayerStitch::GetInstance(false);
  if (layer_stitch_map_.at(display) == nullptr) {
    ALOGW("Unable to initialize layer stitch: display %lu", display);
    layer_stitch_map_.erase(display);
  }
}

void DisplayConfigAIDL::DestroyLayerStitch(uint64_t display) {
  auto stitch_layer = layer_stitch_map_.find(display);
  if (stitch_layer == layer_stitch_map_.end()) {
    return;
  }

  GLLayerStitch::Destroy(stitch_layer->second);
  layer_stitch_map_.erase(stitch_layer);
}

void DisplayConfigAIDL::InitColorConvert(uint64_t display, bool secure) {
  if (color_convert_map_.find(display) == color_convert_map_.end()) {
    color_convert_map_.insert({display, nullptr});
  }

  color_convert_map_.at(display) = GLColorConvert::GetInstance(sdm::kTargetYUV, secure);
}

void DisplayConfigAIDL::ColorConvertBlit(uint64_t display, sdm::ColorConvertBlitContext *ctx) {
  if (color_convert_map_.find(display) == color_convert_map_.end()) {
    ALOGW("Display %lu: GL Color convert is not initialized", display);
    return;
  }

  GLRect src_rect = {FLOAT(ctx->src_rect.left), FLOAT(ctx->src_rect.top),
                     FLOAT(ctx->src_rect.right), FLOAT(ctx->src_rect.bottom)};
  GLRect dst_rect = {FLOAT(ctx->dst_rect.left), FLOAT(ctx->dst_rect.top),
                     FLOAT(ctx->dst_rect.right), FLOAT(ctx->dst_rect.bottom)};
  native_handle_t *legacy_src_handle =
      sdm::SnapHandleToLegacyHandle(reinterpret_cast<SnapHandle *>(ctx->src_hnd));
  native_handle_t *legacy_dst_handle =
      sdm::SnapHandleToLegacyHandle(reinterpret_cast<SnapHandle *>(ctx->dst_hnd));

  color_convert_map_.at(display)->Blit(legacy_src_handle, legacy_dst_handle, src_rect, dst_rect,
                                       ctx->src_acquire_fence, ctx->dst_acquire_fence,
                                       &(ctx->release_fence));
}

void DisplayConfigAIDL::ResetColorConvert(uint64_t display) {
  if (color_convert_map_.find(display) == color_convert_map_.end()) {
    return;
  }

  color_convert_map_.at(display)->Reset();
}

void DisplayConfigAIDL::DestroyColorConvert(uint64_t display) {
  if (color_convert_map_.find(display) == color_convert_map_.end()) {
    return;
  }

  auto convert = color_convert_map_.find(display);
  GLColorConvert::Destroy(convert->second);
  color_convert_map_.erase(convert);
}

void DisplayConfigAIDL::OnCECMessageReceived(char *message, int len) {
  qservice_->OnCECMessageReceived(message, len);
}

void DisplayConfigAIDL::StartHistogram(uint64_t display, int max_frames) {
  // if this is
  if (histogram_map_.find(display) == histogram_map_.end()) {
    histogram_map_.emplace(display, new histogram::HistogramCollector());
  }

  if (max_frames == 0) {
    histogram_map_.at(display)->start();
  } else {
    histogram_map_.at(display)->start(max_frames);
  }
}

void DisplayConfigAIDL::StopHistogram(uint64_t display, bool teardown) {
  if (histogram_map_.find(display) == histogram_map_.end()) {
    return;
  }

  histogram_map_.at(display)->stop();

  if (teardown) {
    delete histogram_map_.at(display);
    histogram_map_.erase(histogram_map_.find(display));
  }
}

void DisplayConfigAIDL::NotifyHistogram(uint64_t display, int fd, uint64_t blob_id,
                                        uint32_t panel_width, uint32_t panel_height) {
  if (histogram_map_.find(display) != histogram_map_.end()) {
    histogram_map_.at(display)->notify_histogram_event(fd, blob_id, panel_width, panel_height);
  }
}
std::string DisplayConfigAIDL::DumpHistogram(uint64_t display) {
  if (histogram_map_.find(display) != histogram_map_.end()) {
    return histogram_map_.at(display)->Dump();
  }

  return "";
}

void DisplayConfigAIDL::CollectHistogram(uint64_t display, uint64_t max_frames, uint64_t timestamp,
                                         int32_t samples_size[NUM_HISTOGRAM_COLOR_COMPONENTS],
                                         uint64_t *samples[NUM_HISTOGRAM_COLOR_COMPONENTS],
                                         uint64_t *numFrames) {
  if (histogram_map_.find(display) == histogram_map_.end()) {
    ALOGW("Display %lu: Histogram not initialized!", display);
    return;
  }

  histogram_map_.at(display)->collect(max_frames, timestamp, samples_size, samples, numFrames);
}

sdm::DisplayError DisplayConfigAIDL::GetHistogramAttributes(uint64_t display, int32_t *format,
                                                            int32_t *dataspace,
                                                            uint8_t *supported_components) {
  if (histogram_map_.find(display) == histogram_map_.end()) {
    return sdm::kErrorNotSupported;
  }

  return static_cast<sdm::DisplayError>(
      histogram_map_.at(display)->getAttributes(format, dataspace, supported_components));
}

sdm::nsecs_t DisplayConfigAIDL::SystemTime(int clock) {
  return systemTime(clock);
}

}  // namespace config
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
}  // namespace aidl
