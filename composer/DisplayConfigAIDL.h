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
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <aidl/vendor/qti/hardware/display/config/BnDisplayConfig.h>
#include <aidl/vendor/qti/hardware/display/config/BnDisplayConfigCallback.h>
#include <aidlcommonsupport/NativeHandle.h>
#include <binder/Status.h>
#include <log/log.h>
#include <utils/locker.h>

#include "hwc_session.h"

namespace aidl {
namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace config {

using ::android::binder::Status;
using ndk::ScopedAStatus;
using sdm::HWCSession;

class DisplayConfigCallback : public BnDisplayConfigCallback {
  public:
    DisplayConfigCallback(const std::function<void()>& callback) : mCallback(callback) {}

  private:
    std::function<void()> mCallback;
};

class DisplayConfigAIDL : public BnDisplayConfig {
  public:
    DisplayConfigAIDL();
    DisplayConfigAIDL(sdm::HWCSession *hwc_session);
    int GetDispTypeFromPhysicalId(uint64_t physical_disp_id, DisplayType *disp_type);
    ScopedAStatus isDisplayConnected(DisplayType dpy, bool* connected) override;
    ScopedAStatus setDisplayStatus(DisplayType dpy, ExternalStatus status) override;
    ScopedAStatus configureDynRefreshRate(DynRefreshRateOp op, int refreshRate) override;
    ScopedAStatus getConfigCount(DisplayType dpy, int* count) override;
    ScopedAStatus getActiveConfig(DisplayType dpy, int* config) override;
    ScopedAStatus setActiveConfig(DisplayType dpy, int config) override;
    ScopedAStatus getDisplayAttributes(int configIndex, DisplayType dpy,
                                       Attributes* attributes) override;
    ScopedAStatus setPanelBrightness(int level) override;
    ScopedAStatus getPanelBrightness(int* level) override;
    ScopedAStatus minHdcpEncryptionLevelChanged(DisplayType dpy, int minEncLevel) override;
    ScopedAStatus refreshScreen() override;
    ScopedAStatus controlPartialUpdate(DisplayType dpy, bool enable) override;
    ScopedAStatus toggleScreenUpdate(bool on) override;
    ScopedAStatus setIdleTimeout(int value) override;
    ScopedAStatus getHDRCapabilities(DisplayType dpy, HDRCapsParams* aidl_return) override;
    ScopedAStatus setCameraLaunchStatus(int on) override;
    ScopedAStatus displayBWTransactionPending(bool* status) override;
    ScopedAStatus setDisplayAnimating(long displayId, bool animating) override;
    ScopedAStatus controlIdlePowerCollapse(bool enable, bool synchronous) override;
    ScopedAStatus getWriteBackCapabilities(bool* is_wb_ubwc_supported);
    ScopedAStatus setDisplayDppsAdROI(int displayId, int hStart, int hEnd, int vStart, int vEnd,
                                      int factorIn, int factorOut) override;
    ScopedAStatus updateVSyncSourceOnPowerModeOff() override;
    ScopedAStatus updateVSyncSourceOnPowerModeDoze() override;
    ScopedAStatus setPowerMode(int dispId, PowerMode powerMode) override;
    ScopedAStatus isPowerModeOverrideSupported(int dispId, bool* supported) override;
    ScopedAStatus isHDRSupported(int dispId, bool* supported) override;
    ScopedAStatus isWCGSupported(int dispId, bool* supported) override;
    ScopedAStatus setLayerAsMask(int dispId, long layerId) override;
    ScopedAStatus getDebugProperty(const std::string& propName, std::string* value) override;
    ScopedAStatus getActiveBuiltinDisplayAttributes(Attributes* attr) override;
    ScopedAStatus setPanelLuminanceAttributes(int dispId, float minLum, float maxLum) override;
    ScopedAStatus isBuiltInDisplay(int dispId, bool* _aidl_return) override;
    ScopedAStatus isAsyncVDSCreationSupported(bool* _aidl_return) override;
    ScopedAStatus createVirtualDisplay(int width, int height, int format) override;
    ScopedAStatus getSupportedDSIBitClks(int dispId, std::vector<long>* bitClks) override;
    ScopedAStatus getDSIClk(int dispId, long* bitClk) override;
    ScopedAStatus setDSIClk(int dispId, long bitClk) override;
    ScopedAStatus setQsyncMode(int dispId, QsyncMode mode) override;
    ScopedAStatus isSmartPanelConfig(int dispId, int configId, bool* isSmart) override;
    ScopedAStatus isRotatorSupportedFormat(int halFormat, bool ubwc, bool* supported) override;
    ScopedAStatus controlQsyncCallback(bool enable) override;
    ScopedAStatus sendTUIEvent(DisplayType dpy, TUIEventType eventType) override;
    ScopedAStatus getDisplayHwId(int dispId, int* displayHwId) override;
    ScopedAStatus getSupportedDisplayRefreshRates(DisplayType dpy,
                                                 std::vector<int>* supportedRefreshRates) override;
    ScopedAStatus isRCSupported(int dispId, bool* supported) override;
    ScopedAStatus controlIdleStatusCallback(bool enable) override;
    ScopedAStatus isSupportedConfigSwitch(int dispId, int config, bool* supported) override;
    ScopedAStatus getDisplayType(long physicalDispId, DisplayType* displayType) override;
    ScopedAStatus setCWBOutputBuffer(
          const std::shared_ptr<IDisplayConfigCallback>& in_callback,
          int32_t in_dispId, const Rect& in_rect, bool in_postProcessed,
          const ::aidl::android::hardware::common::NativeHandle& in_buffer) override;
    ScopedAStatus setCameraSmoothInfo(CameraSmoothOp in_op, int32_t in_fps);
    ScopedAStatus registerCallback(const std::shared_ptr<IDisplayConfigCallback>& in_callback,
                                    int64_t* _aidl_return);
    ScopedAStatus unRegisterCallback(int64_t in_handle);
    ScopedAStatus notifyDisplayIdleState(const std::vector<int32_t>& displayIds) {
      return ScopedAStatus::ok();
    }

  private:
    sdm::HWCSession *hwc_session_;
    std::weak_ptr<DisplayConfig::ConfigCallback> callback_;
};

} // namespace config
} // namespace display
} // namespace hardware
} // namespace qti
} // namespace vendor
} // namespace aidl
