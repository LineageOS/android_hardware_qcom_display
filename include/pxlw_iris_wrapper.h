/*
 * SPDX-FileCopyrightText: 2025 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

namespace android {
namespace hardware {
namespace graphics {
namespace common {
namespace V1_1 {
enum class RenderIntent : int32_t;
}
namespace V1_2 {
enum class ColorMode : int32_t;
}
}
}
}
}

namespace pxlw {

class PxlwIrisWrapper {
public:
    static PxlwIrisWrapper* GetInstance();
    static int GetIrisDisplayType(int param_1, int param_2);

private:
    PxlwIrisWrapper() = default;
};

class PxlwSoftirisWrapper {
public:
    PxlwSoftirisWrapper();
    virtual ~PxlwSoftirisWrapper();

    bool HasSoftIris();
    int InitPrimaryDisplay(int32_t vsync_period_ns, uint32_t width, uint32_t height);
    int SetActiveConfig(uint32_t width, uint32_t height, sdm::DisplayConfigVariableInfo *variable_info);
    int SetActiveConfig(uint32_t width, bool height, uint32_t alwaysInBypassMode, sdm::DisplayConfigVariableInfo *variable_info);
    void SetColorModeWithRenderIntent(int32_t display_id, int32_t param_2, 
                                    android::hardware::graphics::common::V1_2::ColorMode colorMode,
                                    android::hardware::graphics::common::V1_1::RenderIntent renderIntent);
    int BeforeSetPowerMode(unsigned long param_1, int param_2, bool param_3);
    int AfterSetPowerMode(unsigned long param_1, int param_2, bool param_3);
    int SetPanelBrightness(int param_1, int param_2, float param_3);
    int BeforeCommitLayerStack(int param_1, int param_2, int& param_3, int& param_4, int& param_5);
    int AfterCommitLayerStack(int param_1, int param_2, int param_3, int param_4);
    int Dump(int param_1, int param_2, int param_3);

private:
    friend class PxlwIrisWrapper;
};

} // namespace pxlw
