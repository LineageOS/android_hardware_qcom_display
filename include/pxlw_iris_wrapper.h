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

private:
    PxlwIrisWrapper() = default;
};

class PxlwSoftirisWrapper {
public:
    PxlwSoftirisWrapper();
    virtual ~PxlwSoftirisWrapper();

    bool HasSoftIris();
    int InitPrimaryDisplay(int32_t vsync_period_ns, uint32_t width, uint32_t height);
    void SetColorModeWithRenderIntent(int32_t display_id, int32_t,
                                    android::hardware::graphics::common::V1_2::ColorMode colorMode,
                                    android::hardware::graphics::common::V1_1::RenderIntent renderIntent);

private:
    friend class PxlwIrisWrapper;
};

} // namespace pxlw
