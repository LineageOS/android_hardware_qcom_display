/*
 * Copyright 2022 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef __HWCCOMMON_H__
#define __HWCCOMMON_H__

#include "aidl/vendor/qti/hardware/display/composer3/QtiLayerFlags.h"
#pragma once

#include <aidl/android/hardware/graphics/composer3/BnComposerClient.h>
#include <aidl/android/hardware/graphics/composer3/DisplayConfiguration.h>
#include <aidl/vendor/qti/hardware/display/composer3/BnQtiComposer3Client.h>
#include <aidl/android/hardware/graphics/composer3/Capability.h>
#include <aidl/android/hardware/graphics/common/ColorTransform.h>
#include <functional>
#include <string>

// TODO: Change to HWC3 namespace
namespace sdm {

using aidl::android::hardware::graphics::common::AlphaInterpretation;
using aidl::android::hardware::graphics::common::BlendMode;
using aidl::android::hardware::graphics::common::ColorTransform;
using aidl::android::hardware::graphics::common::Dataspace;
using aidl::android::hardware::graphics::common::FRect;
using aidl::android::hardware::graphics::common::Rect;
using aidl::android::hardware::graphics::common::Transform;
using aidl::android::hardware::graphics::composer3::Capability;
using aidl::android::hardware::graphics::composer3::ColorMode;
using aidl::android::hardware::graphics::composer3::Composition;
using aidl::android::hardware::graphics::composer3::DisplayRequest;
using aidl::android::hardware::graphics::composer3::FormatColorComponent;
using aidl::android::hardware::graphics::composer3::IComposerClient;
using aidl::android::hardware::graphics::composer3::OverlayProperties;
using aidl::android::hardware::graphics::composer3::PowerMode;
using aidl::android::hardware::graphics::composer3::VsyncPeriodChangeTimeline;

using aidl::vendor::qti::hardware::display::composer3::QtiDisplayCommand;
using DrawMethod = aidl::vendor::qti::hardware::display::composer3::QtiDrawMethod;
using LayerFlag = aidl::vendor::qti::hardware::display::composer3::QtiLayerFlags;
using LayerType = aidl::vendor::qti::hardware::display::composer3::QtiLayerType;

using FColor = aidl::android::hardware::graphics::composer3::Color;

using onHotplug_func_t = std::function<void(void *, int64_t, bool)>;
using onRefresh_func_t = std::function<void(void *, int64_t)>;
using onSeamlessPossible_func_t = onRefresh_func_t;
using onVsync_func_t = std::function<void(void *, int64_t, int64_t, int32_t)>;
using onVsyncPeriodTimingChanged_func_t =
    std::function<void(void *, int64_t, const VsyncPeriodChangeTimeline &)>;
using onVsyncIdle_func_t = std::function<void(void *, int64_t)>;

typedef uint64_t Display;
typedef uint32_t Config;
typedef int64_t LayerId;

struct Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a;
};

struct Region {
  size_t num_rects;
  Rect const *rects;
};

namespace HWC3 {
enum class Error : int32_t {
  None = 0,
  BadConfig = IComposerClient::EX_BAD_CONFIG,
  BadDisplay = IComposerClient::EX_BAD_DISPLAY,
  BadLayer = IComposerClient::EX_BAD_LAYER,
  BadParameter = IComposerClient::EX_BAD_PARAMETER,
  HasChanges,
  NoResources = IComposerClient::EX_NO_RESOURCES,
  NotValidated = IComposerClient::EX_NOT_VALIDATED,
  Unsupported = IComposerClient::EX_UNSUPPORTED,
  SeamlessNotAllowed = IComposerClient::EX_SEAMLESS_NOT_ALLOWED,
};
}  // namespace HWC3

enum CallbackCommand {
  CALLBACK_INVALID = 0,
  CALLBACK_HOTPLUG = 1,
  CALLBACK_REFRESH = 2,
  CALLBACK_VSYNC = 3,
  CALLBACK_VSYNC_PERIOD_TIMING_CHANGED = 5,
  CALLBACK_SEAMLESS_POSSIBLE = 6,
  CALLBACK_VSYNC_IDLE = 7,
};

enum class DisplayBasicType : int32_t {
  kInvalid = 0,
  kPhysical = 1,
  kVirtual = 2,
};

/* Display types and associated mask bits. */
enum {
  HWC_DISPLAY_PRIMARY = 0,
  HWC_DISPLAY_EXTERNAL = 1,  // HDMI, DP, etc.

  HWC_DISPLAY_EXTERNAL_2 = 2,
  HWC_DISPLAY_EXTERNAL_3 = 3,
  HWC_DISPLAY_EXTERNAL_4 = 4,

  HWC_DISPLAY_BUILTIN_2 = 5,
  HWC_DISPLAY_BUILTIN_3 = 6,
  HWC_DISPLAY_BUILTIN_4 = 7,

  HWC_DISPLAY_VIRTUAL = 8,

  HWC_NUM_PHYSICAL_DISPLAY_TYPES = 8,
  HWC_NUM_DISPLAY_TYPES = 9,
};

std::string to_string(HWC3::Error error);
std::string to_string(PowerMode mode);
std::string to_string(Composition composition);

}  // namespace sdm

#endif  // __HWCCOMMON_H__
