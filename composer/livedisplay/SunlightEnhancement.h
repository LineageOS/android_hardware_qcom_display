/*
 * Copyright (C) 2019-2020 The LineageOS Project
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

#pragma once

#include <vendor/lineage/livedisplay/2.0/ISunlightEnhancement.h>

#include "../hwc_session.h"

namespace vendor {
namespace lineage {
namespace livedisplay {
namespace V2_0 {
namespace drm {

using ::android::hardware::Return;

class SunlightEnhancement : public ISunlightEnhancement {
  public:
    explicit SunlightEnhancement(sdm::HWCSession *hwc_session);

    // Methods from ::vendor::lineage::livedisplay::V2_0::ISunlightEnhancement follow.
    Return<bool> isEnabled() override;
    Return<bool> setEnabled(bool enabled) override;

  private:
    sdm::HWCSession* const hwc_session_;
};

}  // namespace drm
}  // namespace V2_0
}  // namespace livedisplay
}  // namespace lineage
}  // namespace vendor
