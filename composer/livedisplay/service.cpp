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

#include <hidl/HidlTransportSupport.h>

#include "SunlightEnhancement.h"

using ::android::OK;
using ::android::sp;
using ::android::status_t;

using ::vendor::lineage::livedisplay::V2_0::ISunlightEnhancement;
using ::vendor::lineage::livedisplay::V2_0::drm::SunlightEnhancement;

namespace sdm {

status_t DisplayInit(sdm::HWCSession *hwc_session) {
    status_t status = OK;

    sp<ISunlightEnhancement> se = new SunlightEnhancement(hwc_session);
    status = se->registerAsService();
    if (status != OK) {
        ALOGE("Could not register service for LiveDisplay HAL SunlightEnhancement Iface (%s)",
                android::statusToString(status).c_str());
        return status;
    }

    return OK;
}

}  // namespace sdm
