/*
*Copyright (c) 2020, The Linux Foundation. All rights reserved.
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
*    * Neither the name of The Linux Foundation nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*
*THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
*WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
*MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
*ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
*BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
*BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
*WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
*OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
*IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <vendor/qti/hardware/display/demura/1.0/IDemuraFileFinder.h>
#include <hidl/Status.h>
#include <hidl/MQDescriptor.h>
#include <binder/ProcessState.h>
#include <hidl/LegacySupport.h>
#include <log/log.h>
#include "demura_file_finder.h"


using vendor::qti::hardware::display::demura::V1_0::IDemuraFileFinder;
using vendor::qti::hardware::display::demura::V1_0::implementation::DemuraFileFinder;

using android::sp;
using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;

using android::status_t;
using android::OK;

int main() {
    android::ProcessState::initWithDriver("/dev/vndbinder");
    android::sp<IDemuraFileFinder> demura_file_finder = DemuraFileFinder::GetInstance();
    if (!demura_file_finder) {
      ALOGE("Could not create the IDemuraFileFinder, not registering process!!");
      return -1;
    }
    configureRpcThreadpool(1, true /*callerWillJoin*/);
    if (demura_file_finder->registerAsService() != OK) {
        ALOGE("Cannot register DemuraFileFinder service");
        return -EINVAL;
    }
    joinRpcThreadpool();
    return 0;
}
