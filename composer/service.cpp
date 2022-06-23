/*
 * Copyright (c) 2019, 2021 The Linux Foundation. All rights reserved.
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

/*
Changes from Qualcomm Innovation Center are provided under the following license:
Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the
disclaimer below) provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of Qualcomm Innovation Center, Inc. nor the
      names of its contributors may be used to endorse or promote
      products derived from this software without specific prior
      written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <hidl/LegacySupport.h>

#include "DisplayConfigAIDL.h"
#include "QtiComposer.h"
#include "hwc_debugger.h"

using aidl::vendor::qti::hardware::display::config::DisplayConfigAIDL;
using android::ProcessState;
using android::sp;
using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;
using android::hardware::graphics::composer::V2_3::IComposer;
using vendor::qti::hardware::display::composer::V3_1::IQtiComposer;
using vendor::qti::hardware::display::composer::V3_1::implementation::QtiComposer;

int main(int, char **) {
  ALOGI("Creating Display HW Composer HAL");

  int composer_thread_count = 4;
  sdm::HWCDebugHandler::Get()->GetProperty(COMPOSER_THREAD_COUNT, &composer_thread_count);
  if (composer_thread_count < 4 || composer_thread_count > 15) {
    composer_thread_count = 4;
  }
  ALOGI("composer_thread_count: %d", composer_thread_count);

  // TODO(user): double-check for SCHED_FIFO logic
  // the conventional HAL might start binder services
  ProcessState::initWithDriver("/dev/vndbinder");
  sp<ProcessState> ps(ProcessState::self());
  ps->setThreadPoolMaxThreadCount(composer_thread_count);
  ps->startThreadPool();
  ALOGI("ProcessState initialization completed");

  // same as SF main thread
  struct sched_param param = {0};
  param.sched_priority = 2;
  if (sched_setscheduler(0, SCHED_FIFO | SCHED_RESET_ON_FORK, &param) != 0) {
    ALOGE("Couldn't set SCHED_FIFO: %d", errno);
  } else {
    ALOGI("Scheduler priority settings completed");
  }

  ALOGI("Initializing QtiComposer");
  sp<IQtiComposer> composer = QtiComposer::initialize();
  if (composer == nullptr) {
    ALOGE("Initializing QtiComposer...failed!");
    return -EINVAL;
  } else {
    ALOGI("Initializing QtiComposer...done!");
  }

  ALOGI("Configuring RPC threadpool");
  configureRpcThreadpool(composer_thread_count, true /*callerWillJoin*/);
  ALOGI("Configuring RPC threadpool...done!");

  ALOGI("Registering Display HW Composer HAL as a service");
  if (composer->registerAsService() != android::OK) {
    ALOGE("Registering Display HW Composer HAL as a service...failed!");
    return -EINVAL;
  }
  ALOGI("Registering Display HW Composer HAL as a service...done!");

  ALOGI("Registering DisplayConfig AIDL as a service");
  ABinderProcess_setThreadPoolMaxThreadCount(0);
  std::shared_ptr<DisplayConfigAIDL> displayConfig = ndk::SharedRefBase::make<DisplayConfigAIDL>();
  const std::string instance = std::string() + DisplayConfigAIDL::descriptor + "/default";
  if (!displayConfig->asBinder().get()) {
    ALOGW("Display Config AIDL's binder is null");
  }

  binder_status_t status = AServiceManager_addService(displayConfig->asBinder().get(),
                                                      instance.c_str());
  if (status != STATUS_OK) {
    ALOGW("Failed to register DisplayConfig AIDL as a service (status:%d)", status);
  } else {
    ALOGI("Successfully registered DisplayConfig AIDL as a service");
  }

  ALOGI("Joining RPC threadpool...");
  ABinderProcess_joinThreadPool();

  return 0;
}
