/*
 * Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
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
 *
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <hidl/LegacySupport.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <aidlcommonsupport/NativeHandle.h>

#include <string>

#include "QtiAllocator.h"
#include "QtiAllocatorAIDL.h"

using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;
using IQtiAllocator4 = vendor::qti::hardware::display::allocator::V4_0::IQtiAllocator;
using aidl::android::hardware::graphics::allocator::impl::QtiAllocatorAIDL;

int main(int, char **) {
  // same as SF main thread
  struct sched_param param = {0};
  param.sched_priority = 2;
  if (sched_setscheduler(0, SCHED_FIFO | SCHED_RESET_ON_FORK, &param) != 0) {
    ALOGI("%s: failed to set priority: %s", __FUNCTION__, strerror(errno));
  }

  ALOGI("Registering qti-allocator 4");
  android::sp<IQtiAllocator4> service4 =
      new vendor::qti::hardware::display::allocator::V4_0::implementation::QtiAllocator();
  if (service4->registerAsService() != android::OK) {
    ALOGE("Cannot register QTI Allocator 4 service");
    return -EINVAL;
  }
  ALOGI("Initialized qti-allocator 4");

  ALOGI("Registering QTI Allocator AIDL as a service");
  auto allocator = ndk::SharedRefBase::make<QtiAllocatorAIDL>();
  const std::string instance = std::string() + QtiAllocatorAIDL::descriptor + "/default";
  if (!allocator->asBinder().get()) {
    ALOGW("QTI Allocator AIDL's binder is null");
    return EXIT_FAILURE;
  }

  binder_status_t status = AServiceManager_addServiceWithFlags(
      allocator->asBinder().get(), instance.c_str(),
      AServiceManager_AddServiceFlag::ADD_SERVICE_ALLOW_ISOLATED);
  if (status != STATUS_OK) {
    ALOGW("Failed to register QTI Allocator AIDL as a service (status:%d)", status);
    return EXIT_FAILURE;
  }
  ALOGI("Initialized QTI Allocator AIDL");

  ABinderProcess_setThreadPoolMaxThreadCount(4);
  ABinderProcess_startThreadPool();
  ABinderProcess_joinThreadPool();

  return 0;
}
