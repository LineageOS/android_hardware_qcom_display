/*
 *Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
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

#include <string>
#include "demura_file_finder.h"

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace demura {
namespace V2_0 {
namespace implementation {

using sdm::FileFinderInterface;
using sdm::GenericPayload;
using sdm::kFileFinderFileData;

IDemuraFileFinder *DemuraFileFinder::file_finder_ = NULL;
FileFinderInterface *DemuraFileFinder::file_intf_ = NULL;
DestroyFileFinderIntf DemuraFileFinder::destroy_ff_intf_ = NULL;

IDemuraFileFinder *DemuraFileFinder::GetInstance() {
  if (!file_finder_) {
    sdm::DynLib demura_file_intf_lib;
    if (!demura_file_intf_lib.Open(OEM_FILE_FINDER_LIB_NAME)) {
      ALOGE("Failed to load lib %s", OEM_FILE_FINDER_LIB_NAME);
      return nullptr;
    }

    GetFileFinderIntf get_ff_intf = NULL;

    if (!demura_file_intf_lib.Sym(GET_FILE_FINDER_INTF_NAME,
                                  reinterpret_cast<void **>(&(get_ff_intf)))) {
      ALOGE("Unable to load symbols, err %s", demura_file_intf_lib.Error());
      return nullptr;
    }
    if (!demura_file_intf_lib.Sym(DESTROY_FILE_FINDER_INTF_NAME,
                                  reinterpret_cast<void **>(&(destroy_ff_intf_)))) {
      ALOGE("Unable to load symbols, err %s", demura_file_intf_lib.Error());
      return nullptr;
    }

    file_intf_ = get_ff_intf();
    if (!file_intf_) {
      ALOGE("Failed to get FileFinder Interface!");
      return nullptr;
    }

    int error = file_intf_->Init();
    if (error) {
      ALOGE("Failed to initalize FileFinderInterface error = %d", error);
      return nullptr;
    }
    file_finder_ = new DemuraFileFinder();
  }

  return file_finder_;
}

DemuraFileFinder::~DemuraFileFinder() {
  if (file_intf_) {
    file_intf_->Deinit();
    if (destroy_ff_intf_) {
      destroy_ff_intf_();
    }
  }
}

Return<void> DemuraFileFinder::getDemuraFilePaths(uint64_t panel_id,
                                                  getDemuraFilePaths_cb _hidl_cb) {
  int ret = 0;
  DemuraFilePaths file_paths = {};

  if (!file_intf_) {
    ALOGE("file_intf_ was not initialized, or not found. Command not supported");
    ret = -EINVAL;
    _hidl_cb(ret, file_paths);
    return Void();
  }

  if (panel_id == UINT64_MAX) {
    ret = -EINVAL;
    _hidl_cb(ret, file_paths);
    return Void();
  }

  uint64_t *input = nullptr;
  DemuraFilePaths *out_paths = nullptr;
  GenericPayload in;
  GenericPayload out;

  int error = 0;
  if ((error = in.CreatePayload(input))) {
    ret = -ENOMEM;
    ALOGE("Failed to create input payload error = %d", error);
    _hidl_cb(ret, file_paths);
    return Void();
  }

  if ((error = out.CreatePayload(out_paths))) {
    ret = -ENOMEM;
    ALOGE("Failed to create output payload error = %d", error);
    _hidl_cb(ret, file_paths);
    return Void();
  }

  *input = panel_id;
  if ((error = file_intf_->ProcessOps(kFileFinderFileData, in, &out))) {
    ret = error;
    ALOGE("Failed to process ops error = %d", error);
    _hidl_cb(ret, file_paths);
    return Void();
  }

  file_paths = *out_paths;
  ALOGI("Demura file paths : config file %s signature file %s public key file %s",
        file_paths.configFilePath.c_str(), file_paths.signatureFilePath.c_str(),
        file_paths.publickeyFilePath.c_str());

  _hidl_cb(ret, file_paths);

  return Void();
}

}  // namespace implementation
}  // namespace V2_0
}  // namespace demura
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
