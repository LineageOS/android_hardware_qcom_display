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

#include <sstream>
#include <iomanip>
#include <string>
#include "file_finder_oem_extension.h"

#define __CLASS__ "FileFinderOemExtn"

#define LOCAL_SOURCE_PATH "/mnt/vendor/persist/display/"

#define FILE_CHUNK_SIZE 8192

namespace sdm {

FileFinderOemExtn *FileFinderOemExtn::file_finder_ = NULL;
uint32_t FileFinderOemExtn::ref_count_ = 0;
std::mutex FileFinderOemExtn::lock_ = {};

FileFinderInterface *GetFileFinderIntf() {
  std::lock_guard<std::mutex> g(FileFinderOemExtn::lock_);
  if (FileFinderOemExtn::file_finder_ == NULL) {
    FileFinderOemExtn::file_finder_ = new FileFinderOemExtn();
  }
  FileFinderOemExtn::ref_count_++;
  return FileFinderOemExtn::file_finder_;
}

void DestroyFileFinderIntf() {
  std::lock_guard<std::mutex> g(FileFinderOemExtn::lock_);
  FileFinderOemExtn::ref_count_--;
  if (FileFinderOemExtn::ref_count_ == 0) {
    delete FileFinderOemExtn::file_finder_;
    FileFinderOemExtn::file_finder_ = NULL;
  }
}

FileFinderOemExtn::FileFinderOemExtn() {
  std::function<int(FileFinderOemExtn *, const GenericPayload &, GenericPayload *)> ffd =
      &FileFinderOemExtn::FindFileData;
  ops_fcns_.emplace(kFileFinderFileData, ffd);
}

int FileFinderOemExtn::Init() {
  return 0;
}

int FileFinderOemExtn::Deinit() {
  return 0;
}

int FileFinderOemExtn::SetParameter(FileFinderParams param, const GenericPayload &in) {
  (void)param;
  (void)in;
  DLOGE("SetParameter on param %d not supported", param);
  return -ENOTSUP;
}

int FileFinderOemExtn::GetParameter(FileFinderParams param, GenericPayload *out) {
  (void)param;
  (void)out;
  DLOGE("GetParameter on param %d not supported", param);
  return -ENOTSUP;
}

int FileFinderOemExtn::ProcessOps(FileFinderOps op, const GenericPayload &in, GenericPayload *out) {
  if (out == NULL) {
    return -EINVAL;
  }

  if (op < kFileFinderOpMax) {
    auto fcn = ops_fcns_.at(op);
    return fcn(this, in, out);
  } else {
    DLOGE("Invalid op %d", op);
    return -EINVAL;
  }
}

int FileFinderOemExtn::FindFileData(const GenericPayload &in, GenericPayload *out) {
  int status = 0;
  uint32_t sz = 0;
  uint64_t *panel_id = nullptr;
  std::string panel_id_hex_str = "";
  DemuraFilePaths *file_paths = nullptr;

  status = in.GetPayload(panel_id, &sz);
  if ((status != 0) || sz != 1) {
    return -EINVAL;
  }
  std::stringstream temp;
  uint64_t id = *panel_id;
  temp << std::setfill('0') << std::setw(16) << std::hex << id << std::dec;
  panel_id_hex_str = temp.str();

  status = out->GetPayload(file_paths, &sz);
  if ((status != 0) || sz != 1) {
    return -EINVAL;
  }

  errno = 0;

  *file_paths = getSrcFilePaths(panel_id_hex_str);

  if (file_paths->configFilePath.empty() || file_paths->signatureFilePath.empty() ||
      file_paths->publickeyFilePath.empty()) {
    DLOGE("Demura missing file - config file %d signature file %d public key file %d",
          file_paths->configFilePath.empty(), file_paths->signatureFilePath.empty(),
          file_paths->publickeyFilePath.empty());
    return -EINVAL;
  }

  return 0;
}

DemuraFilePaths FileFinderOemExtn::getSrcFilePaths(const std::string &panel_id_hex_str) {
  DemuraFilePaths paths = {};
  FILE *calib_file = NULL;
  FILE *signature_file = NULL;
  FILE *publickey_file = NULL;

  // Build path strings and check if the file is available
  std::string sp = LOCAL_SOURCE_PATH;
  errno = 0;
  std::string src_path_calib = sp + "demura_config_" + panel_id_hex_str;
  calib_file = fopen(src_path_calib.c_str(), "rb");

  std::string src_path_sig = sp + "demura_signature_" + panel_id_hex_str;
  signature_file = fopen(src_path_sig.c_str(), "rb");

  std::string src_path_pk = sp + "demura_publickey_" + panel_id_hex_str;
  publickey_file = fopen(src_path_pk.c_str(), "rb");

  // Get files OTA if any file is missing
  if (calib_file == NULL || signature_file == NULL || publickey_file == NULL) {
    DLOGW("Failed to open files locally, attempting OTA");
    paths = getFileOTA(panel_id_hex_str);
    errno = 0;
    calib_file = fopen(paths.configFilePath.c_str(), "rb");
    if (calib_file == NULL) {
      paths.configFilePath = "";
      DLOGE("Failed to open file after OTA at %s. Error = %s", paths.configFilePath.c_str(),
            strerror(errno));
    }
    signature_file = fopen(paths.signatureFilePath.c_str(), "rb");
    if (signature_file == NULL) {
      paths.signatureFilePath = "";
      DLOGE("Failed to open file after OTA at %s. Error = %s", paths.signatureFilePath.c_str(),
            strerror(errno));
    }
    publickey_file = fopen(paths.publickeyFilePath.c_str(), "rb");
    if (publickey_file == NULL) {
      paths.publickeyFilePath = "";
      DLOGE("Failed to open file after OTA at %s. Error = %s", paths.publickeyFilePath.c_str(),
            strerror(errno));
    }
  } else {
    paths.configFilePath = src_path_calib;
    paths.signatureFilePath = src_path_sig;
    paths.publickeyFilePath = src_path_pk;
  }

  if (calib_file != NULL) {
    fclose(calib_file);
  }
  if (signature_file != NULL) {
    fclose(signature_file);
  }
  if (publickey_file != NULL) {
    fclose(publickey_file);
  }

  return paths;
}

DemuraFilePaths FileFinderOemExtn::getFileOTA(const std::string &panel_id_hex_str) {
  /*
   * Communication to a server shall begin here.
   * API shall securely contact the server and download the data to a file on the device
   * and the location must accessible for both read and write by this process.
   * This API shall return the paths of the files to the caller.
   */
  DemuraFilePaths paths = {};
  return paths;
}

}  // namespace sdm
