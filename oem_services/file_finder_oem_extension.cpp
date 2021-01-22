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

#include <sstream>
#include <iomanip>
#include <string>
#include "file_finder_oem_extension.h"

#define __CLASS__ "FileFinderOemExtn"

#define DESTINATION_PATH "/data/vendor/display/demura/"
#define LOCAL_SOURCE_PATH "/vendor/etc/"
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
  FILE *file_in = nullptr;
  FILE *file_out = nullptr;
  std::string *file_path = nullptr;
  std::string file_path_out = "";

  status = in.GetPayload(panel_id, &sz);
  if ((status != 0) || sz != 1) {
    return -EINVAL;
  }
  std::stringstream temp;
  uint64_t id = *panel_id;
  temp << std::setfill('0') << std::setw(16) << std::hex << id << std::dec;
  panel_id_hex_str = temp.str();

  status = out->GetPayload(file_path, &sz);
  if ((status != 0) || sz != 1) {
    return -EINVAL;
  }

  file_in = getSrcFile(panel_id_hex_str);
  if (!file_in) {
    DLOGE("Did not get a correction file");
    *file_path = "";
    return -EINVAL;
  }

  errno = 0;
  status = mkdir(DESTINATION_PATH, 755);
  if ((status != 0) && errno != EEXIST) {
    DLOGE("Failed to create %s directory errno = %d, desc = %s", DESTINATION_PATH, errno,
          strerror(errno));
    fclose(file_in);
    return -EPERM;
  }

  // Even if directory exists already, need to explicitly change the permission.
  if (chmod(DESTINATION_PATH, 0755) != 0) {
    DLOGE("Failed to change permissions on %s directory", DESTINATION_PATH);
    fclose(file_in);
    return -EACCES;
  }

  std::string ds = DESTINATION_PATH;
  file_path_out = ds + "demura_config_" + panel_id_hex_str;
  file_out = fopen(file_path_out.c_str(), "wb+");
  if (file_out == NULL) {
    DLOGI("cannot create file for writing in /data\n");
    fclose(file_in);
    return -ENOENT;
  }

  // Read and write contents from file
  size_t bytes = 0;
  uint8_t buffer[FILE_CHUNK_SIZE] = {};
  while ((bytes = fread(buffer, 1, sizeof(buffer), file_in)) > 0) {
    fwrite(buffer, 1, bytes, file_out);
  }

  fclose(file_in);
  fclose(file_out);

  *file_path = file_path_out;
  return 0;
}

FILE *FileFinderOemExtn::getSrcFile(const std::string &panel_id_hex_str) {
  FILE *file = NULL;
  std::string sp = LOCAL_SOURCE_PATH;
  std::string src_path = sp + "demura_config_" + panel_id_hex_str;
  errno = 0;
  file = fopen(src_path.c_str(), "rb");
  if (file == NULL) {
    DLOGW("Failed to open file locally at %s. Error = %s", src_path.c_str(), strerror(errno));
    src_path = getFileOTA(panel_id_hex_str);
    errno = 0;
    file = fopen(src_path.c_str(), "rb");
    if (file == NULL) {
      DLOGE("Failed to open file after OTA at %s. Error = %s", src_path.c_str(), strerror(errno));
    }
  }

  return file;
}

std::string FileFinderOemExtn::getFileOTA(const std::string &panel_id_hex_str) {
  /*
   * Communication to a server shall begin here.
   * API shall securely contact the server and download the data to a file on the device
   * and the location must accessible for both read and write by this process.
   * This API shall return the path to the file to the caller.
   *
   * The stub impl of this API simply returns the local path the caller would have already checked
   * before requesting server download of the file.
   */
  std::string sp = LOCAL_SOURCE_PATH;
  return sp + "demura_config_" + panel_id_hex_str;
}

}  // namespace sdm
