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

#ifndef __FILE_FINDER_OEM_EXTENSION_H__
#define __FILE_FINDER_OEM_EXTENSION_H__

#include <vendor/qti/hardware/display/demura/2.0/IDemuraFileFinder.h>
#include <log/log.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <mutex>
#include <map>
#include <string>
#include "file_finder_interface.h"
#include "debug_handler.h"

using ::vendor::qti::hardware::display::demura::V2_0::IDemuraFileFinder;
using DemuraFilePaths = IDemuraFileFinder::DemuraFilePaths;

namespace sdm {

class FileFinderOemExtn : public FileFinderInterface {
 public:
  virtual ~FileFinderOemExtn() {}
  FileFinderOemExtn();
  int Init();
  int Deinit();
  int SetParameter(FileFinderParams param, const GenericPayload &in);
  int GetParameter(FileFinderParams param, GenericPayload *out);
  int ProcessOps(FileFinderOps op, const GenericPayload &in, GenericPayload *out);
  static FileFinderOemExtn *file_finder_;
  static uint32_t ref_count_;
  static std::mutex lock_;

 private:
  int FindFileData(const GenericPayload &in, GenericPayload *out);
  DemuraFilePaths getSrcFilePaths(const std::string &panel_id_hex_str);
  DemuraFilePaths getFileOTA(const std::string &panel_id_hex_str);
  std::map<FileFinderOps,
           std::function<int(FileFinderOemExtn *, const GenericPayload &, GenericPayload *)>>
      ops_fcns_;
};
}  // namespace sdm

#endif  // __FILE_FINDER_OEM_EXTENSION_H__
