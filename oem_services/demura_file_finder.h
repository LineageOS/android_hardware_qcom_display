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

#ifndef __DEMURA_FILE_FINDER_H__
#define __DEMURA_FILE_FINDER_H__

#include <vendor/qti/hardware/display/demura/2.0/IDemuraFileFinder.h>
#include <hidl/MQDescriptor.h>
#include <hidl/Status.h>
#include <log/log.h>
#include <utils/sys.h>
#include <file_finder_interface.h>

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace demura {
namespace V2_0 {
namespace implementation {
using ::android::hardware::Return;
using ::android::hardware::Void;
using sdm::FileFinderInterface;
using ::vendor::qti::hardware::display::demura::V2_0::IDemuraFileFinder;

#define OEM_FILE_FINDER_LIB_NAME "libfilefinder.so"
#define GET_FILE_FINDER_INTF_NAME "GetFileFinderIntf"
#define DESTROY_FILE_FINDER_INTF_NAME "DestroyFileFinderIntf"
typedef FileFinderInterface *(*GetFileFinderIntf)();
typedef void *(*DestroyFileFinderIntf)();

class DemuraFileFinder : public IDemuraFileFinder {
 public:
  virtual ~DemuraFileFinder();
  static IDemuraFileFinder *GetInstance();
  static FileFinderInterface *file_intf_;
  static IDemuraFileFinder *file_finder_;
  static DestroyFileFinderIntf destroy_ff_intf_;

  // IDemuraFileFinder
  Return<void> getDemuraFilePaths(uint64_t panel_id, getDemuraFilePaths_cb _hidl_cb) override;

 private:
};

}  // namespace implementation
}  // namespace V2_0
}  // namespace demura
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor

#endif  // __DEMURA_FILE_FINDER_H__
