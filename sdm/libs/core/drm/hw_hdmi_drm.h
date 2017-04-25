/*
* Copyright (c) 2017, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted
* provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright notice, this list of
*      conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright notice, this list of
*      conditions and the following disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation nor the names of its contributors may be used to
*      endorse or promote products derived from this software without specific prior written
*      permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __HW_HDMI_DRM_H__
#define __HW_HDMI_DRM_H__

#include <map>
#include <vector>

#include "hw_device_drm.h"

namespace sdm {

using std::vector;

class HWHDMIDRM : public HWDeviceDRM {
 public:
  explicit HWHDMIDRM(BufferSyncHandler *buffer_sync_handler, BufferAllocator *buffer_allocator,
                     HWInfoInterface *hw_info_intf);

 protected:
  virtual DisplayError Init();
  virtual DisplayError GetNumDisplayAttributes(uint32_t *count);
  // Requirement to call this only after the first config has been explicitly set by client
  virtual DisplayError GetActiveConfig(uint32_t *active_config);
  virtual DisplayError SetDisplayAttributes(uint32_t index);
  virtual DisplayError GetConfigIndex(uint32_t mode, uint32_t *index);
  virtual DisplayError Validate(HWLayers *hw_layers);
  virtual DisplayError Commit(HWLayers *hw_layers);

 private:
  uint32_t active_config_index_;
  uint32_t frame_rate_ = 0;
};

}  // namespace sdm

#endif  // __HW_HDMI_DRM_H__

