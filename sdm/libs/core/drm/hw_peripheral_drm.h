/*
Copyright (c) 2017-2018, 2020, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __HW_PERIPHERAL_DRM_H__
#define __HW_PERIPHERAL_DRM_H__

#include <vector>
#include <string>
#include "hw_device_drm.h"

namespace sdm {

class HWPeripheralDRM : public HWDeviceDRM {
 public:
  explicit HWPeripheralDRM(int32_t display_id, BufferSyncHandler *buffer_sync_handler,
                           BufferAllocator *buffer_allocator,
                           HWInfoInterface *hw_info_intf);
  virtual ~HWPeripheralDRM() {}

 protected:
  virtual DisplayError Init();
  virtual DisplayError Validate(HWLayers *hw_layers);
  virtual DisplayError Commit(HWLayers *hw_layers);
  virtual DisplayError Flush(HWLayers *hw_layers);
  virtual DisplayError SetDynamicDSIClock(uint64_t bit_clk_rate);
  virtual DisplayError GetDynamicDSIClock(uint64_t *bit_clk_rate);
  virtual DisplayError SetPanelBrightness(int level);
  virtual DisplayError GetPanelBrightness(int *level);
  virtual void GetHWPanelMaxBrightness();

 private:
  void SetDestScalarData(HWLayersInfo hw_layer_info);
  void ResetDisplayParams();
  void PopulateBitClkRates();
  sde_drm_dest_scaler_data sde_dest_scalar_data_ = {};
  std::vector<SDEScaler> scalar_data_ = {};
  std::vector<uint64_t> bitclk_rates_;
  std::string brightness_base_path_ = "";
};

}  // namespace sdm

#endif  // __HW_PERIPHERAL_DRM_H__
