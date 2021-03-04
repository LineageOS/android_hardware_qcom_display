/* Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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
 */

#ifndef __COLOR_MANAGER_H__
#define __COLOR_MANAGER_H__

#include <stdlib.h>
#include <core/sdm_types.h>
#include <utils/locker.h>
#include <private/color_interface.h>
#include <private/snapdragon_color_intf.h>
#include <utils/sys.h>
#include <utils/debug.h>
#include <array>
#include <vector>
#include <map>
#include <string>
#include <mutex>

#include "hw_interface.h"

namespace sdm {

using snapdragoncolor::ColorMode;
using snapdragoncolor::ColorModeList;
using snapdragoncolor::GammaPostBlendConfig;
using snapdragoncolor::GamutConfig;
using snapdragoncolor::HwConfigOutputParams;
using snapdragoncolor::HwConfigPayload;
using snapdragoncolor::kHwConfigPayloadParam;
using snapdragoncolor::kModeList;
using snapdragoncolor::kModeRenderInputParams;
using snapdragoncolor::kNeedsUpdate;
using snapdragoncolor::kNotifyDisplayCalibrationMode;
using snapdragoncolor::kPbGamut;
using snapdragoncolor::kPbGC;
using snapdragoncolor::kPbIgc;
using snapdragoncolor::kPostBlendGammaHwConfig;
using snapdragoncolor::kPostBlendGamutHwConfig;
using snapdragoncolor::kPostBlendInverseGammaHwConfig;
using snapdragoncolor::kScModeRenderIntent;
using snapdragoncolor::kScModeSwAssets;
using snapdragoncolor::kSupportToneMap;
using snapdragoncolor::ModeRenderInputParams;
using snapdragoncolor::PostBlendGammaHwConfig;
using snapdragoncolor::PostBlendGamutHwConfig;
using snapdragoncolor::PostBlendInverseGammaHwConfig;
using snapdragoncolor::ScOps;
using snapdragoncolor::ScPayload;
using snapdragoncolor::ScPostBlendInterface;
using std::lock_guard;
using std::mutex;

enum FeatureOps {
  kFeatureSwitchMode,
  kFeatureOpsMax,
};

enum ControlOps {
  kControlNonPostedStart,
  kControlWithPostedStartDynSwitch,
  kControlPostedStart,
  kControlOpsMax,
};

class FeatureInterface {
 public:
  virtual ~FeatureInterface() {}
  virtual DisplayError Init() = 0;
  virtual DisplayError Deinit() = 0;
  virtual DisplayError SetParams(FeatureOps param_type, void *payload) = 0;
  virtual DisplayError GetParams(FeatureOps param_type, void *payload) = 0;
};

FeatureInterface* GetPostedStartFeatureCheckIntf(HWInterface *intf,
                                                 PPFeaturesConfig *config, bool dyn_switch);

/*
 * ColorManager proxy to maintain necessary information to interact with underlying color service.
 * Each display object has its own proxy.
 */
class ColorManagerProxy {
 public:
  static DisplayError Init(const HWResourceInfo &hw_res_info);
  static void Deinit();

  /* Create ColorManagerProxy for this display object, following things need to be happening
   * 1. Instantiates concrete ColorInerface implementation.
   * 2. Pass all display object specific informations into it.
   * 3. Populate necessary resources.
   * 4. Need get panel name for hw_panel_info_.
   */
  static ColorManagerProxy *CreateColorManagerProxy(DisplayType type, HWInterface *hw_intf,
                                                    const HWDisplayAttributes &attribute,
                                                    const HWPanelInfo &panel_info,
                                                    DppsControlInterface *dpps_intf);

  /* need reverse the effect of CreateColorManagerProxy. */
  ~ColorManagerProxy();

  DisplayError ColorSVCRequestRoute(const PPDisplayAPIPayload &in_payload,
                                    PPDisplayAPIPayload *out_payload,
                                    PPPendingParams *pending_action);
  DisplayError ApplyDefaultDisplayMode();
  DisplayError ColorMgrGetNumOfModes(uint32_t *mode_cnt);
  DisplayError ColorMgrGetModes(uint32_t *mode_cnt, SDEDisplayMode *modes);
  DisplayError ColorMgrSetMode(int32_t color_mode_id);
  DisplayError ColorMgrGetModeInfo(int32_t mode_id, AttrVal *query);
  DisplayError ColorMgrSetColorTransform(uint32_t length, const double *trans_data);
  DisplayError ColorMgrGetDefaultModeID(int32_t *mode_id);
  DisplayError ColorMgrCombineColorModes();
  bool NeedsPartialUpdateDisable();
  DisplayError Commit();
  DisplayError ColorMgrSetModeWithRenderIntent(int32_t color_mode_id,
                                               const PrimariesTransfer &blend_space,
                                               uint32_t intent);
  DisplayError Validate(DispLayerStack *disp_layer_stack);
  bool IsSupportStcTonemap();
  bool GameEnhanceSupported();
  DisplayError ColorMgrGetStcModes(ColorModeList *mode_list);
  DisplayError ColorMgrSetStcMode(const ColorMode &color_mode);
  DisplayError PrePrepare();
  DisplayError NotifyDisplayCalibrationMode(bool in_calibration);

 protected:
  ColorManagerProxy() {}
  ColorManagerProxy(int32_t id, DisplayType type, HWInterface *intf,
                    const HWDisplayAttributes &attr, const HWPanelInfo &info);

 private:
  static DynLib color_lib_;
  static DynLib stc_lib_;
  static CreateColorInterface create_intf_;
  static DestroyColorInterface destroy_intf_;
  static HWResourceInfo hw_res_info_;
  static GetScPostBlendInterface create_stc_intf_;

  typedef DisplayError (ColorManagerProxy::*ConvertProc)(const HwConfigPayload &in_data,
                                        PPFeaturesConfig *out_data);
  typedef std::map<std::string, ConvertProc> ConvertTable;

  bool NeedHwAssetsUpdate();
  DisplayError UpdateModeHwassets(int32_t mode_id, snapdragoncolor::ColorMode color_mode,
                                  bool valid_meta_data, const ColorMetaData &meta_data);
  DisplayError ConvertToPPFeatures(const HwConfigOutputParams &params, PPFeaturesConfig *out_data);
  void DumpColorMetaData(const ColorMetaData &color_metadata);
  bool HasNativeModeSupport();

  int32_t display_id_;
  DisplayType device_type_;
  PPHWAttributes pp_hw_attributes_;
  HWInterface *hw_intf_;
  ColorInterface *color_intf_;
  PPFeaturesConfig pp_features_;
  FeatureInterface *feature_intf_;
  bool apply_mode_ = false;
  PrimariesTransfer cur_blend_space_ = {};
  uint32_t cur_intent_ = 0;
  int32_t cur_mode_id_ = -1;
  ColorMetaData meta_data_ = {};
  snapdragoncolor::ScPostBlendInterface *stc_intf_ = NULL;
  snapdragoncolor::ColorMode curr_mode_;
  bool needs_update_ = false;
};

class ColorFeatureCheckingImpl : public FeatureInterface {
 public:
  explicit ColorFeatureCheckingImpl(HWInterface *hw_intf, PPFeaturesConfig *pp_features,
    bool dyn_switch);
  virtual ~ColorFeatureCheckingImpl() { }

  DisplayError Init();
  DisplayError Deinit();
  DisplayError SetParams(FeatureOps param_type, void *payload);
  DisplayError GetParams(FeatureOps param_type, void *payload);

 private:
  friend class FeatureStatePostedStart;
  friend class FeatureStateDefaultTrigger;
  friend class FeatureStateSerializedTrigger;

  HWInterface *hw_intf_;
  PPFeaturesConfig *pp_features_;
  std::array<FeatureInterface*, kFrameTriggerMax> states_ = {{NULL}};
  FeatureInterface *curr_state_ = NULL;
  std::vector<PPGlobalColorFeatureID> single_buffer_feature_;
  void CheckColorFeature(FrameTriggerMode *mode);
  bool dyn_switch_ = false;
};

class FeatureStatePostedStart : public FeatureInterface {
 public:
  explicit FeatureStatePostedStart(ColorFeatureCheckingImpl *obj);
  virtual ~FeatureStatePostedStart() {}

  DisplayError Init();
  DisplayError Deinit();
  DisplayError SetParams(FeatureOps param_type, void *payload);
  DisplayError GetParams(FeatureOps param_type, void *payload);

 private:
  ColorFeatureCheckingImpl *obj_;
};

class FeatureStateDefaultTrigger : public FeatureInterface {
 public:
  explicit FeatureStateDefaultTrigger(ColorFeatureCheckingImpl *obj);
  virtual ~FeatureStateDefaultTrigger() {}

  DisplayError Init();
  DisplayError Deinit();
  DisplayError SetParams(FeatureOps param_type, void *payload);
  DisplayError GetParams(FeatureOps param_type, void *payload);

 private:
  ColorFeatureCheckingImpl *obj_;
};

class FeatureStateSerializedTrigger : public FeatureInterface {
 public:
  explicit FeatureStateSerializedTrigger(ColorFeatureCheckingImpl *obj);
  virtual ~FeatureStateSerializedTrigger() {}

  DisplayError Init();
  DisplayError Deinit();
  DisplayError SetParams(FeatureOps param_type, void *payload);
  DisplayError GetParams(FeatureOps param_type, void *payload);

 private:
  ColorFeatureCheckingImpl *obj_;
};

}  // namespace sdm

#endif  // __COLOR_MANAGER_H__
