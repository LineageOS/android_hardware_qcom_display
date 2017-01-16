/*
* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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
*/

#define __CLASS__ "HWColorManagerDRM"

#ifdef PP_DRM_ENABLE
#include <drm/msm_drm_pp.h>
#endif
#include "hw_color_manager_drm.h"

using sde_drm::kFeaturePcc;
using sde_drm::kFeatureIgc;
using sde_drm::kFeaturePgc;
using sde_drm::kFeatureMixerGc;
using sde_drm::kFeaturePaV2;
using sde_drm::kFeatureDither;
using sde_drm::kFeatureGamut;
using sde_drm::kFeaturePADither;
using sde_drm::kPPFeaturesMax;

namespace sdm {

DisplayError (*HWColorManagerDrm::GetDrmFeature[])(const PPFeatureInfo &, DRMPPFeatureInfo *) = {
        [kGlobalColorFeaturePcc] = &HWColorManagerDrm::GetDrmPCC,
        [kGlobalColorFeatureIgc] = &HWColorManagerDrm::GetDrmIGC,
        [kGlobalColorFeaturePgc] = &HWColorManagerDrm::GetDrmPGC,
        [kMixerColorFeatureGc] = &HWColorManagerDrm::GetDrmMixerGC,
        [kGlobalColorFeaturePaV2] = &HWColorManagerDrm::GetDrmPAV2,
        [kGlobalColorFeatureDither] = &HWColorManagerDrm::GetDrmDither,
        [kGlobalColorFeatureGamut] = &HWColorManagerDrm::GetDrmGamut,
        [kGlobalColorFeaturePADither] = &HWColorManagerDrm::GetDrmPADither,
};

void HWColorManagerDrm::FreeDrmFeatureData(DRMPPFeatureInfo *feature) {
  if (feature->payload)
    free(feature->payload);
}

uint32_t HWColorManagerDrm::GetFeatureVersion(DRMPPFeatureInfo &feature) {
  uint32_t version = PPFeatureVersion::kSDEPpVersionInvalid;

  switch (feature.id) {
    case kFeaturePcc:
      break;
    case kFeatureIgc:
      break;
    case kFeaturePgc:
      break;
    case kFeatureMixerGc:
      break;
    case kFeaturePaV2:
      break;
    case kFeatureDither:
      break;
    case kFeatureGamut:
      break;
    case kFeaturePADither:
      break;
    default:
      break;
  }
  return version;
}

DRMPPFeatureID HWColorManagerDrm::ToDrmFeatureId(uint32_t id) {
  DRMPPFeatureID ret = kPPFeaturesMax;

  switch (id) {
    case kGlobalColorFeaturePcc:
      ret = kFeaturePcc;
      break;
    case kGlobalColorFeatureIgc:
      ret = kFeatureIgc;
      break;
    case kGlobalColorFeaturePgc:
      ret = kFeaturePgc;
      break;
    case kMixerColorFeatureGc:
      ret = kFeatureMixerGc;
      break;
    case kGlobalColorFeaturePaV2:
      ret = kFeaturePaV2;
      break;
    case kGlobalColorFeatureDither:
      ret = kFeatureDither;
      break;
    case kGlobalColorFeatureGamut:
      ret = kFeatureGamut;
      break;
    case kGlobalColorFeaturePADither:
      ret = kFeaturePADither;
      break;
    default:
      break;
  }
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmPCC(const PPFeatureInfo &in_data,
                                          DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmIGC(const PPFeatureInfo &in_data,
                                          DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmPGC(const PPFeatureInfo &in_data,
                                          DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmMixerGC(const PPFeatureInfo &in_data,
                                              DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmPAV2(const PPFeatureInfo &in_data,
                                           DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmDither(const PPFeatureInfo &in_data,
                                             DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmGamut(const PPFeatureInfo &in_data,
                                            DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmPADither(const PPFeatureInfo &in_data,
                                               DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
  return ret;
}

}  // namespace sdm
