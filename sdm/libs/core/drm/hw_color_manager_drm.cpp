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
#include <utils/debug.h>
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

#ifdef PP_DRM_ENABLE
static const uint32_t kPgcDataMask = 0x3FF;
static const uint32_t kPgcShift = 16;

static const uint32_t kIgcDataMask = 0xFFF;
static const uint32_t kIgcShift = 16;
#endif

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
  feature->payload = NULL;
}

uint32_t HWColorManagerDrm::GetFeatureVersion(const DRMPPFeatureInfo &feature) {
  uint32_t version = PPFeatureVersion::kSDEPpVersionInvalid;

  switch (feature.id) {
    case kFeaturePcc:
      if (feature.version == 1) {
        version = PPFeatureVersion::kSDEPccV17;
      } else if (feature.version == 4) {
        version = PPFeatureVersion::kSDEPccV4;
      }
      break;
    case kFeatureIgc:
      if (feature.version == 3)
        version = PPFeatureVersion::kSDEIgcV30;
      break;
    case kFeaturePgc:
      if (feature.version == 1)
        version = PPFeatureVersion::kSDEPgcV17;
      break;
    case kFeatureMixerGc:
        version = PPFeatureVersion::kSDEPgcV17;
      break;
    case kFeaturePaV2:
        version = PPFeatureVersion::kSDEPaV17;
      break;
    case kFeatureDither:
        version = PPFeatureVersion::kSDEDitherV17;
      break;
    case kFeatureGamut:
      if (feature.version == 1)
        version = PPFeatureVersion::kSDEGamutV17;
      else if (feature.version == 4)
        version = PPFeatureVersion::kSDEGamutV4;
      break;
    case kFeaturePADither:
        version = PPFeatureVersion::kSDEPADitherV17;
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
#ifdef PP_DRM_ENABLE
  struct SDEPccV4Cfg *sde_pcc = NULL;
  struct SDEPccV4Coeff *sde_pcc_coeffs = NULL;
  struct drm_msm_pcc *mdp_pcc = NULL;
  struct drm_msm_pcc_coeff *mdp_pcc_coeffs = NULL;
  uint32_t i = 0;

  if (!out_data) {
    DLOGE("Invalid input parameter for pcc");
    return kErrorParameters;
  }

  switch (in_data.feature_version_) {
  case PPFeatureVersion::kSDEPccV4:
    sde_pcc = (struct SDEPccV4Cfg *) in_data.GetConfigData();
    break;
  default:
    DLOGE("Unsupported pcc feature version: %d", in_data.feature_version_);
    return kErrorParameters;
  }

  out_data->id = kFeaturePcc;
  out_data->type = sde_drm::kPropBlob;
  out_data->version = in_data.feature_version_;
  out_data->payload_size = sizeof(struct drm_msm_pcc);

  if (in_data.enable_flags_ & kOpsDisable) {
    /* feature disable case */
    out_data->payload = NULL;
    return ret;
  } else if (!(in_data.enable_flags_ & kOpsEnable)) {
    out_data->payload = NULL;
    return kErrorParameters;
  }

  mdp_pcc = new drm_msm_pcc();
  if (!mdp_pcc) {
    DLOGE("Failed to allocate memory for pcc");
    return kErrorMemory;
  }

  mdp_pcc->flags = 0;

  for (i = 0; i < kMaxPCCChanel; i++) {
    switch (i) {
    case 0:
      sde_pcc_coeffs = &sde_pcc->red;
      mdp_pcc_coeffs = &mdp_pcc->r;
      mdp_pcc->r_rr = sde_pcc_coeffs->rr;
      mdp_pcc->r_gg = sde_pcc_coeffs->gg;
      mdp_pcc->r_bb = sde_pcc_coeffs->bb;
      break;
    case 1:
        sde_pcc_coeffs = &sde_pcc->green;
        mdp_pcc_coeffs = &mdp_pcc->g;
        mdp_pcc->g_rr = sde_pcc_coeffs->rr;
        mdp_pcc->g_gg = sde_pcc_coeffs->gg;
        mdp_pcc->g_bb = sde_pcc_coeffs->bb;
      break;
    case 2:
        sde_pcc_coeffs = &sde_pcc->blue;
        mdp_pcc_coeffs = &mdp_pcc->b;
        mdp_pcc->b_rr = sde_pcc_coeffs->rr;
        mdp_pcc->b_gg = sde_pcc_coeffs->gg;
        mdp_pcc->b_bb = sde_pcc_coeffs->bb;
      break;
    }
    mdp_pcc_coeffs->c = sde_pcc_coeffs->c;
    mdp_pcc_coeffs->r = sde_pcc_coeffs->r;
    mdp_pcc_coeffs->g = sde_pcc_coeffs->g;
    mdp_pcc_coeffs->b = sde_pcc_coeffs->b;
    mdp_pcc_coeffs->rg = sde_pcc_coeffs->rg;
    mdp_pcc_coeffs->gb = sde_pcc_coeffs->gb;
    mdp_pcc_coeffs->rb = sde_pcc_coeffs->rb;
    mdp_pcc_coeffs->rgb = sde_pcc_coeffs->rgb;
  }
  out_data->payload = mdp_pcc;
#endif
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmIGC(const PPFeatureInfo &in_data,
                                          DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
#ifdef PP_DRM_ENABLE
  struct SDEIgcV30LUTData *sde_igc;
  struct drm_msm_igc_lut *mdp_igc;
  uint32_t *c0_c1_data_ptr = NULL;
  uint32_t *c2_data_ptr = NULL;


  if (!out_data) {
    DLOGE("Invalid input parameter for igc");
    return kErrorParameters;
  }

  switch (in_data.feature_version_) {
  case PPFeatureVersion::kSDEIgcV30:
    sde_igc = (struct SDEIgcV30LUTData *) in_data.GetConfigData();
    break;
  default:
    DLOGE("Unsupported igc feature version: %d", in_data.feature_version_);
    return kErrorParameters;
  }

  out_data->id = kFeatureIgc;
  out_data->type = sde_drm::kPropBlob;
  out_data->version = in_data.feature_version_;
  out_data->payload_size = sizeof(struct drm_msm_igc_lut);

  if (in_data.enable_flags_ & kOpsDisable) {
    /* feature disable case */
    out_data->payload = NULL;
    return ret;
  } else if (!(in_data.enable_flags_ & kOpsEnable)) {
    out_data->payload = NULL;
    return kErrorParameters;
  }

  mdp_igc = new drm_msm_igc_lut();
  if (!mdp_igc) {
    DLOGE("Failed to allocate memory for igc");
    return kErrorMemory;
  }

  mdp_igc->flags = IGC_DITHER_ENABLE;
  mdp_igc->strength = sde_igc->strength;

  c0_c1_data_ptr = reinterpret_cast<uint32_t*>(sde_igc->c0_c1_data);
  c2_data_ptr = reinterpret_cast<uint32_t*>(sde_igc->c2_data);

  if (!c0_c1_data_ptr || !c2_data_ptr) {
    DLOGE("Invaid igc data pointer");
    delete mdp_igc;
    out_data->payload = NULL;
    return kErrorParameters;
  }

  for (int i = 0; i < IGC_TBL_LEN; i++) {
    mdp_igc->c0[i] = c0_c1_data_ptr[i] & kIgcDataMask;
    mdp_igc->c1[i] = (c0_c1_data_ptr[i] >> kIgcShift) & kIgcDataMask;
    mdp_igc->c2[i] = c2_data_ptr[i] & kIgcDataMask;
  }
  out_data->payload = mdp_igc;
#endif
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmPGC(const PPFeatureInfo &in_data,
                                          DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
#ifdef PP_DRM_ENABLE
  struct SDEPgcLUTData *sde_pgc;
  struct drm_msm_pgc_lut *mdp_pgc;

  if (!out_data) {
    DLOGE("Invalid input parameter for gamut");
    return kErrorParameters;
  }
  sde_pgc = (struct SDEPgcLUTData *)in_data.GetConfigData();
  out_data->id = kFeaturePgc;
  out_data->type = sde_drm::kPropBlob;
  out_data->version = in_data.feature_version_;
  out_data->payload_size = sizeof(struct drm_msm_pgc_lut);

  if (in_data.enable_flags_ & kOpsDisable) {
    /* feature disable case */
    out_data->payload = NULL;
    return ret;
  } else if (!(in_data.enable_flags_ & kOpsEnable)) {
    out_data->payload = NULL;
    return kErrorParameters;
  }

  mdp_pgc = new drm_msm_pgc_lut();
  if (!mdp_pgc) {
    DLOGE("Failed to allocate memory for pgc");
    return kErrorMemory;
  }

  if (in_data.enable_flags_ & kOpsEnable)
    mdp_pgc->flags = PGC_8B_ROUND;
  else
    mdp_pgc->flags = 0;

  for (int i = 0, j = 0; i < PGC_TBL_LEN; i++, j += 2) {
    mdp_pgc->c0[i] = (sde_pgc->c0_data[j] & kPgcDataMask) |
        (sde_pgc->c0_data[j + 1] & kPgcDataMask) << kPgcShift;
    mdp_pgc->c1[i] = (sde_pgc->c1_data[j] & kPgcDataMask) |
        (sde_pgc->c1_data[j + 1] & kPgcDataMask) << kPgcShift;
    mdp_pgc->c2[i] = (sde_pgc->c2_data[j] & kPgcDataMask) |
        (sde_pgc->c2_data[j + 1] & kPgcDataMask) << kPgcShift;
  }
  out_data->payload = mdp_pgc;
#endif
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmMixerGC(const PPFeatureInfo &in_data,
                                              DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
#ifdef PP_DRM_ENABLE
  if (!out_data) {
    DLOGE("Invalid input parameter for Mixer GC");
    return kErrorParameters;
  }

  out_data->id = kPPFeaturesMax;
  out_data->type = sde_drm::kPropBlob;
  out_data->version = in_data.feature_version_;
#endif
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmPAV2(const PPFeatureInfo &in_data,
                                           DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
#ifdef PP_DRM_ENABLE
  if (!out_data) {
    DLOGE("Invalid input parameter for PA V2");
    return kErrorParameters;
  }

  out_data->id = kPPFeaturesMax;
  out_data->type = sde_drm::kPropBlob;
  out_data->version = in_data.feature_version_;

#endif
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmDither(const PPFeatureInfo &in_data,
                                             DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
#ifdef PP_DRM_ENABLE
  if (!out_data) {
    DLOGE("Invalid input parameter for dither");
    return kErrorParameters;
  }

  out_data->id = kPPFeaturesMax;
  out_data->type = sde_drm::kPropBlob;
  out_data->version = in_data.feature_version_;
#endif
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmGamut(const PPFeatureInfo &in_data,
                                            DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
#ifdef PP_DRM_ENABLE
  struct SDEGamutCfg *sde_gamut = NULL;
  struct drm_msm_3d_gamut *mdp_gamut = NULL;
  uint32_t size = 0;

  if (!out_data) {
    DLOGE("Invalid input parameter for gamut");
    return kErrorParameters;
  }
  sde_gamut = (struct SDEGamutCfg *)in_data.GetConfigData();
  out_data->id = kFeatureGamut;
  out_data->type = sde_drm::kPropBlob;
  out_data->version = in_data.feature_version_;
  out_data->payload_size = sizeof(struct drm_msm_3d_gamut);
  if (in_data.enable_flags_ & kOpsDisable) {
    /* feature disable case */
    out_data->payload = NULL;
    return ret;
  } else if (!(in_data.enable_flags_ & kOpsEnable)) {
    out_data->payload = NULL;
    return kErrorParameters;
  }

  mdp_gamut = new drm_msm_3d_gamut();
  if (!mdp_gamut) {
    DLOGE("Failed to allocate memory for gamut");
    return kErrorMemory;
  }

  if (sde_gamut->map_en)
    mdp_gamut->flags = GAMUT_3D_MAP_EN;
  else
    mdp_gamut->flags = 0;

  switch (sde_gamut->mode) {
    case SDEGamutCfgWrapper::GAMUT_FINE_MODE:
      mdp_gamut->mode = GAMUT_3D_MODE_17;
      size = GAMUT_3D_MODE17_TBL_SZ;
      break;
    case SDEGamutCfgWrapper::GAMUT_COARSE_MODE:
      mdp_gamut->mode = GAMUT_3D_MODE_5;
      size = GAMUT_3D_MODE5_TBL_SZ;
      break;
    case SDEGamutCfgWrapper::GAMUT_COARSE_MODE_13:
      mdp_gamut->mode = GAMUT_3D_MODE_13;
      size = GAMUT_3D_MODE13_TBL_SZ;
      break;
    default:
      DLOGE("Invalid gamut mode %d", sde_gamut->mode);
      free(mdp_gamut);
      return kErrorParameters;
  }

  if (sde_gamut->map_en) {
    std::memcpy(&mdp_gamut->scale_off[0][0], sde_gamut->scale_off_data[0],
                sizeof(uint32_t) * GAMUT_3D_SCALE_OFF_SZ);
    std::memcpy(&mdp_gamut->scale_off[1][0], sde_gamut->scale_off_data[1],
                sizeof(uint32_t) * GAMUT_3D_SCALE_OFF_SZ);
    std::memcpy(&mdp_gamut->scale_off[2][0], sde_gamut->scale_off_data[2],
                sizeof(uint32_t) * GAMUT_3D_SCALE_OFF_SZ);
  }

  for (uint32_t row = 0; row < GAMUT_3D_TBL_NUM; row++) {
    for (uint32_t col = 0; col < size; col++) {
      mdp_gamut->col[row][col].c0 = sde_gamut->c0_data[row][col];
      mdp_gamut->col[row][col].c2_c1 = sde_gamut->c1_c2_data[row][col];
    }
  }
  out_data->payload = mdp_gamut;
#endif
  return ret;
}

DisplayError HWColorManagerDrm::GetDrmPADither(const PPFeatureInfo &in_data,
                                               DRMPPFeatureInfo *out_data) {
  DisplayError ret = kErrorNone;
#ifdef PP_DRM_ENABLE
  if (!out_data) {
    DLOGE("Invalid input parameter for PA dither");
    return kErrorParameters;
  }

  out_data->id = kPPFeaturesMax;
  out_data->type = sde_drm::kPropBlob;
  out_data->version = in_data.feature_version_;
#endif
  return ret;
}

}  // namespace sdm
