/*
 * Copyright (c) 2021 The Linux Foundation. All rights reserved.
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

// LCOV_EXCL_START
#include <private/noise_plugin_intf.h>
#include <private/noise_plugin_dbg.h>
#include <string.h>
#include <map>
#include <string>
#include <sstream>
#include <vector>

#define NOISE_PLUGIN_VERSION_MAJOR (1)       // Noise Plugin major version number
#define NOISE_PLUGIN_VERSION_MINOR (0)       // Noise Plugin minor version number

using sdm::GenericPayload;
using sdm::kDimLayer;
using sdm::kFodLayer;
using sdm::kGraphicsLayer;
using sdm::kGameLayer;
using sdm::kMaskLayer;
using sdm::kVideoLayer;
using std::string;
using std::stringstream;
using std::vector;

struct TestCase {
  bool enable;
  uint32_t attn;
  uint32_t noise_layer;
  uint32_t attn_layer;
  int32_t backlight_max;
  uint32_t backlight;
  vector<sdm::NoisePlugInLayerType> layers;
  bool override_en;
  int32_t override_attn;
  int32_t override_noise_zpos;
  int32_t override_bl_thr;
};

uint32_t total_test_cases = 32;

TestCase *test_cases;

void init_test_cases() {
  // {noise enable(exp), attn(exp), noisezpos(exp), attnzpos(exp), input bl, input layers,
  // max backlight, override enable, attn (override), noise zpos (override), bl_thr (override)}
  // backlight threshold is percentage. attn, input bl and max backlight are actual values.
  // 'X' represents a don't care condition.

  // Override disabled case - Override values won't be used

  // Fod layer is present (at the bottom), no global dimming
  // Noise at LM is not required for this use case as FOD itself is bottom most
  TestCase test_case_0{0, 125, 0, 1, 255, 200, {kFodLayer, kDimLayer, kVideoLayer, kGraphicsLayer,
                       kGameLayer}, 0, 0, 0, 0};
  // Fod layer is present (in the middle), no global dimming
  TestCase test_case_1{1, 125, 3, 4, 255, 225, {kDimLayer, kVideoLayer, kGraphicsLayer, kFodLayer,
                       kGameLayer}, 0, 0, 0, 0};
  // Fod layer is present (at the top), no global dimming
  TestCase test_case_2{1, 125, 3, 4, 255, 225, {kGameLayer, kDimLayer, kGraphicsLayer, kFodLayer},
                       0, 0, 0, 0};
  // Fod layer is present, global dimming
  TestCase test_case_3{1, 125, 1, 2, 255, 15, {kDimLayer, kFodLayer, kVideoLayer, kGraphicsLayer,
                       kGameLayer}, 0, 0, 0, 0};
  // No Fod layer, global dimming
  TestCase test_case_4{1, 125, 3, 4, 255, 15, {kGameLayer, kDimLayer, kGraphicsLayer}, 0, 0, 0, 0};
  // No Fod layer, no global dimming
  TestCase test_case_5{0, 'X', 'X', 'X', 255, 225, {kVideoLayer, kGraphicsLayer, kGameLayer},
                       0, 0, 0, 0};
  // Multiple Fod layers, no global dimming
  TestCase test_case_6{1, 125, 5, 6, 255, 225, {kGraphicsLayer, kVideoLayer, kFodLayer, kGameLayer,
                       kDimLayer, kFodLayer}, 0, 0, 0, 0};
  // Multiple Fod layers, global dimming
  TestCase test_case_7{1, 125, 2, 3, 255, 15, {kFodLayer, kVideoLayer, kFodLayer, kGraphicsLayer,
                       kGameLayer, kDimLayer}, 0, 0, 0, 0};
  // No Fod layer, no global dimming, mask layer is present
  TestCase test_case_8{0, 'X', 'X', 'X', 255, 75, {kDimLayer, kVideoLayer, kGraphicsLayer,
                       kMaskLayer}, 0, 0, 0, 0};
  // Fod layer is present, global dimming, mask layer is present
  TestCase test_case_9{1, 125, 2, 3, 255, 15, {kDimLayer, kVideoLayer, kFodLayer, kGameLayer,
                       kMaskLayer}, 0, 0, 0, 0};
  // Fod layer is present, no global dimming, mask layer is present
  TestCase test_case_10{1, 125, 5, 6, 255, 225, {kDimLayer, kVideoLayer, kGraphicsLayer, kGameLayer,
                        kMaskLayer, kFodLayer}, 0, 0, 0, 0};
  // No Fod layer, global dimming, mask layer is present
  TestCase test_case_11{1, 125, 3, 4, 255, 15, {kGameLayer, kDimLayer, kGraphicsLayer, kMaskLayer},
                        0, 0, 0, 0};
  // No Fod layer, global dimming, multiple mask layers
  TestCase test_case_12{1, 125, 4, 5, 255, 15, {kDimLayer, kVideoLayer, kGraphicsLayer, kGameLayer,
                        kMaskLayer, kMaskLayer}, 0, 0, 0, 0};
  // Global dimming use case, max backlight is not set
  TestCase test_case_13{0, 'X', 'X', 'X', -1, 15, {kGameLayer, kDimLayer, kGraphicsLayer,
                        kMaskLayer}, 0, 0, 0, 0};
  // FOD use case, max backlight is not set
  TestCase test_case_14{1, 125, 3, 4, -1, 100, {kGameLayer, kDimLayer, kVideoLayer, kFodLayer,
                        kMaskLayer}, 0, 0, 0, 0};
  // FOD use case, input backlight is 0
  TestCase test_case_15{1, 125, 1, 2, 255, 0, {kGraphicsLayer, kFodLayer, kGameLayer, kMaskLayer},
                        0, 0, 0, 0};
  // FOD use case, max backlight is not set and input backlight is 0
  TestCase test_case_16{1, 125, 3, 4, -1, 0, {kDimLayer, kVideoLayer, kGraphicsLayer, kFodLayer,
                        kMaskLayer}, 0, 0, 0, 0};
  // multiple FOD layers, max backlight is not set
  TestCase test_case_17{1, 125, 4, 5, -1, 15, {kVideoLayer, kGameLayer, kFodLayer, kDimLayer,
                        kFodLayer}, 0, 0, 0, 0};

  // Override enabled case - One or more among {attn, noise zpos, bl thr} will be overriden

  // only override flag is enabled
  TestCase test_case_18{0, 125, 0, 1, 255, 200, {kFodLayer, kDimLayer, kVideoLayer, kGraphicsLayer,
                        kGameLayer}, 1, -1, -1, -1};
  // override attenuation factor
  TestCase test_case_19{1, 150, 3, 4, 255, 255, {kDimLayer, kVideoLayer, kGraphicsLayer, kFodLayer,
                        kGameLayer}, 1, 150, -1, -1};
  // override noise z-position
  TestCase test_case_20{1, 125, 2, 3, 255, 225, {kDimLayer, kFodLayer, kVideoLayer, kGraphicsLayer,
                        kGameLayer}, 1, -1, 2, -1};
  // override backlight threshold (global dimming and no Fod layer)
  TestCase test_case_21{1, 125, 4, 5, 255, 100, {kVideoLayer, kDimLayer, kGraphicsLayer,
                        kGameLayer, kMaskLayer}, 1, -1, -1, 50};
  // override attenuation factor and noise z-position
  TestCase test_case_22{1, 80, 1, 2, 255, 175, {kVideoLayer, kGraphicsLayer, kDimLayer}, 1, 80, 1,
                        -1};
  // override attenuation factor and backlight threshold (global dimming, no Fod layer)
  TestCase test_case_23{1, 180, 4, 5, 255, 150, {kGraphicsLayer, kVideoLayer, kGameLayer,
                        kDimLayer}, 1, 180, -1, 90};
  // override noise z-position and backlight threshold
  TestCase test_case_24{1, 125, 2, 3, 255, 150, {kGameLayer, kDimLayer, kGraphicsLayer}, 1,
                        -1, 2, 90};
  // override attenuation factor, noise z-position and backlight threshold (GD, Fod is present)
  TestCase test_case_25{1, 150, 4, 5, 255, 50, {kGraphicsLayer, kVideoLayer, kFodLayer, kGameLayer,
                        kDimLayer, kFodLayer}, 1, 150, 4, 50};
  // override attenuation factor, noise z-position and backlight threshold (GD, No Fod)
  TestCase test_case_26{1, 70, 3, 4, 255, 25, {kDimLayer, kVideoLayer, kGraphicsLayer, kGameLayer,
                        kMaskLayer}, 1, 70, 3, 80};
  // override backlight threshold (global dimming, mask layer is present, no Fod layer)
  TestCase test_case_27{1, 100, 3, 4, 255, 120, {kGameLayer, kDimLayer, kGraphicsLayer,
                        kMaskLayer}, 1, 100, -1, 70};
  // override backlight threshold (global dimming, Fod layers are present)
  TestCase test_case_28{1, 100, 2, 3, 255, 25, {kFodLayer, kVideoLayer, kFodLayer, kGraphicsLayer,
                        kGameLayer, kDimLayer}, 1, 100, -1, 50};
  // Override noise zpos, max backlight not set
  TestCase test_case_29{1, 125, 2, 3, -1, 150, {kGameLayer, kDimLayer, kGraphicsLayer}, 1, -1,
                        2, -1};
  // Override backlight thr, but max backlight not set, Fod layer present
  TestCase test_case_30{1, 150, 5, 6, -1, 50, {kGraphicsLayer, kVideoLayer, kFodLayer, kGameLayer,
                        kDimLayer, kFodLayer}, 1, 150, -1, 50};
  // Override backlight thr, but max backlight not set, Fod layer not present
  TestCase test_case_31{0, 'X', 'X', 'X', -1, 150, {kGraphicsLayer, kVideoLayer, kGameLayer,
                        kMaskLayer}, 1, 180, -1, 90};

  test_cases = new TestCase[total_test_cases];
  if (!test_cases) {
    printf("Failed to allocate memory for test_cases\n");
    return;
  }

  test_cases[0] = test_case_0;
  test_cases[1] = test_case_1;
  test_cases[2] = test_case_2;
  test_cases[3] = test_case_3;
  test_cases[4] = test_case_4;
  test_cases[5] = test_case_5;
  test_cases[6] = test_case_6;
  test_cases[7] = test_case_7;
  test_cases[8] = test_case_8;
  test_cases[9] = test_case_9;
  test_cases[10] = test_case_10;
  test_cases[11] = test_case_11;
  test_cases[12] = test_case_12;
  test_cases[13] = test_case_13;
  test_cases[14] = test_case_14;
  test_cases[15] = test_case_15;
  test_cases[16] = test_case_16;
  test_cases[17] = test_case_17;
  test_cases[18] = test_case_18;
  test_cases[19] = test_case_19;
  test_cases[20] = test_case_20;
  test_cases[21] = test_case_21;
  test_cases[22] = test_case_22;
  test_cases[23] = test_case_23;
  test_cases[24] = test_case_24;
  test_cases[25] = test_case_25;
  test_cases[26] = test_case_26;
  test_cases[27] = test_case_27;
  test_cases[28] = test_case_28;
  test_cases[29] = test_case_29;
  test_cases[30] = test_case_30;
  test_cases[31] = test_case_31;
}

void deinit_test_cases() {
  if (test_cases) {
    delete[] test_cases;
  }
}

int TestCaseParser(sdm::NoisePlugInInputParams *in, sdm::NoisePlugInOutputParams *out,
                  TestCase test_case, bool *enable, uint32_t *attn, uint32_t *noise_layer,
                  uint32_t *attn_layer, int32_t *backlight_max, bool *override_en,
                  int32_t *override_attn, int32_t *override_noise_zpos, int32_t *override_bl_thr) {
  if (!in || !out || !attn || !noise_layer || !attn_layer) {
    printf("%s: invalid params\n", __FUNCTION__);
    return -EINVAL;
  }

  in->backlight = test_case.backlight;
  int32_t num_layers = test_case.layers.size();
  int32_t idx = 0;
  while (idx < num_layers) {
    sdm::NoisePlugInInputLayers layer;
    layer.layer_type = test_case.layers[idx];
    layer.zorder = idx++;
    in->layers.push_back(layer);
  }

  *enable = test_case.enable;
  *attn = test_case.attn;
  *noise_layer = test_case.noise_layer;
  *attn_layer = test_case.attn_layer;
  *backlight_max = test_case.backlight_max;
  *override_en = test_case.override_en;
  *override_attn = test_case.override_attn;
  *override_noise_zpos = test_case.override_noise_zpos;
  *override_bl_thr = test_case.override_bl_thr;

  return 0;
}

static bool TestStatusCheck(bool enable, uint32_t attn, uint32_t noise_layer, uint32_t attn_layer,
                            sdm::NoisePlugInOutputParams *p) {
  bool failed = true;
  if (!p) {
    return failed;
  }
  if ((p->enabled == enable) && ((enable == 0) || ((p->attn == attn) && (p->zpos[0] == noise_layer)
       && (p->zpos[1] == attn_layer)))) {
    failed = false;
  } else {
    printf(
        "test case failed! act en %d exp en %d act attn %d expected attn = %d act noise_layer %d "
        "expected noise_layer = %d act attn_layer %d expected attn_layer = %d\n",
        p->enabled, enable, p->attn, attn, p->zpos[0], noise_layer, p->zpos[1], attn_layer);
    failed = true;
  }

  return failed;
}

static void ExecuteTestCase(std::unique_ptr<sdm::NoisePlugInIntf> &plugin_intf, bool &failure) {
  init_test_cases();
  if (!plugin_intf) {
    printf("%s: invalid interface\n", __FUNCTION__);
    return;
  }
  printf("number of test-cases is %d\n", total_test_cases);
  uint32_t cur = 0;
  while (cur < total_test_cases) {
    plugin_intf->Init();

    sdm::NoisePlugInInputParams *in;
    sdm::NoisePlugInOutputParams *out;
    GenericPayload in_payload, out_payload;

    int32_t ret = in_payload.CreatePayload<sdm::NoisePlugInInputParams>(in);
    if (ret) {
      printf("failed to create input payload. Error:%d\n", ret);
      return;
    }

    ret = out_payload.CreatePayload<sdm::NoisePlugInOutputParams>(out);
    if (ret) {
      printf("failed to create output payload. Error:%d\n", ret);
      return;
    }

    uint32_t exp_attn, exp_noise_layer, exp_attn_layer;
    bool enable, override_en;
    int32_t backlight_max, override_attn, override_noise_zpos, override_bl_thr;
    TestCaseParser(in, out, test_cases[cur], &enable, &exp_attn, &exp_noise_layer, &exp_attn_layer,
                   &backlight_max, &override_en, &override_attn, &override_noise_zpos,
                   &override_bl_thr);

    // payload is reused as all the set params are of same data type.
    GenericPayload payload;
    int32_t *val = nullptr;
    ret = payload.CreatePayload<int32_t>(val);
    if (ret) {
      printf("failed to create the payload. Error:%d\n", ret);
      return;
    }

    // Set maximum backlight value
    *val = backlight_max;
    ret = plugin_intf->SetParameter(sdm::kNoisePlugInBackLightMax, payload);
    if ((ret) && (backlight_max > 0)) {
      printf("failed to set max backlight value\n");
      payload.DeletePayload();
      return;
    }

    // set override enable/disable flag
    sdm::NoisePlugInParams param;
    if ((sdm::kNoisePlugInDebugOverride >= sdm::kNoisePlugInDebugPropertyStart) &&
        (sdm::kNoisePlugInDebugOverride < sdm::kNoisePlugInDebugPropertyEnd)) {
      *val = override_en;
      param = static_cast<sdm::NoisePlugInParams>(sdm::kNoisePlugInDebugOverride);
      ret = plugin_intf->SetParameter(param, payload);
      if (ret) {
        printf("Failed to set override enable flag\n");
        payload.DeletePayload();
        return;
      }
    }

    if (override_en) {
      // over-ride enable case
      if ((sdm::kNoisePlugInDebugAttn >= sdm::kNoisePlugInDebugPropertyStart) &&
          (sdm::kNoisePlugInDebugAttn < sdm::kNoisePlugInDebugPropertyEnd)) {
        // override attenuation factor
        *val = override_attn;
        param = static_cast<sdm::NoisePlugInParams>(sdm::kNoisePlugInDebugAttn);
        ret = plugin_intf->SetParameter(param, payload);
        if ((ret) && (override_attn > 0)) {
          printf("Failed to set noise attn\n");
          payload.DeletePayload();
          return;
        }
      }

      if ((sdm::kNoisePlugInDebugNoiseZpos >= sdm::kNoisePlugInDebugPropertyStart) &&
          (sdm::kNoisePlugInDebugNoiseZpos < sdm::kNoisePlugInDebugPropertyEnd)) {
        // override noise layer z position
        *val = override_noise_zpos;
        param = static_cast<sdm::NoisePlugInParams>(sdm::kNoisePlugInDebugNoiseZpos);
        ret = plugin_intf->SetParameter(param, payload);
        if ((ret) && (override_noise_zpos > 0)) {
          printf("Failed to set noise zpos\n");
          payload.DeletePayload();
          return;
        }
      }

      if ((sdm::kNoisePlugInDebugBacklightThr >= sdm::kNoisePlugInDebugPropertyStart) &&
          (sdm::kNoisePlugInDebugBacklightThr < sdm::kNoisePlugInDebugPropertyEnd)) {
        // override backlight threshold
        *val = override_bl_thr;
        param = static_cast<sdm::NoisePlugInParams>(sdm::kNoisePlugInDebugBacklightThr);
        ret = plugin_intf->SetParameter(param, payload);
        if ((ret) && (override_bl_thr > 0) && (backlight_max > 0)) {
          printf("Failed to set backlight threshold\n");
          payload.DeletePayload();
          return;
        }
      }
    }

    ret = plugin_intf->ProcessOps(sdm::kOpsRunNoisePlugIn, in_payload, &out_payload);
    if (ret)
      printf("Call ops failed\n");
    failure = TestStatusCheck(enable, exp_attn, exp_noise_layer, exp_attn_layer, out);

    in_payload.DeletePayload();
    out_payload.DeletePayload();
    payload.DeletePayload();

    // De-initialize plugin interface so that max backlight is unset
    plugin_intf->Deinit();

    printf("test case: %d resut: %s\n", cur, failure ? "failed":"passed");
    if (failure) {
      break;
    }
    cur++;
  }
  deinit_test_cases();
}


int main(int argc, char *argv[]) {
  std::unique_ptr<sdm::NoisePlugInIntf> plugin_intf = nullptr;
  sdm::NoisePlugInFactoryIntf *plugin_factory_intf = nullptr;

  bool failure = true;

  // create noise plugin factory
  plugin_factory_intf = sdm::GetNoisePlugInFactoryIntf();
  if (!plugin_factory_intf) {
    printf("Failed to create noise plugin factory\n");
    return -1;
  }

  // create noise plugin interface
  plugin_intf = plugin_factory_intf->CreateNoisePlugInIntf(NOISE_PLUGIN_VERSION_MAJOR,
                                                           NOISE_PLUGIN_VERSION_MINOR);
  if (!plugin_intf) {
    printf("Failed to create noise plugin interface\n");
    return -1;
  }

  ExecuteTestCase(plugin_intf, failure);

  if (failure) {
    printf("TEST FAILED\n");
  }
  return (failure ? -1 : 0);
}
// LCOV_EXCL_STOP
