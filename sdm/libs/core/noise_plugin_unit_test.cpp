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
  uint32_t exp_attn;
  uint32_t exp_noise_zpos;
  uint32_t exp_attn_zpos;
  uint32_t exp_strength;
  uint32_t exp_alpha_noise;
  uint32_t exp_temporal_en;
  bool override_en;
  int32_t override_attn;
  int32_t override_noise_zpos;
  vector<sdm::NoisePlugInLayerType> layers;
};

uint32_t total_test_cases = 11;

TestCase *test_cases;

void init_test_cases() {
  // refer to TestCase declaration
  // 'X' represents a don't care condition.

  // Override disabled case - Override values won't be used

  // Fod layer is present at the bottom
  // Noise at LM is not required for this use case as FOD itself is bottom most
  TestCase test_case_0{0, 125, 0, 1, 0, 35, 1, 0, 0, 0,
                      {kFodLayer, kDimLayer, kVideoLayer, kGraphicsLayer, kGameLayer}};
  // Fod layer is present in the middle
  TestCase test_case_1{1, 125, 3, 4, 0, 35, 1, 0, 0, 0,
                      {kDimLayer, kVideoLayer, kGraphicsLayer, kFodLayer, kGameLayer}};
  // Fod layer is present at the top
  TestCase test_case_2{1, 125, 3, 4, 0, 35, 1, 0, 0, 0,
                      {kGameLayer, kDimLayer, kGraphicsLayer, kFodLayer}};
  // No Fod layer
  TestCase test_case_3{0, 'X', 'X', 'X', 0, 35, 1, 0, 0, 0,
                      {kVideoLayer, kGraphicsLayer, kGameLayer}};
  // Multiple Fod layers
  TestCase test_case_4{1, 125, 5, 6, 0, 35, 1, 0, 0, 0,
                      {kGraphicsLayer, kVideoLayer, kFodLayer, kGameLayer, kDimLayer, kFodLayer}};
  // No Fod layer, mask layer is present
  TestCase test_case_5{0, 'X', 'X', 'X', 0, 35, 1, 0, 0, 0,
                      {kDimLayer, kVideoLayer, kGraphicsLayer, kMaskLayer}};
  // Fod layer is present, mask layer is present
  TestCase test_case_6{1, 125, 2, 3, 0, 35, 1, 0, 0, 0,
                      {kDimLayer, kVideoLayer, kFodLayer, kGameLayer, kMaskLayer}};

  // Override enabled case - One or more among {attn, noise zpos} will be overriden

  // only override flag is enabled
  TestCase test_case_7{0, 125, 0, 1, 0, 35, 1, 1, -1, -1,
                       {kFodLayer, kDimLayer, kVideoLayer, kGraphicsLayer, kGameLayer}};
  // override attenuation factor
  TestCase test_case_8{1, 150, 3, 4, 0, 29, 1, 1, 150, -1,
                       {kDimLayer, kVideoLayer, kGraphicsLayer, kFodLayer, kGameLayer}};
  // override noise z-position
  TestCase test_case_9{1, 125, 2, 3, 0, 35, 1, 1, -1, 2,
                       {kDimLayer, kFodLayer, kVideoLayer, kGraphicsLayer, kGameLayer}};
  // override attenuation factor and noise z-position
  TestCase test_case_10{1, 75, 1, 2, 0, 58, 1, 1, 75, 1,
                       {kVideoLayer, kGraphicsLayer, kDimLayer}};

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
}

void deinit_test_cases() {
  if (test_cases) {
    delete[] test_cases;
  }
}

int TestCaseParser(sdm::NoisePlugInInputParams *in, sdm::NoisePlugInOutputParams *out,
                  TestCase test_case) {
  if (!in || !out) {
    printf("%s: invalid params\n", __FUNCTION__);
    return -EINVAL;
  }

  int32_t num_layers = test_case.layers.size();
  int32_t idx = 0;
  while (idx < num_layers) {
    sdm::NoisePlugInInputLayers layer;
    layer.layer_type = test_case.layers[idx];
    layer.zorder = idx++;
    in->layers.push_back(layer);
  }

  return 0;
}

// function returns true when test fails.
static bool TestStatusCheck(TestCase test_case, sdm::NoisePlugInOutputParams *p) {
  if (!p) {
    return true;
  }

  // check actual enable and expected enable
  if (p->enabled != test_case.enable) {
    printf("test case failed! act en = %d exp en = %d \n", p->enabled, test_case.enable);
    return true;
  }

  // no need to check remaining parameters when noise is disabled.
  if (test_case.enable == 0) {
    // test case passed
    return false;
  }

  // check actual and expected attenuation
  if (p->attn != test_case.exp_attn) {
    printf("test case failed! act attn = %d exp attn = %d \n", p->attn, test_case.exp_attn);
    return true;
  }

  // check actual and expected noise layer z-position
  if (p->zpos[0] != test_case.exp_noise_zpos) {
    printf("test case failed! act noise zpos = %d expected noise zpos = %d",
        p->zpos[0], test_case.exp_noise_zpos);
    return true;
  }

  // check actual and expected attenuation layer z-position
  if (p->zpos[1] != test_case.exp_attn_zpos) {
    printf("test case failed! act attn zpos = %d expected attn zpos = %d\n",
        p->zpos[1], test_case.exp_attn_zpos);
    return true;
  }

  // check actual and expected noise strength
  if (p->strength != test_case.exp_strength) {
    printf("test case failed! act noise strength = %d expected noise strength = %d\n",
        p->strength, test_case.exp_strength);
    return true;
  }

  // check actual and expected alpha noise
  if (p->alpha_noise != test_case.exp_alpha_noise) {
    printf("test case failed! act alpha noise = %d expected alpha noise = %d\n",
        p->alpha_noise, test_case.exp_alpha_noise);
    return true;
  }

  // check actual and expected temporal enable
  if (p->temporal_en != test_case.exp_temporal_en) {
    printf("test case failed! act temporal enable = %d expected temporal enable = %d\n",
        p->temporal_en, test_case.exp_temporal_en);
    return true;
  }

  // test case passed
  return false;
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

    TestCaseParser(in, out, test_cases[cur]);

    // payload is reused as all the set params are of same data type.
    GenericPayload payload;
    int32_t *val = nullptr;
    ret = payload.CreatePayload<int32_t>(val);
    if (ret) {
      printf("failed to create the payload. Error:%d\n", ret);
      return;
    }

    // set override enable/disable flag
    sdm::NoisePlugInParams param;
    if ((sdm::kNoisePlugInDebugOverride >= sdm::kNoisePlugInDebugPropertyStart) &&
        (sdm::kNoisePlugInDebugOverride < sdm::kNoisePlugInDebugPropertyEnd)) {
      *val = test_cases[cur].override_en;
      param = static_cast<sdm::NoisePlugInParams>(sdm::kNoisePlugInDebugOverride);
      ret = plugin_intf->SetParameter(param, payload);
      if (ret) {
        printf("Failed to set override enable flag\n");
        payload.DeletePayload();
        return;
      }
    }

    if (test_cases[cur].override_en) {
      // over-ride enable case
      if ((sdm::kNoisePlugInDebugAttn >= sdm::kNoisePlugInDebugPropertyStart) &&
          (sdm::kNoisePlugInDebugAttn < sdm::kNoisePlugInDebugPropertyEnd)) {
        // override attenuation factor
        *val = test_cases[cur].override_attn;
        param = static_cast<sdm::NoisePlugInParams>(sdm::kNoisePlugInDebugAttn);
        ret = plugin_intf->SetParameter(param, payload);
        if ((ret) && (test_cases[cur].override_attn > 0)) {
          printf("Failed to set noise attn\n");
          payload.DeletePayload();
          return;
        }
      }

      if ((sdm::kNoisePlugInDebugNoiseZpos >= sdm::kNoisePlugInDebugPropertyStart) &&
          (sdm::kNoisePlugInDebugNoiseZpos < sdm::kNoisePlugInDebugPropertyEnd)) {
        // override noise layer z position
        *val = test_cases[cur].override_noise_zpos;
        param = static_cast<sdm::NoisePlugInParams>(sdm::kNoisePlugInDebugNoiseZpos);
        ret = plugin_intf->SetParameter(param, payload);
        if ((ret) && (test_cases[cur].override_noise_zpos > 0)) {
          printf("Failed to set noise zpos\n");
          payload.DeletePayload();
          return;
        }
      }
    }

    ret = plugin_intf->ProcessOps(sdm::kOpsRunNoisePlugIn, in_payload, &out_payload);
    if (ret) {
      printf("Call ops failed\n");
      break;
    }

    failure = TestStatusCheck(test_cases[cur], out);

    in_payload.DeletePayload();
    out_payload.DeletePayload();
    payload.DeletePayload();

    // De-initialize plugin interface so that max backlight is unset
    plugin_intf->Deinit();

    printf("test case: %d result: %s\n", cur, failure ? "failed":"passed");
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
