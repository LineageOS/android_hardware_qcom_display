/*
* Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <math.h>
#include <tinyxml2.h>
#include <utils/constants.h>
#include <utils/debug.h>

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "hwc_display_resolution_extn.h"

#define __CLASS__ "HWCDisplayResolutionExtn"
#define DISPLAY_RESOLUTION_EXTN_FILE "/vendor/etc/display/sdm_display_resolution_extn.xml"

using tinyxml2::XMLElement;
using tinyxml2::XMLDocument;
using tinyxml2::XML_SUCCESS;

namespace sdm {

static const char *GetSocName() {
  std::ifstream soc_id_file("/sys/devices/soc0/soc_id");
  std::string soc_id;

  if (!soc_id_file.is_open() || !std::getline(soc_id_file, soc_id) || soc_id.empty()) {
    return "";
  }

  switch (std::atoi(soc_id.c_str())) {
    case 557:  // SM8650
    case 577:  // SM8650P
    case 614:  // SM8635
    case 632:  // SM7675
    case 642:  // SM8635P
    case 643:  // SM7675P
      return "pineapple";
    default:
      return "";
  }
}

DisplayError HWCDisplayResolutionExtn::GetExtendedDisplayResolutions(uint32_t panel_width,
                                                                     uint32_t panel_height,
                                          std::vector<std::pair<uint32_t, uint32_t>> *extended_disp_res) {
  DisplayError error = kErrorNotSupported;
  const char *soc_name = GetSocName();

  if (soc_name[0] == '\0') {
    return error;
  }

  XMLDocument document;
  if (document.LoadFile(DISPLAY_RESOLUTION_EXTN_FILE) != XML_SUCCESS) {
    DLOGI("Could not load %s", DISPLAY_RESOLUTION_EXTN_FILE);
    return error;
  }

  XMLElement *targets = document.RootElement();
  if (targets == nullptr) {
    DLOGE("No targets configuration specified");
    return error;
  }

  for (XMLElement *target_node = targets->FirstChildElement("Target"); target_node != nullptr;
       target_node = target_node->NextSiblingElement("Target")) {
    const char *target_name = target_node->Attribute("name");
    if (target_name == nullptr || strcmp(target_name, soc_name)) {
      continue;
    }

    for (XMLElement *panel_res_node = target_node->FirstChildElement("PanelResolution");
         panel_res_node != nullptr;
         panel_res_node = panel_res_node->NextSiblingElement("PanelResolution")) {
      const char *width = panel_res_node->Attribute("width");
      const char *height = panel_res_node->Attribute("height");
      if (width == nullptr || height == nullptr) {
        continue;
      }

      const uint32_t p_width = UINT32(std::atoi(width));
      const uint32_t p_height = UINT32(std::atoi(height));
      if (p_width != panel_width || p_height != panel_height) {
        continue;
      }

      for (XMLElement *scaling_node = panel_res_node->FirstChildElement();
           scaling_node != nullptr; scaling_node = scaling_node->NextSiblingElement()) {
        double res_x = 0.0, res_y = 0.0;

        if (!strcmp(scaling_node->Name(), "ScalingFactor")) {
          const char *x = scaling_node->Attribute("x");
          const char *y = scaling_node->Attribute("y");
          if (x == nullptr || y == nullptr) {
            continue;
          }
          const double x_factor = std::atof(x);
          const double y_factor = std::atof(y);
          if (x_factor < 1 || y_factor < 1) {
            DLOGW("Unsupported scaling factor");
            continue;
          }
          res_x = p_width / x_factor;
          res_y = p_height / y_factor;
        } else if (!strcmp(scaling_node->Name(), "ScalingResolution")) {
          const char *w = scaling_node->Attribute("w");
          const char *h = scaling_node->Attribute("h");
          if (w == nullptr || h == nullptr) {
            continue;
          }
          res_x = std::atof(w);
          res_y = std::atof(h);
        }

        if (!res_x || !res_y) {
          continue;
        }

        if ((floor(res_x) == res_x) && (floor(res_y) == res_y) && (UINT32(res_x) % 2 == 0) &&
            (UINT32(res_y) % 2 == 0) && ((p_width / res_x) == (p_height / res_y))) {
          extended_disp_res->push_back(std::make_pair(UINT32(res_x), UINT32(res_y)));
        } else {
          DLOGI("scaling resolution: %f x %f is invalid", res_x, res_y);
        }
      }

      return extended_disp_res->empty() ? kErrorNotSupported : kErrorNone;
    }
  }

  return error;
}

}  // namespace sdm
