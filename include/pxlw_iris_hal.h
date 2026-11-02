/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <vector>

namespace pxlw {

class IrisHal {
 public:
  IrisHal();

  bool ready();
  int irisConfigureSet(int type, std::vector<int> values);

 private:
  uint8_t reserved_[32];
};

}  // namespace pxlw
