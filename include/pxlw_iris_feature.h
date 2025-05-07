/*
 * SPDX-FileCopyrightText: 2025 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

namespace pxlw {

class IrisFeature {
 public:
  static IrisFeature* getInstance();
  bool hasSoftIris();
  bool hasIrisDual();

 private:
  IrisFeature() = default;
  static IrisFeature* instance_;
};

}  // namespace pxlw
