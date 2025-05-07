/*
 * SPDX-FileCopyrightText: 2025 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

namespace pxlw {

class IrisFeature {
 public:
  static IrisFeature *getInstance();
  static IrisFeature *getFeature();

  ~IrisFeature() = default;

  bool hasIrisDual();
  bool hasSoftIris();
  bool hasIris5Dual();
  bool hasIris7Dual();
  bool hasIrisDualWithoutCSC();
  bool hasIris();
  bool hasIris5();
  bool hasIris7();

 private:
  IrisFeature() = default;
  static IrisFeature *mInstance;

  IrisFeature(const IrisFeature &) = delete;
  IrisFeature &operator=(const IrisFeature &) = delete;
  IrisFeature(IrisFeature &&) = delete;
  IrisFeature &operator=(IrisFeature &&) = delete;
};

}  // namespace pxlw
