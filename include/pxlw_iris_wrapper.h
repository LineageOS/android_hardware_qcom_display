/*
 * SPDX-FileCopyrightText: 2025 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>

namespace pxlw {

class PxlwIrisWrapper {
 public:
  static PxlwIrisWrapper *GetInstance();

 private:
  PxlwIrisWrapper() = default;
};

// HW iris 7 wrapper entry points exported by vendor libpwirishalwrapper.so.
// Only used when SUPPORTS_PXLW_IRIS7 is defined.
class PxlwIris7Wrapper {
 public:
  PxlwIris7Wrapper();
  virtual ~PxlwIris7Wrapper();

  bool HasSoftIris();

  int InitPrimaryDisplay(int32_t vsync_period_ns, uint32_t width, uint32_t height);
  void SetColorModeWithRenderIntent(int32_t display_id, int32_t, int32_t colorMode,
                                    int32_t renderIntent);

 private:
  friend class PxlwIrisWrapper;
};

class PxlwSoftirisWrapper {
 public:
  PxlwSoftirisWrapper();
  virtual ~PxlwSoftirisWrapper();

  bool HasSoftIris();

  int InitPrimaryDisplay(int32_t vsync_period_ns, uint32_t width, uint32_t height);
  void SetColorModeWithRenderIntent(int32_t display_id, int32_t, int32_t colorMode,
                                    int32_t renderIntent);

 private:
  friend class PxlwIrisWrapper;
};

}  // namespace pxlw
