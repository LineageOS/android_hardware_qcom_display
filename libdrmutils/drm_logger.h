/*
* Copyright (c) 2017, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of The Linux Foundation nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
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

#ifndef __DRM_LOGGER_H__
#define __DRM_LOGGER_H__

#include <utility>

namespace drm_utils {

class DRMLogger {
 public:
  virtual ~DRMLogger() {}
  virtual void Error(const char *format, ...) = 0;
  virtual void Info(const char *format, ...) = 0;
  virtual void Debug(const char *format, ...) = 0;

  static void Set(DRMLogger *logger) { s_instance = logger; }
  static DRMLogger *Get() { return s_instance; }

 private:
  static DRMLogger *s_instance;
};

template <typename... T>
void DRM_LOGE(const char *format, T&&... args) {
  if (DRMLogger::Get()) {
    DRMLogger::Get()->Error(format, std::forward<T>(args)...);
  }
}

template <typename... T>
void DRM_LOGI(const char *format, T&&... args) {
  if (DRMLogger::Get()) {
    DRMLogger::Get()->Info(format, std::forward<T>(args)...);
  }
}

template <typename... T>
void DRM_LOGD_IF(bool pred, const char *format, T&&... args) {
  if (pred && DRMLogger::Get()) {
    DRMLogger::Get()->Debug(format, std::forward<T>(args)...);
  }
}

}  // namespace drm_utils

#endif  // __DRM_LOGGER_H__

