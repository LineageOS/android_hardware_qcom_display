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

#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm/drm_fourcc.h>

#include <algorithm>
#include <iterator>

#include "drm_master.h"

#define __CLASS__ "DRMMaster"

using std::mutex;
using std::lock_guard;
using std::begin;
using std::copy;
using std::end;
using std::fill;

namespace drm_utils {

DRMLogger *DRMLogger::s_instance = nullptr;
DRMMaster *DRMMaster::s_instance = nullptr;
mutex DRMMaster::s_lock;

int DRMMaster::GetInstance(DRMMaster **master) {
  lock_guard<mutex> obj(s_lock);

  if (!s_instance) {
    s_instance = new DRMMaster();
    if (s_instance->Init() < 0) {
      delete s_instance;
      s_instance = nullptr;
      return -ENODEV;
    }
  }

  *master = s_instance;
  return 0;
}

int DRMMaster::Init() {
  dev_fd_ = drmOpen("msm_drm", nullptr);
  if (dev_fd_ < 0) {
    DRM_LOGE("%s::%s: drmOpen failed with error %d", __CLASS__, __FUNCTION__, dev_fd_);
    return -ENODEV;
  }

  return 0;
}

DRMMaster::~DRMMaster() {
  drmClose(dev_fd_);
  dev_fd_ = -1;
}

int DRMMaster::CreateFbId(const DRMBuffer &drm_buffer, uint32_t *gem_handle, uint32_t *fb_id) {
  int ret = drmPrimeFDToHandle(dev_fd_, drm_buffer.fd, gem_handle);
  if (ret) {
    DRM_LOGE("%s::%s: drmPrimeFDToHandle failed with error %d", __CLASS__, __FUNCTION__, ret);
    return ret;
  }

  uint32_t gem_handles[4] = {0};
  uint32_t pitches[4] = {0};
  uint32_t offsets[4] = {0};
  uint64_t modifier[4] = {0};

  fill(begin(gem_handles), begin(gem_handles) + drm_buffer.num_planes, *gem_handle);
  copy(begin(drm_buffer.stride), end(drm_buffer.stride), begin(pitches));
  copy(begin(drm_buffer.offset), end(drm_buffer.offset), begin(offsets));
  fill(begin(modifier), begin(modifier) + drm_buffer.num_planes, drm_buffer.drm_format_modifier);

  ret = drmModeAddFB3(dev_fd_, drm_buffer.width, drm_buffer.height, drm_buffer.drm_format,
                      gem_handles, pitches, offsets, modifier, fb_id, DRM_MODE_FB_MODIFIERS);
  if (ret) {
    DRM_LOGE("%s::%s: drmModeAddFB3 failed with error %d", __CLASS__, __FUNCTION__, ret);
    struct drm_gem_close gem_close = {};
    gem_close.handle = *gem_handle;
    int ret1 = drmIoctl(dev_fd_, DRM_IOCTL_GEM_CLOSE, &gem_close);
    if (ret1) {
      DRM_LOGE("drmIoctl::DRM_IOCTL_GEM_CLOSE failed with error %d", ret1);
    }
  }

  return ret;
}

int DRMMaster::RemoveFbId(uint32_t gem_handle, uint32_t fb_id) {
  int ret = drmModeRmFB(dev_fd_, fb_id);
  if (ret) {
    DRM_LOGE("%s::%s: drmModeRmFB failed with error %d", __CLASS__, __FUNCTION__, ret);
  }

  struct drm_gem_close gem_close = {};
  gem_close.handle = gem_handle;
  ret = drmIoctl(dev_fd_, DRM_IOCTL_GEM_CLOSE, &gem_close);
  if (ret) {
    DRM_LOGE("drmIoctl::DRM_IOCTL_GEM_CLOSE failed with error %d", ret);
  }

  return ret;
}

}  // namespace drm_utils
