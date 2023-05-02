/*
 * Copyright (c) 2016-2020 The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *
 * Copyright 2015 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "EGLImageWrapper.h"
#include <cutils/native_handle.h>
#include <QtiGralloc.h>
#include <QtiGrallocPriv.h>
#include <ui/GraphicBuffer.h>
#include <fcntl.h>
#include <string>
#include <map>
#include <utility>
#include <sys/stat.h>

using std::string;
using std::map;
using std::pair;
using aidl::android::hardware::graphics::common::StandardMetadataType;
using private_handle_t = qtigralloc::private_handle_t;

static string pidString = std::to_string(getpid());

//-----------------------------------------------------------------------------
static string get_ion_buff_str(int buff_fd)
//-----------------------------------------------------------------------------
{
  string retStr = {};
  if (buff_fd >= 0) {
    struct stat stat1;
    fstat(buff_fd, &stat1);
    retStr = std::to_string(stat1.st_ino);
  }

  return retStr;
}

//-----------------------------------------------------------------------------
void EGLImageWrapper::DeleteEGLImageCallback::operator()(int& buffInt, EGLImageBuffer*& eglImage)
//-----------------------------------------------------------------------------
{
  if (eglImage != 0) {
    delete eglImage;
  }

  if (!mapClearPending) {
    for (auto it = buffStrbuffIntMapPtr->begin(); it != buffStrbuffIntMapPtr->end(); it++) {
      if (it->second == buffInt /* counter */) {
        buffStrbuffIntMapPtr->erase(it);
        return;
      }
    }
  }
}

//-----------------------------------------------------------------------------
EGLImageWrapper::EGLImageWrapper()
//-----------------------------------------------------------------------------
{
  Init();
}

//-----------------------------------------------------------------------------
EGLImageWrapper::~EGLImageWrapper()
//-----------------------------------------------------------------------------
{
  Deinit();
}

//-----------------------------------------------------------------------------
void EGLImageWrapper::Init()
//-----------------------------------------------------------------------------
{
  eglImageBufferCache = new android::LruCache<int, EGLImageBuffer*>(32);
  callback = new DeleteEGLImageCallback(&buffStrbuffIntMap);
  eglImageBufferCache->setOnEntryRemovedListener(callback);
}

//-----------------------------------------------------------------------------
void EGLImageWrapper::Deinit()
//-----------------------------------------------------------------------------
{
  if (eglImageBufferCache != 0) {
    if (callback != 0) {
      callback->mapClearPending = true;
    }
    eglImageBufferCache->clear();
    delete eglImageBufferCache;
    eglImageBufferCache = 0;
    buffStrbuffIntMap.clear();
  }

  if (callback != 0) {
    delete callback;
    callback = 0;
  }

}

//-----------------------------------------------------------------------------
static EGLImageBuffer *L_wrap(const private_handle_t *src)
//-----------------------------------------------------------------------------
{
  EGLImageBuffer* result = 0;

  uint32_t unaligned_width = src->unaligned_width;
  uint32_t unaligned_height = src->unaligned_height;
  uint32_t stride = src->width;
  native_handle_t *native_handle = const_cast<private_handle_t *>(src);

  CropRectangle_t crop;
  if (gralloc::GetMetaDataValue(const_cast<private_handle_t *>(src),
                                (int64_t)StandardMetadataType::CROP,
                                &crop) == gralloc::Error::NONE) {
    unaligned_width = crop.right;
    unaligned_height = crop.bottom;
    uint32_t aligned_height = 0;
    gralloc::BufferInfo info(unaligned_width, unaligned_height, src->format, src->usage);
    gralloc::GetAlignedWidthAndHeight(info, &stride, &aligned_height);
  }

  int flags = android::GraphicBuffer::USAGE_HW_TEXTURE |
              android::GraphicBuffer::USAGE_SW_READ_NEVER |
              android::GraphicBuffer::USAGE_SW_WRITE_NEVER;

  if (src->flags & qtigralloc::PRIV_FLAGS_SECURE_BUFFER) {
    flags |= android::GraphicBuffer::USAGE_PROTECTED;
  }

  android::sp<android::GraphicBuffer> graphicBuffer =
    new android::GraphicBuffer(unaligned_width, unaligned_height, src->format,
                               1,  // Layer count
                               flags, stride /*src->stride*/,
                               native_handle, false);

  result = new EGLImageBuffer(graphicBuffer);

  return result;
}

//-----------------------------------------------------------------------------
EGLImageBuffer *EGLImageWrapper::wrap(const void *pvt_handle)
//-----------------------------------------------------------------------------
{
  const private_handle_t *src = static_cast<const private_handle_t *>(pvt_handle);

  string buffStr = get_ion_buff_str(src->fd);
  EGLImageBuffer* eglImage = nullptr;
  if (!buffStr.empty()) {
    auto it = buffStrbuffIntMap.find(buffStr);
    if (it != buffStrbuffIntMap.end()) {
      eglImage = eglImageBufferCache->get(it->second);
    } else {
        eglImage = L_wrap(src);
        buffStrbuffIntMap.insert(pair<string, int>(buffStr, buffInt));
        eglImageBufferCache->put(buffInt, eglImage);
        buffInt++;
    }
  } else {
    ALOGE("Could not provide an eglImage for fd = %d, EGLImageWrapper = %p", src->fd, this);
  }

  return eglImage;
}
