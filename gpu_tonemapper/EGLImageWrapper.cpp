/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#include "EGLImageWrapper.h"
#include <cutils/native_handle.h>
#include <gralloc_priv.h>
#include <ui/GraphicBuffer.h>

//-----------------------------------------------------------------------------
EGLImageBuffer *EGLImageWrapper::wrap(const void *pvt_handle)
//-----------------------------------------------------------------------------
{
  const private_handle_t *src = static_cast<const private_handle_t *>(pvt_handle);

  EGLImageBuffer *result = 0;
  std::map<int, EGLImageBuffer *>::iterator it = eglImageBufferMap.find(src->fd);
  if (it == eglImageBufferMap.end()) {
    native_handle_t *native_handle = const_cast<private_handle_t *>(src);

    int flags = android::GraphicBuffer::USAGE_HW_TEXTURE |
                android::GraphicBuffer::USAGE_SW_READ_NEVER |
                android::GraphicBuffer::USAGE_SW_WRITE_NEVER;
    if (src->flags & private_handle_t::PRIV_FLAGS_SECURE_BUFFER) {
      flags |= android::GraphicBuffer::USAGE_PROTECTED;
    }

    android::sp<android::GraphicBuffer> graphicBuffer =
        new android::GraphicBuffer(src->width, src->height, src->format,
#ifndef __NOUGAT__
                                   1, // Layer count
#endif
                                   flags, src->width /*src->stride*/,
                                   native_handle, false);

    result = new EGLImageBuffer(graphicBuffer);

    eglImageBufferMap[src->fd] = result;
  } else {
    result = it->second;
  }

  return result;
}

//-----------------------------------------------------------------------------
void EGLImageWrapper::destroy()
//-----------------------------------------------------------------------------
{
  std::map<int, EGLImageBuffer *>::iterator it = eglImageBufferMap.begin();
  for (; it != eglImageBufferMap.end(); it++) {
    delete it->second;
  }
  eglImageBufferMap.clear();
}
