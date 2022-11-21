/*
 * Copyright (c) 2011-2021, The Linux Foundation. All rights reserved.

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

#include <log/log.h>
#include <cutils/properties.h>
#include <algorithm>
#include <vector>
#include <string>

#ifndef QMAA
#include <linux/msm_ion.h>
#endif

#include "gr_allocator.h"
#include "gr_utils.h"
#include "gralloc_priv.h"

#include "qd_utils.h"
#include "gr_alloc_interface.h"

using std::shared_ptr;
using std::vector;

namespace gralloc {

static BufferInfo GetBufferInfo(const BufferDescriptor &descriptor) {
  return BufferInfo(descriptor.GetWidth(), descriptor.GetHeight(), descriptor.GetFormat(),
                    descriptor.GetUsage());
}

void Allocator::SetProperties(gralloc::GrallocProperties props) {
  use_system_heap_for_sensors_ = props.use_system_heap_for_sensors;
}

int Allocator::AllocateMem(AllocData *alloc_data, uint64_t usage, int format) {
  int ret;
  int err = 0;
  alloc_data->uncached = UseUncached(format, usage);

  AllocInterface *alloc_intf = AllocInterface::GetInstance();
  if (!alloc_intf) {
    return -ENOMEM;
  }

  // After this point we should have the right heap set, there is no fallback

  alloc_intf->GetHeapInfo(usage, use_system_heap_for_sensors_, &alloc_data->heap_name,
                          &alloc_data->vm_names, &alloc_data->alloc_type, &alloc_data->flags,
                          &alloc_data->size);

  ret = alloc_intf->AllocBuffer(alloc_data);
  if (ret >= 0) {
    alloc_data->alloc_type |= private_handle_t::PRIV_FLAGS_USES_ION;
  } else {
    ALOGE("%s: Failed to allocate buffer - heap name: %s flags: 0x%x ret: %d", __FUNCTION__,
          alloc_data->heap_name.c_str(), alloc_data->flags, ret);
  }

  if (!alloc_data->vm_names.empty()) {
    err = alloc_intf->SecureMemPerms(alloc_data);
  }

  if (err) {
    ALOGE("%s: Failed to modify secure use permissions - heap name: %s flags: 0x%x, err: %d"
          , __FUNCTION__, alloc_data->heap_name.c_str(), alloc_data->flags, err);
  }

  return ret;
}

int Allocator::MapBuffer(void **base, unsigned int size, unsigned int offset, int fd) {
  AllocInterface *alloc_intf = AllocInterface::GetInstance();
  if (!alloc_intf) {
    return -ENOMEM;
  }
  if (alloc_intf) {
    return alloc_intf->MapBuffer(base, size, offset, fd);
  }

  return -EINVAL;
}

int Allocator::ImportBuffer(int fd) {
  AllocInterface *alloc_intf = AllocInterface::GetInstance();
  if (!alloc_intf) {
    return -ENOMEM;
  }
  if (alloc_intf) {
    return alloc_intf->ImportBuffer(fd);
  }

  return -EINVAL;
}

int Allocator::FreeBuffer(void *base, unsigned int size, unsigned int offset, int fd, int handle) {
  AllocInterface *alloc_intf = AllocInterface::GetInstance();
  if (!alloc_intf) {
    return -ENOMEM;
  }
  if (alloc_intf) {
    return alloc_intf->FreeBuffer(base, size, offset, fd, handle);
  }

  return -EINVAL;
}

int Allocator::CleanBuffer(void *base, unsigned int size, unsigned int offset, int handle, int op,
                           int fd) {
  AllocInterface *alloc_intf = AllocInterface::GetInstance();
  if (!alloc_intf) {
    return -ENOMEM;
  }
  if (alloc_intf) {
    return alloc_intf->CleanBuffer(base, size, offset, handle, op, fd);
  }

  return -EINVAL;
}

bool Allocator::CheckForBufferSharing(uint32_t num_descriptors,
                                      const vector<shared_ptr<BufferDescriptor>> &descriptors,
                                      ssize_t *max_index) {
  std::string cur_heap_name = "", prev_heap_name = "";
  std::vector<std::string> cur_vm_names, prev_vm_names;
  unsigned int cur_alloc_type = 0, prev_alloc_type = 0;
  unsigned int cur_flags = 0, prev_flags = 0;
  bool cur_uncached = false, prev_uncached = false;
  unsigned int alignedw, alignedh;
  unsigned int max_size = 0;
  unsigned int cur_size = 0, prev_size = 0;

  *max_index = -1;

  AllocInterface *alloc_intf = AllocInterface::GetInstance();
  if (!alloc_intf) {
    return false;
  }

  for (uint32_t i = 0; i < num_descriptors; i++) {
    // Check Cached vs non-cached and all the flags
    cur_uncached = UseUncached(descriptors[i]->GetFormat(), descriptors[i]->GetUsage());
    alloc_intf->GetHeapInfo(descriptors[i]->GetUsage(), use_system_heap_for_sensors_,
                            &cur_heap_name, &cur_vm_names, &cur_alloc_type, &cur_flags, &cur_size);

    if (i > 0 && (cur_heap_name != prev_heap_name || cur_alloc_type != prev_alloc_type ||
                  cur_flags != prev_flags || cur_vm_names != prev_vm_names ||
                  cur_size != prev_size)) {
      return false;
    }

    // For same format type, find the descriptor with bigger size
    int err = GetAlignedWidthAndHeight(GetBufferInfo(*descriptors[i]), &alignedw, &alignedh);
    if (err) {
      return false;
    }
    unsigned int size = GetSize(GetBufferInfo(*descriptors[i]), alignedw, alignedh);
    if (max_size < size) {
      *max_index = INT(i);
      max_size = size;
    }

    prev_heap_name = cur_heap_name;
    prev_uncached = cur_uncached;
    prev_flags = cur_flags;
    prev_alloc_type = cur_alloc_type;
    prev_vm_names = cur_vm_names;
    prev_size = cur_size;
  }

  return true;
}
}  // namespace gralloc
