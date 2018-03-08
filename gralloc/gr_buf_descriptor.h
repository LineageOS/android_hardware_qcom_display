/*
 * Copyright (c) 2016-2018, The Linux Foundation. All rights reserved.

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

#ifndef __GR_BUF_DESCRIPTOR_H__
#define __GR_BUF_DESCRIPTOR_H__

#include <android/hardware/graphics/mapper/2.1/IMapper.h>
#include <atomic>

namespace gralloc {
using android::hardware::graphics::mapper::V2_0::Error;
using android::hardware::graphics::mapper::V2_1::IMapper;
using android::hardware::hidl_vec;

const uint32_t kBufferDescriptorSize = 7;
const uint32_t kMagicVersion = 0x76312E30;  // v1.0

class BufferDescriptor {
 public:
  BufferDescriptor() {}
  explicit BufferDescriptor(uint64_t id) : id_(id) {}

  static hidl_vec<uint32_t> Encode(const IMapper::BufferDescriptorInfo &bd_info) {
    hidl_vec<uint32_t> out;
    out.resize(kBufferDescriptorSize);
    out[0] = kMagicVersion;
    out[1] = bd_info.width;
    out[2] = bd_info.height;
    out[3] = bd_info.layerCount;
    out[4] = static_cast<uint32_t>(bd_info.format);
    out[5] = static_cast<uint32_t>(bd_info.usage);
    out[6] = static_cast<uint32_t>(bd_info.usage >> 32);
    return out;
  }

  Error Decode(const hidl_vec<uint32_t> &in) {
    if (in.size() != kBufferDescriptorSize || in[0] != kMagicVersion) {
      return Error::BAD_DESCRIPTOR;
    }
    width_ = static_cast<int32_t>(in[1]);
    height_ = static_cast<int32_t>(in[2]);
    layer_count_ = in[3];
    format_ = static_cast<int32_t>(in[4]);
    usage_ = static_cast<uint64_t>(in[6]) << 32 | in[5];
    return Error::NONE;
  }

  void SetUsage(uint64_t usage) { usage_ |= usage; }

  void SetDimensions(int w, int h) {
    width_ = w;
    height_ = h;
  }

  void SetColorFormat(int format) { format_ = format; }

  void SetLayerCount(uint32_t layer_count) { layer_count_ = layer_count; }

  uint64_t GetUsage() const { return usage_; }

  int GetWidth() const { return width_; }

  int GetHeight() const { return height_; }

  int GetFormat() const { return format_; }

  uint32_t GetLayerCount() const { return layer_count_; }

  uint64_t GetId() const { return id_; }

 private:
  int width_ = -1;
  int height_ = -1;
  int format_ = -1;
  uint32_t layer_count_ = 1;
  uint64_t usage_ = 0;
  const uint64_t id_ = 0;
};
};      // namespace gralloc
#endif  // __GR_BUF_DESCRIPTOR_H__
