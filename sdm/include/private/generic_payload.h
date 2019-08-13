/* Copyright (c) 2020, The Linux Foundataion. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
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
*
*/

#ifndef __GENERIC_PAYLOAD_H__
#define __GENERIC_PAYLOAD_H__

#include <errno.h>
#include <functional>

namespace sdm {

struct GenericPayload {
 public:
  GenericPayload():
    type_size(0), payload(nullptr), array_size(0) {}

  template<typename A> int CreatePayload(A *&p) {
    if (payload) {
      p = nullptr;
      return -EALREADY;
    }

    p = new A();
    if (p == nullptr) {
      return -ENOMEM;
    }

    type_size = sizeof(A);
    array_size = 1;
    payload = reinterpret_cast<uint8_t *>(p);
    release = std::function<void(void)>([p]() -> void {delete p;});

    return 0;
  }

  template<typename A> int CreatePayload(A *&p, uint32_t sz) {
    if (payload) {
      p = nullptr;
      return -EALREADY;
    }

    if (!sz) {
      return -EINVAL;
    }

    p = new A[sz];
    if (p == nullptr) {
      return -ENOMEM;
    }

    type_size = sizeof(A);
    array_size = sz;
    payload = reinterpret_cast<uint8_t *>(p);
    release = std::function<void(void)>([p]() -> void {delete [] p;});

    return 0;
  }

  template<typename A> int GetPayload(A *&p, uint32_t *sz) const {
    if ((sz == nullptr) || (sizeof(A) != type_size)) {
      p = nullptr;
      return -EINVAL;
    }

    p = reinterpret_cast<A *>(payload);
    *sz = array_size;

    return 0;
  }

  void DeletePayload() {
    if (payload != nullptr) {
      release();
    }

    type_size = 0;
    payload = nullptr;
    array_size = 0;
  }

  ~GenericPayload() {
    DeletePayload();
  }

 private:
  uint32_t type_size;
  uint8_t *payload;
  uint32_t array_size;
  std::function<void(void)> release;
};

}  // namespace sdm

#endif  // __GENERIC_PAYLOAD_H__

