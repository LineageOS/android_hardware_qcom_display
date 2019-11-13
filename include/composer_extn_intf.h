/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
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
 */

#ifndef __COMPOSER_EXTN_INTF_H__
#define __COMPOSER_EXTN_INTF_H__

#include <dlfcn.h>

namespace composer {

class ComposerExtnIntf {
 protected:
  virtual ~ComposerExtnIntf() { }
};

class ComposerExtnLib {
 public:
  static ComposerExtnIntf * GetInstance() {
    return g_composer_ext_lib_.composer_ext_intf_;
  }

 private:
  const char *lib_name = "libcomposerextn.qti.so";

  typedef int (*CreateComposerExtnIntf)(ComposerExtnIntf **intf);
  typedef void (*DestroyComposerExtnIntf)(ComposerExtnIntf *intf);

  ComposerExtnLib() {
    lib_obj_ = ::dlopen(lib_name, RTLD_NOW);
    if (!lib_obj_) {
      return;
    }

    create_composer_ext_intf_ = reinterpret_cast<CreateComposerExtnIntf>(
                                    ::dlsym(lib_obj_, "CreateComposerExtnIntf"));
    destroy_composer_ext_intf_ = reinterpret_cast<DestroyComposerExtnIntf>(
                                    ::dlsym(lib_obj_, "DestroyComposerExtnIntf"));
    if (create_composer_ext_intf_ && destroy_composer_ext_intf_) {
      create_composer_ext_intf_(&composer_ext_intf_);
    }
  }

  ~ComposerExtnLib() {
    if (composer_ext_intf_) {
      destroy_composer_ext_intf_(composer_ext_intf_);
    }

    if (lib_obj_) {
      ::dlclose(lib_obj_);
    }
  }

  static ComposerExtnLib g_composer_ext_lib_;
  void *lib_obj_ = nullptr;
  CreateComposerExtnIntf create_composer_ext_intf_ = nullptr;
  DestroyComposerExtnIntf destroy_composer_ext_intf_ = nullptr;
  ComposerExtnIntf *composer_ext_intf_ = nullptr;
};

}  // namespace composer

#endif  // __COMPOSER_EXTN_INTF_H__
