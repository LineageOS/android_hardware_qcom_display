/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

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

#include "gl_layer_stitch_impl.h"

#define __CLASS__ "GLLayerStitchImpl"

namespace sdm {

const float kFullScreenVertices[] = {
  -1.0f,  3.0f,
  -1.0f, -1.0f,
  3.0f, -1.0f
};

const float kFullScreenTexCoords[] = {
  0.0f, 2.0f,
  0.0f, 0.0f,
  2.0f, 0.0f
};

const char *kVertexShader1 = ""
  "#version 300 es                                                       \n"
  "precision highp float;                                                \n"
  "layout(location = 0) in vec2 in_pos;                                  \n"
  "layout(location = 1) in vec2 in_uv;                                   \n"
  "                                                                      \n"
  "out vec2 uv;                                                          \n"
  "                                                                      \n"
  "void main()                                                           \n"
  "{                                                                     \n"
  "    gl_Position = vec4(in_pos, 0.0, 1.0);                             \n"
  "    uv = in_uv;                                                       \n"
  "}                                                                     \n";

const char *kConvertRenderRGBShader = ""
  "precision highp float;                                                \n"
  "                                                                      \n"
  "layout(binding = 0) uniform sampler2D u_sTexture;                     \n"
  "                                                                      \n"
  "in vec2 uv;                                                           \n"
  "out vec4 color;                                                       \n"
  "                                                                      \n"
  "void main()                                                           \n"
  "{                                                                     \n"
  "    color = texture(u_sTexture, uv);                                  \n"
  "}                                                                     \n";

int GLLayerStitchImpl::CreateContext(bool secure) {
  ctx_.egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  EGL(eglBindAPI(EGL_OPENGL_ES_API));

  // Initialize current display.
  EGL(eglInitialize(ctx_.egl_display, nullptr, nullptr));

  // Get attributes corresponing to render target.
  // Describes Framebuffer attributes like buffer depth, color space etc;
  EGLConfig eglConfig;
  int numConfig = 0;
  EGLint eglConfigAttribList[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
                                  EGL_RED_SIZE,     8,
                                  EGL_GREEN_SIZE,   8,
                                  EGL_BLUE_SIZE,    8,
                                  EGL_ALPHA_SIZE,   8,
                                  EGL_NONE};
  EGL(eglChooseConfig(ctx_.egl_display, eglConfigAttribList, &eglConfig, 1, &numConfig));

  // When GPU runs in protected context it can read from
  //  - Protected sources
  //  - UnProtected source
  // and writes into Protected buffer.
  // VS in UnProtected context it can read/write happen from/to Unprotected sources.
  EGLint egl_contextAttribList[] = {EGL_CONTEXT_CLIENT_VERSION, 3,
                                    secure ? EGL_PROTECTED_CONTENT_EXT : EGL_NONE,
                                    secure ? EGL_TRUE : EGL_NONE,
                                    EGL_NONE};
  ctx_.egl_context = eglCreateContext(ctx_.egl_display, eglConfig, NULL, egl_contextAttribList);

  // eglCreatePbufferSurface creates an off-screen pixel buffer surface and returns its handle
  EGLint egl_surfaceAttribList[] = {EGL_WIDTH, 1,
                                    EGL_HEIGHT, 1,
                                    secure ? EGL_PROTECTED_CONTENT_EXT : EGL_NONE,
                                    secure ? EGL_TRUE : EGL_NONE,
                                    EGL_NONE};
  ctx_.egl_surface = eglCreatePbufferSurface(ctx_.egl_display, eglConfig, egl_surfaceAttribList);

  // eglMakeCurrent attaches rendering context to rendering surface.
  MakeCurrent(&ctx_);

  DLOGI("Created context = %p", (void *)(&ctx_.egl_context));

  // Load Vertex and Fragment shaders.
  const char *fragment_shaders[2] = { };
  int count = 0;
  const char *version = "#version 300 es\n";

  fragment_shaders[count++] = version;

  // ToDo: Add support to yuv_to_rgb shader.
  fragment_shaders[count++] = kConvertRenderRGBShader;

  ctx_.program_id = LoadProgram(1, &kVertexShader1, count, fragment_shaders);

  return 0;
}

int GLLayerStitchImpl::Blit(const private_handle_t *src_hnd, const private_handle_t *dst_hnd,
                            const GLRect &src_rect, const GLRect &dst_rect,
                            int src_acquire_fence_fd, int dst_acquire_fence_fd,
                            int *release_fence_fd) {
  DTRACE_SCOPED();
  // eglMakeCurrent attaches rendering context to rendering surface.
  MakeCurrent(&ctx_);

  SetProgram(ctx_.program_id);

  SetSourceBuffer(src_hnd);
  SetDestinationBuffer(dst_hnd, dst_rect);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, kFullScreenVertices);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, kFullScreenTexCoords);
  glDrawArrays(GL_TRIANGLES, 0, 3);

  // Dst. is always guaranteed to be signaled.
  int in_fence_fd = dup(src_acquire_fence_fd);
  if (in_fence_fd >= 0) {
    WaitOnInputFence(in_fence_fd);
  }

  // Create output fence for client to wait on.
  *release_fence_fd = CreateOutputFence();

  return 0;
}

int GLLayerStitchImpl::Init() {
  return CreateContext(secure_);
}

int GLLayerStitchImpl::Deinit() {
  MakeCurrent(&ctx_);
  DestroyContext(&ctx_);

  return 0;
}

GLLayerStitchImpl::~GLLayerStitchImpl() {}

GLLayerStitchImpl::GLLayerStitchImpl(bool secure) {
  secure_ = secure;
}

}  // namespace sdm

