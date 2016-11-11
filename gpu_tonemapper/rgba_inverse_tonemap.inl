/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

const char* rgba_inverse_tonemap_shader = ""
    "#extension GL_OES_EGL_image_external_essl3 : require                                               \n"
    "precision highp float;                                                                             \n"
    "precision highp sampler2D;                                                                         \n"
    "layout(binding = 0) uniform samplerExternalOES externalTexture;                                    \n"
    "layout(binding = 1) uniform sampler3D tonemapper;                                                  \n"
    "layout(binding = 2) uniform sampler2D xform;                                                       \n"
    "in vec2 uv;                                                                                        \n"
    "out vec4 fs_color;                                                                                 \n"
    "void main()                                                                                        \n"
    "{                                                                                                  \n"
    "vec2 flipped = uv;                                                                                 \n"
    "flipped.y = 1.0 - flipped.y;                                                                       \n"
    "flipped.x = flipped.x;                                                                             \n"
    "vec4 rgb_premulalpha = texture(externalTexture, flipped);                                          \n"
    "fs_color = rgb_premulalpha;                                                                        \n"
    "if( rgb_premulalpha.a > 0.0 ) {                                                                    \n"
    "vec3 rgb = rgb_premulalpha.rgb/rgb_premulalpha.a;                                                  \n"
    "#if defined(USE_NONUNIFORM_SAMPLING)                                                               \n"
    "float r = texture(xform, vec2(rgb.r, 0.0f)).r;                                                     \n"
    "float g = texture(xform, vec2(rgb.g, 0.0f)).g;                                                     \n"
    "float b = texture(xform, vec2(rgb.b, 0.0f)).b;                                                     \n"
    "#else                                                                                              \n"
    "float r = rgb.r;                                                                                   \n"
    "float g = rgb.g;                                                                                   \n"
    "float b = rgb.b;                                                                                   \n"
    "#endif                                                                                             \n"
    "fs_color.rgb = texture(tonemapper, vec3(r, g, b)).rgb * rgb_premulalpha.a;                         \n"
    "fs_color.a = rgb_premulalpha.a;                                                                    \n"
    "}                                                                                                  \n"
    "}                                                                                                  \n";
