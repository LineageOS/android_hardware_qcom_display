# Copyright (C) 2018 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE := libhistogram
LOCAL_VENDOR_MODULE := true
LOCAL_MODULE_TAGS := optional
LOCAL_C_INCLUDES  := external/libdrm \
                     $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_HEADER_LIBRARIES := display_headers
LOCAL_SHARED_LIBRARIES := libdrm liblog libcutils libutils libbase
LOCAL_CFLAGS := -DLOG_TAG=\"SDM-histogram\" -Wall -std=c++14 -Werror -fno-operator-names \
	-Wthread-safety
LOCAL_CLANG  := true
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_SRC_FILES := histogram_collector.cpp ringbuffer.cpp

include $(BUILD_STATIC_LIBRARY)


include $(CLEAR_VARS)

LOCAL_HEADER_LIBRARIES := display_headers
LOCAL_C_INCLUDES  := external/libdrm \
                     $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_MODULE := color_sampling_tool
LOCAL_SRC_FILES := color_sampling_tool.cpp

LOCAL_STATIC_LIBRARIES := libhistogram
LOCAL_SHARED_LIBRARIES := libdrm liblog libcutils libutils libbase
LOCAL_CFLAGS := -DLOG_TAG=\"SDM-histogram\" -Wall -std=c++14 -Werror -fno-operator-names \
	-Wthread-safety
LOCAL_CLANG  := true
LOCAL_MODULE_TAGS := optional
LOCAL_VENDOR_MODULE := true

include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)

LOCAL_HEADER_LIBRARIES := display_headers
LOCAL_C_INCLUDES  := external/libdrm \
                     $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_MODULE := color_sampling_test
LOCAL_SRC_FILES := ringbuffer_test.cpp

LOCAL_STATIC_LIBRARIES := libhistogram libgtest libgmock
LOCAL_SHARED_LIBRARIES := libdrm liblog libcutils libutils libbase
LOCAL_CFLAGS := -DLOG_TAG=\"SDM-histogram\" -Wall -std=c++14 -Werror -fno-operator-names \
	-Wthread-safety
LOCAL_CLANG  := true
LOCAL_MODULE_TAGS := optional
LOCAL_VENDOR_MODULE := true

include $(BUILD_EXECUTABLE)
