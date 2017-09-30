# Copyright (C) 2008 The Android Open Source Project
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
# HAL module implemenation stored in
# hw/<COPYPIX_HARDWARE_MODULE_ID>.<ro.board.platform>.so
include $(CLEAR_VARS)

ifneq ($(TARGET_USES_LEGACY_LIBLIGHT),true)
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/common/inc
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/qdcm/inc
endif

LOCAL_SRC_FILES := lights.c
ifneq ($(TARGET_USES_LEGACY_LIBLIGHT),true)
LOCAL_SRC_FILES += lights_prv.cpp
endif

LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_SHARED_LIBRARIES := liblog libcutils
ifneq ($(TARGET_USES_LEGACY_LIBLIGHT),true)
LOCAL_SHARED_LIBRARIES += libsdm-disp-vndapis
endif

LOCAL_CFLAGS := -DLOG_TAG=\"qdlights\"
ifeq ($(TARGET_USES_LEGACY_LIBLIGHT),true)
LOCAL_CFLAGS += -DLEGACY_COMPATIBILITY
endif

LOCAL_CLANG  := true
LOCAL_MODULE := lights.$(TARGET_BOARD_PLATFORM)
LOCAL_MODULE_TAGS := optional
LOCAL_VENDOR_MODULE := true

include $(BUILD_SHARED_LIBRARY)
