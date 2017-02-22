LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE                  := libsdmutils

ifneq ($(TARGET_IS_HEADLESS), true)
LOCAL_MODULE_PATH_32          := $(TARGET_OUT_VENDOR)/lib
LOCAL_MODULE_PATH_64          := $(TARGET_OUT_VENDOR)/lib64
endif

LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(call project-path-for,qcom-display)/sdm/include/
LOCAL_CFLAGS                  := -Wno-missing-field-initializers -Wno-unused-parameter \
                                 -Wconversion -Wall -Werror \
                                 -DLOG_TAG=\"SDM\"
LOCAL_SRC_FILES               := debug.cpp rect.cpp

include $(BUILD_SHARED_LIBRARY)
