LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE                  := libdisplaydebug
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_TAGS             := optional
LOCAL_SHARED_LIBRARIES        := libdl
LOCAL_CFLAGS                  := -DLOG_TAG=\"SDM\" -Wall -Werror -fno-operator-names
LOCAL_CLANG                   := true
LOCAL_SRC_FILES               := debug_handler.cpp
LOCAL_COPY_HEADERS_TO         := qcom/display
LOCAL_COPY_HEADERS            := debug_handler.h

include $(BUILD_SHARED_LIBRARY)
