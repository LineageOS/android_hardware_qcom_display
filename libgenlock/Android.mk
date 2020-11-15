LOCAL_PATH := $(call my-dir)
include $(LOCAL_PATH)/../common.mk
include $(CLEAR_VARS)

LOCAL_MODULE                  := libgenlock
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_TAGS             := optional
LOCAL_SHARED_LIBRARIES        := liblog libcutils
LOCAL_CFLAGS                  := $(common_flags) -DLOG_TAG=\"qdgenlock\"
LOCAL_HEADER_LIBRARIES        := generated_kernel_headers display_headers
LOCAL_SRC_FILES               := genlock.cpp

include $(BUILD_SHARED_LIBRARY)
