LOCAL_PATH := $(call my-dir)
include $(LOCAL_PATH)/../common.mk
include $(CLEAR_VARS)

LOCAL_MODULE                  := libexternal
LOCAL_MODULE_PATH             := $(TARGET_OUT_SHARED_LIBRARIES)
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes)
LOCAL_SHARED_LIBRARIES        := $(common_libs) libmedia
LOCAL_CFLAGS                  := $(common_flags) -DLOG_TAG=\"qdexternal\"
LOCAL_HEADER_LIBRARIES        := generated_kernel_headers
LOCAL_SRC_FILES               := external.cpp

include $(BUILD_SHARED_LIBRARY)
