LOCAL_PATH := $(call my-dir)
include $(LOCAL_PATH)/../common.mk
include $(CLEAR_VARS)

LOCAL_MODULE                  := libvirtual
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_TAGS             := optional
LOCAL_SHARED_LIBRARIES        := $(common_libs) liboverlay libqdutils libmedia
LOCAL_CFLAGS                  := $(common_flags) -DLOG_TAG=\"qdvirtual\"
LOCAL_HEADER_LIBRARIES        := generated_kernel_headers display_headers
LOCAL_SRC_FILES               := virtual.cpp
LOCAL_STATIC_LIBRARIES        := libbase

include $(BUILD_SHARED_LIBRARY)
