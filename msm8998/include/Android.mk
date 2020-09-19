LOCAL_PATH:= $(call my-dir)
include $(LOCAL_PATH)/../common.mk

include $(CLEAR_VARS)
#TODO move all exported headers to this directory
LOCAL_MODULE                  := display_headers
LOCAL_LICENSE_KINDS           := SPDX-license-identifier-BSD
LOCAL_LICENSE_CONDITIONS      := notice
LOCAL_VENDOR_MODULE           := true
LOCAL_EXPORT_C_INCLUDE_DIRS   := $(LOCAL_PATH) \
                                 $(display_top)/libcopybit \
                                 $(display_top)/libdrmutils \
                                 $(display_top)/libqdutils \
                                 $(display_top)/libqservice \
                                 $(display_top)/gpu_tonemapper \
                                 $(display_top)/sdm/include \
                                 $(display_top)/libgralloc1

LOCAL_EXPORT_HEADER_LIBRARY_HEADERS := libhardware_headers
include $(BUILD_HEADER_LIBRARY)
