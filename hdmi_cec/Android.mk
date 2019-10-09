LOCAL_PATH := $(call my-dir)
include $(LOCAL_PATH)/../common.mk
include $(CLEAR_VARS)

LOCAL_MODULE                  := hdmi_cec.$(TARGET_BOARD_PLATFORM)
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_RELATIVE_PATH    := hw
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes)
LOCAL_SHARED_LIBRARIES        := $(common_libs) libqservice libbinder libqdutils

LOCAL_CFLAGS                  := $(common_flags) -DLOG_TAG=\"qdhdmi_cec\"
LOCAL_HEADER_LIBRARIES        := generated_kernel_headers
LOCAL_SRC_FILES               := qhdmi_cec.cpp \
                                 QHDMIClient.cpp
include $(BUILD_SHARED_LIBRARY)
