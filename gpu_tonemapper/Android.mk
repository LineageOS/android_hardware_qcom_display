LOCAL_PATH := $(call my-dir)
include $(LOCAL_PATH)/../common.mk

include $(CLEAR_VARS)
LOCAL_COPY_HEADERS_TO     := $(common_header_export_path)
LOCAL_COPY_HEADERS        := TonemapFactory.h Tonemapper.h
LOCAL_VENDOR_MODULE       := true
include $(BUILD_COPY_HEADERS)

include $(CLEAR_VARS)
LOCAL_MODULE              := libgpu_tonemapper
LOCAL_VENDOR_MODULE       := true
LOCAL_MODULE_TAGS         := optional
LOCAL_HEADER_LIBRARIES    := display_headers
LOCAL_C_INCLUDES          += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_SHARED_LIBRARIES    := libEGL libGLESv2 libGLESv3 libui libutils liblog
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_CFLAGS              := $(version_flag) -Wno-missing-field-initializers -Wall \
                             -Wno-unused-parameter -DLOG_TAG=\"GPU_TONEMAPPER\"

LOCAL_SRC_FILES           := TonemapFactory.cpp \
                             glengine.cpp \
                             EGLImageBuffer.cpp \
                             EGLImageWrapper.cpp \
                             Tonemapper.cpp

include $(BUILD_SHARED_LIBRARY)
