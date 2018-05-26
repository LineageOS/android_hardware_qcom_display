LOCAL_PATH := $(call my-dir)
include $(LOCAL_PATH)/../common.mk
include $(CLEAR_VARS)

LOCAL_MODULE                  := libqdutils
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_TAGS             := optional
LOCAL_SHARED_LIBRARIES        := $(common_libs) libbinder libqservice
LOCAL_C_INCLUDES              := $(common_includes) $(kernel_includes)
LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_HEADER_LIBRARIES        += libhardware_headers
LOCAL_CFLAGS                  := $(common_flags) -DLOG_TAG=\"qdutils\" -Wno-sign-conversion
LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)
LOCAL_COPY_HEADERS_TO         := $(common_header_export_path)
LOCAL_COPY_HEADERS            := display_config.h qd_utils.h
LOCAL_SRC_FILES               := profiler.cpp \
                                 qd_utils.cpp \
                                 display_config.cpp
include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)

LOCAL_COPY_HEADERS_TO           := $(common_header_export_path)
LOCAL_COPY_HEADERS              := qdMetaData.h
LOCAL_SHARED_LIBRARIES          := $(common_libs)
LOCAL_C_INCLUDES                := $(common_includes)
LOCAL_HEADER_LIBRARIES          := display_headers
LOCAL_ADDITIONAL_DEPENDENCIES   := $(common_deps)
LOCAL_SRC_FILES                 := qdMetaData.cpp
LOCAL_CFLAGS                    := $(common_flags) -Wno-sign-conversion
LOCAL_CFLAGS                    += -DLOG_TAG=\"DisplayMetaData\"

LOCAL_MODULE_TAGS               := optional
LOCAL_MODULE                    := libqdMetaData
LOCAL_VENDOR_MODULE             := true
include $(BUILD_SHARED_LIBRARY)


include $(CLEAR_VARS)

LOCAL_SHARED_LIBRARIES          := liblog libcutils
LOCAL_C_INCLUDES                := $(common_includes)
LOCAL_HEADER_LIBRARIES          := display_headers
LOCAL_ADDITIONAL_DEPENDENCIES   := $(common_deps)
LOCAL_SRC_FILES                 := qdMetaData.cpp qd_utils.cpp
LOCAL_CFLAGS                    := $(common_flags) -Wno-sign-conversion
LOCAL_CFLAGS                    += -DLOG_TAG=\"DisplayMetaData\"

LOCAL_MODULE_TAGS               := optional
LOCAL_MODULE                    := libqdMetaData.system

include $(BUILD_SHARED_LIBRARY)
