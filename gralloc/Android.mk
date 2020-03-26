# Gralloc module
LOCAL_PATH := $(call my-dir)
include $(LOCAL_PATH)/../common.mk
include $(CLEAR_VARS)

LOCAL_MODULE                  := gralloc.$(TARGET_BOARD_PLATFORM)
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_RELATIVE_PATH    := hw
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes) $(kernel_includes)

LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_SHARED_LIBRARIES        := $(common_libs) libqdMetaData libsync libgrallocutils \
                                 libgralloccore \
                                 android.hardware.graphics.mapper@2.0 \
                                 android.hardware.graphics.mapper@2.1
LOCAL_CFLAGS                  := $(common_flags) -DLOG_TAG=\"qdgralloc\" -Wall -Werror
LOCAL_CLANG                   := true
LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)
LOCAL_SRC_FILES               := gr_device_impl.cpp
include $(BUILD_SHARED_LIBRARY)

#libgrallocutils
include $(CLEAR_VARS)
LOCAL_MODULE                  := libgrallocutils
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes) $(kernel_includes)
LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_SHARED_LIBRARIES        := $(common_libs) libqdMetaData libdl  \
                                  android.hardware.graphics.mapper@2.0 \
                                  android.hardware.graphics.mapper@2.1
LOCAL_CFLAGS                  := $(common_flags) -DLOG_TAG=\"qdgralloc\" -Wno-sign-conversion

ifeq ($(TARGET_USES_YCRCB_CAMERA_PREVIEW),true)
    LOCAL_CFLAGS              += -DUSE_YCRCB_CAMERA_PREVIEW
else ifeq ($(TARGET_USES_YCRCB_VENUS_CAMERA_PREVIEW),true)
    LOCAL_CFLAGS              += -DUSE_YCRCB_CAMERA_PREVIEW_VENUS
endif

ifeq ($(TARGET_USES_UNALIGNED_NV21_ZSL),true)
    LOCAL_CFLAGS              += -DUSE_UNALIGNED_NV21_ZSL
endif
ifeq ($(TARGET_USES_UNALIGNED_YCRCB),true)
    LOCAL_CFLAGS              += -DUSE_UNALIGNED_YCRCB
endif

LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)
LOCAL_SRC_FILES               := gr_utils.cpp gr_adreno_info.cpp
include $(BUILD_SHARED_LIBRARY)

#libgralloccore
include $(CLEAR_VARS)
LOCAL_MODULE                  := libgralloccore
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes) \
                                 system/core/libion/include \
                                 system/core/libion/kernel-headers \
                                 $(kernel_includes)

LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_SHARED_LIBRARIES        := $(common_libs) libqdMetaData libdl libgrallocutils libion \
                                  android.hardware.graphics.mapper@2.1
LOCAL_CFLAGS                  := $(common_flags) -DLOG_TAG=\"qdgralloc\" -Wno-sign-conversion
LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)
LOCAL_SRC_FILES               := gr_allocator.cpp gr_buf_mgr.cpp gr_ion_alloc.cpp
include $(BUILD_SHARED_LIBRARY)

#mapper
include $(CLEAR_VARS)
LOCAL_MODULE                  := android.hardware.graphics.mapper@2.0-impl-qti-display
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_RELATIVE_PATH    := hw
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes) $(kernel_includes)
LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_SHARED_LIBRARIES        := $(common_libs) \
                                  libhidlbase \
                                  libhidltransport \
                                  libqdMetaData \
                                  libgrallocutils \
                                  libgralloccore \
                                  libsync \
                                  vendor.qti.hardware.display.mapper@2.0 \
                                  vendor.qti.hardware.display.mapperextensions@1.0 \
                                  android.hardware.graphics.mapper@2.0 \
                                  android.hardware.graphics.mapper@2.1 \
                                  vendor.qti.hardware.display.mapperextensions@1.1
LOCAL_CFLAGS                  := $(common_flags) -DLOG_TAG=\"qdgralloc\" -Wno-sign-conversion
LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)
LOCAL_SRC_FILES               := QtiMapper.cpp QtiMapperExtensions.cpp
include $(BUILD_SHARED_LIBRARY)

#allocator
include $(CLEAR_VARS)
LOCAL_MODULE                  := vendor.qti.hardware.display.allocator-service
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_RELATIVE_PATH    := hw
LOCAL_MODULE_TAGS             := optional
LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_SHARED_LIBRARIES        := $(common_libs) \
                                 libhidlbase \
                                 libhidltransport\
                                 libqdMetaData \
                                 libgrallocutils \
                                 libgralloccore \
                                 vendor.qti.hardware.display.allocator@1.0 \
                                 android.hardware.graphics.mapper@2.1 \
                                 android.hardware.graphics.allocator@2.0
LOCAL_CFLAGS                  := -DLOG_TAG=\"qdgralloc\" $(common_flags)
LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)
LOCAL_C_INCLUDES              := $(common_includes) $(kernel_includes)
LOCAL_SRC_FILES               := QtiAllocator.cpp service.cpp
LOCAL_INIT_RC                 := vendor.qti.hardware.display.allocator-service.rc
include $(BUILD_EXECUTABLE)
