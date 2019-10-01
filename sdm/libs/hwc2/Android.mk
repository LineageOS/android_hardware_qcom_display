LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
include $(LOCAL_PATH)/../../../common.mk

LOCAL_MODULE                  := hwcomposer.$(TARGET_BOARD_PLATFORM)
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_RELATIVE_PATH    := hw
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes) \
                                 $(kernel_includes)
LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_CFLAGS                  := -Wno-missing-field-initializers -Wno-unused-parameter \
                                 -fcolor-diagnostics\
                                 -DLOG_TAG=\"SDM\" $(common_flags)

ifeq ($(TARGET_KERNEL_VERSION),$(filter $(TARGET_KERNEL_VERSION),4.14 4.19))
LOCAL_CFLAGS += -DTARGET_MIN_KERNEL_4_14
endif

LOCAL_CLANG                   := true

# TODO: Remove libui after addressing gpu_tonemapper issues
LOCAL_SHARED_LIBRARIES        := libsdmcore libqservice libbinder libhardware libhardware_legacy \
                                 libutils libcutils libsync libqdutils libqdMetaData libdl libdrmutils \
                                 libsdmutils libc++ liblog libgrallocutils libdl \
                                 vendor.display.config@2.0 libhidlbase \
                                 libdisplayconfig.qti libui libgpu_tonemapper

LOCAL_SRC_FILES               := hwc_session.cpp \
                                 hwc_session_services.cpp \
                                 hwc_display.cpp \
                                 hwc_display_primary.cpp \
                                 hwc_display_external.cpp \
                                 hwc_display_dummy.cpp \
                                 hwc_display_external_test.cpp \
                                 hwc_display_virtual.cpp \
                                 hwc_debugger.cpp \
                                 hwc_buffer_sync_handler.cpp \
                                 hwc_color_manager.cpp \
                                 hwc_layers.cpp \
                                 hwc_callbacks.cpp \
                                 cpuhint.cpp \
                                 hwc_tonemapper.cpp \
                                 display_null.cpp \
                                 hwc_socket_handler.cpp \
                                 hwc_buffer_allocator.cpp

ifeq ($(TARGET_HAS_WIDE_COLOR_DISPLAY), true)
    LOCAL_CFLAGS += -DFEATURE_WIDE_COLOR
endif

include $(BUILD_SHARED_LIBRARY)
