LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
include $(LOCAL_PATH)/../../../common.mk

ifeq ($(use_hwc2),true)

LOCAL_MODULE                  := hwcomposer.$(TARGET_BOARD_PLATFORM)
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_RELATIVE_PATH    := hw
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes)
LOCAL_C_INCLUDES              += $(kernel_includes)
LOCAL_HEADER_LIBRARIES        := display_headers

LOCAL_CFLAGS                  := -Wno-missing-field-initializers -Wno-unused-parameter \
                                 -fcolor-diagnostics\
                                 -DLOG_TAG=\"SDM\" $(common_flags)
LOCAL_CLANG                   := true

LOCAL_SHARED_LIBRARIES        := libsdmcore libqservice libbinder libhardware libhardware_legacy \
                                 libutils libcutils libsync libqdutils libqdMetaData \
                                 libdisplaydebug libsdmutils libc++ liblog libgrallocutils libui \
                                 libgpu_tonemapper libhidlbase libhidltransport libdisplayconfig.qti \
                                 vendor.display.config@2.0 \
                                 android.hardware.graphics.mapper@2.0 \
                                 android.hardware.graphics.mapper@2.1 \
                                 android.hardware.graphics.allocator@2.0 \
                                 android.hardware.graphics.composer@2.2 \

ifeq ($(TARGET_USES_FOD_ZPOS), true)
LOCAL_CFLAGS                  += -DFOD_ZPOS
endif

LOCAL_SRC_FILES               := hwc_session.cpp \
                                 hwc_session_services.cpp \
                                 hwc_display.cpp \
                                 hwc_display_builtin.cpp \
                                 hwc_display_pluggable.cpp \
                                 hwc_display_dummy.cpp \
                                 hwc_display_pluggable_test.cpp \
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
endif
