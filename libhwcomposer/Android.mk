LOCAL_PATH := $(call my-dir)
include $(LOCAL_PATH)/../common.mk
include $(CLEAR_VARS)

LOCAL_MODULE                  := hwcomposer.$(TARGET_BOARD_PLATFORM)
LOCAL_MODULE_RELATIVE_PATH    := hw
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes) $(kernel_includes) \
                                 $(TOP)/external/skia/include/core \
                                 $(TOP)/external/skia/include/images
LOCAL_SHARED_LIBRARIES        := $(common_libs) libEGL liboverlay \
                                 libexternal libqdutils libhardware_legacy \
                                 libdl libmemalloc libqservice libsync \
                                 libbinder libmedia libvirtual libhwui
LOCAL_CFLAGS                  := $(common_flags) -DLOG_TAG=\"qdhwcomposer\"
ifeq ($(GET_DISPLAY_SECURE_STATUS_FROM_HWC),true)
    LOCAL_CFLAGS += -DGET_DISPLAY_SECURE_STATUS_FROM_HWC
endif
LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)
LOCAL_SRC_FILES               := hwc.cpp          \
                                 hwc_utils.cpp    \
                                 hwc_uevents.cpp  \
                                 hwc_vsync.cpp    \
                                 hwc_fbupdate.cpp \
                                 hwc_mdpcomp.cpp  \
                                 hwc_copybit.cpp  \
                                 hwc_qclient.cpp  \
                                 hwc_dump_layers.cpp

include $(BUILD_SHARED_LIBRARY)
