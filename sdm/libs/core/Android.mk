LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE                  := libsdmcore
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(call project-path-for,qcom-display)/sdm/include/
LOCAL_CFLAGS                  := -Wno-missing-field-initializers -Wno-unused-parameter \
                                 -Wconversion -Wall -Werror \
                                 -DLOG_TAG=\"SDM\"
LOCAL_HW_INTF_PATH            := fb
LOCAL_SHARED_LIBRARIES        := libdl libsdmutils
LOCAL_HEADER_LIBRARIES        := generated_kernel_headers
LOCAL_SRC_FILES               := core_interface.cpp \
                                 core_impl.cpp \
                                 display_base.cpp \
                                 display_primary.cpp \
                                 display_hdmi.cpp \
                                 display_virtual.cpp \
                                 comp_manager.cpp \
                                 strategy.cpp \
                                 resource_default.cpp \
                                 dump_impl.cpp \
                                 color_manager.cpp \
                                 $(LOCAL_HW_INTF_PATH)/hw_info.cpp \
                                 $(LOCAL_HW_INTF_PATH)/hw_device.cpp \
                                 $(LOCAL_HW_INTF_PATH)/hw_primary.cpp \
                                 $(LOCAL_HW_INTF_PATH)/hw_hdmi.cpp \
                                 $(LOCAL_HW_INTF_PATH)/hw_virtual.cpp \
                                 $(LOCAL_HW_INTF_PATH)/hw_color_manager.cpp

include $(BUILD_SHARED_LIBRARY)
