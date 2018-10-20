LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE                  := libsde
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := hardware/qcom/display/displayengine/include/
LOCAL_CFLAGS                  := -Wno-missing-field-initializers -Wno-unused-parameter \
                                 -Wconversion -Wall -Werror \
                                 -DLOG_TAG=\"SDE\"
LOCAL_HEADER_LIBRARIES        := generated_kernel_headers
LOCAL_SHARED_LIBRARIES        := libdl libsdeutils
LOCAL_SRC_FILES               := core_interface.cpp \
                                 core_impl.cpp \
                                 display_base.cpp \
                                 display_primary.cpp \
                                 display_hdmi.cpp \
                                 display_virtual.cpp \
                                 comp_manager.cpp \
                                 strategy_default.cpp \
                                 res_manager.cpp \
                                 res_config.cpp \
                                 offline_ctrl.cpp \
                                 hw_interface.cpp \
                                 hw_framebuffer.cpp \
                                 dump_impl.cpp \
                                 buffer_manager.cpp

include $(BUILD_SHARED_LIBRARY)
