LOCAL_PATH := $(call my-dir)

display_config_version := $(shell \
    if [ -d "$(TOP)/vendor/qcom/opensource/interfaces/display/config/1.4" ];\
    then echo DISPLAY_CONFIG_1_4; fi)

include $(CLEAR_VARS)
LOCAL_MODULE                  := libdisplayconfig
LOCAL_MODULE_TAGS             := optional
LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_SRC_FILES               := DisplayConfig.cpp
LOCAL_SHARED_LIBRARIES        := libhidlbase libhidltransport libutils \
                                 vendor.display.config@1.0

ifeq ($(display_config_version), DISPLAY_CONFIG_1_4)
    LOCAL_SHARED_LIBRARIES        += vendor.display.config@1.4
    LOCAL_CFLAGS                  += -DDISPLAY_CONFIG_1_4
endif

LOCAL_EXPORT_C_INCLUDE_DIRS   := $(LOCAL_PATH)

ifeq ($(LLVM_SA), true)
    LOCAL_CFLAGS += --compile-and-analyze --analyzer-perf --analyzer-Werror
endif

include $(BUILD_SHARED_LIBRARY)
