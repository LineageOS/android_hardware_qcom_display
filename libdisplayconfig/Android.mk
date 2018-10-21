LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE                  := libdisplayconfig
LOCAL_MODULE_TAGS             := optional
LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_COPY_HEADERS            := DisplayConfig.h
LOCAL_SRC_FILES               := DisplayConfig.cpp
LOCAL_SHARED_LIBRARIES        := libhidlbase libhidltransport libutils \
                                 vendor.display.config@1.0_vendor

ifeq ($(LLVM_SA), true)
    LOCAL_CFLAGS += --compile-and-analyze --analyzer-perf --analyzer-Werror
endif

include $(BUILD_SHARED_LIBRARY)
