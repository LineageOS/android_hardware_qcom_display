LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE                  := libdisplayconfig
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes)
LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_COPY_HEADERS            := DisplayConfig.h
LOCAL_SRC_FILES               := DisplayConfig.cpp
LOCAL_SHARED_LIBRARIES        := libhidlbase libhidltransport libutils \
                                 vendor.display.config@1.0 android.hidl.base@1.0

include $(BUILD_SHARED_LIBRARY)
