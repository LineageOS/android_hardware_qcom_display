LOCAL_PATH:= $(call my-dir)
include $(LOCAL_PATH)/../common.mk
include $(CLEAR_VARS)

# Legacy header copy. This is deprecated.
# Modules using these headers should shift to using
# LOCAL_HEADER_LIBRARIES := display_headers
LOCAL_VENDOR_MODULE           := true
LOCAL_COPY_HEADERS_TO         := $(common_header_export_path)
LOCAL_COPY_HEADERS            := color_metadata.h \
                                 display_properties.h \
                                 ../libqdutils/qd_utils.h \
                                 ../libqdutils/qdMetaData.h \
                                 ../libqdutils/display_config.h \
                                 ../libqservice/QServiceUtils.h \
                                 ../libqservice/IQService.h \
                                 ../libqservice/IQHDMIClient.h \
                                 ../libqservice/IQClient.h

include $(BUILD_COPY_HEADERS)
