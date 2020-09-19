LOCAL_PATH:= $(call my-dir)
include $(LOCAL_PATH)/../common.mk
include $(CLEAR_VARS)

# Legacy header copy. This is deprecated.
# Modules using these headers should shift to using
# LOCAL_HEADER_LIBRARIES := display_headers
