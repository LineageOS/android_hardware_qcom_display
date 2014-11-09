ifeq ($(call my-dir),$(call project-path-for,qcom-display))

LOCAL_PATH := $(call my-dir)
include $(LOCAL_PATH)/qmcs.mk

endif
