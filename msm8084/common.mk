display_top := $(call my-dir)

ifeq ($(TARGET_USES_POST_PROCESSING),true)
    common_flags     += -DUSES_POST_PROCESSING
endif

#Common libraries external to display HAL
common_libs := liblog libutils libcutils libhardware

#Common C flags
common_flags := -DDEBUG_CALC_FPS -Wno-missing-field-initializers
#TODO: Add -Werror back once all the current warnings are fixed
common_flags += -Wno-error -Wno-sign-compare -Wno-sign-conversion -Wno-float-conversion

ifeq ($(ARCH_ARM_HAVE_NEON),true)
    common_flags += -D__ARM_HAVE_NEON
endif

ifneq ($(filter msm8974 msm8226 msm8610 msm8084 msm8916, \
       $(TARGET_BOARD_PLATFORM)),)
    common_flags += -DVENUS_COLOR_FORMAT
    common_flags += -DMDSS_TARGET
endif

ifeq ($(DISPLAY_DEBUG_SWAPINTERVAL),true)
    common_flags += -DDEBUG_SWAPINTERVAL
endif

common_flags += -D__STDC_FORMAT_MACROS

# Executed only on QCOM BSPs
ifeq ($(TARGET_USES_QCOM_BSP),true)
# Enable QCOM Display features
    common_flags += -DQCOM_BSP
endif
ifneq ($(call is-platform-sdk-version-at-least,18),true)
    common_flags += -DANDROID_JELLYBEAN_MR1=1
endif
