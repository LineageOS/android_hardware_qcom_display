display_top := $(call my-dir)

ifeq ($(TARGET_USES_POST_PROCESSING),true)
    common_flags     += -DUSES_POST_PROCESSING
endif

#Common libraries external to display HAL
common_libs := liblog libutils libcutils libhardware

#Common C flags
common_flags := -DDEBUG_CALC_FPS -Wno-missing-field-initializers
common_flags += -Wconversion -Wall -Werror -Wno-sign-conversion

ifeq ($(ARCH_ARM_HAVE_NEON),true)
    common_flags += -D__ARM_HAVE_NEON
endif

ifeq ($(call is-board-platform-in-list, $(MSM_VIDC_TARGET_LIST)), true)
    common_flags += -DVENUS_COLOR_FORMAT
endif

ifeq ($(call is-board-platform-in-list, $(MASTER_SIDE_CP_TARGET_LIST)), true)
    common_flags += -DMASTER_SIDE_CP
endif

# Executed only on QCOM BSPs
ifeq ($(TARGET_USES_QCOM_BSP),true)
# Enable QCOM Display features
    common_flags += -DQTI_BSP
endif

ifneq ($(call is-platform-sdk-version-at-least,18),true)
    common_flags += -DANDROID_JELLYBEAN_MR1=1
endif
