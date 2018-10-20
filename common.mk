#Common headers
display_top := $(call my-dir)

#Common headers
common_includes := $(display_top)/libgralloc
common_includes += $(display_top)/liboverlay
common_includes += $(display_top)/libcopybit
common_includes += $(display_top)/libqdutils
common_includes += $(display_top)/libhwcomposer
common_includes += $(display_top)/libhdmi
common_includes += $(display_top)/libqservice
common_includes += $(display_top)/include

common_header_export_path := qcom/display

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

common_flags += -isystem $(TARGET_OUT_HEADERS)/qcom/display

ifneq ($(call is-platform-sdk-version-at-least,18),true)
    common_flags += -DANDROID_JELLYBEAN_MR1=1
endif
