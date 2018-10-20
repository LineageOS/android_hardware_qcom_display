#Common headers
common_includes := $(call project-path-for,qcom-display)/libgralloc
common_includes += $(call project-path-for,qcom-display)/liboverlay
common_includes += $(call project-path-for,qcom-display)/libcopybit
common_includes += $(call project-path-for,qcom-display)/libqdutils
common_includes += $(call project-path-for,qcom-display)/libhwcomposer
common_includes += $(call project-path-for,qcom-display)/libhdmi
common_includes += $(call project-path-for,qcom-display)/libqservice

common_header_export_path := qcom/display

#Common libraries external to display HAL
common_libs := liblog libutils libcutils libhardware

#Common C flags
common_flags := -DDEBUG_CALC_FPS -Wno-missing-field-initializers
common_flags += -Wconversion -Wall -Werror -Wno-sign-conversion

ifeq ($(TARGET_USES_POST_PROCESSING),true)
    common_flags     += -DUSES_POST_PROCESSING
    common_includes  += $(TARGET_OUT_HEADERS)/pp/inc
endif

ifeq ($(ARCH_ARM_HAVE_NEON),true)
    common_flags += -D__ARM_HAVE_NEON
endif

ifeq ($(call is-board-platform-in-list, msm8974 msm8226 msm8610 apq8084 \
        mpq8092 msm_bronze msm8916 msm8994 msm8992), true)
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
    common_flags += -DQTI_BSP
endif
