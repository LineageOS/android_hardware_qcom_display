ifeq ($(call my-dir),$(call project-path-for,qcom-display))

ifeq ($(call is-board-platform-in-list, msm8996),true)
    TARGET_USES_SDM = true
else
    TARGET_USES_SDM = false
endif

display-hals := libgralloc libcopybit libmemtrack libqservice libqdutils
display-hals += hdmi_cec include

ifeq ($(TARGET_USES_SDM), true)
    sdm-libs := sdm/libs
    display-hals += $(sdm-libs)/utils $(sdm-libs)/core $(sdm-libs)/hwc
else
    display-hals += libgenlock libhwcomposer liboverlay libhdmi
endif

ifneq ($(TARGET_PROVIDES_LIBLIGHT),true)
display-hals += liblight
endif
ifeq ($(call is-vendor-board-platform,QCOM),true)
    include $(call all-named-subdir-makefiles,$(display-hals))
else
ifneq ($(filter msm% apq%,$(TARGET_BOARD_PLATFORM)),)
    include $(call all-named-subdir-makefiles,$(display-hals))
endif
endif

include $(CLEAR_VARS)
LOCAL_MODULE := display_headers
LOCAL_EXPORT_C_INCLUDE_DIRS := \
    $(display_top)/libcopybit \
    $(display_top)/libexternal \
    $(display_top)/libgralloc \
    $(display_top)/libhdmi \
    $(display_top)/libhwcomposer \
    $(display_top)/liboverlay \
    $(display_top)/libqdutils \
    $(display_top)/libqservice \
    $(display_top)/libvirtual
include $(BUILD_HEADER_LIBRARY)

endif
