sdm-libs := sdm/libs
display-hals := include libqservice libqdutils $(sdm-libs)/utils $(sdm-libs)/core

ifneq ($(TARGET_IS_HEADLESS), true)
    display-hals += libcopybit libmemtrack hdmi_cec \
                    $(sdm-libs)/hwc $(sdm-libs)/hwc2 gpu_tonemapper libdrmutils
ifneq ($(TARGET_PROVIDES_LIBLIGHT),true)
    display-hals += liblight
endif
endif

ifneq ($(TARGET_USES_GRALLOC1), true)
    display-hals += libgralloc
else
    display-hals += libgralloc1
endif

ifeq ($(call is-vendor-board-platform,QCOM),true)
    include $(call all-named-subdir-makefiles,$(display-hals))
else
ifneq ($(filter msm% apq%,$(TARGET_BOARD_PLATFORM)),)
    include $(call all-named-subdir-makefiles,$(display-hals))
endif
endif
