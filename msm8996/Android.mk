display-hals := libcopybit liblight libmemtrack libqservice libqdutils
ifneq ($(TARGET_USES_GRALLOC1), true)
    display-hals += libgralloc
else
    display-hals += libgralloc1
endif

display-hals += hdmi_cec
sdm-libs := sdm/libs
display-hals += $(sdm-libs)/utils $(sdm-libs)/core $(sdm-libs)/hwc $(sdm-libs)/hwc2

ifneq (,$(call is-vendor-board-qcom))
    include $(call all-named-subdir-makefiles,$(display-hals))
else
ifneq ($(filter msm% apq%,$(TARGET_BOARD_PLATFORM)),)
    include $(call all-named-subdir-makefiles,$(display-hals))
endif
endif
