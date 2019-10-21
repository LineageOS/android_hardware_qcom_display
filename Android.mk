ifeq ($(call my-dir),$(call project-path-for,qcom-display))

ifneq ($(TARGET_DISABLE_DISPLAY),true)
sdm-libs := sdm/libs
display-hals := include $(sdm-libs)/utils $(sdm-libs)/core libdebug

ifneq ($(TARGET_IS_HEADLESS), true)
    display-hals += libmemtrack hdmi_cec \
                    $(sdm-libs)/hwc2 gpu_tonemapper libdrmutils
endif

display-hals += gralloc
display-hals += commonsys-intf/libdisplayconfig

ifneq ($(TARGET_PROVIDES_LIBLIGHT),true)
    display-hals += liblight
endif
else
ifneq ($(TARGET_IS_HEADLESS), true)
    display-hals := libcopybit
endif
endif #TARGET_DISABLE_DISPLAY

ifeq ($(call is-vendor-board-platform,QCOM),true)
    include $(call all-named-subdir-makefiles,$(display-hals))
else
ifneq ($(filter msm% apq%,$(TARGET_BOARD_PLATFORM)),)
    include $(call all-named-subdir-makefiles,$(display-hals))
endif
endif

endif
