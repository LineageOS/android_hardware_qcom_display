ifeq ($(call my-dir),$(call project-path-for,qcom-display))

display-hals := libgralloc libgenlock libcopybit libvirtual
display-hals += libhwcomposer liboverlay libqdutils libhdmi libqservice
display-hals += libmemtrack
ifneq ($(TARGET_PROVIDES_LIBLIGHT),true)
display-hals += liblight
endif
ifneq ($(filter msm8960 msm8974,$(TARGET_BOARD_PLATFORM)),)
    #This is for mako since it doesn't have the QCOM make functions
    include $(call all-named-subdir-makefiles,$(display-hals))
endif

endif
