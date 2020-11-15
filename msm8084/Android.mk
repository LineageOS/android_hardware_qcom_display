display-hals := libgralloc libcopybit libvirtual
display-hals += libhwcomposer liboverlay libqdutils libexternal libqservice
display-hals += libmemtrack
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
    $(display_top)/libhwcomposer \
    $(display_top)/liboverlay \
    $(display_top)/libqdutils \
    $(display_top)/libqservice \
    $(display_top)/libvirtual
include $(BUILD_HEADER_LIBRARY)
