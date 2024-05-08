HEADER_PATH := ${WORKSPACE}/hardware/qcom/display/sdm/include

c_sources = core_interface.cpp \
            core_impl.cpp \
            display_base.cpp \
            display_builtin.cpp \
            noise_plugin_intf_impl.cpp \
            display_pluggable.cpp \
            display_virtual.cpp \
            display_null.cpp \
            comp_manager.cpp \
            strategy.cpp \
            resource_default.cpp \
            color_manager.cpp \
            hw_interface.cpp \
            hw_info_interface.cpp \
            hw_events_interface.cpp \
            hw_info_default.cpp \
            drm/hw_color_manager_drm.cpp \
            drm/hw_device_drm.cpp \
            drm/hw_events_drm.cpp \
            drm/hw_info_drm.cpp \
            drm/hw_peripheral_drm.cpp \
            drm/hw_scale_drm.cpp \
            drm/hw_tv_drm.cpp \
            drm/hw_virtual_drm.cpp

core_h_sources = $(HEADER_PATH)/core/*.h

core_includedir = $(includedir)/sdm/core
core_include_HEADERS = $(core_h_sources)

private_h_sources = $(HEADER_PATH)/private/*.h

private_includedir = $(includedir)/sdm/private
private_include_HEADERS = $(private_h_sources)

utils_h_sources = $(HEADER_PATH)/utils/*.h

utils_includedir = $(includedir)/sdm/utils
utils_include_HEADERS = $(utils_h_sources)

lib_LTLIBRARIES = libsdmcore.la
libsdmcore_la_CC = @CC@
libsdmcore_la_SOURCES = $(c_sources)
libsdmcore_la_CFLAGS = $(COMMON_CFLAGS) -DLOG_TAG=\"SDM\"
libsdmcore_la_CPPFLAGS = $(AM_CPPFLAGS) -DPP_DRM_ENABLE
libsdmcore_la_LIBADD = ../utils/libsdmutils.la ../../../sde-drm/libsdedrm.la -ldl -ldisplaydebug
libsdmcore_la_LDFLAGS = -shared -avoid-version
