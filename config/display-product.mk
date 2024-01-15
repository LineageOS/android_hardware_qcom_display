# Display product definitions
DISPLAY_HAL_DIR := hardware/qcom-caf/sm8550/display

include $(DISPLAY_HAL_DIR)/config/display-modules.mk
PRODUCT_PACKAGES += $(DISPLAY_MODULES_HARDWARE)

ifneq ($(TARGET_HAS_LOW_RAM),true)
#Multi-stc libraries config xml file
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/snapdragon_color_libs_config.xml:$(TARGET_COPY_OUT_VENDOR)/etc/snapdragon_color_libs_config.xml

#Clstc library config xml file
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/clstc_config_library.xml:$(TARGET_COPY_OUT_VENDOR)/etc/clstc_config_library.xml
endif

#QDCM calibration json file for r66451 panel
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_r66451_amoled_cmd_mode_dsi_visionox_panel_with_DSC.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_r66451_amoled_cmd_mode_dsi_visionox_panel_with_DSC.json
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_r66451_amoled_video_mode_dsi_visionox_panel_with_DSC.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_r66451_amoled_video_mode_dsi_visionox_panel_with_DSC.json
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_r66451_amoled_cmd_mode_dsi_visionox_panel_without_DSC.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_r66451_amoled_cmd_mode_dsi_visionox_panel_without_DSC.json
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_r66451_amoled_video_mode_dsi_visionox_panel_without_DSC.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_r66451_amoled_video_mode_dsi_visionox_panel_without_DSC.json

#QDCM calibration json file for Sharp panel
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_Sharp_4k_cmd_mode_dsc_dsi_panel.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_Sharp_4k_cmd_mode_dsc_dsi_panel.json
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_Sharp_4k_video_mode_dsc_dsi_panel.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_Sharp_4k_video_mode_dsc_dsi_panel.json
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_Sharp_2k_cmd_mode_qsync_dsi_panel.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_Sharp_2k_cmd_mode_qsync_dsi_panel.json
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_Sharp_2k_video_mode_qsync_dsi_panel.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_Sharp_2k_video_mode_qsync_dsi_panel.json
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_Sharp_qhd_cmd_mode_dsi_panel.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_Sharp_qhd_cmd_mode_dsi_panel.json
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_sharp_1080p_cmd_mode_dsi_panel.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_sharp_1080p_cmd_mode_dsi_panel.json

#QDCM calibration JSON file for nt36672e LCD video mode single dsi with DSC panel.
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_nt36672e_lcd_video_mode_dsi_novatek_panel_with_DSC.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_nt36672e_lcd_video_mode_dsi_novatek_panel_with_DSC.json
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_nt36672e_lcd_video_mode_dsi_novatek_panel_without_DSC.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_nt36672e_lcd_video_mode_dsi_novatek_panel_without_DSC.json

#Backlight calibration xml file for r66451 amoled panels
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/backlight_calib_r66451_amoled_cmd_mode_dsi_visionox_panel_with_DSC.xml:$(TARGET_COPY_OUT_VENDOR)/etc/display/backlight_calib_r66451_amoled_cmd_mode_dsi_visionox_panel_with_DSC.xml
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/backlight_calib_r66451_amoled_cmd_mode_dsi_visionox_panel_with_DSC.xml:$(TARGET_COPY_OUT_VENDOR)/etc/display/backlight_calib_r66451_amoled_video_mode_dsi_visionox_panel_with_DSC.xml

#QDCM calibration json file for vtdr6130 panel
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_vtdr6130_amoled_cmd_mode_dsi_visionox_panel_with_DSC.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_vtdr6130_amoled_cmd_mode_dsi_visionox_panel_with_DSC.json
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_vtdr6130_amoled_video_mode_dsi_visionox_panel_with_DSC.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_vtdr6130_amoled_video_mode_dsi_visionox_panel_with_DSC.json
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_vtdr6130_amoled_qsync_cmd_mode_dsi_visionox_panel_with_DSC.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_vtdr6130_amoled_qsync_cmd_mode_dsi_visionox_panel_with_DSC.json
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/qdcm_calib_data_vtdr6130_amoled_qsync_video_mode_dsi_visionox_panel_with_DSC.json:$(TARGET_COPY_OUT_VENDOR)/etc/display/qdcm_calib_data_vtdr6130_amoled_qsync_video_mode_dsi_visionox_panel_with_DSC.json

#Backlight calibration xml file for vtdr6130 amoled panels
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/backlight_calib_vtdr6130_amoled_cmd_mode_dsi_visionox_panel_with_DSC.xml:$(TARGET_COPY_OUT_VENDOR)/etc/display/backlight_calib_vtdr6130_amoled_cmd_mode_dsi_visionox_panel_with_DSC.xml
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/backlight_calib_vtdr6130_amoled_cmd_mode_dsi_visionox_panel_with_DSC.xml:$(TARGET_COPY_OUT_VENDOR)/etc/display/backlight_calib_vtdr6130_amoled_video_mode_dsi_visionox_panel_with_DSC.xml
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/backlight_calib_vtdr6130_amoled_cmd_mode_dsi_visionox_panel_with_DSC.xml:$(TARGET_COPY_OUT_VENDOR)/etc/display/backlight_calib_vtdr6130_amoled_qsync_cmd_mode_dsi_visionox_panel_with_DSC.xml
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/backlight_calib_vtdr6130_amoled_cmd_mode_dsi_visionox_panel_with_DSC.xml:$(TARGET_COPY_OUT_VENDOR)/etc/display/backlight_calib_vtdr6130_amoled_qsync_video_mode_dsi_visionox_panel_with_DSC.xml

#Smomo config xml file
PRODUCT_COPY_FILES += $(DISPLAY_HAL_DIR)/config/smomo_setting.xml:$(TARGET_COPY_OUT_VENDOR)/etc/smomo_setting.xml

PRODUCT_VENDOR_PROPERTIES += \
    persist.demo.hdmirotationlock=false \
    persist.sys.sf.color_saturation=1.0 \
    persist.sys.sf.color_mode=9 \
    debug.sf.hw=0 \
    debug.egl.hw=0 \
    debug.sf.latch_unsignaled=1 \
    debug.sf.auto_latch_unsignaled=0 \
    debug.mdpcomp.logs=0 \
    vendor.gralloc.disable_ubwc=0 \
    vendor.gralloc.enable_logs=0 \
    vendor.display.disable_scaler=0 \
    vendor.display.disable_excl_rect=0 \
    vendor.display.disable_excl_rect_partial_fb=1 \
    vendor.display.comp_mask=0 \
    vendor.display.enable_optimize_refresh=0 \
    vendor.display.use_smooth_motion=1 \
    vendor.display.disable_stc_dimming=1 \
    vendor.display.enable_dpps_dynamic_fps=1 \
    debug.sf.disable_client_composition_cache=1 \
    debug.sf.enable_gl_backpressure=1 \
    debug.sf.enable_hwc_vds=0 \
    debug.sf.enable_advanced_sf_phase_offset=1 \
    vendor.display.vds_allow_hwc=1 \
    debug.sf.use_phase_offsets_as_durations=1 \
    debug.sf.late.app.duration=13666666 \
    debug.sf.early.app.duration=13666666 \
    debug.sf.earlyGl.app.duration=13666666 \
    debug.sf.early.sf.duration=10500000 \
    debug.sf.earlyGl.sf.duration=10500000 \
    debug.sf.late.sf.duration=10500000 \
    vendor.display.enable_async_vds_creation=1 \
    vendor.display.enable_rounded_corner=1 \
    vendor.display.disable_3d_adaptive_tm=1 \
    vendor.display.disable_sdr_dimming=1 \
    vendor.display.enable_rc_support=1 \
    vendor.display.disable_sdr_histogram=1 \
    vendor.display.enable_hdr10_gpu_target=1 \
    debug.sf.predict_hwc_composition_strategy=0 \
    debug.sf.treat_170m_as_sRGB=1 \
    vendor.display.enable_display_extensions=1

# Enable offline rotator for Bengal.
ifneq ($(TARGET_BOARD_PLATFORM),bengal)
PRODUCT_VENDOR_PROPERTIES += \
    vendor.display.disable_offline_rotator=1
else
PRODUCT_VENDOR_PROPERTIES += \
    vendor.display.disable_rotator_ubwc=1 \
    vendor.display.disable_layer_stitch=0
endif

ifeq ($(TARGET_BOARD_PLATFORM),holi)
PRODUCT_VENDOR_PROPERTIES += \
    vendor.display.secure_preview_buffer_format=420_sp \
    vendor.gralloc.secure_preview_buffer_format=420_sp \
    vendor.gralloc.secure_preview_only=1
    PRODUCT_VENDOR_PROPERTIES += vendor.display.enable_rounded_corner=1
    PRODUCT_VENDOR_PROPERTIES += vendor.display.disable_rounded_corner_thread=0
endif

ifneq ($(PLATFORM_VERSION), 10)
    PRODUCT_VENDOR_PROPERTIES +=  vendor.display.enable_async_powermode=0
endif

#Set WCG properties
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += ro.surface_flinger.has_wide_color_display=true
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += ro.surface_flinger.has_HDR_display=true
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += ro.surface_flinger.use_color_management=true
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += ro.surface_flinger.wcg_composition_dataspace=143261696
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += ro.surface_flinger.protected_contents=true
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += ro.surface_flinger.use_content_detection_for_refresh_rate=true
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += ro.surface_flinger.set_touch_timer_ms=200
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += ro.surface_flinger.force_hwc_copy_for_virtual_displays=true
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += ro.surface_flinger.max_frame_buffer_acquired_buffers=3
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += ro.surface_flinger.max_virtual_display_dimension=4096

#BG blur support
ifeq ($(TARGET_BOARD_PLATFORM),kalama)
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += ro.surface_flinger.supports_background_blur=1
else
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += ro.surface_flinger.supports_background_blur=0
endif

ifneq (,$(filter userdebug eng, $(TARGET_BUILD_VARIANT)))
# Recovery is enabled, logging is enabled
PRODUCT_VENDOR_PROPERTIES += \
    vendor.display.disable_hw_recovery_dump=0
PRODUCT_VENDOR_PROPERTIES += \
    vendor.display.hw_recovery_threshold=5
else
# Recovery is enabled, logging is disabled
PRODUCT_VENDOR_PROPERTIES += \
    vendor.display.disable_hw_recovery_dump=1
endif

ifeq ($(TARGET_USES_QMAA),true)
    ifneq ($(TARGET_USES_QMAA_OVERRIDE_DISPLAY),true)
        #QMAA Mode is enabled
        TARGET_IS_HEADLESS := true
    endif
endif

# Soong Namespace
SOONG_CONFIG_NAMESPACES += qtidisplay

# Soong Keys
SOONG_CONFIG_qtidisplay := drmpp headless llvmsa gralloc4 displayconfig_enabled default var1 var2 var3 llvmcov

# Soong Values
SOONG_CONFIG_qtidisplay_drmpp := true
SOONG_CONFIG_qtidisplay_headless := false
SOONG_CONFIG_qtidisplay_llvmsa := false
SOONG_CONFIG_qtidisplay_gralloc4 := true
SOONG_CONFIG_qtidisplay_displayconfig_enabled := false
SOONG_CONFIG_qtidisplay_default := true
SOONG_CONFIG_qtidisplay_var1 := false
SOONG_CONFIG_qtidisplay_var2 := false
SOONG_CONFIG_qtidisplay_var3 := false
SOONG_CONFIG_qtidisplay_llvmcov := false

ifeq ($(call is-vendor-board-platform,QCOM),true)
    SOONG_CONFIG_qtidisplay_displayconfig_enabled := true
endif

# Techpack values

ifeq ($(TARGET_IS_HEADLESS), true)
    # TODO: QMAA prebuilts
    PRODUCT_SOONG_NAMESPACES += $(DISPLAY_HAL_DIR)/qmaa
    SOONG_CONFIG_qtidisplay_headless := true
    SOONG_CONFIG_qtidisplay_default := false
else
    #Packages that should not be installed in QMAA are enabled here.
    PRODUCT_PACKAGES += libdrmutils
    PRODUCT_PACKAGES += libsdedrm
    PRODUCT_PACKAGES += libgpu_tonemapper
    #Properties that should not be set in QMAA are enabled here.
    PRODUCT_VENDOR_PROPERTIES += \
        vendor.display.enable_early_wakeup=1
    ifneq ($(BUILD_DISPLAY_TECHPACK_SOURCE), true)
        SOONG_CONFIG_qtidisplay_var1 := true
        SOONG_CONFIG_qtidisplay_var2 := true
        SOONG_CONFIG_qtidisplay_var3 := true
    endif
    ifneq ($(BUILD_DISPLAY_TECHPACK_SOURCE_VARIANT), true)
        SOONG_CONFIG_qtidisplay_var1 := true
        SOONG_CONFIG_qtidisplay_var2 := true
    endif

    ifeq ($(PROFILE_COVERAGE_DATA), true)
        SOONG_CONFIG_qtidisplay_llvmcov := true
    endif
    ifeq (,$(wildcard $(QCPATH)/display-noship))
        SOONG_CONFIG_qtidisplay_var1 := true
    endif

    ifeq (,$(wildcard $(QCPATH)/display))
        SOONG_CONFIG_qtidisplay_var2 := true
    endif
endif




QMAA_ENABLED_HAL_MODULES += display

# Properties using default value:
#    vendor.display.disable_hw_recovery=0
