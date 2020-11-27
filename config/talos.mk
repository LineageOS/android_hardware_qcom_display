#Display related packages and configuration

PRODUCT_PACKAGES += \
    android.hardware.graphics.composer@2.3-impl \
    android.hardware.graphics.composer@2.3-service \
    android.hardware.graphics.mapper@2.0-impl-qti-display \
    vendor.qti.hardware.display.allocator-service \
    android.hardware.memtrack@1.0-impl \
    android.hardware.memtrack@1.0-service \
    android.hardware.light@2.0-impl \
    android.hardware.light@2.0-service \
    gralloc.sm6150 \
    lights.sm6150 \
    hwcomposer.sm6150 \
    memtrack.sm6150 \
    libqdutils \
    libqdMetaData \
    libqdMetaData.system \
    modetest

#QDCM calibration xml file base on Talos panel type hx83112a
PRODUCT_COPY_FILES += hardware/qcom/display/config/qdcm_calib_data_talos_default.xml:$(TARGET_COPY_OUT_VENDOR)/etc/qdcm_calib_data_hx83112a_video_mode_dsi_truly_panel.xml
#QDCM calibration xml file base on Talos panel type td4328
PRODUCT_COPY_FILES += hardware/qcom/display/config/qdcm_calib_data_talos_default.xml:$(TARGET_COPY_OUT_VENDOR)/etc/qdcm_calib_data_td4328_cmd_mode_dsi_truly_panel.xml
PRODUCT_COPY_FILES += hardware/qcom/display/config/qdcm_calib_data_talos_default.xml:$(TARGET_COPY_OUT_VENDOR)/etc/qdcm_calib_data_td4328_video_mode_dsi_truly_panel.xml

#QDCM calibration xml file base on Moorea panel type fhd+
PRODUCT_COPY_FILES += hardware/qcom/display/config/qdcm_calib_data_default.xml:$(TARGET_COPY_OUT_VENDOR)/etc/qdcm_calib_data_sw43404_amoled_boe_fhd+_panel_with_DSC.xml
#QDCM calibration xml file base on Moorea panel type sw43404
PRODUCT_COPY_FILES += hardware/qcom/display/config/qdcm_calib_data_sw43404_amoled_cmd_mode_dsi_boe_panel_with_DSC.xml:$(TARGET_COPY_OUT_VENDOR)/etc/qdcm_calib_data_sw43404_amoled_cmd_mode_dsi_boe_panel_with_DSC.xml
PRODUCT_COPY_FILES += hardware/qcom/display/config/qdcm_calib_data_sw43404_amoled_cmd_mode_dsi_boe_panel_with_DSC.xml:$(TARGET_COPY_OUT_VENDOR)/etc/qdcm_calib_data_sw43404_amoled_video_mode_dsi_boe_panel_with_DSC.xml
#QDCM calibration xml file base on Moorea panel type wqhd
PRODUCT_COPY_FILES += hardware/qcom/display/config/qdcm_calib_data_default.xml:$(TARGET_COPY_OUT_VENDOR)/etc/qdcm_calib_data_Dual_Sharp_WQHD_cmd_mode_dsi_panel.xml
PRODUCT_COPY_FILES += hardware/qcom/display/config/qdcm_calib_data_default.xml:$(TARGET_COPY_OUT_VENDOR)/etc/qdcm_calib_data_Dual_Sharp_wqhd_video_mode_dsi_panel.xml

#QDCM calibration xml file for secondary panel nt35695b
PRODUCT_COPY_FILES += hardware/qcom/display/config/qdcm_calib_data_default.xml:$(TARGET_COPY_OUT_VENDOR)/etc/qdcm_calib_data_nt35695b_truly_fhd_command_mode_dsi_panel.xml
PRODUCT_COPY_FILES += hardware/qcom/display/config/qdcm_calib_data_default.xml:$(TARGET_COPY_OUT_VENDOR)/etc/qdcm_calib_data_nt35695b_truly_fhd_video_mode_dsi_panel.xml

#Enable Charging Icon
TARGET_RECOVERY_PIXEL_FORMAT := RGBX_8888

TARGET_USES_GRALLOC1 := true
TARGET_USES_DRM_PP := true
TARGET_FORCE_HWC_FOR_VIRTUAL_DISPLAYS := true
MAX_VIRTUAL_DISPLAY_DIMENSION := 4096
NUM_FRAMEBUFFER_SURFACE_BUFFERS := 3
TARGET_USES_HWC2 := true
TARGET_USES_QCOM_DISPLAY_BSP := true
TARGET_USES_COLOR_METADATA := true
TARGET_USES_DISPLAY_RENDER_INTENTS := true
TARGET_USES_QTI_MAPPER_2_0 := true
TARGET_USES_QTI_MAPPER_EXTENSIONS_1_1 := true

PRODUCT_PROPERTY_OVERRIDES += \
    persist.demo.hdmirotationlock=false \
    persist.sys.sf.color_saturation=1.0 \
    debug.sf.hw=0 \
    debug.egl.hw=0 \
    debug.sf.latch_unsignaled=1 \
    debug.mdpcomp.logs=0 \
    vendor.gralloc.disable_ubwc=0 \
    vendor.display.disable_scaler=0 \
    vendor.display.disable_inline_rotator=1 \
    vendor.display.disable_decimation=1 \
    vendor.display.disable_excl_rect_partial_fb=1 \
    vendor.display.enable_null_display=0 \
    vendor.display.disable_excl_rect=0 \
    vendor.display.comp_mask=0 \
    vendor.display.enable_default_color_mode=1 \
    vendor.display.enable_optimize_refresh=1

ifneq (,$(filter userdebug eng, $(TARGET_BUILD_VARIANT)))
# Recovery is enabled, logging is enabled
PRODUCT_PROPERTY_OVERRIDES += \
    vendor.display.disable_hw_recovery_dump=0
else
# Recovery is enabled, logging is disabled
PRODUCT_PROPERTY_OVERRIDES += \
    vendor.display.disable_hw_recovery_dump=1
endif

# Properties using default value:
#    vendor.display.disable_hw_recovery=0

# This matrix should be in column major order, per SurfaceFlinger requirement
#  1.0   0.0   0.0
#  0.0   1.0   0.0
#  0.0   0.0   1.0
PRODUCT_PROPERTY_OVERRIDES += \
    vendor.display.dataspace_saturation_matrix=1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0
