#Display related packages and configuration

PRODUCT_PACKAGES += \
    android.hardware.graphics.composer@2.2-impl \
    android.hardware.graphics.composer@2.2-service \
    android.hardware.graphics.mapper@2.0-impl-qti-display \
    vendor.qti.hardware.display.allocator@1.0-service \
    android.hardware.memtrack@1.0-impl \
    android.hardware.memtrack@1.0-service \
    android.hardware.light@2.0-impl \
    android.hardware.light@2.0-service \
    gralloc.trinket \
    lights.trinket \
    hwcomposer.trinket \
    memtrack.trinket \
    libqdutils \
    libqdMetaData \
    libqdMetaData.system \
    modetest

#QDCM calibration xml file base on td4330
PRODUCT_COPY_FILES += hardware/qcom/display/config/qdcm_calib_data_trinket_default.xml:$(TARGET_COPY_OUT_VENDOR)/etc/qdcm_calib_data_td4330_cmd_mode_dsi_truly_panel.xml
PRODUCT_COPY_FILES += hardware/qcom/display/config/qdcm_calib_data_trinket_default.xml:$(TARGET_COPY_OUT_VENDOR)/etc/qdcm_calib_data_td4330_video_mode_dsi_truly_panel.xml
#QDCM calibration xml file base on nt36672
PRODUCT_COPY_FILES += hardware/qcom/display/config/qdcm_calib_data_trinket_default.xml:$(TARGET_COPY_OUT_VENDOR)/etc/qdcm_calib_data_nt36672_truly_fhd_video_mode_dsi_panel.xml

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
    vendor.display.enable_null_display=0 \
    vendor.display.disable_excl_rect=0 \
    vendor.display.comp_mask=0 \
    vendor.display.enable_default_color_mode=1

# This matrix should be in column major order, per SurfaceFlinger requirement
#  1.0   0.0   0.0
#  0.0   1.0   0.0
#  0.0   0.0   1.0
PRODUCT_PROPERTY_OVERRIDES += \
    vendor.display.dataspace_saturation_matrix=1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0
