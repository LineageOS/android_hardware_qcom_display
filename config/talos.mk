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
    gralloc.talos \
    lights.talos \
    hwcomposer.talos \
    memtrack.talos \
    libqdutils \
    libqdMetaData \
    libqdMetaData.system \
    modetest

#QDCM calibration xml file
PRODUCT_COPY_FILES += hardware/qcom/display/config/qdcm_calib_data_nt35597_cmd_mode_dsi_truly_panel_with_DSC.xml:$(TARGET_COPY_OUT_VENDOR)/etc/qdcm_calib_data_hx83112a_video_mode_dsi_truly_panel.xml

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
TARGET_HAS_WIDE_COLOR_DISPLAY := true

PRODUCT_PROPERTY_OVERRIDES += \
    persist.demo.hdmirotationlock=false \
    debug.sf.hw=0 \
    debug.egl.hw=0 \
    debug.sf.latch_unsignaled=1 \
    debug.mdpcomp.logs=0 \
    debug.sf.enable_hwc_vds=1 \
    ro.vendor.display.cabl=2 \
    vendor.gralloc.disable_ubwc=0 \
    vendor.display.disable_scaler=0 \
    vendor.display.disable_inline_rotator=1 \
    vendor.display.disable_decimation=1 \
    vendor.display.disable_excl_rect_partial_fb=1 \
    vendor.display.enable_null_display=0 \
    vendor.display.disable_excl_rect=0 \
    vendor.display.comp_mask=0 \
    vendor.display.disable_hw_recovery=1 \
    vendor.display.enable_default_color_mode=1 \
    vendor.display.drop_skewed_vsync=1

# This matrix should be in column major order, per SurfaceFlinger requirement
#  1.16868   -0.16868    0.00000
# -0.03155    1.03155    0.00000
# -0.01473   -0.05899    1.07372
PRODUCT_PROPERTY_OVERRIDES += \
    vendor.display.dataspace_saturation_matrix=1.16868,-0.03155,-0.01473,-0.16868,1.03155,-0.05899,0.00000,0.00000,1.07372
