# Clean old composer
$(call add-clean-step, rm -rf $(PRODUCT_OUT)/vendor/bin/hw/android.hardware.graphics.composer@2.1-service)
$(call add-clean-step, rm -rf $(PRODUCT_OUT)/vendor/etc/init/android.hardware.graphics.composer@2.1-service.rc)
$(call add-clean-step, rm -rf $(PRODUCT_OUT)/vendor/lib64/hw/android.hardware.graphics.composer@2.1-impl.so)
#Clean display includes
$(call add-clean-step, rm -rf $(PRODUCT_OUT)/obj/include/qcom/display)
