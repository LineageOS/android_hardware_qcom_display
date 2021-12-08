include hardware/qcom/display/config/display-modules.mk

ifneq (,$(wildcard $(QCPATH)/display))
include $(QCPATH)/display/config/display-vendor-modules.mk
endif

.PHONY: display_tp display_tp_hal display_tp_dlkm

display_tp: display_tp_hal display_tp_dlkm

display_tp_hal: $(DISPLAY_MODULES_HARDWARE) $(DISPLAY_MODULES_VENDOR)

display_tp_dlkm: $(DISPLAY_MODULES_DRIVER)
