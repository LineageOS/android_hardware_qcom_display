#-----------------------------------------------------------------------------------
# Generate qmcs.img FAT image for Display QMCS partition
# This image is to be populated during runtime and is initially empty as a result
#-----------------------------------------------------------------------------------
#TODO: (CBRAGA) This needs to be converted to soong via genrules. Pending due to gbug 161152718

#Configurable partition variables
BOARD_QMCSIMAGE_PARTITION_SIZE_KB ?= 30720
QMCS_SECTOR_SIZE ?= 4096

#Input output objects
INSTALLED_QMCSIMAGE_TARGET := $(PRODUCT_OUT)/qmcs.img
NEWFSTOOL := $(HOST_OUT_EXECUTABLES)/newfs_msdos$(HOST_EXECUTABLE_SUFFIX)

define build-qmcsimage-target
        $(hide) rm -rf ${INSTALLED_QMCSIMAGE_TARGET}
	$(hide) $(NEWFSTOOL) -L "QMCS" -O "QTI" -S ${QMCS_SECTOR_SIZE} -c 1 -r 32 -o 0 -h 64 -u 32 -s $$(( ${BOARD_QMCSIMAGE_PARTITION_SIZE_KB} * 1024 / ${QMCS_SECTOR_SIZE} )) -C ${BOARD_QMCSIMAGE_PARTITION_SIZE_KB}K ${INSTALLED_QMCSIMAGE_TARGET}
endef

#Custom build target
$(INSTALLED_QMCSIMAGE_TARGET) : $(NEWFSTOOL)
	$(build-qmcsimage-target)

ALL_DEFAULT_INSTALLED_MODULES += $(INSTALLED_QMCSIMAGE_TARGET)
ALL_MODULES.$(LOCAL_MODULE).INSTALLED += $(INSTALLED_QMCSIMAGE_TARGET)
