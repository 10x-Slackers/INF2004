################################################################################
# micro T-Kernel 3.00.03  makefile
################################################################################

include $(ROOT)/build_make/mtkernel_3/device/common/drvif/subdir.mk

-include $(ROOT)/build_make/mtkernel_3/device/adc/subdir.mk
-include $(ROOT)/build_make/mtkernel_3/device/i2c/subdir.mk
-include $(ROOT)/build_make/mtkernel_3/device/ser/subdir.mk
