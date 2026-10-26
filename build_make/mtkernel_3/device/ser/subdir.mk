################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

OBJS += \
./mtkernel_3/device/ser/ser.o

C_DEPS += \
./mtkernel_3/device/ser/ser.d


# Each subdirectory must supply rules for building sources it contributes
mtkernel_3/device/ser/%.o: $(ROOT)/device/ser/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

ifeq ($(TARGET), _IOTE_M367_)
-include $(ROOT)/build_make/mtkernel_3/device/ser/sysdepend/tx03_m367/subdir.mk
endif

ifeq ($(TARGET), _IOTE_RX231_)
-include $(ROOT)/build_make/mtkernel_3/device/ser/sysdepend/rx231/subdir.mk
endif

ifeq ($(TARGET), _IOTE_STM32L4_)
-include $(ROOT)/build_make/mtkernel_3/device/ser/sysdepend/stm32l4/subdir.mk
endif

ifeq ($(TARGET), _IOTE_RZA2M_)
-include $(ROOT)/build_make/mtkernel_3/device/ser/sysdepend/rza2m/subdir.mk
endif

ifeq ($(TARGET), _NUCLEO_H723_)
-include $(ROOT)/build_make/mtkernel_3/device/ser/sysdepend/stm32h7/subdir.mk
endif

ifeq ($(TARGET), _PICO_RP2040_)
-include $(ROOT)/build_make/mtkernel_3/device/ser/sysdepend/rp2040/subdir.mk
endif
