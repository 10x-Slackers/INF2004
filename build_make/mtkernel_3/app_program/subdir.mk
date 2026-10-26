################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# app_program/*.c plus one level of subfolders (one per subsystem)
TEMP_SRCS = $(wildcard $(ROOT)/app_program/*.c) $(wildcard $(ROOT)/app_program/*/*.c)
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(patsubst $(ROOT)/%,./mtkernel_3/%,$(TEMP_OBJS))
C_DEPS += $(patsubst $(ROOT)/%,./mtkernel_3/%,$(TEMP_DEPS))

mtkernel_3/app_program/%.o: $(ROOT)/app_program/%.c
	@echo 'Building file: $<'
	@mkdir -p $(dir $@)
	$(GCC) $(CFLAGS) $(APP_CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

mtkernel_3/app_program/subsystem2/%.o: APP_CFLAGS = -std=c99 -Wall -Wextra -Werror
