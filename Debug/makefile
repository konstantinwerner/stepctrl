################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include usb/vendor/subdir.mk
-include usb/subdir.mk
-include startup/subdir.mk
-include src/subdir.mk
-include modules/subdir.mk
-include inc/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: stepctrl.axf

# Tool invocations
stepctrl.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -Xlinker -Map="stepctrl.map" -Xlinker --gc-sections -mcpu=cortex-m0 -mthumb -T ../linker/LPC11Uxx.ld -o"stepctrl.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) stepctrl.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "stepctrl.axf"; arm-none-eabi-objcopy -O binary "stepctrl.axf" "stepctrl.bin" ; checksum -p LPC11U24_401 -d "stepctrl.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
