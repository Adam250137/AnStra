################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
RF-2-nadajnik.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
RF-2-nadajnik.list \

OBJCOPY_BIN += \
RF-2-nadajnik.bin \


# All Target
all: RF-2-nadajnik.elf secondary-outputs

# Tool invocations
RF-2-nadajnik.elf: $(OBJS) $(USER_OBJS) C:\Users\micha\Documents\PWr\Sterowniki\ robotow\RF-2-nadajnik\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "RF-2-nadajnik.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\micha\Documents\PWr\Sterowniki robotow\RF-2-nadajnik\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="RF-2-nadajnik.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

RF-2-nadajnik.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "RF-2-nadajnik.list"
	@echo 'Finished building: $@'
	@echo ' '

RF-2-nadajnik.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "RF-2-nadajnik.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
