################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32F042CANI2C.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32F042CANI2C.list \

OBJCOPY_BIN += \
STM32F042CANI2C.bin \


# All Target
all: STM32F042CANI2C.elf secondary-outputs

# Tool invocations
STM32F042CANI2C.elf: $(OBJS) $(USER_OBJS) /home/igor/STM32CubeIDE/workspace_1.3.0/STM32F042CANI2C/STM32F042F6PX_FLASH.ld
	arm-none-eabi-gcc -o "STM32F042CANI2C.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"/home/igor/STM32CubeIDE/workspace_1.3.0/STM32F042CANI2C/STM32F042F6PX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32F042CANI2C.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32F042CANI2C.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32F042CANI2C.list"
	@echo 'Finished building: $@'
	@echo ' '

STM32F042CANI2C.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "STM32F042CANI2C.bin"
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
