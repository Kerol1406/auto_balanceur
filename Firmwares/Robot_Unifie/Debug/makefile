################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USER/subdir.mk
-include Startup/subdir.mk
-include STM32F10x_StdPeriph_Driver/src/subdir.mk
-include Controllers/subdir.mk
-include CMSIS/subdir.mk
-include BSP/Usart1/subdir.mk
-include BSP/Timer/subdir.mk
-include BSP/OLED/subdir.mk
-include BSP/Motor/subdir.mk
-include BSP/MPU6050/DMP/subdir.mk
-include BSP/MPU6050/subdir.mk
-include BSP/LED/subdir.mk
-include BSP/Key/subdir.mk
-include BSP/INT_Sever/subdir.mk
-include BSP/IIC_Software/subdir.mk
-include BSP/Enconder/subdir.mk
-include BSP/Delay/subdir.mk
-include BSP/Beep/subdir.mk
-include BSP/Battery/subdir.mk
-include BSP/subdir.mk
-include APP/filter/subdir.mk
-include APP/KF/subdir.mk
-include APP/subdir.mk
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

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Robot_Unifie
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Robot_Unifie.elf \

MAP_FILES += \
Robot_Unifie.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Robot_Unifie.list \

OBJCOPY_HEX += \
Robot_Unifie.hex \

OBJCOPY_BIN += \
Robot_Unifie.bin \


# All Target
all: main-build

# Main-build Target
main-build: Robot_Unifie.elf secondary-outputs

# Tool invocations
Robot_Unifie.elf Robot_Unifie.map: $(OBJS) $(USER_OBJS) /home/mr_sun/STM32CubeIDE/workspace_2.2.0/Robot_Unifie/STM32F103RCTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Robot_Unifie.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"/home/mr_sun/STM32CubeIDE/workspace_2.2.0/Robot_Unifie/STM32F103RCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Robot_Unifie.map" -Wl,--gc-sections -static -u _printf_float --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Robot_Unifie.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Robot_Unifie.list"
	@echo 'Finished building: $@'
	@echo ' '

Robot_Unifie.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Robot_Unifie.hex"
	@echo 'Finished building: $@'
	@echo ' '

Robot_Unifie.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Robot_Unifie.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Robot_Unifie.bin Robot_Unifie.elf Robot_Unifie.hex Robot_Unifie.list Robot_Unifie.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
