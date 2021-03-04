################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include Thirdparty/VL53L0X_1.0.2/Api/platform/src/subdir.mk
-include Thirdparty/VL53L0X_1.0.2/Api/core/src/subdir.mk
-include Thirdparty/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Thirdparty/Middleware/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0/subdir.mk
-include Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Thirdparty/Middleware/FreeRTOS/Source/subdir.mk
-include Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/subdir.mk
-include H08R6/subdir.mk
-include Debug/User/subdir.mk
-include Debug/Thirdparty/VL53L0X_1.0.2/Api/platform/src/subdir.mk
-include Debug/Thirdparty/VL53L0X_1.0.2/Api/core/src/subdir.mk
-include Debug/Thirdparty/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Debug/Thirdparty/Middleware/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Debug/Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0/subdir.mk
-include Debug/Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Debug/Thirdparty/Middleware/FreeRTOS/Source/subdir.mk
-include Debug/Thirdparty/CMSIS/Device/ST/STM32F0xx/Source/Templates/subdir.mk
-include Debug/H08R6/subdir.mk
-include Debug/BOS/subdir.mk
-include Debug/subdir.mk
-include BOS/subdir.mk
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

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := H08R6x
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
H08R6x.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
H08R6x.list \

OBJCOPY_HEX += \
H08R6x.hex \


# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: H08R6x.elf secondary-outputs

# Tool invocations
H08R6x.elf: $(OBJS) $(USER_OBJS) D:\Hexabitz\for\ Release\Modules\ firmware\H08R6x\STM32CubeIDE\STM32F091RCTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "H08R6x.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"D:\Hexabitz\for Release\Modules firmware\H08R6x\STM32CubeIDE\STM32F091RCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="H08R6x.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

H08R6x.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "H08R6x.list"
	@echo 'Finished building: $@'
	@echo ' '

H08R6x.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "H08R6x.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	arm-none-eabi-objcopy -O ihex H08R6x.elf H08R6x.hex
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY: post-build

-include ../makefile.targets
