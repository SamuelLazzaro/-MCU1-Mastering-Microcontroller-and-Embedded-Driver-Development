################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := STM32NUCLEO_H563ZI_Drivers
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32NUCLEO_H563ZI_Drivers.elf \

MAP_FILES += \
STM32NUCLEO_H563ZI_Drivers.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32NUCLEO_H563ZI_Drivers.list \


# All Target
all: main-build

# Main-build Target
main-build: STM32NUCLEO_H563ZI_Drivers.elf secondary-outputs

# Tool invocations
STM32NUCLEO_H563ZI_Drivers.elf STM32NUCLEO_H563ZI_Drivers.map: $(OBJS) $(USER_OBJS) C:\Users\Samuel.Lazzaro\OneDrive\ -\ Carlo\ Gavazzi\ Automation\ SpA\Online\ Courses\(MCU1)\ Mastering\ Microcontroller\ and\ Embedded\ Driver\ Development\Project\STM32NUCLEO-H563ZI\STM32NUCLEO_H563ZI_Drivers\STM32H563ZITX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "STM32NUCLEO_H563ZI_Drivers.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m33 -T"C:\Users\Samuel.Lazzaro\OneDrive - Carlo Gavazzi Automation SpA\Online Courses\(MCU1) Mastering Microcontroller and Embedded Driver Development\Project\STM32NUCLEO-H563ZI\STM32NUCLEO_H563ZI_Drivers\STM32H563ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32NUCLEO_H563ZI_Drivers.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32NUCLEO_H563ZI_Drivers.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32NUCLEO_H563ZI_Drivers.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) STM32NUCLEO_H563ZI_Drivers.elf STM32NUCLEO_H563ZI_Drivers.list STM32NUCLEO_H563ZI_Drivers.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
