################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
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
NucleoPinout.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
NucleoPinout.list \

OBJCOPY_BIN += \
NucleoPinout.bin \


# All Target
all: NucleoPinout.elf secondary-outputs

# Tool invocations
NucleoPinout.elf: $(OBJS) $(USER_OBJS) C:\Users\Ben\Documents\Thunderbots\PCB_NucleoClone\STM32CubeIDE\STM32H743ZITX_FLASH.ld
	arm-none-eabi-gcc -o "NucleoPinout.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\Ben\Documents\Thunderbots\PCB_NucleoClone\STM32CubeIDE\STM32H743ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="NucleoPinout.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

NucleoPinout.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "NucleoPinout.list"
	@echo 'Finished building: $@'
	@echo ' '

NucleoPinout.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "NucleoPinout.bin"
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
