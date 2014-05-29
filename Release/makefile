################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
dongfangMartinezRewrite.lss \

FLASH_IMAGE += \
dongfangMartinezRewrite.hex \

SIZEDUMMY += \
sizedummy \


# All Target
all: dongfangMartinezRewrite.elf secondary-outputs

# Tool invocations
dongfangMartinezRewrite.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -lm -Wl,-Map,dongfangMartinezRewrite.map,--cref -mmcu=atmega328p -o "dongfangMartinezRewrite.elf" $(OBJS) $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

dongfangMartinezRewrite.lss: dongfangMartinezRewrite.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S dongfangMartinezRewrite.elf  >"dongfangMartinezRewrite.lss"
	@echo 'Finished building: $@'
	@echo ' '

dongfangMartinezRewrite.hex: dongfangMartinezRewrite.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex dongfangMartinezRewrite.elf  "dongfangMartinezRewrite.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: dongfangMartinezRewrite.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p dongfangMartinezRewrite.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(CXX_DEPS)$(S_UPPER_DEPS)$(C++_DEPS)$(ASM_DEPS)$(CC_DEPS)$(CPP_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) dongfangMartinezRewrite.elf
	-@echo ' '

post-build:

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
