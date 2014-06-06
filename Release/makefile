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
dongfangMartinezBleeding.lss \

FLASH_IMAGE += \
dongfangMartinezBleeding.hex \

SIZEDUMMY += \
sizedummy \

AVRDUDEDUMMY += \
avrdudedummy \


# All Target
all: dongfangMartinezBleeding.elf secondary-outputs

# Tool invocations
dongfangMartinezBleeding.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -lm -Wl,-Map,dongfangMartinezBleeding.map,--cref -mmcu=atmega328p -o "dongfangMartinezBleeding.elf" $(OBJS) $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

dongfangMartinezBleeding.lss: dongfangMartinezBleeding.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S dongfangMartinezBleeding.elf  >"dongfangMartinezBleeding.lss"
	@echo 'Finished building: $@'
	@echo ' '

dongfangMartinezBleeding.hex: dongfangMartinezBleeding.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex dongfangMartinezBleeding.elf  "dongfangMartinezBleeding.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: dongfangMartinezBleeding.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p dongfangMartinezBleeding.elf
	@echo 'Finished building: $@'
	@echo ' '

avrdudedummy: dongfangMartinezBleeding.elf
	@echo 'Invoking: AVRDude'
	/usr/bin/avrdude -pm328p -cavrisp2 -Pusb -Uflash:w:dongfangMartinezBleeding.hex:a
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(CXX_DEPS)$(S_UPPER_DEPS)$(C++_DEPS)$(ASM_DEPS)$(CC_DEPS)$(AVRDUDEDUMMY)$(CPP_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) dongfangMartinezBleeding.elf
	-@echo ' '

post-build:
	-scp dongfangMartinezBleeding.hex git@usg20:
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(SIZEDUMMY) $(AVRDUDEDUMMY)

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
