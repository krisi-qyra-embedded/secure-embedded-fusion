################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Middlewares/ST/STM32_Cryptographic/legacy_v3/src/rsa/subdir.mk
-include Middlewares/ST/STM32_Cryptographic/legacy_v3/src/membuf/subdir.mk
-include Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/subdir.mk
-include Middlewares/ST/STM32_Cryptographic/legacy_v3/src/hash/subdir.mk
-include Middlewares/ST/STM32_Cryptographic/legacy_v3/src/ecc/subdir.mk
-include Middlewares/ST/STM32_Cryptographic/legacy_v3/src/drbg/subdir.mk
-include Middlewares/ST/STM32_Cryptographic/legacy_v3/src/cipher/subdir.mk
-include Middlewares/ST/STM32_Cryptographic/interface/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.elf \

MAP_FILES += \
secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.list \

OBJCOPY_BIN += \
secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.bin \


# All Target
all: main-build

# Main-build Target
main-build: secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.elf secondary-outputs

# Tool invocations
secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.elf secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.map: $(OBJS) $(USER_OBJS) C:\Users\user\Documents\stm32_projects\secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject\STM32F411RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\user\Documents\stm32_projects\secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject\STM32F411RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.map" -Wl,--gc-sections -static -L../Middlewares/ST/STM32_Cryptographic/lib -u _printf_float --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.list"
	@echo 'Finished building: $@'
	@echo ' '

secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.bin secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.elf secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.list secure_embedded_fusion_ESP32_UART_AES_GCM_FULL_DUPLEX_RECEIVER_TRANSMITTER_auth_fault_inject.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
