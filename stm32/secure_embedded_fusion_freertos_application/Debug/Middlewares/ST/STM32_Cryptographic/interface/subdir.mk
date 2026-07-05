################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_Cryptographic/interface/cmox_low_level_template.c 

OBJS += \
./Middlewares/ST/STM32_Cryptographic/interface/cmox_low_level_template.o 

C_DEPS += \
./Middlewares/ST/STM32_Cryptographic/interface/cmox_low_level_template.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_Cryptographic/interface/%.o Middlewares/ST/STM32_Cryptographic/interface/%.su Middlewares/ST/STM32_Cryptographic/interface/%.cyclo: ../Middlewares/ST/STM32_Cryptographic/interface/%.c Middlewares/ST/STM32_Cryptographic/interface/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Middlewares/ST/STM32_Cryptographic/legacy_v3/include -I../Middlewares/ST/STM32_Cryptographic/legacy_v3/src -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_Cryptographic/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_Cryptographic-2f-interface

clean-Middlewares-2f-ST-2f-STM32_Cryptographic-2f-interface:
	-$(RM) ./Middlewares/ST/STM32_Cryptographic/interface/cmox_low_level_template.cyclo ./Middlewares/ST/STM32_Cryptographic/interface/cmox_low_level_template.d ./Middlewares/ST/STM32_Cryptographic/interface/cmox_low_level_template.o ./Middlewares/ST/STM32_Cryptographic/interface/cmox_low_level_template.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_Cryptographic-2f-interface

