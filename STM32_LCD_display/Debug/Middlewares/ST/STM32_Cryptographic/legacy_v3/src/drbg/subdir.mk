################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_Cryptographic/legacy_v3/src/drbg/legacy_v3_ctr_drbg.c 

OBJS += \
./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/drbg/legacy_v3_ctr_drbg.o 

C_DEPS += \
./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/drbg/legacy_v3_ctr_drbg.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_Cryptographic/legacy_v3/src/drbg/%.o Middlewares/ST/STM32_Cryptographic/legacy_v3/src/drbg/%.su Middlewares/ST/STM32_Cryptographic/legacy_v3/src/drbg/%.cyclo: ../Middlewares/ST/STM32_Cryptographic/legacy_v3/src/drbg/%.c Middlewares/ST/STM32_Cryptographic/legacy_v3/src/drbg/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Middlewares/ST/STM32_Cryptographic/interface -I../Middlewares/ST/STM32_Cryptographic/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/lvgl -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_Cryptographic/legacy_v3/include -I../Middlewares/ST/STM32_Cryptographic/legacy_v3/src -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -include../Drivers/lvgl/lvgl.h -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_Cryptographic-2f-legacy_v3-2f-src-2f-drbg

clean-Middlewares-2f-ST-2f-STM32_Cryptographic-2f-legacy_v3-2f-src-2f-drbg:
	-$(RM) ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/drbg/legacy_v3_ctr_drbg.cyclo ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/drbg/legacy_v3_ctr_drbg.d ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/drbg/legacy_v3_ctr_drbg.o ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/drbg/legacy_v3_ctr_drbg.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_Cryptographic-2f-legacy_v3-2f-src-2f-drbg

