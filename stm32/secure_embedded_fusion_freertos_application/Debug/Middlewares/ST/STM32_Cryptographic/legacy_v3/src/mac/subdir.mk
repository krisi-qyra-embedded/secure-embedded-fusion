################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_cmac.c \
../Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha1.c \
../Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha224.c \
../Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha256.c \
../Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha384.c \
../Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha512.c 

OBJS += \
./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_cmac.o \
./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha1.o \
./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha224.o \
./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha256.o \
./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha384.o \
./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha512.o 

C_DEPS += \
./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_cmac.d \
./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha1.d \
./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha224.d \
./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha256.d \
./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha384.d \
./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha512.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/%.o Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/%.su Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/%.cyclo: ../Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/%.c Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Middlewares/ST/STM32_Cryptographic/legacy_v3/include -I../Middlewares/ST/STM32_Cryptographic/legacy_v3/src -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_Cryptographic/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_Cryptographic-2f-legacy_v3-2f-src-2f-mac

clean-Middlewares-2f-ST-2f-STM32_Cryptographic-2f-legacy_v3-2f-src-2f-mac:
	-$(RM) ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_cmac.cyclo ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_cmac.d ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_cmac.o ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_cmac.su ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha1.cyclo ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha1.d ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha1.o ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha1.su ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha224.cyclo ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha224.d ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha224.o ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha224.su ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha256.cyclo ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha256.d ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha256.o ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha256.su ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha384.cyclo ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha384.d ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha384.o ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha384.su ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha512.cyclo ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha512.d ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha512.o ./Middlewares/ST/STM32_Cryptographic/legacy_v3/src/mac/legacy_v3_hmac_sha512.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_Cryptographic-2f-legacy_v3-2f-src-2f-mac

