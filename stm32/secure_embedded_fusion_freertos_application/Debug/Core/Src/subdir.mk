################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DMA.c \
../Core/Src/ESP32_uart.c \
../Core/Src/adc.c \
../Core/Src/adc_dma.c \
../Core/Src/bme680.c \
../Core/Src/bsp.c \
../Core/Src/clock.c \
../Core/Src/cmox_low_level.c \
../Core/Src/device_uid_entropy.c \
../Core/Src/ds18b20.c \
../Core/Src/ecdh.c \
../Core/Src/entropy_personalization_nonce_gen.c \
../Core/Src/fpu.c \
../Core/Src/i2c_dma.c \
../Core/Src/iwdg.c \
../Core/Src/main.c \
../Core/Src/max31865.c \
../Core/Src/mpu9250.c \
../Core/Src/nonce.c \
../Core/Src/onewire.c \
../Core/Src/pm25_uart.c \
../Core/Src/pm2_5.c \
../Core/Src/pwm_dma.c \
../Core/Src/pwr.c \
../Core/Src/sig_sensors.c \
../Core/Src/spi_dma.c \
../Core/Src/sram_startup_entropy.c \
../Core/Src/stm32_gcm_nvm.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/systick.c \
../Core/Src/telemetry.c \
../Core/Src/tim.c \
../Core/Src/tim3.c \
../Core/Src/uart.c 

OBJS += \
./Core/Src/DMA.o \
./Core/Src/ESP32_uart.o \
./Core/Src/adc.o \
./Core/Src/adc_dma.o \
./Core/Src/bme680.o \
./Core/Src/bsp.o \
./Core/Src/clock.o \
./Core/Src/cmox_low_level.o \
./Core/Src/device_uid_entropy.o \
./Core/Src/ds18b20.o \
./Core/Src/ecdh.o \
./Core/Src/entropy_personalization_nonce_gen.o \
./Core/Src/fpu.o \
./Core/Src/i2c_dma.o \
./Core/Src/iwdg.o \
./Core/Src/main.o \
./Core/Src/max31865.o \
./Core/Src/mpu9250.o \
./Core/Src/nonce.o \
./Core/Src/onewire.o \
./Core/Src/pm25_uart.o \
./Core/Src/pm2_5.o \
./Core/Src/pwm_dma.o \
./Core/Src/pwr.o \
./Core/Src/sig_sensors.o \
./Core/Src/spi_dma.o \
./Core/Src/sram_startup_entropy.o \
./Core/Src/stm32_gcm_nvm.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/systick.o \
./Core/Src/telemetry.o \
./Core/Src/tim.o \
./Core/Src/tim3.o \
./Core/Src/uart.o 

C_DEPS += \
./Core/Src/DMA.d \
./Core/Src/ESP32_uart.d \
./Core/Src/adc.d \
./Core/Src/adc_dma.d \
./Core/Src/bme680.d \
./Core/Src/bsp.d \
./Core/Src/clock.d \
./Core/Src/cmox_low_level.d \
./Core/Src/device_uid_entropy.d \
./Core/Src/ds18b20.d \
./Core/Src/ecdh.d \
./Core/Src/entropy_personalization_nonce_gen.d \
./Core/Src/fpu.d \
./Core/Src/i2c_dma.d \
./Core/Src/iwdg.d \
./Core/Src/main.d \
./Core/Src/max31865.d \
./Core/Src/mpu9250.d \
./Core/Src/nonce.d \
./Core/Src/onewire.d \
./Core/Src/pm25_uart.d \
./Core/Src/pm2_5.d \
./Core/Src/pwm_dma.d \
./Core/Src/pwr.d \
./Core/Src/sig_sensors.d \
./Core/Src/spi_dma.d \
./Core/Src/sram_startup_entropy.d \
./Core/Src/stm32_gcm_nvm.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/systick.d \
./Core/Src/telemetry.d \
./Core/Src/tim.d \
./Core/Src/tim3.d \
./Core/Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Middlewares/ST/STM32_Cryptographic/interface -I../Middlewares/ST/STM32_Cryptographic/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_Cryptographic/legacy_v3/include -I../Middlewares/ST/STM32_Cryptographic/legacy_v3/src -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/DMA.cyclo ./Core/Src/DMA.d ./Core/Src/DMA.o ./Core/Src/DMA.su ./Core/Src/ESP32_uart.cyclo ./Core/Src/ESP32_uart.d ./Core/Src/ESP32_uart.o ./Core/Src/ESP32_uart.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/adc_dma.cyclo ./Core/Src/adc_dma.d ./Core/Src/adc_dma.o ./Core/Src/adc_dma.su ./Core/Src/bme680.cyclo ./Core/Src/bme680.d ./Core/Src/bme680.o ./Core/Src/bme680.su ./Core/Src/bsp.cyclo ./Core/Src/bsp.d ./Core/Src/bsp.o ./Core/Src/bsp.su ./Core/Src/clock.cyclo ./Core/Src/clock.d ./Core/Src/clock.o ./Core/Src/clock.su ./Core/Src/cmox_low_level.cyclo ./Core/Src/cmox_low_level.d ./Core/Src/cmox_low_level.o ./Core/Src/cmox_low_level.su ./Core/Src/device_uid_entropy.cyclo ./Core/Src/device_uid_entropy.d ./Core/Src/device_uid_entropy.o ./Core/Src/device_uid_entropy.su ./Core/Src/ds18b20.cyclo ./Core/Src/ds18b20.d ./Core/Src/ds18b20.o ./Core/Src/ds18b20.su ./Core/Src/ecdh.cyclo ./Core/Src/ecdh.d ./Core/Src/ecdh.o ./Core/Src/ecdh.su ./Core/Src/entropy_personalization_nonce_gen.cyclo ./Core/Src/entropy_personalization_nonce_gen.d ./Core/Src/entropy_personalization_nonce_gen.o ./Core/Src/entropy_personalization_nonce_gen.su ./Core/Src/fpu.cyclo ./Core/Src/fpu.d ./Core/Src/fpu.o ./Core/Src/fpu.su ./Core/Src/i2c_dma.cyclo ./Core/Src/i2c_dma.d ./Core/Src/i2c_dma.o ./Core/Src/i2c_dma.su ./Core/Src/iwdg.cyclo ./Core/Src/iwdg.d ./Core/Src/iwdg.o ./Core/Src/iwdg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/max31865.cyclo ./Core/Src/max31865.d ./Core/Src/max31865.o ./Core/Src/max31865.su ./Core/Src/mpu9250.cyclo ./Core/Src/mpu9250.d ./Core/Src/mpu9250.o ./Core/Src/mpu9250.su ./Core/Src/nonce.cyclo ./Core/Src/nonce.d ./Core/Src/nonce.o ./Core/Src/nonce.su ./Core/Src/onewire.cyclo ./Core/Src/onewire.d ./Core/Src/onewire.o ./Core/Src/onewire.su ./Core/Src/pm25_uart.cyclo ./Core/Src/pm25_uart.d ./Core/Src/pm25_uart.o ./Core/Src/pm25_uart.su ./Core/Src/pm2_5.cyclo ./Core/Src/pm2_5.d ./Core/Src/pm2_5.o ./Core/Src/pm2_5.su ./Core/Src/pwm_dma.cyclo ./Core/Src/pwm_dma.d ./Core/Src/pwm_dma.o ./Core/Src/pwm_dma.su ./Core/Src/pwr.cyclo ./Core/Src/pwr.d ./Core/Src/pwr.o ./Core/Src/pwr.su ./Core/Src/sig_sensors.cyclo ./Core/Src/sig_sensors.d ./Core/Src/sig_sensors.o ./Core/Src/sig_sensors.su ./Core/Src/spi_dma.cyclo ./Core/Src/spi_dma.d ./Core/Src/spi_dma.o ./Core/Src/spi_dma.su ./Core/Src/sram_startup_entropy.cyclo ./Core/Src/sram_startup_entropy.d ./Core/Src/sram_startup_entropy.o ./Core/Src/sram_startup_entropy.su ./Core/Src/stm32_gcm_nvm.cyclo ./Core/Src/stm32_gcm_nvm.d ./Core/Src/stm32_gcm_nvm.o ./Core/Src/stm32_gcm_nvm.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/systick.cyclo ./Core/Src/systick.d ./Core/Src/systick.o ./Core/Src/systick.su ./Core/Src/telemetry.cyclo ./Core/Src/telemetry.d ./Core/Src/telemetry.o ./Core/Src/telemetry.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/tim3.cyclo ./Core/Src/tim3.d ./Core/Src/tim3.o ./Core/Src/tim3.su ./Core/Src/uart.cyclo ./Core/Src/uart.d ./Core/Src/uart.o ./Core/Src/uart.su

.PHONY: clean-Core-2f-Src

