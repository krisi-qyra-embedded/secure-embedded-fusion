################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc_dma.c \
../Core/Src/bme680.c \
../Core/Src/clock.c \
../Core/Src/dma.c \
../Core/Src/i2c_dma.c \
../Core/Src/iwdg.c \
../Core/Src/main.c \
../Core/Src/mpu9250.c \
../Core/Src/pwr.c \
../Core/Src/sig_sensors.c \
../Core/Src/spi_dma.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/systick.c \
../Core/Src/tim.c \
../Core/Src/uart.c 

OBJS += \
./Core/Src/adc_dma.o \
./Core/Src/bme680.o \
./Core/Src/clock.o \
./Core/Src/dma.o \
./Core/Src/i2c_dma.o \
./Core/Src/iwdg.o \
./Core/Src/main.o \
./Core/Src/mpu9250.o \
./Core/Src/pwr.o \
./Core/Src/sig_sensors.o \
./Core/Src/spi_dma.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/systick.o \
./Core/Src/tim.o \
./Core/Src/uart.o 

C_DEPS += \
./Core/Src/adc_dma.d \
./Core/Src/bme680.d \
./Core/Src/clock.d \
./Core/Src/dma.d \
./Core/Src/i2c_dma.d \
./Core/Src/iwdg.d \
./Core/Src/main.d \
./Core/Src/mpu9250.d \
./Core/Src/pwr.d \
./Core/Src/sig_sensors.d \
./Core/Src/spi_dma.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/systick.d \
./Core/Src/tim.d \
./Core/Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc_dma.cyclo ./Core/Src/adc_dma.d ./Core/Src/adc_dma.o ./Core/Src/adc_dma.su ./Core/Src/bme680.cyclo ./Core/Src/bme680.d ./Core/Src/bme680.o ./Core/Src/bme680.su ./Core/Src/clock.cyclo ./Core/Src/clock.d ./Core/Src/clock.o ./Core/Src/clock.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/i2c_dma.cyclo ./Core/Src/i2c_dma.d ./Core/Src/i2c_dma.o ./Core/Src/i2c_dma.su ./Core/Src/iwdg.cyclo ./Core/Src/iwdg.d ./Core/Src/iwdg.o ./Core/Src/iwdg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mpu9250.cyclo ./Core/Src/mpu9250.d ./Core/Src/mpu9250.o ./Core/Src/mpu9250.su ./Core/Src/pwr.cyclo ./Core/Src/pwr.d ./Core/Src/pwr.o ./Core/Src/pwr.su ./Core/Src/sig_sensors.cyclo ./Core/Src/sig_sensors.d ./Core/Src/sig_sensors.o ./Core/Src/sig_sensors.su ./Core/Src/spi_dma.cyclo ./Core/Src/spi_dma.d ./Core/Src/spi_dma.o ./Core/Src/spi_dma.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/systick.cyclo ./Core/Src/systick.d ./Core/Src/systick.o ./Core/Src/systick.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/uart.cyclo ./Core/Src/uart.d ./Core/Src/uart.o ./Core/Src/uart.su

.PHONY: clean-Core-2f-Src

