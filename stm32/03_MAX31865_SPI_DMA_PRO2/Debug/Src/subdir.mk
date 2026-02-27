################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/clock.c \
../Src/cmox_low_level.c \
../Src/dma.c \
../Src/iwdg.c \
../Src/main.c \
../Src/max31865.c \
../Src/pwr.c \
../Src/spi_dma.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/systick.c \
../Src/tim.c \
../Src/uart.c 

OBJS += \
./Src/clock.o \
./Src/cmox_low_level.o \
./Src/dma.o \
./Src/iwdg.o \
./Src/main.o \
./Src/max31865.o \
./Src/pwr.o \
./Src/spi_dma.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/systick.o \
./Src/tim.o \
./Src/uart.o 

C_DEPS += \
./Src/clock.d \
./Src/cmox_low_level.d \
./Src/dma.d \
./Src/iwdg.d \
./Src/main.d \
./Src/max31865.d \
./Src/pwr.d \
./Src/spi_dma.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/systick.d \
./Src/tim.d \
./Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I../Inc -I"C:/Users/user/Documents/Advanced_BareMetal_Programming/MAX31865_SPI_DMA_PRO/chip_headers/CMSIS/Include" -I"C:/Users/user/Documents/Advanced_BareMetal_Programming/MAX31865_SPI_DMA_PRO/chip_headers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/user/Documents/Advanced_BareMetal_Programming/MAX31865_SPI_DMA_PRO/Middlewares/ST/STM32_Cryptographic/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/clock.cyclo ./Src/clock.d ./Src/clock.o ./Src/clock.su ./Src/cmox_low_level.cyclo ./Src/cmox_low_level.d ./Src/cmox_low_level.o ./Src/cmox_low_level.su ./Src/dma.cyclo ./Src/dma.d ./Src/dma.o ./Src/dma.su ./Src/iwdg.cyclo ./Src/iwdg.d ./Src/iwdg.o ./Src/iwdg.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/max31865.cyclo ./Src/max31865.d ./Src/max31865.o ./Src/max31865.su ./Src/pwr.cyclo ./Src/pwr.d ./Src/pwr.o ./Src/pwr.su ./Src/spi_dma.cyclo ./Src/spi_dma.d ./Src/spi_dma.o ./Src/spi_dma.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/systick.cyclo ./Src/systick.d ./Src/systick.o ./Src/systick.su ./Src/tim.cyclo ./Src/tim.d ./Src/tim.o ./Src/tim.su ./Src/uart.cyclo ./Src/uart.d ./Src/uart.o ./Src/uart.su

.PHONY: clean-Src

