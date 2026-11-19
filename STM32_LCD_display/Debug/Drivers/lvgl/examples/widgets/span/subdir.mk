################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/lvgl/examples/widgets/span/lv_example_span_1.c 

OBJS += \
./Drivers/lvgl/examples/widgets/span/lv_example_span_1.o 

C_DEPS += \
./Drivers/lvgl/examples/widgets/span/lv_example_span_1.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/lvgl/examples/widgets/span/%.o Drivers/lvgl/examples/widgets/span/%.su Drivers/lvgl/examples/widgets/span/%.cyclo: ../Drivers/lvgl/examples/widgets/span/%.c Drivers/lvgl/examples/widgets/span/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Middlewares/ST/STM32_Cryptographic/interface -I../Middlewares/ST/STM32_Cryptographic/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/lvgl -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_Cryptographic/legacy_v3/include -I../Middlewares/ST/STM32_Cryptographic/legacy_v3/src -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -include../Drivers/lvgl/lvgl.h -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-lvgl-2f-examples-2f-widgets-2f-span

clean-Drivers-2f-lvgl-2f-examples-2f-widgets-2f-span:
	-$(RM) ./Drivers/lvgl/examples/widgets/span/lv_example_span_1.cyclo ./Drivers/lvgl/examples/widgets/span/lv_example_span_1.d ./Drivers/lvgl/examples/widgets/span/lv_example_span_1.o ./Drivers/lvgl/examples/widgets/span/lv_example_span_1.su

.PHONY: clean-Drivers-2f-lvgl-2f-examples-2f-widgets-2f-span

