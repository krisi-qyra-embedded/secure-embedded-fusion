################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/lvgl/src/extra/widgets/list/lv_list.c 

OBJS += \
./Drivers/lvgl/src/extra/widgets/list/lv_list.o 

C_DEPS += \
./Drivers/lvgl/src/extra/widgets/list/lv_list.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/lvgl/src/extra/widgets/list/%.o Drivers/lvgl/src/extra/widgets/list/%.su Drivers/lvgl/src/extra/widgets/list/%.cyclo: ../Drivers/lvgl/src/extra/widgets/list/%.c Drivers/lvgl/src/extra/widgets/list/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Middlewares/ST/STM32_Cryptographic/interface -I../Middlewares/ST/STM32_Cryptographic/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/lvgl -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_Cryptographic/legacy_v3/include -I../Middlewares/ST/STM32_Cryptographic/legacy_v3/src -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -include../Drivers/lvgl/lvgl.h -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-lvgl-2f-src-2f-extra-2f-widgets-2f-list

clean-Drivers-2f-lvgl-2f-src-2f-extra-2f-widgets-2f-list:
	-$(RM) ./Drivers/lvgl/src/extra/widgets/list/lv_list.cyclo ./Drivers/lvgl/src/extra/widgets/list/lv_list.d ./Drivers/lvgl/src/extra/widgets/list/lv_list.o ./Drivers/lvgl/src/extra/widgets/list/lv_list.su

.PHONY: clean-Drivers-2f-lvgl-2f-src-2f-extra-2f-widgets-2f-list

