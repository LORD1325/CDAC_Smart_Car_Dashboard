################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middleware/SEGGER/FreeRTOSV11/SEGGER_SYSVIEW_FreeRTOS.c 

OBJS += \
./Middleware/SEGGER/FreeRTOSV11/SEGGER_SYSVIEW_FreeRTOS.o 

C_DEPS += \
./Middleware/SEGGER/FreeRTOSV11/SEGGER_SYSVIEW_FreeRTOS.d 


# Each subdirectory must supply rules for building sources it contributes
Middleware/SEGGER/FreeRTOSV11/%.o Middleware/SEGGER/FreeRTOSV11/%.su Middleware/SEGGER/FreeRTOSV11/%.cyclo: ../Middleware/SEGGER/FreeRTOSV11/%.c Middleware/SEGGER/FreeRTOSV11/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../Core/Inc -I"C:/Users/shiva/Downloads/Sensor_Integration_CAN (2)/Sensor_Integration_CAN/Middleware/FreeRTOS" -I"C:/Users/shiva/Downloads/Sensor_Integration_CAN (2)/Sensor_Integration_CAN/Middleware/FreeRTOS/include" -I"C:/Users/shiva/Downloads/Sensor_Integration_CAN (2)/Sensor_Integration_CAN/Middleware/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/Users/shiva/Downloads/Sensor_Integration_CAN (2)/Sensor_Integration_CAN/Middleware/SEGGER/Config" -I"C:/Users/shiva/Downloads/Sensor_Integration_CAN (2)/Sensor_Integration_CAN/Middleware/SEGGER/FreeRTOSV11" -I"C:/Users/shiva/Downloads/Sensor_Integration_CAN (2)/Sensor_Integration_CAN/Middleware/SEGGER/SEGGER" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middleware-2f-SEGGER-2f-FreeRTOSV11

clean-Middleware-2f-SEGGER-2f-FreeRTOSV11:
	-$(RM) ./Middleware/SEGGER/FreeRTOSV11/SEGGER_SYSVIEW_FreeRTOS.cyclo ./Middleware/SEGGER/FreeRTOSV11/SEGGER_SYSVIEW_FreeRTOS.d ./Middleware/SEGGER/FreeRTOSV11/SEGGER_SYSVIEW_FreeRTOS.o ./Middleware/SEGGER/FreeRTOSV11/SEGGER_SYSVIEW_FreeRTOS.su

.PHONY: clean-Middleware-2f-SEGGER-2f-FreeRTOSV11

