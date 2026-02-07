################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middleware/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.c 

OBJS += \
./Middleware/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o 

C_DEPS += \
./Middleware/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d 


# Each subdirectory must supply rules for building sources it contributes
Middleware/SEGGER/SEGGER/Syscalls/%.o Middleware/SEGGER/SEGGER/Syscalls/%.su Middleware/SEGGER/SEGGER/Syscalls/%.cyclo: ../Middleware/SEGGER/SEGGER/Syscalls/%.c Middleware/SEGGER/SEGGER/Syscalls/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../Core/Inc -I"C:/Users/shiva/Downloads/Sensor_Integration_CAN (2)/Sensor_Integration_CAN/Middleware/FreeRTOS" -I"C:/Users/shiva/Downloads/Sensor_Integration_CAN (2)/Sensor_Integration_CAN/Middleware/FreeRTOS/include" -I"C:/Users/shiva/Downloads/Sensor_Integration_CAN (2)/Sensor_Integration_CAN/Middleware/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/Users/shiva/Downloads/Sensor_Integration_CAN (2)/Sensor_Integration_CAN/Middleware/SEGGER/Config" -I"C:/Users/shiva/Downloads/Sensor_Integration_CAN (2)/Sensor_Integration_CAN/Middleware/SEGGER/FreeRTOSV11" -I"C:/Users/shiva/Downloads/Sensor_Integration_CAN (2)/Sensor_Integration_CAN/Middleware/SEGGER/SEGGER" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middleware-2f-SEGGER-2f-SEGGER-2f-Syscalls

clean-Middleware-2f-SEGGER-2f-SEGGER-2f-Syscalls:
	-$(RM) ./Middleware/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.cyclo ./Middleware/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d ./Middleware/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o ./Middleware/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.su

.PHONY: clean-Middleware-2f-SEGGER-2f-SEGGER-2f-Syscalls

