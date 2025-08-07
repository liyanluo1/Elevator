################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Modules/Local_RS485/rs485_slave_adapter.c 

OBJS += \
./Core/Modules/Local_RS485/rs485_slave_adapter.o 

C_DEPS += \
./Core/Modules/Local_RS485/rs485_slave_adapter.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Modules/Local_RS485/%.o Core/Modules/Local_RS485/%.su Core/Modules/Local_RS485/%.cyclo: ../Core/Modules/Local_RS485/%.c Core/Modules/Local_RS485/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Modules-2f-Local_RS485

clean-Core-2f-Modules-2f-Local_RS485:
	-$(RM) ./Core/Modules/Local_RS485/rs485_slave_adapter.cyclo ./Core/Modules/Local_RS485/rs485_slave_adapter.d ./Core/Modules/Local_RS485/rs485_slave_adapter.o ./Core/Modules/Local_RS485/rs485_slave_adapter.su

.PHONY: clean-Core-2f-Modules-2f-Local_RS485

