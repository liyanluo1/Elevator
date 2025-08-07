################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Modules/RS485_Common/Core/rs485_core.c 

OBJS += \
./Core/Modules/RS485_Common/Core/rs485_core.o 

C_DEPS += \
./Core/Modules/RS485_Common/Core/rs485_core.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Modules/RS485_Common/Core/%.o Core/Modules/RS485_Common/Core/%.su Core/Modules/RS485_Common/Core/%.cyclo: ../Core/Modules/RS485_Common/Core/%.c Core/Modules/RS485_Common/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Modules-2f-RS485_Common-2f-Core

clean-Core-2f-Modules-2f-RS485_Common-2f-Core:
	-$(RM) ./Core/Modules/RS485_Common/Core/rs485_core.cyclo ./Core/Modules/RS485_Common/Core/rs485_core.d ./Core/Modules/RS485_Common/Core/rs485_core.o ./Core/Modules/RS485_Common/Core/rs485_core.su

.PHONY: clean-Core-2f-Modules-2f-RS485_Common-2f-Core

