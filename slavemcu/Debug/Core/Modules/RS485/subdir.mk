################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Modules/RS485/rs485.c \
../Core/Modules/RS485/rs485_driver.c \
../Core/Modules/RS485/rs485_protocol.c \
../Core/Modules/RS485/rs485_test.c 

OBJS += \
./Core/Modules/RS485/rs485.o \
./Core/Modules/RS485/rs485_driver.o \
./Core/Modules/RS485/rs485_protocol.o \
./Core/Modules/RS485/rs485_test.o 

C_DEPS += \
./Core/Modules/RS485/rs485.d \
./Core/Modules/RS485/rs485_driver.d \
./Core/Modules/RS485/rs485_protocol.d \
./Core/Modules/RS485/rs485_test.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Modules/RS485/%.o Core/Modules/RS485/%.su Core/Modules/RS485/%.cyclo: ../Core/Modules/RS485/%.c Core/Modules/RS485/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Modules-2f-RS485

clean-Core-2f-Modules-2f-RS485:
	-$(RM) ./Core/Modules/RS485/rs485.cyclo ./Core/Modules/RS485/rs485.d ./Core/Modules/RS485/rs485.o ./Core/Modules/RS485/rs485.su ./Core/Modules/RS485/rs485_driver.cyclo ./Core/Modules/RS485/rs485_driver.d ./Core/Modules/RS485/rs485_driver.o ./Core/Modules/RS485/rs485_driver.su ./Core/Modules/RS485/rs485_protocol.cyclo ./Core/Modules/RS485/rs485_protocol.d ./Core/Modules/RS485/rs485_protocol.o ./Core/Modules/RS485/rs485_protocol.su ./Core/Modules/RS485/rs485_test.cyclo ./Core/Modules/RS485/rs485_test.d ./Core/Modules/RS485/rs485_test.o ./Core/Modules/RS485/rs485_test.su

.PHONY: clean-Core-2f-Modules-2f-RS485

