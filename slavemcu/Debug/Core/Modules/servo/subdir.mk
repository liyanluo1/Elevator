################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Modules/servo/servo.c 

OBJS += \
./Core/Modules/servo/servo.o 

C_DEPS += \
./Core/Modules/servo/servo.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Modules/servo/%.o Core/Modules/servo/%.su Core/Modules/servo/%.cyclo: ../Core/Modules/servo/%.c Core/Modules/servo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Modules-2f-servo

clean-Core-2f-Modules-2f-servo:
	-$(RM) ./Core/Modules/servo/servo.cyclo ./Core/Modules/servo/servo.d ./Core/Modules/servo/servo.o ./Core/Modules/servo/servo.su

.PHONY: clean-Core-2f-Modules-2f-servo

