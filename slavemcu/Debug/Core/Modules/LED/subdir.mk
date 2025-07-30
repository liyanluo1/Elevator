################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Modules/LED/LED.c 

OBJS += \
./Core/Modules/LED/LED.o 

C_DEPS += \
./Core/Modules/LED/LED.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Modules/LED/%.o Core/Modules/LED/%.su Core/Modules/LED/%.cyclo: ../Core/Modules/LED/%.c Core/Modules/LED/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Modules-2f-LED

clean-Core-2f-Modules-2f-LED:
	-$(RM) ./Core/Modules/LED/LED.cyclo ./Core/Modules/LED/LED.d ./Core/Modules/LED/LED.o ./Core/Modules/LED/LED.su

.PHONY: clean-Core-2f-Modules-2f-LED

