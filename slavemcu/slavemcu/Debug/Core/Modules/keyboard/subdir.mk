################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Modules/keyboard/keyboard.c 

OBJS += \
./Core/Modules/keyboard/keyboard.o 

C_DEPS += \
./Core/Modules/keyboard/keyboard.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Modules/keyboard/%.o Core/Modules/keyboard/%.su Core/Modules/keyboard/%.cyclo: ../Core/Modules/keyboard/%.c Core/Modules/keyboard/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Modules-2f-keyboard

clean-Core-2f-Modules-2f-keyboard:
	-$(RM) ./Core/Modules/keyboard/keyboard.cyclo ./Core/Modules/keyboard/keyboard.d ./Core/Modules/keyboard/keyboard.o ./Core/Modules/keyboard/keyboard.su

.PHONY: clean-Core-2f-Modules-2f-keyboard

