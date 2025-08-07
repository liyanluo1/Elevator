################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Modules/Local_BB/local_blackboard.c 

OBJS += \
./Core/Modules/Local_BB/local_blackboard.o 

C_DEPS += \
./Core/Modules/Local_BB/local_blackboard.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Modules/Local_BB/%.o Core/Modules/Local_BB/%.su Core/Modules/Local_BB/%.cyclo: ../Core/Modules/Local_BB/%.c Core/Modules/Local_BB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Modules-2f-Local_BB

clean-Core-2f-Modules-2f-Local_BB:
	-$(RM) ./Core/Modules/Local_BB/local_blackboard.cyclo ./Core/Modules/Local_BB/local_blackboard.d ./Core/Modules/Local_BB/local_blackboard.o ./Core/Modules/Local_BB/local_blackboard.su

.PHONY: clean-Core-2f-Modules-2f-Local_BB

