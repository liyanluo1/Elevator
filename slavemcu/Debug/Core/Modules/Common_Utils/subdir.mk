################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Modules/Common_Utils/debounce.c \
../Core/Modules/Common_Utils/gpio_utils.c \
../Core/Modules/Common_Utils/ring_buffer.c 

OBJS += \
./Core/Modules/Common_Utils/debounce.o \
./Core/Modules/Common_Utils/gpio_utils.o \
./Core/Modules/Common_Utils/ring_buffer.o 

C_DEPS += \
./Core/Modules/Common_Utils/debounce.d \
./Core/Modules/Common_Utils/gpio_utils.d \
./Core/Modules/Common_Utils/ring_buffer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Modules/Common_Utils/%.o Core/Modules/Common_Utils/%.su Core/Modules/Common_Utils/%.cyclo: ../Core/Modules/Common_Utils/%.c Core/Modules/Common_Utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Modules-2f-Common_Utils

clean-Core-2f-Modules-2f-Common_Utils:
	-$(RM) ./Core/Modules/Common_Utils/debounce.cyclo ./Core/Modules/Common_Utils/debounce.d ./Core/Modules/Common_Utils/debounce.o ./Core/Modules/Common_Utils/debounce.su ./Core/Modules/Common_Utils/gpio_utils.cyclo ./Core/Modules/Common_Utils/gpio_utils.d ./Core/Modules/Common_Utils/gpio_utils.o ./Core/Modules/Common_Utils/gpio_utils.su ./Core/Modules/Common_Utils/ring_buffer.cyclo ./Core/Modules/Common_Utils/ring_buffer.d ./Core/Modules/Common_Utils/ring_buffer.o ./Core/Modules/Common_Utils/ring_buffer.su

.PHONY: clean-Core-2f-Modules-2f-Common_Utils

