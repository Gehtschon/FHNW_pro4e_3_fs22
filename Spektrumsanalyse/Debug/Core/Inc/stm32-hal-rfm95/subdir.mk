################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/stm32-hal-rfm95/rfm95.c 

C_DEPS += \
./Core/Inc/stm32-hal-rfm95/rfm95.d 

OBJS += \
./Core/Inc/stm32-hal-rfm95/rfm95.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/stm32-hal-rfm95/%.o Core/Inc/stm32-hal-rfm95/%.su: ../Core/Inc/stm32-hal-rfm95/%.c Core/Inc/stm32-hal-rfm95/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L471xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-stm32-2d-hal-2d-rfm95

clean-Core-2f-Inc-2f-stm32-2d-hal-2d-rfm95:
	-$(RM) ./Core/Inc/stm32-hal-rfm95/rfm95.d ./Core/Inc/stm32-hal-rfm95/rfm95.o ./Core/Inc/stm32-hal-rfm95/rfm95.su

.PHONY: clean-Core-2f-Inc-2f-stm32-2d-hal-2d-rfm95

