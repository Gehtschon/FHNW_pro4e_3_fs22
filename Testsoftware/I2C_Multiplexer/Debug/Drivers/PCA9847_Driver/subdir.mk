################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/PCA9847_Driver/PCA9847.c 

C_DEPS += \
./Drivers/PCA9847_Driver/PCA9847.d 

OBJS += \
./Drivers/PCA9847_Driver/PCA9847.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/PCA9847_Driver/%.o Drivers/PCA9847_Driver/%.su: ../Drivers/PCA9847_Driver/%.c Drivers/PCA9847_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L452xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-PCA9847_Driver

clean-Drivers-2f-PCA9847_Driver:
	-$(RM) ./Drivers/PCA9847_Driver/PCA9847.d ./Drivers/PCA9847_Driver/PCA9847.o ./Drivers/PCA9847_Driver/PCA9847.su

.PHONY: clean-Drivers-2f-PCA9847_Driver

