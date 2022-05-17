################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/MS5607_Driver/MS5607.cpp 

OBJS += \
./Drivers/MS5607_Driver/MS5607.o 

CPP_DEPS += \
./Drivers/MS5607_Driver/MS5607.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MS5607_Driver/%.o Drivers/MS5607_Driver/%.su: ../Drivers/MS5607_Driver/%.cpp Drivers/MS5607_Driver/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L452xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MS5607_Driver

clean-Drivers-2f-MS5607_Driver:
	-$(RM) ./Drivers/MS5607_Driver/MS5607.d ./Drivers/MS5607_Driver/MS5607.o ./Drivers/MS5607_Driver/MS5607.su

.PHONY: clean-Drivers-2f-MS5607_Driver

