################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/AS7341_Driver/DEV_Config.c \
../Drivers/AS7341_Driver/Waveshare_AS7341.c 

C_DEPS += \
./Drivers/AS7341_Driver/DEV_Config.d \
./Drivers/AS7341_Driver/Waveshare_AS7341.d 

OBJS += \
./Drivers/AS7341_Driver/DEV_Config.o \
./Drivers/AS7341_Driver/Waveshare_AS7341.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/AS7341_Driver/%.o Drivers/AS7341_Driver/%.su: ../Drivers/AS7341_Driver/%.c Drivers/AS7341_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L471xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-AS7341_Driver

clean-Drivers-2f-AS7341_Driver:
	-$(RM) ./Drivers/AS7341_Driver/DEV_Config.d ./Drivers/AS7341_Driver/DEV_Config.o ./Drivers/AS7341_Driver/DEV_Config.su ./Drivers/AS7341_Driver/Waveshare_AS7341.d ./Drivers/AS7341_Driver/Waveshare_AS7341.o ./Drivers/AS7341_Driver/Waveshare_AS7341.su

.PHONY: clean-Drivers-2f-AS7341_Driver

