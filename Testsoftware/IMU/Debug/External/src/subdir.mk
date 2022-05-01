################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../External/src/bmi160.c 

OBJS += \
./External/src/bmi160.o 

C_DEPS += \
./External/src/bmi160.d 


# Each subdirectory must supply rules for building sources it contributes
External/src/%.o External/src/%.su: ../External/src/%.c External/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g1 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L053xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/rolan/Documents/FHNW/2022 FS/pro4E/02_Unterricht/Software/pro4e_3_fs22/Testsoftware/IMU/External/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-External-2f-src

clean-External-2f-src:
	-$(RM) ./External/src/bmi160.d ./External/src/bmi160.o ./External/src/bmi160.su

.PHONY: clean-External-2f-src

