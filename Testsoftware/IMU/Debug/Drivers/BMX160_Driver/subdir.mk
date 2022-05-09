################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BMX160_Driver/bmx160.c 

OBJS += \
./Drivers/BMX160_Driver/bmx160.o 

C_DEPS += \
./Drivers/BMX160_Driver/bmx160.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BMX160_Driver/%.o Drivers/BMX160_Driver/%.su: ../Drivers/BMX160_Driver/%.c Drivers/BMX160_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g1 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L053xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/rolan/Documents/FHNW/2022 FS/pro4E/02_Unterricht/Software/pro4e_3_fs22/Testsoftware/IMU/External/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-BMX160_Driver

clean-Drivers-2f-BMX160_Driver:
	-$(RM) ./Drivers/BMX160_Driver/bmx160.d ./Drivers/BMX160_Driver/bmx160.o ./Drivers/BMX160_Driver/bmx160.su

.PHONY: clean-Drivers-2f-BMX160_Driver

