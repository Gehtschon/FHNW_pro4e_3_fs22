################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LSM303AGR.c 

OBJS += \
./Drivers/LSM303AGR.o 

C_DEPS += \
./Drivers/LSM303AGR.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/%.o Drivers/%.su: ../Drivers/%.c Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L053xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/rolan/Documents/FHNW/2022 FS/pro4E/02_Unterricht/Software/pro4e_3_fs22/Testsoftware/IMU_STM/Drivers/LSM303_AGR" -I"C:/Users/rolan/Documents/FHNW/2022 FS/pro4E/02_Unterricht/Software/pro4e_3_fs22/Testsoftware/IMU_STM/Drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers

clean-Drivers:
	-$(RM) ./Drivers/LSM303AGR.d ./Drivers/LSM303AGR.o ./Drivers/LSM303AGR.su

.PHONY: clean-Drivers

