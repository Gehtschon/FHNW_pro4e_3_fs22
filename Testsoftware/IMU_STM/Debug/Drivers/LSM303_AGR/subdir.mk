################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LSM303_AGR/lsm303agr.c 

OBJS += \
./Drivers/LSM303_AGR/lsm303agr.o 

C_DEPS += \
./Drivers/LSM303_AGR/lsm303agr.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LSM303_AGR/%.o Drivers/LSM303_AGR/%.su: ../Drivers/LSM303_AGR/%.c Drivers/LSM303_AGR/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L053xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/rolan/Documents/FHNW/2022 FS/pro4E/02_Unterricht/Software/pro4e_3_fs22/Testsoftware/IMU_STM/Drivers/LSM303_AGR" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-LSM303_AGR

clean-Drivers-2f-LSM303_AGR:
	-$(RM) ./Drivers/LSM303_AGR/lsm303agr.d ./Drivers/LSM303_AGR/lsm303agr.o ./Drivers/LSM303_AGR/lsm303agr.su

.PHONY: clean-Drivers-2f-LSM303_AGR

