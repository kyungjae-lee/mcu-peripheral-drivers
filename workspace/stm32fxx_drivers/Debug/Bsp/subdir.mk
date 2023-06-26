################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Bsp/ds1307.c \
../Bsp/lcd.c 

OBJS += \
./Bsp/ds1307.o \
./Bsp/lcd.o 

C_DEPS += \
./Bsp/ds1307.d \
./Bsp/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Bsp/%.o Bsp/%.su: ../Bsp/%.c Bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"/home/klee/repos/mcu-peripheral-drivers/workspace/stm32fxx_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Bsp

clean-Bsp:
	-$(RM) ./Bsp/ds1307.d ./Bsp/ds1307.o ./Bsp/ds1307.su ./Bsp/lcd.d ./Bsp/lcd.o ./Bsp/lcd.su

.PHONY: clean-Bsp

