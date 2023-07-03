################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Bsp/lcd_hd44780u.c \
../Bsp/rtc_ds1307.c 

OBJS += \
./Bsp/lcd_hd44780u.o \
./Bsp/rtc_ds1307.o 

C_DEPS += \
./Bsp/lcd_hd44780u.d \
./Bsp/rtc_ds1307.d 


# Each subdirectory must supply rules for building sources it contributes
Bsp/%.o Bsp/%.su: ../Bsp/%.c Bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"/home/klee/repos/mcu-peripheral-drivers/workspace/stm32fxx_drivers/Drivers/Inc" -I"/home/klee/repos/mcu-peripheral-drivers/workspace/stm32fxx_drivers/Bsp" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Bsp

clean-Bsp:
	-$(RM) ./Bsp/lcd_hd44780u.d ./Bsp/lcd_hd44780u.o ./Bsp/lcd_hd44780u.su ./Bsp/rtc_ds1307.d ./Bsp/rtc_ds1307.o ./Bsp/rtc_ds1307.su

.PHONY: clean-Bsp

