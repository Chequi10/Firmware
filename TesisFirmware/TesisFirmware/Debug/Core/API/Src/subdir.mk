################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/API/Src/API_delay.c \
../Core/API/Src/API_uart.c \
../Core/API/Src/decode.c 

OBJS += \
./Core/API/Src/API_delay.o \
./Core/API/Src/API_uart.o \
./Core/API/Src/decode.o 

C_DEPS += \
./Core/API/Src/API_delay.d \
./Core/API/Src/API_uart.d \
./Core/API/Src/decode.d 


# Each subdirectory must supply rules for building sources it contributes
Core/API/Src/%.o Core/API/Src/%.su: ../Core/API/Src/%.c Core/API/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/ezequiel/Firmware/Firmware/TesisFirmware/TesisFirmware/Core/API" -I"/home/ezequiel/Firmware/Firmware/TesisFirmware/TesisFirmware/Core/API/Src" -I"/home/ezequiel/Firmware/Firmware/TesisFirmware/TesisFirmware/Core/API/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-API-2f-Src

clean-Core-2f-API-2f-Src:
	-$(RM) ./Core/API/Src/API_delay.d ./Core/API/Src/API_delay.o ./Core/API/Src/API_delay.su ./Core/API/Src/API_uart.d ./Core/API/Src/API_uart.o ./Core/API/Src/API_uart.su ./Core/API/Src/decode.d ./Core/API/Src/decode.o ./Core/API/Src/decode.su

.PHONY: clean-Core-2f-API-2f-Src

