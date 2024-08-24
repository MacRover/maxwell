################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/PID_lib/Src/PID_lib.c 

OBJS += \
./Drivers/PID_lib/Src/PID_lib.o 

C_DEPS += \
./Drivers/PID_lib/Src/PID_lib.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/PID_lib/Src/%.o Drivers/PID_lib/Src/%.su Drivers/PID_lib/Src/%.cyclo: ../Drivers/PID_lib/Src/%.c Drivers/PID_lib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Ethan/Documents/Repos/maxwell/firmware/STM32 Drivers/PID_lib/Drivers/PID_lib/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-PID_lib-2f-Src

clean-Drivers-2f-PID_lib-2f-Src:
	-$(RM) ./Drivers/PID_lib/Src/PID_lib.cyclo ./Drivers/PID_lib/Src/PID_lib.d ./Drivers/PID_lib/Src/PID_lib.o ./Drivers/PID_lib/Src/PID_lib.su

.PHONY: clean-Drivers-2f-PID_lib-2f-Src

