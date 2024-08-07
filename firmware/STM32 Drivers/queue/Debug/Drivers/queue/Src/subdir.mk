################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/queue/Src/queue.c 

OBJS += \
./Drivers/queue/Src/queue.o 

C_DEPS += \
./Drivers/queue/Src/queue.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/queue/Src/%.o Drivers/queue/Src/%.su Drivers/queue/Src/%.cyclo: ../Drivers/queue/Src/%.c Drivers/queue/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Ethan/Documents/Repos/maxwell/firmware/STM32 Drivers/queue/Drivers/queue/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-queue-2f-Src

clean-Drivers-2f-queue-2f-Src:
	-$(RM) ./Drivers/queue/Src/queue.cyclo ./Drivers/queue/Src/queue.d ./Drivers/queue/Src/queue.o ./Drivers/queue/Src/queue.su

.PHONY: clean-Drivers-2f-queue-2f-Src

