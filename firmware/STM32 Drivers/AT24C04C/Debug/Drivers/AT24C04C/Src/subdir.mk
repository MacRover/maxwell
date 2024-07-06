################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/AT24C04C/Src/stm32f1xx_at24c04c.c 

OBJS += \
./Drivers/AT24C04C/Src/stm32f1xx_at24c04c.o 

C_DEPS += \
./Drivers/AT24C04C/Src/stm32f1xx_at24c04c.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/AT24C04C/Src/%.o Drivers/AT24C04C/Src/%.su Drivers/AT24C04C/Src/%.cyclo: ../Drivers/AT24C04C/Src/%.c Drivers/AT24C04C/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/zokur/OneDrive/Documents/MMRT/maxwell/firmware/STM32 Drivers/AT24C04C/Drivers/AT24C04C/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-AT24C04C-2f-Src

clean-Drivers-2f-AT24C04C-2f-Src:
	-$(RM) ./Drivers/AT24C04C/Src/stm32f1xx_at24c04c.cyclo ./Drivers/AT24C04C/Src/stm32f1xx_at24c04c.d ./Drivers/AT24C04C/Src/stm32f1xx_at24c04c.o ./Drivers/AT24C04C/Src/stm32f1xx_at24c04c.su

.PHONY: clean-Drivers-2f-AT24C04C-2f-Src

