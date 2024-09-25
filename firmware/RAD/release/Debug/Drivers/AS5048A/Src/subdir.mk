################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/zokur/OneDrive/Documents/MMRT/maxwell/firmware/STM32\ Drivers/AS5048A/Drivers/AS5048A/Src/stm32f1xx_as5048a.c 

OBJS += \
./Drivers/AS5048A/Src/stm32f1xx_as5048a.o 

C_DEPS += \
./Drivers/AS5048A/Src/stm32f1xx_as5048a.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/AS5048A/Src/stm32f1xx_as5048a.o: C:/Users/zokur/OneDrive/Documents/MMRT/maxwell/firmware/STM32\ Drivers/AS5048A/Drivers/AS5048A/Src/stm32f1xx_as5048a.c Drivers/AS5048A/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/zokur/OneDrive/Documents/MMRT/maxwell/firmware/STM32 Drivers/TMC_2590/Drivers/TMC_2590/Inc" -I"C:/Users/zokur/OneDrive/Documents/MMRT/maxwell/firmware/STM32 Drivers/AS5048A/Drivers/AS5048A/Inc" -I"C:/Users/zokur/OneDrive/Documents/MMRT/maxwell/firmware/STM32 Drivers/PID_lib/Drivers/PID_lib/Inc" -I"C:/Users/zokur/OneDrive/Documents/MMRT/maxwell/firmware/STM32 Drivers/queue/Drivers/queue/Inc" -I"C:/Users/zokur/OneDrive/Documents/MMRT/maxwell/firmware/STM32 Drivers/AT24C04C/Drivers/AT24C04C/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-AS5048A-2f-Src

clean-Drivers-2f-AS5048A-2f-Src:
	-$(RM) ./Drivers/AS5048A/Src/stm32f1xx_as5048a.cyclo ./Drivers/AS5048A/Src/stm32f1xx_as5048a.d ./Drivers/AS5048A/Src/stm32f1xx_as5048a.o ./Drivers/AS5048A/Src/stm32f1xx_as5048a.su

.PHONY: clean-Drivers-2f-AS5048A-2f-Src

