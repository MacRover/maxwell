################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Ethan/Documents/Repos/maxwell/firmware/STM32\ Drivers/TMC_2590/Drivers/TMC_2590/Src/stm32f1xx_tmc_2590.c 

OBJS += \
./Drivers/TMC_2590/Src/stm32f1xx_tmc_2590.o 

C_DEPS += \
./Drivers/TMC_2590/Src/stm32f1xx_tmc_2590.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/TMC_2590/Src/stm32f1xx_tmc_2590.o: C:/Users/Ethan/Documents/Repos/maxwell/firmware/STM32\ Drivers/TMC_2590/Drivers/TMC_2590/Src/stm32f1xx_tmc_2590.c Drivers/TMC_2590/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Ethan/Documents/Repos/maxwell/firmware/STM32 Drivers/TMC_2590/Drivers/TMC_2590/Inc" -I"C:/Users/Ethan/Documents/Repos/maxwell/firmware/STM32 Drivers/AS5048A/Drivers/AS5048A/Inc" -I"C:/Users/Ethan/Documents/Repos/maxwell/firmware/STM32 Drivers/PID_lib/Drivers/PID_lib/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-TMC_2590-2f-Src

clean-Drivers-2f-TMC_2590-2f-Src:
	-$(RM) ./Drivers/TMC_2590/Src/stm32f1xx_tmc_2590.cyclo ./Drivers/TMC_2590/Src/stm32f1xx_tmc_2590.d ./Drivers/TMC_2590/Src/stm32f1xx_tmc_2590.o ./Drivers/TMC_2590/Src/stm32f1xx_tmc_2590.su

.PHONY: clean-Drivers-2f-TMC_2590-2f-Src

