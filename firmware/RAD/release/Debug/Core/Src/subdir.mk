################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/as5048a.c \
../Core/Src/at24c04c.c \
../Core/Src/can.c \
../Core/Src/dma.c \
../Core/Src/enc_dec_utils.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/queue.c \
../Core/Src/spi.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/tim.c \
../Core/Src/tmc_2590.c 

OBJS += \
./Core/Src/as5048a.o \
./Core/Src/at24c04c.o \
./Core/Src/can.o \
./Core/Src/dma.o \
./Core/Src/enc_dec_utils.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/queue.o \
./Core/Src/spi.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/tim.o \
./Core/Src/tmc_2590.o 

C_DEPS += \
./Core/Src/as5048a.d \
./Core/Src/at24c04c.d \
./Core/Src/can.d \
./Core/Src/dma.d \
./Core/Src/enc_dec_utils.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/queue.d \
./Core/Src/spi.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/tim.d \
./Core/Src/tmc_2590.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/Users/vaibhavgopal/Projects/maxwell/firmware/STM32 Drivers/TMC_2590/Drivers/TMC_2590/Inc" -I"/Users/vaibhavgopal/Projects/maxwell/firmware/STM32 Drivers/AS5048A/Drivers/AS5048A/Inc" -I"/Users/vaibhavgopal/Projects/maxwell/firmware/STM32 Drivers/PID_lib/Drivers/PID_lib/Inc" -I"/Users/vaibhavgopal/Projects/maxwell/firmware/STM32 Drivers/queue/Drivers/queue/Inc" -I"/Users/vaibhavgopal/Projects/maxwell/firmware/STM32 Drivers/AT24C04C/Drivers/AT24C04C/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/as5048a.cyclo ./Core/Src/as5048a.d ./Core/Src/as5048a.o ./Core/Src/as5048a.su ./Core/Src/at24c04c.cyclo ./Core/Src/at24c04c.d ./Core/Src/at24c04c.o ./Core/Src/at24c04c.su ./Core/Src/can.cyclo ./Core/Src/can.d ./Core/Src/can.o ./Core/Src/can.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/enc_dec_utils.cyclo ./Core/Src/enc_dec_utils.d ./Core/Src/enc_dec_utils.o ./Core/Src/enc_dec_utils.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/queue.cyclo ./Core/Src/queue.d ./Core/Src/queue.o ./Core/Src/queue.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/tmc_2590.cyclo ./Core/Src/tmc_2590.d ./Core/Src/tmc_2590.o ./Core/Src/tmc_2590.su

.PHONY: clean-Core-2f-Src

