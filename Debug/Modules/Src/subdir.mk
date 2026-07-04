################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Modules/Src/ADC_CMSIS.c \
../Modules/Src/Clock_CMSIS.c \
../Modules/Src/DAC_CMSIS.c \
../Modules/Src/DMA_CMSIS.c \
../Modules/Src/Flash_CMSIS.c \
../Modules/Src/GPIO_Interrupt_CMSIS.c \
../Modules/Src/I2C_CMSIS.c \
../Modules/Src/LowPower_CMSIS.c \
../Modules/Src/PWM_CMSIS.c \
../Modules/Src/RTC_CMSIS.c \
../Modules/Src/SPI_CMSIS.c \
../Modules/Src/SYSTICK_CMSIS.c \
../Modules/Src/Timer_CMSIS.c \
../Modules/Src/UART_CMSIS.c \
../Modules/Src/Watchdog_CMSIS.c 

OBJS += \
./Modules/Src/ADC_CMSIS.o \
./Modules/Src/Clock_CMSIS.o \
./Modules/Src/DAC_CMSIS.o \
./Modules/Src/DMA_CMSIS.o \
./Modules/Src/Flash_CMSIS.o \
./Modules/Src/GPIO_Interrupt_CMSIS.o \
./Modules/Src/I2C_CMSIS.o \
./Modules/Src/LowPower_CMSIS.o \
./Modules/Src/PWM_CMSIS.o \
./Modules/Src/RTC_CMSIS.o \
./Modules/Src/SPI_CMSIS.o \
./Modules/Src/SYSTICK_CMSIS.o \
./Modules/Src/Timer_CMSIS.o \
./Modules/Src/UART_CMSIS.o \
./Modules/Src/Watchdog_CMSIS.o 

C_DEPS += \
./Modules/Src/ADC_CMSIS.d \
./Modules/Src/Clock_CMSIS.d \
./Modules/Src/DAC_CMSIS.d \
./Modules/Src/DMA_CMSIS.d \
./Modules/Src/Flash_CMSIS.d \
./Modules/Src/GPIO_Interrupt_CMSIS.d \
./Modules/Src/I2C_CMSIS.d \
./Modules/Src/LowPower_CMSIS.d \
./Modules/Src/PWM_CMSIS.d \
./Modules/Src/RTC_CMSIS.d \
./Modules/Src/SPI_CMSIS.d \
./Modules/Src/SYSTICK_CMSIS.d \
./Modules/Src/Timer_CMSIS.d \
./Modules/Src/UART_CMSIS.d \
./Modules/Src/Watchdog_CMSIS.d 


# Each subdirectory must supply rules for building sources it contributes
Modules/Src/%.o Modules/Src/%.su Modules/Src/%.cyclo: ../Modules/Src/%.c Modules/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Talha/Desktop/Calismalar/StmCalismalar/CMSIS_Practices/Modules/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Modules-2f-Src

clean-Modules-2f-Src:
	-$(RM) ./Modules/Src/ADC_CMSIS.cyclo ./Modules/Src/ADC_CMSIS.d ./Modules/Src/ADC_CMSIS.o ./Modules/Src/ADC_CMSIS.su ./Modules/Src/Clock_CMSIS.cyclo ./Modules/Src/Clock_CMSIS.d ./Modules/Src/Clock_CMSIS.o ./Modules/Src/Clock_CMSIS.su ./Modules/Src/DAC_CMSIS.cyclo ./Modules/Src/DAC_CMSIS.d ./Modules/Src/DAC_CMSIS.o ./Modules/Src/DAC_CMSIS.su ./Modules/Src/DMA_CMSIS.cyclo ./Modules/Src/DMA_CMSIS.d ./Modules/Src/DMA_CMSIS.o ./Modules/Src/DMA_CMSIS.su ./Modules/Src/Flash_CMSIS.cyclo ./Modules/Src/Flash_CMSIS.d ./Modules/Src/Flash_CMSIS.o ./Modules/Src/Flash_CMSIS.su ./Modules/Src/GPIO_Interrupt_CMSIS.cyclo ./Modules/Src/GPIO_Interrupt_CMSIS.d ./Modules/Src/GPIO_Interrupt_CMSIS.o ./Modules/Src/GPIO_Interrupt_CMSIS.su ./Modules/Src/I2C_CMSIS.cyclo ./Modules/Src/I2C_CMSIS.d ./Modules/Src/I2C_CMSIS.o ./Modules/Src/I2C_CMSIS.su ./Modules/Src/LowPower_CMSIS.cyclo ./Modules/Src/LowPower_CMSIS.d ./Modules/Src/LowPower_CMSIS.o ./Modules/Src/LowPower_CMSIS.su ./Modules/Src/PWM_CMSIS.cyclo ./Modules/Src/PWM_CMSIS.d ./Modules/Src/PWM_CMSIS.o ./Modules/Src/PWM_CMSIS.su ./Modules/Src/RTC_CMSIS.cyclo ./Modules/Src/RTC_CMSIS.d ./Modules/Src/RTC_CMSIS.o ./Modules/Src/RTC_CMSIS.su ./Modules/Src/SPI_CMSIS.cyclo ./Modules/Src/SPI_CMSIS.d ./Modules/Src/SPI_CMSIS.o ./Modules/Src/SPI_CMSIS.su ./Modules/Src/SYSTICK_CMSIS.cyclo ./Modules/Src/SYSTICK_CMSIS.d ./Modules/Src/SYSTICK_CMSIS.o ./Modules/Src/SYSTICK_CMSIS.su ./Modules/Src/Timer_CMSIS.cyclo ./Modules/Src/Timer_CMSIS.d ./Modules/Src/Timer_CMSIS.o ./Modules/Src/Timer_CMSIS.su ./Modules/Src/UART_CMSIS.cyclo ./Modules/Src/UART_CMSIS.d ./Modules/Src/UART_CMSIS.o ./Modules/Src/UART_CMSIS.su ./Modules/Src/Watchdog_CMSIS.cyclo ./Modules/Src/Watchdog_CMSIS.d ./Modules/Src/Watchdog_CMSIS.o ./Modules/Src/Watchdog_CMSIS.su

.PHONY: clean-Modules-2f-Src

