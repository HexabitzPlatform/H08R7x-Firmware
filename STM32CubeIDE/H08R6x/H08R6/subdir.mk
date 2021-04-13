################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/startup_stm32f091xc.s 

C_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6_dma.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6_gpio.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6_i2c.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6_it.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6_rtc.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6_timers.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6_uart.c 

OBJS += \
./H08R6/H08R6.o \
./H08R6/H08R6_dma.o \
./H08R6/H08R6_gpio.o \
./H08R6/H08R6_i2c.o \
./H08R6/H08R6_it.o \
./H08R6/H08R6_rtc.o \
./H08R6/H08R6_timers.o \
./H08R6/H08R6_uart.o \
./H08R6/startup_stm32f091xc.o 

S_DEPS += \
./H08R6/startup_stm32f091xc.d 

C_DEPS += \
./H08R6/H08R6.d \
./H08R6/H08R6_dma.d \
./H08R6/H08R6_gpio.d \
./H08R6/H08R6_i2c.d \
./H08R6/H08R6_it.d \
./H08R6/H08R6_rtc.d \
./H08R6/H08R6_timers.d \
./H08R6/H08R6_uart.d 


# Each subdirectory must supply rules for building sources it contributes
H08R6/H08R6.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"H08R6/H08R6.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H08R6/H08R6_dma.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6_dma.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"H08R6/H08R6_dma.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H08R6/H08R6_gpio.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6_gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"H08R6/H08R6_gpio.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H08R6/H08R6_i2c.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6_i2c.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"H08R6/H08R6_i2c.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H08R6/H08R6_it.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"H08R6/H08R6_it.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H08R6/H08R6_rtc.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6_rtc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"H08R6/H08R6_rtc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H08R6/H08R6_timers.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6_timers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"H08R6/H08R6_timers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H08R6/H08R6_uart.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/H08R6_uart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"H08R6/H08R6_uart.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
H08R6/startup_stm32f091xc.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/H08R6/startup_stm32f091xc.s
	arm-none-eabi-gcc -mcpu=cortex-m0 -g -c -x assembler-with-cpp -MMD -MP -MF"H08R6/startup_stm32f091xc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

