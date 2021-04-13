################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_CLI.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_CLIcommands.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_dma.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_eeprom.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_freertos.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_inputs.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_messaging.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_msgparser.c 

OBJS += \
./BOS/BOS.o \
./BOS/BOS_CLI.o \
./BOS/BOS_CLIcommands.o \
./BOS/BOS_dma.o \
./BOS/BOS_eeprom.o \
./BOS/BOS_freertos.o \
./BOS/BOS_inputs.o \
./BOS/BOS_messaging.o \
./BOS/BOS_msgparser.o 

C_DEPS += \
./BOS/BOS.d \
./BOS/BOS_CLI.d \
./BOS/BOS_CLIcommands.d \
./BOS/BOS_dma.d \
./BOS/BOS_eeprom.d \
./BOS/BOS_freertos.d \
./BOS/BOS_inputs.d \
./BOS/BOS_messaging.d \
./BOS/BOS_msgparser.d 


# Each subdirectory must supply rules for building sources it contributes
BOS/BOS.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"BOS/BOS.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_CLI.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_CLI.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"BOS/BOS_CLI.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_CLIcommands.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_CLIcommands.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"BOS/BOS_CLIcommands.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_dma.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_dma.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"BOS/BOS_dma.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_eeprom.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_eeprom.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"BOS/BOS_eeprom.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_freertos.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_freertos.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"BOS/BOS_freertos.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_inputs.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_inputs.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"BOS/BOS_inputs.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_messaging.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_messaging.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"BOS/BOS_messaging.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
BOS/BOS_msgparser.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/BOS/BOS_msgparser.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -Og -ffunction-sections -fdata-sections -fno-strict-aliasing -fstack-usage -MMD -MP -MF"BOS/BOS_msgparser.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

