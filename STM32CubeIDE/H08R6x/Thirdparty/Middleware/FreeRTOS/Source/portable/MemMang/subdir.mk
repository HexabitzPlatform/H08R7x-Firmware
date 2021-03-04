################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/Thirdparty/Middleware/FreeRTOS/Source/portable/MemMang/heap_4.c 

OBJS += \
./Thirdparty/Middleware/FreeRTOS/Source/portable/MemMang/heap_4.o 

C_DEPS += \
./Thirdparty/Middleware/FreeRTOS/Source/portable/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
Thirdparty/Middleware/FreeRTOS/Source/portable/MemMang/heap_4.o: D:/Hexabitz/for\ Release/Modules\ firmware/H08R6x/Thirdparty/Middleware/FreeRTOS/Source/portable/MemMang/heap_4.c Thirdparty/Middleware/FreeRTOS/Source/portable/MemMang/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_I2C_2V8 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DP08R6 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../BOS -I../../User -I../../H08R6 -I../../Thirdparty/VL53L0X_1.0.2/Api/core/inc -I../../Thirdparty/VL53L0X_1.0.2/Api/platform/inc -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/FreeRTOS/Source/portable/MemMang/heap_4.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

