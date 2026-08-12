################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/OLED/i2c_hw/bsp_oled.c \
../BSP/OLED/i2c_hw/bsp_oled_i2c.c 

OBJS += \
./BSP/OLED/i2c_hw/bsp_oled.o \
./BSP/OLED/i2c_hw/bsp_oled_i2c.o 

C_DEPS += \
./BSP/OLED/i2c_hw/bsp_oled.d \
./BSP/OLED/i2c_hw/bsp_oled_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/OLED/i2c_hw/%.o BSP/OLED/i2c_hw/%.su BSP/OLED/i2c_hw/%.cyclo: ../BSP/OLED/i2c_hw/%.c BSP/OLED/i2c_hw/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER -c -I../USER -I../Controllers -I../APP -I../APP/filter -I../APP/KF -I../BSP -I../BSP/Battery -I../BSP/Beep -I../BSP/Delay -I../BSP/Enconder -I../BSP/IIC_Software -I../BSP/INT_Sever -I../BSP/Key -I../BSP/LED -I../BSP/Motor -I../BSP/MPU6050 -I../BSP/MPU6050/DMP -I../BSP/OLED -I../BSP/Timer -I../BSP/Usart1 -I../CMSIS -I"../STM32F10x_StdPeriph_Driver/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-BSP-2f-OLED-2f-i2c_hw

clean-BSP-2f-OLED-2f-i2c_hw:
	-$(RM) ./BSP/OLED/i2c_hw/bsp_oled.cyclo ./BSP/OLED/i2c_hw/bsp_oled.d ./BSP/OLED/i2c_hw/bsp_oled.o ./BSP/OLED/i2c_hw/bsp_oled.su ./BSP/OLED/i2c_hw/bsp_oled_i2c.cyclo ./BSP/OLED/i2c_hw/bsp_oled_i2c.d ./BSP/OLED/i2c_hw/bsp_oled_i2c.o ./BSP/OLED/i2c_hw/bsp_oled_i2c.su

.PHONY: clean-BSP-2f-OLED-2f-i2c_hw

