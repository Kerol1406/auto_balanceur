################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/INT_Sever/intsever.c 

OBJS += \
./BSP/INT_Sever/intsever.o 

C_DEPS += \
./BSP/INT_Sever/intsever.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/INT_Sever/%.o BSP/INT_Sever/%.su BSP/INT_Sever/%.cyclo: ../BSP/INT_Sever/%.c BSP/INT_Sever/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER -c -I../USER -I../Controllers -I../APP -I../APP/filter -I../APP/KF -I../BSP -I../BSP/Battery -I../BSP/Beep -I../BSP/Delay -I../BSP/Enconder -I../BSP/IIC_Software -I../BSP/INT_Sever -I../BSP/Key -I../BSP/LED -I../BSP/Motor -I../BSP/MPU6050 -I../BSP/MPU6050/DMP -I../BSP/OLED -I../BSP/Timer -I../BSP/Usart1 -I../CMSIS -I"../STM32F10x_StdPeriph_Driver/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-BSP-2f-INT_Sever

clean-BSP-2f-INT_Sever:
	-$(RM) ./BSP/INT_Sever/intsever.cyclo ./BSP/INT_Sever/intsever.d ./BSP/INT_Sever/intsever.o ./BSP/INT_Sever/intsever.su

.PHONY: clean-BSP-2f-INT_Sever

