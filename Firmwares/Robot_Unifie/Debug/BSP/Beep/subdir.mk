################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/Beep/bsp_beep.c 

OBJS += \
./BSP/Beep/bsp_beep.o 

C_DEPS += \
./BSP/Beep/bsp_beep.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/Beep/%.o BSP/Beep/%.su BSP/Beep/%.cyclo: ../BSP/Beep/%.c BSP/Beep/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER -c -I../USER -I../Controllers -I../APP -I../APP/filter -I../APP/KF -I../BSP -I../BSP/Battery -I../BSP/Beep -I../BSP/Delay -I../BSP/Enconder -I../BSP/IIC_Software -I../BSP/INT_Sever -I../BSP/Key -I../BSP/LED -I../BSP/Motor -I../BSP/MPU6050 -I../BSP/MPU6050/DMP -I../BSP/OLED -I../BSP/Timer -I../BSP/Usart1 -I../CMSIS -I"../STM32F10x_StdPeriph_Driver/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-BSP-2f-Beep

clean-BSP-2f-Beep:
	-$(RM) ./BSP/Beep/bsp_beep.cyclo ./BSP/Beep/bsp_beep.d ./BSP/Beep/bsp_beep.o ./BSP/Beep/bsp_beep.su

.PHONY: clean-BSP-2f-Beep

