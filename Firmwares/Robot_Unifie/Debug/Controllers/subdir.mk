################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Controllers/controller_lqr.c \
../Controllers/controller_pid.c \
../Controllers/sac_controller.c 

OBJS += \
./Controllers/controller_lqr.o \
./Controllers/controller_pid.o \
./Controllers/sac_controller.o 

C_DEPS += \
./Controllers/controller_lqr.d \
./Controllers/controller_pid.d \
./Controllers/sac_controller.d 


# Each subdirectory must supply rules for building sources it contributes
Controllers/%.o Controllers/%.su Controllers/%.cyclo: ../Controllers/%.c Controllers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER -c -I../USER -I../Controllers -I../APP -I../APP/filter -I../APP/KF -I../BSP -I../BSP/Battery -I../BSP/Beep -I../BSP/Delay -I../BSP/Enconder -I../BSP/IIC_Software -I../BSP/INT_Sever -I../BSP/Key -I../BSP/LED -I../BSP/Motor -I../BSP/MPU6050 -I../BSP/MPU6050/DMP -I../BSP/OLED -I../BSP/Timer -I../BSP/Usart1 -I../CMSIS -I"../STM32F10x_StdPeriph_Driver/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Controllers

clean-Controllers:
	-$(RM) ./Controllers/controller_lqr.cyclo ./Controllers/controller_lqr.d ./Controllers/controller_lqr.o ./Controllers/controller_lqr.su ./Controllers/controller_pid.cyclo ./Controllers/controller_pid.d ./Controllers/controller_pid.o ./Controllers/controller_pid.su ./Controllers/sac_controller.cyclo ./Controllers/sac_controller.d ./Controllers/sac_controller.o ./Controllers/sac_controller.su

.PHONY: clean-Controllers

