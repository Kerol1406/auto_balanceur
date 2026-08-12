################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USER/main.c \
../USER/stm32f10x_it.c \
../USER/syscalls.c \
../USER/sysmem.c 

OBJS += \
./USER/main.o \
./USER/stm32f10x_it.o \
./USER/syscalls.o \
./USER/sysmem.o 

C_DEPS += \
./USER/main.d \
./USER/stm32f10x_it.d \
./USER/syscalls.d \
./USER/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
USER/%.o USER/%.su USER/%.cyclo: ../USER/%.c USER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RCTx -DSTM32 -DSTM32F1 -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER -c -I../USER -I../Controllers -I../APP -I../APP/filter -I../APP/KF -I../BSP -I../BSP/Battery -I../BSP/Beep -I../BSP/Delay -I../BSP/Enconder -I../BSP/IIC_Software -I../BSP/INT_Sever -I../BSP/Key -I../BSP/LED -I../BSP/Motor -I../BSP/MPU6050 -I../BSP/MPU6050/DMP -I../BSP/OLED -I../BSP/Timer -I../BSP/Usart1 -I../CMSIS -I"../STM32F10x_StdPeriph_Driver/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-USER

clean-USER:
	-$(RM) ./USER/main.cyclo ./USER/main.d ./USER/main.o ./USER/main.su ./USER/stm32f10x_it.cyclo ./USER/stm32f10x_it.d ./USER/stm32f10x_it.o ./USER/stm32f10x_it.su ./USER/syscalls.cyclo ./USER/syscalls.d ./USER/syscalls.o ./USER/syscalls.su ./USER/sysmem.cyclo ./USER/sysmem.d ./USER/sysmem.o ./USER/sysmem.su

.PHONY: clean-USER

