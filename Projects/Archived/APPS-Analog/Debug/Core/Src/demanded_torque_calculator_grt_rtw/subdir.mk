################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/demanded_torque_calculator_grt_rtw/demanded_torque_calculator.c 

OBJS += \
./Core/Src/demanded_torque_calculator_grt_rtw/demanded_torque_calculator.o 

C_DEPS += \
./Core/Src/demanded_torque_calculator_grt_rtw/demanded_torque_calculator.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/demanded_torque_calculator_grt_rtw/%.o: ../Core/Src/demanded_torque_calculator_grt_rtw/%.c Core/Src/demanded_torque_calculator_grt_rtw/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../USB_HOST/App -I../Core/Src/demanded_torque_calculator_grt_rtw -I"C:/Program Files/MATLAB/R2020b/simulink/include" -I"C:/Program Files/MATLAB/R2020b/extern/include" -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-demanded_torque_calculator_grt_rtw

clean-Core-2f-Src-2f-demanded_torque_calculator_grt_rtw:
	-$(RM) ./Core/Src/demanded_torque_calculator_grt_rtw/demanded_torque_calculator.d ./Core/Src/demanded_torque_calculator_grt_rtw/demanded_torque_calculator.o

.PHONY: clean-Core-2f-Src-2f-demanded_torque_calculator_grt_rtw

