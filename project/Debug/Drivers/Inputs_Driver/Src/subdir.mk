################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Inputs_Driver/Src/Inputs.c 

OBJS += \
./Drivers/Inputs_Driver/Src/Inputs.o 

C_DEPS += \
./Drivers/Inputs_Driver/Src/Inputs.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Inputs_Driver/Src/%.o Drivers/Inputs_Driver/Src/%.su Drivers/Inputs_Driver/Src/%.cyclo: ../Drivers/Inputs_Driver/Src/%.c Drivers/Inputs_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L471xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/LS032B7DD02_Driver/Inc -I../Drivers/Inputs_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Inputs_Driver-2f-Src

clean-Drivers-2f-Inputs_Driver-2f-Src:
	-$(RM) ./Drivers/Inputs_Driver/Src/Inputs.cyclo ./Drivers/Inputs_Driver/Src/Inputs.d ./Drivers/Inputs_Driver/Src/Inputs.o ./Drivers/Inputs_Driver/Src/Inputs.su

.PHONY: clean-Drivers-2f-Inputs_Driver-2f-Src

