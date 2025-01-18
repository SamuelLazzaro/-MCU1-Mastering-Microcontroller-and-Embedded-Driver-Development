################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32nucleo_h563zi_gpio_driver.c 

OBJS += \
./Drivers/Src/stm32nucleo_h563zi_gpio_driver.o 

C_DEPS += \
./Drivers/Src/stm32nucleo_h563zi_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32H563ZITx -DSTM32 -DSTM32H5 -DNUCLEO_H563ZI -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32nucleo_h563zi_gpio_driver.cyclo ./Drivers/Src/stm32nucleo_h563zi_gpio_driver.d ./Drivers/Src/stm32nucleo_h563zi_gpio_driver.o ./Drivers/Src/stm32nucleo_h563zi_gpio_driver.su

.PHONY: clean-Drivers-2f-Src

