################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/006_spi_TX_testing.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/006_spi_TX_testing.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/006_spi_TX_testing.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DSTM32H563ZITx -DSTM32 -DSTM32H5 -DNUCLEO_H563ZI -c -I../Inc -I"C:/Users/Samuel.Lazzaro/OneDrive - Carlo Gavazzi Automation SpA/Online Courses/(MCU1) Mastering Microcontroller and Embedded Driver Development/Project/SampleApplication/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/006_spi_TX_testing.cyclo ./Src/006_spi_TX_testing.d ./Src/006_spi_TX_testing.o ./Src/006_spi_TX_testing.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

