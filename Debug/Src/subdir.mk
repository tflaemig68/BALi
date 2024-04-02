################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/i2cDeivces.c \
../Src/main.c \
../Src/system_stm32f4xx.c \
../Src/xyzScope.c 

OBJS += \
./Src/i2cDeivces.o \
./Src/main.o \
./Src/system_stm32f4xx.o \
./Src/xyzScope.o 

C_DEPS += \
./Src/i2cDeivces.d \
./Src/main.d \
./Src/system_stm32f4xx.d \
./Src/xyzScope.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F401RETx -DDEBUG -c -I"C:/GITHUB/STM32/EDS/MCAL" -I"C:/GITHUB/STM32/EDS/CMSIS/Include" -I"C:/GITHUB/STM32/EDS/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/GITHUB/STM32/EDS/MCAL/Inc" -I"C:/GITHUB/STM32/EDS/MCAL/Inc/mcalTimer" -I"C:/GITHUB/STM32/EDS/Balancer" -I"C:/GITHUB/STM32/EDS/Balancer/Inc" -I"C:/GITHUB/STM32/EDS/BALi/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/i2cDeivces.cyclo ./Src/i2cDeivces.d ./Src/i2cDeivces.o ./Src/i2cDeivces.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/system_stm32f4xx.cyclo ./Src/system_stm32f4xx.d ./Src/system_stm32f4xx.o ./Src/system_stm32f4xx.su ./Src/xyzScope.cyclo ./Src/xyzScope.d ./Src/xyzScope.o ./Src/xyzScope.su

.PHONY: clean-Src

