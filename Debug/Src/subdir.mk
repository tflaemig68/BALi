################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/i2cDeivces.c \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/xyzScope.c 

OBJS += \
./Src/i2cDeivces.o \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/xyzScope.o 

C_DEPS += \
./Src/i2cDeivces.d \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/xyzScope.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F401RETx -DDEBUG -c -I"D:/GITHUB/STM32/EDS_2023/CMSIS/Include" -I"D:/GITHUB/STM32/EDS_2023/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/GITHUB/STM32/EDS_2023/MCAL_F40x/Inc" -I"D:/GITHUB/STM32/EDS_2023/MCAL_F40x/Inc/mcalTimer" -I"D:/GITHUB/STM32/EDS_2023/Balancer" -I"D:/GITHUB/STM32/EDS_2023/Balancer/Inc" -I"D:/GITHUB/STM32/EDS_2023/BALi/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/i2cDeivces.cyclo ./Src/i2cDeivces.d ./Src/i2cDeivces.o ./Src/i2cDeivces.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/xyzScope.cyclo ./Src/xyzScope.d ./Src/xyzScope.o ./Src/xyzScope.su

.PHONY: clean-Src

