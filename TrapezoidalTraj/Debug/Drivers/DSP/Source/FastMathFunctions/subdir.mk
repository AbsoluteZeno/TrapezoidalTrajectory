################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/DSP/Source/FastMathFunctions/FastMathFunctionsF16.c \
../Drivers/DSP/Source/FastMathFunctions/arm_atan2_f16.c \
../Drivers/DSP/Source/FastMathFunctions/arm_atan2_f32.c \
../Drivers/DSP/Source/FastMathFunctions/arm_atan2_q15.c \
../Drivers/DSP/Source/FastMathFunctions/arm_atan2_q31.c \
../Drivers/DSP/Source/FastMathFunctions/arm_cos_f32.c \
../Drivers/DSP/Source/FastMathFunctions/arm_cos_q15.c \
../Drivers/DSP/Source/FastMathFunctions/arm_cos_q31.c \
../Drivers/DSP/Source/FastMathFunctions/arm_divide_q15.c \
../Drivers/DSP/Source/FastMathFunctions/arm_divide_q31.c \
../Drivers/DSP/Source/FastMathFunctions/arm_sin_f32.c \
../Drivers/DSP/Source/FastMathFunctions/arm_sin_q15.c \
../Drivers/DSP/Source/FastMathFunctions/arm_sin_q31.c \
../Drivers/DSP/Source/FastMathFunctions/arm_sqrt_q15.c \
../Drivers/DSP/Source/FastMathFunctions/arm_sqrt_q31.c \
../Drivers/DSP/Source/FastMathFunctions/arm_vexp_f16.c \
../Drivers/DSP/Source/FastMathFunctions/arm_vexp_f32.c \
../Drivers/DSP/Source/FastMathFunctions/arm_vexp_f64.c \
../Drivers/DSP/Source/FastMathFunctions/arm_vinverse_f16.c \
../Drivers/DSP/Source/FastMathFunctions/arm_vlog_f16.c \
../Drivers/DSP/Source/FastMathFunctions/arm_vlog_f32.c \
../Drivers/DSP/Source/FastMathFunctions/arm_vlog_f64.c \
../Drivers/DSP/Source/FastMathFunctions/arm_vlog_q15.c \
../Drivers/DSP/Source/FastMathFunctions/arm_vlog_q31.c 

OBJS += \
./Drivers/DSP/Source/FastMathFunctions/FastMathFunctionsF16.o \
./Drivers/DSP/Source/FastMathFunctions/arm_atan2_f16.o \
./Drivers/DSP/Source/FastMathFunctions/arm_atan2_f32.o \
./Drivers/DSP/Source/FastMathFunctions/arm_atan2_q15.o \
./Drivers/DSP/Source/FastMathFunctions/arm_atan2_q31.o \
./Drivers/DSP/Source/FastMathFunctions/arm_cos_f32.o \
./Drivers/DSP/Source/FastMathFunctions/arm_cos_q15.o \
./Drivers/DSP/Source/FastMathFunctions/arm_cos_q31.o \
./Drivers/DSP/Source/FastMathFunctions/arm_divide_q15.o \
./Drivers/DSP/Source/FastMathFunctions/arm_divide_q31.o \
./Drivers/DSP/Source/FastMathFunctions/arm_sin_f32.o \
./Drivers/DSP/Source/FastMathFunctions/arm_sin_q15.o \
./Drivers/DSP/Source/FastMathFunctions/arm_sin_q31.o \
./Drivers/DSP/Source/FastMathFunctions/arm_sqrt_q15.o \
./Drivers/DSP/Source/FastMathFunctions/arm_sqrt_q31.o \
./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f16.o \
./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f32.o \
./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f64.o \
./Drivers/DSP/Source/FastMathFunctions/arm_vinverse_f16.o \
./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f16.o \
./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f32.o \
./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f64.o \
./Drivers/DSP/Source/FastMathFunctions/arm_vlog_q15.o \
./Drivers/DSP/Source/FastMathFunctions/arm_vlog_q31.o 

C_DEPS += \
./Drivers/DSP/Source/FastMathFunctions/FastMathFunctionsF16.d \
./Drivers/DSP/Source/FastMathFunctions/arm_atan2_f16.d \
./Drivers/DSP/Source/FastMathFunctions/arm_atan2_f32.d \
./Drivers/DSP/Source/FastMathFunctions/arm_atan2_q15.d \
./Drivers/DSP/Source/FastMathFunctions/arm_atan2_q31.d \
./Drivers/DSP/Source/FastMathFunctions/arm_cos_f32.d \
./Drivers/DSP/Source/FastMathFunctions/arm_cos_q15.d \
./Drivers/DSP/Source/FastMathFunctions/arm_cos_q31.d \
./Drivers/DSP/Source/FastMathFunctions/arm_divide_q15.d \
./Drivers/DSP/Source/FastMathFunctions/arm_divide_q31.d \
./Drivers/DSP/Source/FastMathFunctions/arm_sin_f32.d \
./Drivers/DSP/Source/FastMathFunctions/arm_sin_q15.d \
./Drivers/DSP/Source/FastMathFunctions/arm_sin_q31.d \
./Drivers/DSP/Source/FastMathFunctions/arm_sqrt_q15.d \
./Drivers/DSP/Source/FastMathFunctions/arm_sqrt_q31.d \
./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f16.d \
./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f32.d \
./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f64.d \
./Drivers/DSP/Source/FastMathFunctions/arm_vinverse_f16.d \
./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f16.d \
./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f32.d \
./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f64.d \
./Drivers/DSP/Source/FastMathFunctions/arm_vlog_q15.d \
./Drivers/DSP/Source/FastMathFunctions/arm_vlog_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/DSP/Source/FastMathFunctions/%.o Drivers/DSP/Source/FastMathFunctions/%.su: ../Drivers/DSP/Source/FastMathFunctions/%.c Drivers/DSP/Source/FastMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Extension/Work/FIBO/2nd_Year/2nd_Semester/FRA262/STM32/TrapezoidalTrajectory/TrapezoidalTraj/Drivers/DSP/ComputeLibrary/Include" -I"C:/Extension/Work/FIBO/2nd_Year/2nd_Semester/FRA262/STM32/TrapezoidalTrajectory/TrapezoidalTraj/Drivers/DSP/Include" -I"C:/Extension/Work/FIBO/2nd_Year/2nd_Semester/FRA262/STM32/TrapezoidalTrajectory/TrapezoidalTraj/Drivers/DSP/PrivateInclude" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-DSP-2f-Source-2f-FastMathFunctions

clean-Drivers-2f-DSP-2f-Source-2f-FastMathFunctions:
	-$(RM) ./Drivers/DSP/Source/FastMathFunctions/FastMathFunctionsF16.d ./Drivers/DSP/Source/FastMathFunctions/FastMathFunctionsF16.o ./Drivers/DSP/Source/FastMathFunctions/FastMathFunctionsF16.su ./Drivers/DSP/Source/FastMathFunctions/arm_atan2_f16.d ./Drivers/DSP/Source/FastMathFunctions/arm_atan2_f16.o ./Drivers/DSP/Source/FastMathFunctions/arm_atan2_f16.su ./Drivers/DSP/Source/FastMathFunctions/arm_atan2_f32.d ./Drivers/DSP/Source/FastMathFunctions/arm_atan2_f32.o ./Drivers/DSP/Source/FastMathFunctions/arm_atan2_f32.su ./Drivers/DSP/Source/FastMathFunctions/arm_atan2_q15.d ./Drivers/DSP/Source/FastMathFunctions/arm_atan2_q15.o ./Drivers/DSP/Source/FastMathFunctions/arm_atan2_q15.su ./Drivers/DSP/Source/FastMathFunctions/arm_atan2_q31.d ./Drivers/DSP/Source/FastMathFunctions/arm_atan2_q31.o ./Drivers/DSP/Source/FastMathFunctions/arm_atan2_q31.su ./Drivers/DSP/Source/FastMathFunctions/arm_cos_f32.d ./Drivers/DSP/Source/FastMathFunctions/arm_cos_f32.o ./Drivers/DSP/Source/FastMathFunctions/arm_cos_f32.su ./Drivers/DSP/Source/FastMathFunctions/arm_cos_q15.d ./Drivers/DSP/Source/FastMathFunctions/arm_cos_q15.o ./Drivers/DSP/Source/FastMathFunctions/arm_cos_q15.su ./Drivers/DSP/Source/FastMathFunctions/arm_cos_q31.d ./Drivers/DSP/Source/FastMathFunctions/arm_cos_q31.o ./Drivers/DSP/Source/FastMathFunctions/arm_cos_q31.su ./Drivers/DSP/Source/FastMathFunctions/arm_divide_q15.d ./Drivers/DSP/Source/FastMathFunctions/arm_divide_q15.o ./Drivers/DSP/Source/FastMathFunctions/arm_divide_q15.su ./Drivers/DSP/Source/FastMathFunctions/arm_divide_q31.d ./Drivers/DSP/Source/FastMathFunctions/arm_divide_q31.o ./Drivers/DSP/Source/FastMathFunctions/arm_divide_q31.su ./Drivers/DSP/Source/FastMathFunctions/arm_sin_f32.d ./Drivers/DSP/Source/FastMathFunctions/arm_sin_f32.o ./Drivers/DSP/Source/FastMathFunctions/arm_sin_f32.su ./Drivers/DSP/Source/FastMathFunctions/arm_sin_q15.d ./Drivers/DSP/Source/FastMathFunctions/arm_sin_q15.o ./Drivers/DSP/Source/FastMathFunctions/arm_sin_q15.su ./Drivers/DSP/Source/FastMathFunctions/arm_sin_q31.d ./Drivers/DSP/Source/FastMathFunctions/arm_sin_q31.o ./Drivers/DSP/Source/FastMathFunctions/arm_sin_q31.su ./Drivers/DSP/Source/FastMathFunctions/arm_sqrt_q15.d ./Drivers/DSP/Source/FastMathFunctions/arm_sqrt_q15.o ./Drivers/DSP/Source/FastMathFunctions/arm_sqrt_q15.su ./Drivers/DSP/Source/FastMathFunctions/arm_sqrt_q31.d ./Drivers/DSP/Source/FastMathFunctions/arm_sqrt_q31.o ./Drivers/DSP/Source/FastMathFunctions/arm_sqrt_q31.su ./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f16.d ./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f16.o ./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f16.su ./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f32.d ./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f32.o ./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f32.su ./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f64.d ./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f64.o ./Drivers/DSP/Source/FastMathFunctions/arm_vexp_f64.su ./Drivers/DSP/Source/FastMathFunctions/arm_vinverse_f16.d ./Drivers/DSP/Source/FastMathFunctions/arm_vinverse_f16.o ./Drivers/DSP/Source/FastMathFunctions/arm_vinverse_f16.su ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f16.d ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f16.o ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f16.su ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f32.d ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f32.o ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f32.su ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f64.d ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f64.o ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_f64.su ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_q15.d ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_q15.o ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_q15.su ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_q31.d ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_q31.o ./Drivers/DSP/Source/FastMathFunctions/arm_vlog_q31.su

.PHONY: clean-Drivers-2f-DSP-2f-Source-2f-FastMathFunctions

