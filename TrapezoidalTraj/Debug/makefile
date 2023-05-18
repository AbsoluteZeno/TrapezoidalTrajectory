################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/DSP/Source/WindowFunctions/subdir.mk
-include Drivers/DSP/Source/TransformFunctions/subdir.mk
-include Drivers/DSP/Source/SupportFunctions/subdir.mk
-include Drivers/DSP/Source/StatisticsFunctions/subdir.mk
-include Drivers/DSP/Source/SVMFunctions/subdir.mk
-include Drivers/DSP/Source/QuaternionMathFunctions/subdir.mk
-include Drivers/DSP/Source/MatrixFunctions/subdir.mk
-include Drivers/DSP/Source/InterpolationFunctions/subdir.mk
-include Drivers/DSP/Source/FilteringFunctions/subdir.mk
-include Drivers/DSP/Source/FastMathFunctions/subdir.mk
-include Drivers/DSP/Source/DistanceFunctions/subdir.mk
-include Drivers/DSP/Source/ControllerFunctions/subdir.mk
-include Drivers/DSP/Source/ComplexMathFunctions/subdir.mk
-include Drivers/DSP/Source/CommonTables/subdir.mk
-include Drivers/DSP/Source/BayesFunctions/subdir.mk
-include Drivers/DSP/Source/BasicMathFunctions/subdir.mk
-include Drivers/DSP/ComputeLibrary/Source/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := TrapezoidalTraj
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
TrapezoidalTraj.elf \

MAP_FILES += \
TrapezoidalTraj.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
TrapezoidalTraj.list \


# All Target
all: main-build

# Main-build Target
main-build: TrapezoidalTraj.elf secondary-outputs

# Tool invocations
TrapezoidalTraj.elf TrapezoidalTraj.map: $(OBJS) $(USER_OBJS) C:\Extension\Work\FIBO\2nd_Year\2nd_Semester\FRA262\STM32\TrapezoidalTrajectory\TrapezoidalTraj\STM32F411RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "TrapezoidalTraj.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Extension\Work\FIBO\2nd_Year\2nd_Semester\FRA262\STM32\TrapezoidalTrajectory\TrapezoidalTraj\STM32F411RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="TrapezoidalTraj.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

TrapezoidalTraj.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "TrapezoidalTraj.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) TrapezoidalTraj.elf TrapezoidalTraj.list TrapezoidalTraj.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
