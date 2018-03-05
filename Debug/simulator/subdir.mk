################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../simulator/main.cpp \
../simulator/model.cpp 

OBJS += \
./simulator/main.o \
./simulator/model.o 

CPP_DEPS += \
./simulator/main.d \
./simulator/model.d 


# Each subdirectory must supply rules for building sources it contributes
simulator/%.o: ../simulator/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


