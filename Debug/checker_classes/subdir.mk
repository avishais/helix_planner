################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../checker_classes/Rod_ODE_class.cpp \
../checker_classes/collisionDetection.cpp \
../checker_classes/model.cpp \
../checker_classes/robots_class.cpp 

OBJS += \
./checker_classes/Rod_ODE_class.o \
./checker_classes/collisionDetection.o \
./checker_classes/model.o \
./checker_classes/robots_class.o 

CPP_DEPS += \
./checker_classes/Rod_ODE_class.d \
./checker_classes/collisionDetection.d \
./checker_classes/model.d \
./checker_classes/robots_class.d 


# Each subdirectory must supply rules for building sources it contributes
checker_classes/%.o: ../checker_classes/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


