################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Output.cpp \
../src/PID.cpp \
../src/RobotIO.cpp \
../src/RobotThread.cpp \
../src/Sensor.cpp \
../src/Store.cpp \
../src/main.cpp 

OBJS += \
./src/Output.o \
./src/PID.o \
./src/RobotIO.o \
./src/RobotThread.o \
./src/Sensor.o \
./src/Store.o \
./src/main.o 

CPP_DEPS += \
./src/Output.d \
./src/PID.d \
./src/RobotIO.d \
./src/RobotThread.d \
./src/Sensor.d \
./src/Store.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/gruppe_9/eclipse-workspace/RobotControl/inc" -O0 -g3 -Wall -c -fmessage-length=0 -pthread -std=c++11 -Wl,--no-as-needed -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


