################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/robotalgorithmfork/src/Calibration.cpp \
../src/robotalgorithmfork/src/Detector.cpp \
../src/robotalgorithmfork/src/Reader.cpp \
../src/robotalgorithmfork/src/Specification.cpp \
../src/robotalgorithmfork/src/main.cpp 

OBJS += \
./src/robotalgorithmfork/src/Calibration.o \
./src/robotalgorithmfork/src/Detector.o \
./src/robotalgorithmfork/src/Reader.o \
./src/robotalgorithmfork/src/Specification.o \
./src/robotalgorithmfork/src/main.o 

CPP_DEPS += \
./src/robotalgorithmfork/src/Calibration.d \
./src/robotalgorithmfork/src/Detector.d \
./src/robotalgorithmfork/src/Reader.d \
./src/robotalgorithmfork/src/Specification.d \
./src/robotalgorithmfork/src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/robotalgorithmfork/src/%.o: ../src/robotalgorithmfork/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/kinetic/include -I/home/thomas/catkin_ws/devel/include/ -I/home/thomas/Documents/School/world/workspace/robotarminterface/devel/include -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


