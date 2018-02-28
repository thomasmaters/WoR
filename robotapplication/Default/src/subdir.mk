################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/AStar.cpp \
../src/Graph.cpp \
../src/RosCommunication.cpp \
../src/motion_control_node.cpp \
../src/tui_node.cpp 

OBJS += \
./src/AStar.o \
./src/Graph.o \
./src/RosCommunication.o \
./src/motion_control_node.o \
./src/tui_node.o 

CPP_DEPS += \
./src/AStar.d \
./src/Graph.d \
./src/RosCommunication.d \
./src/motion_control_node.d \
./src/tui_node.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/kinetic/include -I/home/thomas/catkin_ws/devel/include/ -I/home/thomas/Documents/School/world/workspace/robotarminterface/devel/include -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


