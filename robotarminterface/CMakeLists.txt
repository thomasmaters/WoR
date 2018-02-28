cmake_minimum_required(VERSION 2.8.3)
project(robotarminterface)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


add_message_files(
  FILES
  moveServo.msg
  moveSingleServoMsg.msg
  moveMultipleServosMsg.msg
  moveServoDefinedMsg.msg
  emergencyStopMsg.msg
)

add_service_files(
  FILES
  interfaceStateSrv.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   robotarminterface
 )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable(robotarm_interface_node 
   src/main.cpp 
   src/MotionControl.cpp
   src/SerialControl.cpp
   src/RosCommunication.cpp
   src/Servo.hpp
   src/SmartQueue.hpp
) 

add_executable(robotarm_interface_demo 
   src/demo.cpp
) 

add_dependencies(robotarm_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robotarm_interface_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robotarm_interface_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

target_link_libraries(robotarm_interface_demo
   ${catkin_LIBRARIES}
)

