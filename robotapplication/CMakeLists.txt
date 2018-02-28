cmake_minimum_required(VERSION 2.8.3)
project(robotapplication)

find_package( Boost REQUIRED)
find_package( Threads REQUIRED)
find_package (Doxygen)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rosgraph_msgs
  geometry_msgs
  rospy
  message_generation
  robotarminterface
)

add_message_files(
  FILES
  arm_control_msg.msg
  arm_position.msg
  pickup_target.msg
  PickAndPlace.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  robotapplication
)

catkin_package(
 CATKIN_DEPENDS 
 message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motion_control_node 
src/motion_control_node.cpp 
src/RosCommunication.cpp 
src/InverseKinematics.h )


add_dependencies(motion_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(motion_control_node
  ${catkin_LIBRARIES}
)
