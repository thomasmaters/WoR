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
  pick_and_place.msg
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
 EXPORTED_TARGETS
 robotapplication_generate_messages_cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motion_control_node 
src/motion_control_node.cpp 
src/motion_control_controller.cpp 
src/motion_control_interface.cpp 
src/InverseKinematics.h )


add_dependencies(motion_control_node robotapplication_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages)

target_link_libraries(motion_control_node
  ${catkin_LIBRARIES}
)
