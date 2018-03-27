cmake_minimum_required(VERSION 2.8.3)
project(robotsimulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  std_msgs
  sensor_msgs
  rospy
  message_generation
  interactive_markers
  tf
)

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  ssc32u_command.msg
  ssc32u_feedback.msg
  cup_data.msg
)

generate_messages(
  DEPENDENCIES
  visualization_msgs
  std_msgs
  sensor_msgs
  robotsimulation
)

catkin_package(
 CATKIN_DEPENDS 
 message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(demo_node src/demo_node.cpp)
add_executable(ssc32u_controller_node 
	src/ssc32u_parser_node.cpp 
	src/ssc32u_parser_interface.cpp
	src/ssc32u_parser_to_jointstate.cpp)
add_executable(virtual_cup_node 
	src/virtual_cup_node.cpp
	src/virtual_cup.cpp
	src/virtual_cup_interface.cpp)


add_dependencies(demo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ssc32u_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(virtual_cup_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(demo_node
  ${catkin_LIBRARIES}
)

target_link_libraries(ssc32u_controller_node
  ${catkin_LIBRARIES}
)

target_link_libraries(virtual_cup_node
  ${catkin_LIBRARIES}
)