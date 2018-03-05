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

add_executable(cli_node src/cli_node.cpp)
add_executable(convert_node 
	src/ssc32u_parser_node.cpp 
	src/ssc32u_parser_interface.cpp
	src/ssc32u_parser_to_jointstate.cpp)
add_executable(beker_node src/cup_node.cpp)


add_dependencies(cli_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(convert_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(beker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cli_node
  ${catkin_LIBRARIES}
)

target_link_libraries(convert_node
  ${catkin_LIBRARIES}
)

target_link_libraries(beker_node
  ${catkin_LIBRARIES}
)