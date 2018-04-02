cmake_minimum_required(VERSION 2.8.3)
project(robotvision)

find_package( OpenCV 2 REQUIRED)
find_package( Threads REQUIRED)
find_package( Doxygen)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  sensor_msgs
  visualization_msgs
  geometry_msgs
  robotarminterface
  robotapplication
)

catkin_package(
 CATKIN_DEPENDS 
 message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(robot_vision_node 
src/main.cpp 
src/EnumOperators.hpp 
src/ImageDisplayer.hpp 
src/InputHandler.hpp 
src/ColorFilter.cpp 
src/ImageFilter.cpp 
src/ApplicationManager.cpp 
src/ShapeFilter.cpp 
src/RosCommunication.cpp )

add_dependencies(robot_vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robot_vision_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
