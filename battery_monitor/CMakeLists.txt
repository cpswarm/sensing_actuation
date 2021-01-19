cmake_minimum_required(VERSION 3.0)
project(battery_monitor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS ./src /usr/include
  CATKIN_DEPENDS std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/battery_monitor.cpp)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

