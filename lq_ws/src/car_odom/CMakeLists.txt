cmake_minimum_required(VERSION 3.0.2)
project(car_odom)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES car_odom
  CATKIN_DEPENDS roscpp std_msgs

)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(car_odom_node src/car_odom_node.cpp)

target_link_libraries(car_odom_node
  ${catkin_LIBRARIES}
)
