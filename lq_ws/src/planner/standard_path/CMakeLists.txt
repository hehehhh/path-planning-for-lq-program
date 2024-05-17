cmake_minimum_required(VERSION 3.0.2)
project(standard_path) #包名称，会被 ${PROJECT_NAME} 的方式调用
SET(CMAKE_BUILD_TYPE Debug)

#设置构建所需要的软件包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  path_search
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES standard_path
  CATKIN_DEPENDS roscpp std_msgs rospy path_search
  DEPENDS system_lib

)
# 添加头文件路径，当前程序包的头文件路径位于其他文件路径之前
include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# 声明 C++ 库
add_library(standard_path src/standard_path.cpp)
add_executable(standard_path_test src/test.cpp src/standard_path.cpp)

target_link_libraries(standard_path
  ${catkin_LIBRARIES}
)

target_link_libraries(standard_path_test
  ${catkin_LIBRARIES}
)

