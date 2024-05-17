cmake_minimum_required(VERSION 3.0.2)
project(path_search) #包名称，会被 ${PROJECT_NAME} 的方式调用

#设置构建所需要的软件包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_search
  CATKIN_DEPENDS roscpp std_msgs

)

# 添加头文件路径，当前程序包的头文件路径位于其他文件路径之前
include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
)

## Declare a C++ library
# 声明 C++ 库
add_library(path_search 
  include/path_search/path_search.h
  src/path_search.cpp

  include/path_search/bspine.h
  src/bspine.cpp

  include/path_search/dubins.h
  src/dubins.cpp
)

add_executable(path_search_test src/text.cpp src/path_search.cpp src/bspine.cpp src/dubins.cpp)

add_executable(bspline_local_planner_node src/bspline_local_planner_node.cpp
              src/bspline_local_planner.cpp src/bspine.cpp)

target_link_libraries(path_search
  ${catkin_LIBRARIES}
)

target_link_libraries(path_search_test
  ${catkin_LIBRARIES}
)

target_link_libraries(bspline_local_planner_node
  ${catkin_LIBRARIES}
)