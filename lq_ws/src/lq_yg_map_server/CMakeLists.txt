cmake_minimum_required(VERSION 3.0.2)
project(lq_yg_map_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  cv_bridge
  message_generation
  tf
)
add_service_files(
  FILES
  ClearSubMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES lq_yg_map_server
 CATKIN_DEPENDS roscpp std_msgs cv_bridge
 DEPENDS system_lib
)

include_directories(
  include
  include/lq_yg_map_server
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_server_node src/map_server_node.cpp src/map_server.cpp)


add_dependencies(map_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(map_server_node
  ${catkin_LIBRARIES}
)

