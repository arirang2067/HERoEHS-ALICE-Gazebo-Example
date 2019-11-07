cmake_minimum_required(VERSION 2.8.3)
project(alice_1_algorithm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES alice_1_algorithm
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(alice_1_algorithm_node src/alice_1_algorithm_node.cpp)
add_dependencies(alice_1_algorithm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(alice_1_algorithm_node ${catkin_LIBRARIES})
