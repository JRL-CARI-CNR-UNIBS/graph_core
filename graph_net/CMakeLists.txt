cmake_minimum_required(VERSION 3.0.2)
project(graph_net)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  graph_core
  roscpp
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES graph_net
  CATKIN_DEPENDS graph_core roscpp
  DEPENDS
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/graph_net.cpp
# )
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(        test_net tests/test_net.cpp)
add_dependencies(      test_net ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( test_net
  ${catkin_LIBRARIES}
)
