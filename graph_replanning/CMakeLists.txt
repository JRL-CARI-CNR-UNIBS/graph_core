cmake_minimum_required(VERSION 3.0.2)
project(graph_replanning)
add_compile_options(-std=c++14 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
graph_core
roscpp
object_loader_msgs
moveit_planning_helper
geometry_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES graph_replanning
  CATKIN_DEPENDS  graph_core roscpp object_loader_msgs moveit_planning_helper geometry_msgs
  DEPENDS
  )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
add_library(${PROJECT_NAME}
src/${PROJECT_NAME}/trajectory.cpp
src/${PROJECT_NAME}/replanner.cpp
src/${PROJECT_NAME}/replanner_manager.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(       test_replanner src/test/test_replanner.cpp)
add_dependencies(     test_replanner ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_replanner
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_executable(       test_online_replanner src/test/test_online_replanner.cpp)
add_dependencies(     test_online_replanner ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_online_replanner
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(       test_replanner_manager src/test/test_replanner_manager.cpp)
add_dependencies(     test_replanner_manager ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_replanner_manager
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
