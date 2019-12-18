cmake_minimum_required(VERSION 2.8.3)
project(test_occupancy)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  human_probablistic_occupancy
  roscpp
  moveit_core
  moveit_ros_planning_interface
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_occupancy
  CATKIN_DEPENDS human_probablistic_occupancy roscpp moveit_core moveit_ros_planning_interface
#  DEPENDS
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_node src/test_occupancy_node.cpp)
 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

 install(TARGETS ${PROJECT_NAME}_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

