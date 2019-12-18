cmake_minimum_required(VERSION 2.8.3)
project(graph_core)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rosdyn_core
  )
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES graph_core
  CATKIN_DEPENDS actionlib moveit_core moveit_ros_planning moveit_ros_planning_interface roscpp rosdyn_core
  DEPENDS
  )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/utils.cpp
  src/${PROJECT_NAME}/node.cpp
  src/${PROJECT_NAME}/connection.cpp
  src/${PROJECT_NAME}/path.cpp
  src/${PROJECT_NAME}/graph.cpp
  src/${PROJECT_NAME}/solver.cpp
  src/${PROJECT_NAME}/problem.cpp
  )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(PROGRAMS
  scripts/my_python_script
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

