cmake_minimum_required(VERSION 2.8.3)
project(dgaco_planner)

add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  roscpp
  net_path_core
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dgaco_planner
  CATKIN_DEPENDS moveit_core moveit_ros_planning roscpp net_path_core
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
   src/${PROJECT_NAME}/${PROJECT_NAME}_manager.cpp
 )


 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )


 install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

