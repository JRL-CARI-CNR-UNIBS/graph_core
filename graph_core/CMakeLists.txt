cmake_minimum_required(VERSION 2.8.3)
project(graph_core)
add_compile_options(-std=c++14 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rosdyn_core
  human_probablistic_occupancy
  moveit_visual_tools
  rviz_visual_tools
  object_loader_msgs
  moveit_planning_helper
  eigen_conversions
  geometry_msgs
  )
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES graph_core
  CATKIN_DEPENDS actionlib moveit_core moveit_ros_planning moveit_ros_planning_interface roscpp rosdyn_core human_probablistic_occupancy moveit_visual_tools rviz_visual_tools object_loader_msgs moveit_planning_helper eigen_conversions geometry_msgs
  DEPENDS
  )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/metrics.cpp
  src/${PROJECT_NAME}/avoidance_metrics.cpp
  src/${PROJECT_NAME}/occupancy_metrics.cpp
  src/${PROJECT_NAME}/speed_metrics.cpp
  src/${PROJECT_NAME}/sampler.cpp
  src/${PROJECT_NAME}/local_informed_sampler.cpp
  src/${PROJECT_NAME}/tube_informed_sampler.cpp
  src/${PROJECT_NAME}/scaled_sampler.cpp
  src/${PROJECT_NAME}/util.cpp
  src/${PROJECT_NAME}/moveit_collision_checker.cpp
  src/${PROJECT_NAME}/narrow_pass_checker.cpp
  src/${PROJECT_NAME}/boxes_checker.cpp
  src/${PROJECT_NAME}/collision_checker.cpp
  src/${PROJECT_NAME}/graph/node.cpp
  src/${PROJECT_NAME}/graph/connection.cpp
  src/${PROJECT_NAME}/graph/tree.cpp
  src/${PROJECT_NAME}/graph/path.cpp
  src/${PROJECT_NAME}/graph/trajectory.cpp
  src/${PROJECT_NAME}/graph/graph_display.cpp
  src/${PROJECT_NAME}/solvers/multigoal.cpp
  src/${PROJECT_NAME}/solvers/tree_solver.cpp
  src/${PROJECT_NAME}/solvers/rrt_connect.cpp
  src/${PROJECT_NAME}/solvers/birrt.cpp
  src/${PROJECT_NAME}/solvers/rrt_star.cpp
  src/${PROJECT_NAME}/solvers/path_solver.cpp
  )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(       test_classes src/test/test_classes.cpp)
add_dependencies(     test_classes ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_classes
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(       test_tree src/test/test_tree.cpp)
add_dependencies(     test_tree ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_tree
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_executable(       test_moveit src/test/test_moveit.cpp)
add_dependencies(     test_moveit ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_moveit
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(       test_tube_sampling src/test/test_tube_sampling.cpp)
add_dependencies(     test_tube_sampling ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_tube_sampling
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_executable(       narrow_pass_test src/test/narrow_pass_test.cpp)
add_dependencies(     narrow_pass_test ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(narrow_pass_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(       test_multigoal src/test/test_multicore.cpp)
add_dependencies(     test_multigoal ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_multigoal
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


#install(PROGRAMS
#  scripts/my_python_script
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )

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
