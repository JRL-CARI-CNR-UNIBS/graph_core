cmake_minimum_required(VERSION 2.8.3)
project(human_probablistic_occupancy)

add_compile_options(-std=c++14 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  subscription_notifier rosdyn_core name_sorting
)
find_package( Eigen3 REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES human_probablistic_occupancy
  CATKIN_DEPENDS roscpp sensor_msgs rosdyn_core subscription_notifier name_sorting
  DEPENDS EIGEN3
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/human_probablistic_occupancy.cpp
 )
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 add_executable(${PROJECT_NAME}_node src/human_probablistic_occupancy_node.cpp)
 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )


add_executable(test_point src/test_point.cpp)
add_dependencies(test_point ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_point
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(simulate_occupancy src/simulate_occupancy.cpp)
add_dependencies(simulate_occupancy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulate_occupancy
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
 install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
