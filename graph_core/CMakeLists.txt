cmake_minimum_required(VERSION 2.8.3)
project(graph_core)
add_compile_options(-std=c++17 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(Eigen3 REQUIRED COMPONENTS)
find_package(cnr_logger REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time filesystem)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${cnr_logger_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
 )

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/graph/node.cpp
  src/${PROJECT_NAME}/graph/connection.cpp
  src/${PROJECT_NAME}/graph/tree.cpp
  src/${PROJECT_NAME}/graph/subtree.cpp
  src/${PROJECT_NAME}/graph/path.cpp
  src/${PROJECT_NAME}/graph/net.cpp

  src/${PROJECT_NAME}/informed_sampler.cpp
  src/${PROJECT_NAME}/metrics.cpp
  src/${PROJECT_NAME}/goal_cost_function.cpp

#  src/${PROJECT_NAME}/tube_informed_sampler.cpp

#  src/${PROJECT_NAME}/solvers/tree_solver.cpp
#  src/${PROJECT_NAME}/solvers/rrt.cpp
#  src/${PROJECT_NAME}/solvers/multigoal.cpp
#  src/${PROJECT_NAME}/solvers/birrt.cpp
#  src/${PROJECT_NAME}/solvers/rrt_star.cpp
#  src/${PROJECT_NAME}/solvers/anytime_rrt.cpp
#  src/${PROJECT_NAME}/solvers/path_solver.cpp
  src/${PROJECT_NAME}/datastructure/kdtree.cpp
  src/${PROJECT_NAME}/datastructure/vector.cpp
  )
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
  ${cnr_logger_LIBRARIES}
  ${Boost_LIBRARIES}
  # log4cxx aprutil-1 apr-1
)

add_executable(node_connection_test tests/node_connection_test.cpp)
target_link_libraries(node_connection_test
  ${cnr_logger_LIBRARIES}
  ${PROJECT_NAME}
  # log4cxx aprutil-1 apr-1
)

