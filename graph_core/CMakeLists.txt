cmake_minimum_required(VERSION 2.8.3)
project(graph_core)

add_compile_options(-std=c++17 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(Eigen3 REQUIRED COMPONENTS)
find_package(cnr_logger REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time filesystem)
find_package(PkgConfig REQUIRED)
find_package(pluginlib)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

if (pluginlib_FOUND)
  message("pluginlib found")
  add_definitions(-DPLUGINLIB_AVAILABLE)
else()
  message("pluginlib not found")
  set(pluginlib_INCLUDE_DIRS "")
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${cnr_logger_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
 )

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/graph/node.cpp
  src/${PROJECT_NAME}/graph/connection.cpp
  src/${PROJECT_NAME}/graph/tree.cpp
  src/${PROJECT_NAME}/graph/subtree.cpp
  src/${PROJECT_NAME}/graph/path.cpp
  src/${PROJECT_NAME}/graph/net.cpp

  src/${PROJECT_NAME}/metrics.cpp
  src/${PROJECT_NAME}/goal_cost_function.cpp
  src/${PROJECT_NAME}/informed_sampler.cpp
  src/${PROJECT_NAME}/tube_informed_sampler.cpp

  src/${PROJECT_NAME}/datastructure/kdtree.cpp
  src/${PROJECT_NAME}/datastructure/vector.cpp

  src/${PROJECT_NAME}/solvers/tree_solver.cpp
  src/${PROJECT_NAME}/solvers/rrt.cpp
  src/${PROJECT_NAME}/solvers/birrt.cpp
  src/${PROJECT_NAME}/solvers/rrt_star.cpp
  src/${PROJECT_NAME}/solvers/anytime_rrt.cpp
  src/${PROJECT_NAME}/solvers/path_local_optimizer.cpp
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

# Install
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT graph_coreTargets
  ARCHIVE DESTINATION lib#/${PROJECT_NAME}
  LIBRARY DESTINATION lib#/${PROJECT_NAME}
  RUNTIME DESTINATION bin#/${PROJECT_NAME}
)

# set(CMAKE_INSTALL_PREFIX "/home/riemann/test_cmake")

set(EXPORT_CONFIGDIR "share/${PROJECT_NAME}/cmake")

install(EXPORT graph_coreTargets
  DESTINATION ${EXPORT_CONFIGDIR}
  FILE graph_coreTargets.cmake
)

export(
  EXPORT graph_coreTargets
  FILE "${PROJECT_BINARY_DIR}/graph_coreTargets.cmake")

include(CMakePackageConfigHelpers)
set(INCLUDE_DIR include)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/graph_coreConfig.cmake.in 
  ${CMAKE_CURRENT_BINARY_DIR}/graph_coreConfig.cmake
  INSTALL_DESTINATION ${EXPORT_CONFIGDIR})
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/graph_coreConfigVersion.cmake
  VERSION 0.0.1
  COMPATIBILITY AnyNewerVersion)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/graph_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/graph_coreConfigVersion.cmake
  DESTINATION ${EXPORT_CONFIGDIR})