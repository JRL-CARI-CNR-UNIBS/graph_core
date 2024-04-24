cmake_minimum_required(VERSION 2.8.3)
project(graph_core)

add_compile_options(-std=c++17 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(Eigen3 REQUIRED)
find_package(cnr_logger REQUIRED)
find_package(cnr_param REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time filesystem)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${cnr_logger_INCLUDE_DIRS}
  ${cnr_param_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
 )

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/graph/node.cpp
  src/${PROJECT_NAME}/graph/connection.cpp
  src/${PROJECT_NAME}/graph/tree.cpp
  src/${PROJECT_NAME}/graph/subtree.cpp
  src/${PROJECT_NAME}/graph/path.cpp
  src/${PROJECT_NAME}/graph/net.cpp

  src/${PROJECT_NAME}/samplers/informed_sampler.cpp
  src/${PROJECT_NAME}/samplers/tube_informed_sampler.cpp

  src/${PROJECT_NAME}/datastructure/kdtree.cpp
  src/${PROJECT_NAME}/datastructure/vector.cpp

  src/${PROJECT_NAME}/solvers/tree_solver.cpp
  src/${PROJECT_NAME}/solvers/rrt.cpp
  src/${PROJECT_NAME}/solvers/birrt.cpp
  src/${PROJECT_NAME}/solvers/rrt_star.cpp
  src/${PROJECT_NAME}/solvers/anytime_rrt.cpp

  src/${PROJECT_NAME}/solvers/path_optimizers/path_optimizer_base.cpp
  src/${PROJECT_NAME}/solvers/path_optimizers/path_local_optimizer.cpp
  )
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${cnr_logger_LIBRARIES} #cnr_logger::cnr_logger
  ${cnr_param_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(kdtree_test tests/kdtree_test.cpp)
target_link_libraries(kdtree_test PUBLIC
  ${cnr_logger_LIBRARIES} #cnr_logger::cnr_logger
  ${cnr_param_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(node_connection_test tests/node_connection_test.cpp)
target_link_libraries(node_connection_test PUBLIC
  ${cnr_logger_LIBRARIES} #cnr_logger::cnr_logger
  ${cnr_param_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}
)

# Install
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib#/${PROJECT_NAME}
  LIBRARY DESTINATION lib#/${PROJECT_NAME}
  RUNTIME DESTINATION bin#/${PROJECT_NAME}
)

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

# make uninstall
add_custom_target("uninstall" COMMENT "Uninstall installed files")
add_custom_command(
    TARGET "uninstall"
    POST_BUILD
    COMMENT "Uninstall files with install_manifest.txt"
    COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
            install_manifest.txt to be uninstalled!
)
