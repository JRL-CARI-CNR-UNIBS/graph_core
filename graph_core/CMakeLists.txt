cmake_minimum_required(VERSION 3.16)
project(graph_core)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    #set(CMAKE_BUILD_TYPE "Debug")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if(CMAKE_BUILD_TYPE MATCHES "Release")
        add_compile_options(-funroll-loops -Wall -Ofast -flto) #-O3
    endif()
    if(CMAKE_BUILD_TYPE MATCHES "Debug")
        add_compile_options(-Wall -g -Og)
    endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#Options
option(ADD_CUSTOM_UNINSTALL "Add uninstall target" ON)

find_package(Eigen3 REQUIRED COMPONENTS Core Dense Geometry)
find_package(cnr_param REQUIRED)
find_package(cnr_logger REQUIRED)
find_package(cnr_class_loader REQUIRED)

add_library(${PROJECT_NAME} SHARED
    #Graph components
    src/${PROJECT_NAME}/graph/node.cpp
    src/${PROJECT_NAME}/graph/connection.cpp
    src/${PROJECT_NAME}/graph/tree.cpp
    src/${PROJECT_NAME}/graph/subtree.cpp
    src/${PROJECT_NAME}/graph/path.cpp
    src/${PROJECT_NAME}/graph/net.cpp

    #Samplers
    src/${PROJECT_NAME}/samplers/uniform_sampler.cpp
    src/${PROJECT_NAME}/samplers/ball_sampler.cpp
    src/${PROJECT_NAME}/samplers/informed_sampler.cpp
    src/${PROJECT_NAME}/samplers/tube_informed_sampler.cpp

    #Metrics
    src/${PROJECT_NAME}/metrics/euclidean_metrics.cpp


    #Datastructure
    src/${PROJECT_NAME}/datastructure/kdtree.cpp
    src/${PROJECT_NAME}/datastructure/vector.cpp

    #Solvers
    src/${PROJECT_NAME}/solvers/tree_solver.cpp
    src/${PROJECT_NAME}/solvers/rrt.cpp
    src/${PROJECT_NAME}/solvers/birrt.cpp
    src/${PROJECT_NAME}/solvers/rrt_star.cpp
    src/${PROJECT_NAME}/solvers/anytime_rrt.cpp

    #Path optimizers
    src/${PROJECT_NAME}/solvers/path_optimizers/path_optimizer_base.cpp
    src/${PROJECT_NAME}/solvers/path_optimizers/path_local_optimizer.cpp

    #Plugins
    src/${PROJECT_NAME}/plugins/metrics/euclidean_metrics_plugin.cpp
    src/${PROJECT_NAME}/plugins/metrics/null_goal_cost_function_plugin.cpp
    src/${PROJECT_NAME}/plugins/samplers/uniform_sampler_plugin.cpp
    src/${PROJECT_NAME}/plugins/samplers/ball_sampler_plugin.cpp
    src/${PROJECT_NAME}/plugins/samplers/informed_sampler_plugin.cpp
    src/${PROJECT_NAME}/plugins/solvers/rrt_plugin.cpp
    src/${PROJECT_NAME}/plugins/solvers/birrt_plugin.cpp
    src/${PROJECT_NAME}/plugins/solvers/anytime_rrt_plugin.cpp
    src/${PROJECT_NAME}/plugins/solvers/rrt_star_plugin.cpp
    )
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
# target_compile_options(${PROJECT_NAME} PUBLIC -std=c++20 -funroll-loops -Wall -Ofast)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    cnr_param::cnr_param
    cnr_logger::cnr_logger
    cnr_class_loader::cnr_class_loader
    )

add_library("${PROJECT_NAME}::${PROJECT_NAME}" ALIAS ${PROJECT_NAME})

add_executable(kdtree_test tests/kdtree_test.cpp)
target_compile_definitions(kdtree_test
    PRIVATE
    TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/tests")
target_link_libraries(kdtree_test PUBLIC
    "${PROJECT_NAME}::${PROJECT_NAME}"
    )

add_executable(node_connection_test tests/node_connection_test.cpp)
target_compile_definitions(node_connection_test
    PRIVATE
    TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/tests")
target_link_libraries(node_connection_test PUBLIC
    "${PROJECT_NAME}::${PROJECT_NAME}"
    )

# Install
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include)

install(FILES tests/logger_param.yaml
    DESTINATION "share/${PROJECT_NAME}/tests")

install(
    TARGETS ${PROJECT_NAME} kdtree_test node_connection_test
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    )

install(EXPORT ${PROJECT_NAME}Targets
    DESTINATION "share/${PROJECT_NAME}/cmake"
    NAMESPACE ${PROJECT_NAME}::
    FILE ${PROJECT_NAME}Targets.cmake
    )

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION 0.0.1
    COMPATIBILITY AnyNewerVersion)
configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake_config/${PROJECT_NAME}Config.cmake.in ${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
    )

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION "share/${PROJECT_NAME}/cmake")

# make uninstall
if(ADD_CUSTOM_UNINSTALL)
    message(STATUS "Adding custom uninstall")
    add_custom_target("uninstall" COMMENT "Uninstall installed files")
    add_custom_command(
        TARGET "uninstall"
        POST_BUILD
        COMMENT "Uninstall files with install_manifest.txt"
        COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
        install_manifest.txt to be uninstalled!
        )
endif()

# clang-format
file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h)

add_custom_target(
    format
    COMMAND echo "Searching for .clang-format file..."
    COMMAND find ${CMAKE_SOURCE_DIR} -name ".clang-format" -print
    COMMAND echo "Running clang-format on the following files:"
    COMMAND echo ${ALL_SOURCE_FILES}
    COMMAND clang-format -i --style=file --assume-filename=${CMAKE_SOURCE_DIR}/.clang-format ${ALL_SOURCE_FILES}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Auto clang-format for all *.cpp and *.h files"
)








