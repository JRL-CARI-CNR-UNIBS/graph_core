cmake_minimum_required(VERSION 3.16)
project(graph_core VERSION 0.0.1)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    #set(CMAKE_BUILD_TYPE "Debug")
endif()

# Compiler flags
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

# Options
option(ADD_CUSTOM_UNINSTALL "Add uninstall target" ON)
option(BUILD_TESTS "Build the test executables" ON)

# Dependencies
include(cmake_config/Dependencies.cmake)

# Configure RPATH for libraries
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_RPATH "")

message(STATUS "${BLUE}CMAKE_INSTALL_RPATH=${CMAKE_INSTALL_RPATH}${RESET}")

# Library target
add_library(${PROJECT_NAME} SHARED
    # Graph components
    src/${PROJECT_NAME}/graph/node.cpp
    src/${PROJECT_NAME}/graph/connection.cpp
    src/${PROJECT_NAME}/graph/tree.cpp
    src/${PROJECT_NAME}/graph/subtree.cpp
    src/${PROJECT_NAME}/graph/path.cpp
    src/${PROJECT_NAME}/graph/net.cpp

    # Samplers
    src/${PROJECT_NAME}/samplers/uniform_sampler.cpp
    src/${PROJECT_NAME}/samplers/ball_sampler.cpp
    src/${PROJECT_NAME}/samplers/informed_sampler.cpp
    src/${PROJECT_NAME}/samplers/tube_informed_sampler.cpp

    # Metrics
    src/${PROJECT_NAME}/metrics/euclidean_metrics.cpp

    # Datastructure
    src/${PROJECT_NAME}/datastructure/kdtree.cpp
    src/${PROJECT_NAME}/datastructure/vector.cpp

    # Solvers
    src/${PROJECT_NAME}/solvers/tree_solver.cpp
    src/${PROJECT_NAME}/solvers/rrt.cpp
    src/${PROJECT_NAME}/solvers/birrt.cpp
    src/${PROJECT_NAME}/solvers/rrt_star.cpp
    src/${PROJECT_NAME}/solvers/anytime_rrt.cpp

    # Path optimizers
    src/${PROJECT_NAME}/solvers/path_optimizers/path_optimizer_base.cpp
    src/${PROJECT_NAME}/solvers/path_optimizers/path_local_optimizer.cpp

    # Plugins
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

# Add dependencies
add_dependencies(${PROJECT_NAME} cnr_logger::cnr_logger cnr_param::cnr_param cnr_class_loader::cnr_class_loader)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    cnr_logger::cnr_logger
    cnr_param::cnr_param
    cnr_class_loader::cnr_class_loader
)

# Alias for library
add_library("${PROJECT_NAME}::${PROJECT_NAME}" ALIAS ${PROJECT_NAME})

# Tests
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

# Install headers
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include)

# Install files
install(FILES tests/logger_param.yaml
    DESTINATION "share/${PROJECT_NAME}/tests")

# Install targets
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install exported targets
install(EXPORT ${PROJECT_NAME}Targets
    DESTINATION "share/${PROJECT_NAME}/cmake"
    NAMESPACE ${PROJECT_NAME}::
    FILE ${PROJECT_NAME}Targets.cmake
)

# Package configuration
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake_config/${PROJECT_NAME}Config.cmake.in
    ${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION "share/${PROJECT_NAME}/cmake"
)

# Ensure cnr_yaml cmake_alternative is copied during installation
if(CLONE_CNR_YAML_CMAKE_ALTERNATIVE)
    include(cmake_config/FixInstallationPath.cmake)
endif()

# Custom uninstall target
if(ADD_CUSTOM_UNINSTALL)
    message(STATUS "Adding custom uninstall")
    add_custom_target("uninstall_graph_core" COMMENT "Uninstall installed files")
    add_custom_command(
        TARGET "uninstall_graph_core"
        POST_BUILD
        COMMAND xargs rm -vf < install_manifest.txt || echo Nothing to uninstall!
    )
endif()

# Clang-format target
file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h)

add_custom_target(
    format
    COMMAND chmod +x cmake_config/clang_format_target.sh
    COMMAND cmake_config/clang_format_target.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Running clang-format on all source files"
)









