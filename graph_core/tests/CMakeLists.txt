add_executable(kdtree_test kdtree_test.cpp)
target_compile_definitions(kdtree_test
    PRIVATE
    TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(kdtree_test PUBLIC
    graph_core::graph_core
)
install(TARGETS kdtree_test RUNTIME DESTINATION "bin/graph_core/tests")

add_executable(node_connection_test node_connection_test.cpp)
target_compile_definitions(node_connection_test
    PRIVATE
    TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(node_connection_test PUBLIC
    graph_core::graph_core
)
install(TARGETS node_connection_test RUNTIME DESTINATION "bin/graph_core/tests")

add_executable(compute_path_test compute_path_test.cpp)
target_compile_definitions(compute_path_test
    PRIVATE
    TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(compute_path_test PUBLIC
    graph_core::graph_core
)
install(TARGETS compute_path_test RUNTIME DESTINATION "bin/graph_core/tests")

add_executable(path_post_processing_test path_post_processing_test.cpp)
target_compile_definitions(path_post_processing_test
    PRIVATE
    TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(path_post_processing_test PUBLIC
    graph_core::graph_core
)
install(TARGETS path_post_processing_test RUNTIME DESTINATION "bin/graph_core/tests")

# Install files
install(FILES logger_param.yaml logger_param_gtest.yaml
    DESTINATION "bin/graph_core/tests")

#GTESTS
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(run_all_tests run_all_tests.cpp)
target_link_libraries(run_all_tests PRIVATE ${GTEST_LIBRARIES} pthread graph_core::graph_core)
install(TARGETS run_all_tests RUNTIME DESTINATION "bin/graph_core/tests")
