add_executable(kdtree_test kdtree_test.cpp)
target_compile_definitions(kdtree_test
    PRIVATE
    TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(kdtree_test PUBLIC
    graph_core::graph_core
)

add_executable(node_connection_test node_connection_test.cpp)
target_compile_definitions(node_connection_test
    PRIVATE
    TEST_DIR="${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(node_connection_test PUBLIC
    graph_core::graph_core
)
