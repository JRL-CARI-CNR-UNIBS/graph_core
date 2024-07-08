include(CMakeFindDependencyMacro)

find_dependency(cnr_logger REQUIRED)
find_dependency(cnr_param REQUIRED)
find_dependency(cnr_class_loader REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/graph_coreTargets.cmake")

set(graph_core_LIBRARIES graph_core ${cnr_logger_LIBRARIES} ${cnr_param_LIBRARIES} ${cnr_class_loader_LIBRARIES})
set(graph_core_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include" ${cnr_logger_INCLUDE_DIRS} ${cnr_param_INCLUDE_DIRS} ${cnr_class_loader_INCLUDE_DIRS})
set(graph_core_FOUND True)

