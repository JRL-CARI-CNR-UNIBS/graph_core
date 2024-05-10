include(CMakeFindDependencyMacro)


find_dependency(cnr_logger REQUIRED)
find_dependency(cnr_param REQUIRED)


find_package(PkgConfig REQUIRED)
pkg_check_modules(urdfdom REQUIRED urdfdom IMPORTED_TARGET)

include("${CMAKE_CURRENT_LIST_DIR}/graph_coreTargets.cmake")
