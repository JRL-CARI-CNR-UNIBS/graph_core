# CMake policies
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
  cmake_policy(SET CMP0148 NEW)
endif()

# Deps
find_package(Eigen3 REQUIRED COMPONENTS Core Dense Geometry)
find_package(yaml-cpp REQUIRED)

# yaml-cpp >= 0.8 exports yaml-cpp::yaml-cpp, older versions export yaml-cpp
if(TARGET yaml-cpp::yaml-cpp)
  set(GRAPH_CORE_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(GRAPH_CORE_YAML_CPP_TARGET yaml-cpp)
endif()

file(
  DOWNLOAD
  https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.5/CPM.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
  EXPECTED_HASH SHA256=c46b876ae3b9f994b4f05a4c15553e0485636862064f1fcc9d8b4f832086bc5d
)
include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)

message("\n-----------------------------------------------\n")

string(ASCII 27 Esc)
set(RESET "${Esc}[m")
set(BLUE "${Esc}[34m")

CPMFindPackage(
  NAME cnr_class_loader
  GITHUB_REPOSITORY JRL-CARI-CNR-UNIBS/cnr_class_loader
  GIT_TAG modern_cmake
)

message("\n-----------------------------------------------\n")
