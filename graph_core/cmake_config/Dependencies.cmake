find_package(Eigen3 REQUIRED COMPONENTS Core Dense Geometry)

file(
  DOWNLOAD
  https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.5/CPM.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
  EXPECTED_HASH SHA256=c46b876ae3b9f994b4f05a4c15553e0485636862064f1fcc9d8b4f832086bc5d
)
include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)

message("\n-----------------------------------------------\n")

CPMFindPackage(
  NAME cnr_logger
  GITHUB_REPOSITORY CNR-STIIMA-IRAS/cnr_logger
  GIT_TAG master
  OPTIONS 
    "USE_ROS1 OFF"
)

message("\n-----------------------------------------------\n")

CPMFindPackage(
  NAME cnr_yaml
  GITHUB_REPOSITORY CNR-STIIMA-IRAS/cnr_yaml
  GIT_TAG master
  OPTIONS
    "BUILD_UNIT_TESTS OFF"
)

message("\n-----------------------------------------------\n")

CPMFindPackage(
  NAME cnr_param
  GITHUB_REPOSITORY CNR-STIIMA-IRAS/cnr_param
  GIT_TAG master
  OPTIONS  
    "COMPILE_MAPPED_FILE_MODULE ON"
    "BUILD_UNBUILD_UNIT_TESTS OFF"
    "BUILD_INTEGRATION_TESTS OFF"
)

message("\n-----------------------------------------------\n")

CPMFindPackage(
  NAME cnr_class_loader
  GITHUB_REPOSITORY JRL-CARI-CNR-UNIBS/cnr_class_loader
  GIT_TAG modern_cmake
)

message("\n-----------------------------------------------\n")
