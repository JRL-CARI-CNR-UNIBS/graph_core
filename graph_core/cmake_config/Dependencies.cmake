# CMake policies
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
  cmake_policy(SET CMP0148 NEW)
endif()

# Deps
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
    "ENABLE_TESTING OFF"
    "ENABLE_COVERAGE_TESTING OFF"
    "COMPILE_EXAMPLE OFF"
)

message("\n-----------------------------------------------\n")

# If cnr_param is not already installed on the system, install cnr_yaml before cnr_param.
# cnr_yaml could be already installed or missing.

string(ASCII 27 Esc)
set(RESET "${Esc}[m")
set(BLUE "${Esc}[34m")

find_package(cnr_param QUIET)

if(NOT cnr_param_FOUND)
  # Check if cnr_yaml is already installed on the system
  find_package(cnr_yaml QUIET)

  if(NOT cnr_yaml_FOUND)
    # If cnr_yaml is not found, use CPM to install it
    CPMAddPackage(
      NAME cnr_yaml
      GITHUB_REPOSITORY CNR-STIIMA-IRAS/cnr_yaml
      GIT_TAG master
      OPTIONS
        "BUILD_UNIT_TESTS OFF"
    )
  endif()
endif()

message("\n-----------------------------------------------\n")

CPMFindPackage(
  NAME cnr_param
  GITHUB_REPOSITORY CNR-STIIMA-IRAS/cnr_param
  GIT_TAG master
  OPTIONS  
    "COMPILE_MAPPED_FILE_MODULE ON"
    "BUILD_UNIT_TESTS OFF"
    "BUILD_INTEGRATION_TESTS OFF"
    "RETRIVE_DEPENDENCIES OFF"
    "BUILD_INTEGRATION_TESTS OFF"
    "BUILD_AS_A_CATKIN_PACKAGE OFF"
)

message("\n-----------------------------------------------\n")

CPMFindPackage(
  NAME cnr_class_loader
  GITHUB_REPOSITORY JRL-CARI-CNR-UNIBS/cnr_class_loader
  GIT_TAG modern_cmake
)

message("\n-----------------------------------------------\n")
