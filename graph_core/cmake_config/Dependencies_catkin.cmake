find_package(Eigen3 REQUIRED COMPONENTS Core Dense Geometry)
find_package(catkin REQUIRED COMPONENTS
             cnr_yaml
             cnr_param
             cnr_logger
             cnr_class_loader
            )

list(APPEND DEPENDENCIES_INCLUDE_DIRS   "${catkin_INCLUDE_DIRS}")
list(APPEND DEPENDENCIES_LINK_LIBRARIES "${catkin_LIBRARIES}")
