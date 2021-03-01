Human-aware motion planning.

Configurations:
copy [config/dirrt_config.yaml](config/dirrt_config.yaml) in {your_package}_moveit_config and change the required field

copy [launch/dirrt_planning_pipeline.launch.xml](launch/dirrt_planning_pipeline.launch.xml) in  {your_package}_moveit_config and change the package name at line 30.

in {your_package}_moveit_config/launch_move_group.launch change pipeline from "ompl" to "dirrt"