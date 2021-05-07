## How to configure the human-aware motion planner for your setup

- copy [config/dirrt_planning.yaml](config/dirrt_planning.yaml) in {your_package}_moveit_config and change the required fields (see comments in the file)

- copy [launch/dirrt_planning_pipeline.launch.xml](launch/dirrt_planning_pipeline.launch.xml) in {your_package}_moveit_config and change the package name at line 30.

- in {your_package}_moveit_config/launch_move_group.launch change pipeline from "ompl" to "dirrt"