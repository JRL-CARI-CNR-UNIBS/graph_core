Human-aware motion planning.

Configurations:
copy config/dirrt_config in [your_package]_moveit_config

copy launch/dirrt_planning_pipeline.launch.xml in  [your_package]_moveit_config

in [your_package]_moveit_config/launch_move_group.launch change pipeline from "ompl" to "dirrt"