## How to configure the human-aware motion planner for your setup

- copy [config/dirrt_planning.yaml](config/dirrt_planning.yaml) in `{your_package}`_moveit_config and change the required fields (see comments in the file)

- copy [launch/dirrt_planning_pipeline.launch.xml](launch/dirrt_planning_pipeline.launch.xml) in `{your_package}`_moveit_config and change the required fields (see comments in the file)

- With ROS Noetic you can use multiple pipelines. For do this,
copy [launch/multiple_pipeline.launch.xml](launch/dirrt_planning_pipeline.launch.xml) in `{your_package}`_moveit_config and change the required fields (see comments in the file)


- in `{your_package}`_moveit_config/launch_move_group.launch, set the argument `pipeline` equal to
  - `ompl` to use OMPL library
  - `dirrt` to use this library
  - `multiple` to use both (available from Noetic)
