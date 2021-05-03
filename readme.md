The software can be installed using rosinstall files.

1. Install ros: follow the steps described in http://wiki.ros.org/ROS/Installation
2. Install wstool and initialize the workspace: follow the steps described in http://wiki.ros.org/wstool
3. Install and configure rosdep: follow the steps described in http://wiki.ros.org/rosdep

Then, download and merge the rosinstall file:
```
cd ~/catkin_ws
wget https://bitbucket.org/iras-ind/human_aware_motion_planners/raw/b1d545049aa78ab35e3918e4d30fbc395416ad40/human_aware.rosinstall

cd ~/catkin_ws
 wstool merge -t src ./human_aware.rosinstall
```
Now, do the same with the dependencies required:
```
cd ~/catkin_ws
https://raw.githubusercontent.com/CNR-STIIMA-IRAS/rosdyn/master/rosdyn.rosinstall

cd ~/catkin_ws
wstool merge -t src ./rosdyn.rosinstall
```
Download and install the packages specified in the rosinstall file and the other system dipendencies:
```
cd ~/catkin_ws
wstool update -t src
rosdep install --from-paths src --ignore-src -r -y
```


see [readme](dirrt_star/readme.md) in dirrt_star