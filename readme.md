This package provides a human-aware motion planner for robot manipulators.

## Installation

The software can be installed using rosinstall files.

1. Install ros: follow the steps described in http://wiki.ros.org/ROS/Installation
2. Install wstool and initialize the workspace: follow the steps described in http://wiki.ros.org/wstool
3. Install and configure rosdep: follow the steps described in http://wiki.ros.org/rosdep

Then, download and merge the rosinstall file:
```
cd ~/catkin_ws
wget https://raw.githubusercontent.com/JRL-CARI-CNR-UNIBS/cari_motion_planning/master/human_aware.rosinstall
wstool merge -t src ./human_aware.rosinstall
```
Now, do the same with the required dependencies:
```
cd ~/catkin_ws
wget https://raw.githubusercontent.com/CNR-STIIMA-IRAS/rosdyn/master/rosdyn.rosinstall
wstool merge -t src ./rosdyn.rosinstall
```
Download and install the packages specified in the rosinstall file and the other system dipendencies:
```
cd ~/catkin_ws
wstool update -t src
rosdep install --from-paths src --ignore-src -r -y
```
## Configuration

See [readme](dirrt_star/readme.md) in dirrt_star.

## Ack

This work was partially supported by ShareWork project (H2020, European Commission – G.A.820807) and Pickplace project (H2020, European Commission – G.A.780488).
