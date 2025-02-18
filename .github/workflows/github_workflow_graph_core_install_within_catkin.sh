#!/bin/bash
set -e

# Source ROS Noetic setup
echo "Sourcing ROS Noetic"
source /opt/ros/noetic/setup.bash

# Define workspace paths
WORKSPACE_DIR=$(pwd)/graph_core_ws
SRC_DIR=$WORKSPACE_DIR/src

# Create the workspace and source folder
echo "Setting up Catkin workspace at $WORKSPACE_DIR"
mkdir -p $SRC_DIR
cd $WORKSPACE_DIR

# Clone graph_core into the src folder
echo "Cloning graph_core repository into $SRC_DIR"
git clone https://github.com/JRL-CARI-CNR-UNIBS/graph_core.git $SRC_DIR/graph_core

echo "Cloning deps with vcstool into $SRC_DIR"
vcs import < graph_core/deps.repos

# Build the workspace
echo "Building the Catkin workspace"
catkin config --extend /opt/ros/noetic
catkin build -cs --cmake-args -DUSE_ROS1=OFF

# Source the workspace setup
echo "Sourcing workspace setup"
source $WORKSPACE_DIR/install/setup.bash
