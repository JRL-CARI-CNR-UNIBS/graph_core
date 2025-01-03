#!/bin/bash

mkdir graph_core_ws
cd graph_core_ws
export PATH_TO_WS="$(pwd)"

echo "PATH_TO_WS=$PATH_TO_WS"
echo "Workspace Path: $PATH_TO_WS"

# Clone graph_core
mkdir -p "$PATH_TO_WS"/src
cd "$PATH_TO_WS"/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/graph_core.git

# Build the workspace
echo "Building the workspace..."
cd $PATH_TO_WS
catkin config --install
catkin build -cs --verbose

# Source the setup script
source $PATH_TO_WS/install/setup.bash

echo "graph_core installation completed successfully!"
