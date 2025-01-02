#!/bin/bash

CATKIN_WS=~/graph_core_catkin_ws
GRAPH_CORE_REPO="https://github.com/JRL-CARI-CNR-UNIBS/graph_core.git"

# Create Catkin workspace
echo "Setting up Catkin workspace at $CATKIN_WS..."
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin config --install

# Clone the graph_core repository
if [ ! -d "$CATKIN_WS/src/graph_core" ]; then
  echo "Cloning graph_core repository..."
  cd $CATKIN_WS/src
  git clone $GRAPH_CORE_REPO
fi

# Build the workspace
echo "Building the workspace..."
cd $CATKIN_WS
catkin build -cs --verbose

# Source the setup script
source $CATKIN_WS/install/setup.bash

echo "graph_core installation completed successfully!"
