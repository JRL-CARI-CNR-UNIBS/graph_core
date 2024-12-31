#!/bin/bash

if [ -z "$PATH_TO_WS" ]; then
    mkdir -p graph_core_ws/src
    cd graph_core_ws
    export PATH_TO_WS="$(pwd)"
fi

echo "PATH_TO_WS=$PATH_TO_WS" >> $GITHUB_ENV
echo "Workspace Path: $PATH_TO_WS"

# Install graph_core
cd $PATH_TO_WS/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/graph_core.git

cd $PATH_TO_WS
mkdir -p build/graph_core
cmake -S src/graph_core/graph_core -B build/graph_core -DCMAKE_INSTALL_PREFIX=$PATH_TO_WS/install
make -C build/graph_core install
