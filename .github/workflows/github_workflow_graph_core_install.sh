#!/bin/bash
echo "Workspace Path: $PATH_TO_WS"
cd $PATH_TO_WS

echo "Install Folder: $PATH_TO_WS/install"
cd $PATH_TO_WS/install
echo "PATH: $PATH"
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"

echo "Files in $PATH_TO_WS/install:"
ls -R $PATH_TO_WS/install

# Install graph_core
cd $PATH_TO_WS/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/graph_core.git

cd $PATH_TO_WS
mkdir -p build/graph_core
cmake -S src/graph_core/graph_core -B build/graph_core -DCMAKE_INSTALL_PREFIX=$PATH_TO_WS/install
make -C build/graph_core install
