#!/bin/bash

export PATH_TO_WS="$(pwd)"

#Install cnr_common
mkdir -p $PATH_TO_WS/src
cd $PATH_TO_WS/src
git clone --recurse-submodules https://github.com/JRL-CARI-CNR-UNIBS/cnr_common.git

cd $PATH_TO_WS/src/cnr_common
git submodule update --init --recursive
. update_submodules.sh

cd $PATH_TO_WS
cp src/cnr_common/build_template.sh build.sh && sed -i '1i PATH_TO_WS="$(PATH_TO_WS)"' build.sh
. build.sh


#Install graph_core
cd $PATH_TO_WS
mkdir -p build/graph_core
cmake -S src/graph_core/graph_core -B build/graph_core -DCMAKE_INSTALL_PREFIX=$PATH_TO_WS/install
make -C build/graph_core install

