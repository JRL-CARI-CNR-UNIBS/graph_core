#!/bin/bash

mkdir graph_core_ws
cd graph_core_ws
export PATH_TO_WS="$(pwd)"

echo "PATH_TO_WS=$PATH_TO_WS" >> $GITHUB_ENV
echo "Workspace Path: $PATH_TO_WS"

#Install cnr_common
mkdir -p "$PATH_TO_WS"/src
cd "$PATH_TO_WS"/src
git clone --recurse-submodules https://github.com/JRL-CARI-CNR-UNIBS/cnr_common.git

cd "$PATH_TO_WS"/src/cnr_common
git submodule update --init --recursive
. update_submodules.sh

cd "$PATH_TO_WS"
cp src/cnr_common/build_template.sh build.sh
chmod +x build.sh
. build.sh

#Export path to lib
echo "PATH=$PATH_TO_WS/install/bin:$PATH" >> "$GITHUB_ENV"
echo "LD_LIBRARY_PATH=$PATH_TO_WS/install/lib" >> "$GITHUB_ENV"
echo "CMAKE_PREFIX_PATH=$PATH_TO_WS/install" >> "$GITHUB_ENV"


