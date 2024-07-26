# graph_core
<h1 align="center">🚧 Update in Progress! 🚧</h1>
<p align="center">
  <img src="https://img.shields.io/badge/Status-Updating-blue?style=for-the-badge&logo=github">
</p>
<p align="center">
  We're currently working on documentation. Expect new changes in the next weeks. Stay tuned!
</p>


## Dependencies

`graph_core` relies on the following packages:

- [cnr_logger](https://github.com/CNR-STIIMA-IRAS/cnr_logger): Logging package.
- [cnr_param](https://github.com/CNR-STIIMA-IRAS/cnr_param): Package to read and set parameters. It depends on [cnr_yaml](https://github.com/CNR-STIIMA-IRAS/cnr_yaml).
- [cnr_class_loader](https://github.com/JRL-CARI-CNR-UNIBS/cnr_class_loader): Provides a way to load classes as plugins.

For detailed instructions on downloading and installing these dependencies, refer to the [cnr_common](https://github.com/JRL-CARI-CNR-UNIBS/cnr_common) page.

## Installation 
Follow these steps to compile and install `graph_core`using CMake: 

1. Set the installation directory:
    ```bash
    export PATH_TO_WS=path_to_your_ws
    ```

2. Compile and install `graph_core`:
    ```bash
    cd $PATH_TO_WS
    mkdir -p build/graph_core
    cmake -S cari_motion_planning/graph_core -B build/graph_core -DCMAKE_INSTALL_PREFIX=$PATH_TO_WS/install
    make -C build/graph_core install
    ```

## Environment Configuration

According to the [cnr_common](https://github.com/JRL-CARI-CNR-UNIBS/cnr_common) instructions, add the following lines to your `~.bashrc` file:

```bash
if [[ ":$PATH:" != *":path_to_your_ws/install/bin:"* ]]; then
    export PATH="path_to_your_ws/install/bin:$PATH"
fi
if [[ ":$LD_LIBRARY_PATH:" != *":path_to_your_ws/install/lib:"* ]]; then
    export LD_LIBRARY_PATH="path_to_your_ws/install/lib:$LD_LIBRARY_PATH"
fi
if [[ ":$CMAKE_PREFIX_PATH:" != *":path_to_your_ws/install:"* ]]; then
    export CMAKE_PREFIX_PATH="path_to_your_ws/install:$CMAKE_PREFIX_PATH"
fi
``` 
Replace `path_to_your_ws/install` with the actual path to your install folder. These settings are necessary to make the installed libraries visible. However, `graph_core` can also be compiled in both ROS1 and ROS2 workspaces. For a ROS1 workspace, ensure you have set `catkin config --install`. In this case, you do not need to export the paths as shown above.
