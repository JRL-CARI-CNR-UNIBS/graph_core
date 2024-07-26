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

To download these packages and set the appropriate branches, execute the following command, which will download the dependencies next to the `graph_core` directory:
```bash
cd graph_core && . download_deps.sh
```

## Installation

To compile and install `graph_core` and its dependencies using CMake, follow these steps:

1. Set the installation directory:
    ```bash
    path_to_install_folder=path_to_your_ws/install
    ```

2. Compile and install `cnr_yaml`:
    ```bash
    mkdir -p build/cnr_yaml
    cmake -S cnr_yaml -B build/cnr_yaml -DCMAKE_INSTALL_PREFIX=$path_to_install_folder
    make -C build/cnr_yaml install
    ```

3. Compile and install `cnr_param`:
    ```bash
    mkdir -p build/cnr_param
    cmake -S cnr_param -B build/cnr_param -DCMAKE_INSTALL_PREFIX=$path_to_install_folder -DCOMPILE_MAPPED_FILE_MODULE=ON
    make -C build/cnr_param install
    ```

Note that `cnr_param` needs the environment variable `CNR_PARAM_ROOT_DIRECTORY` to be defined. For example, you can define it in the `~/.bashrc` file as follows:

```bash
export CNR_PARAM_ROOT_DIRECTORY="/tmp/cnr_param"
```
This is the folder used by `cnr_param` to save parameters. See the dedicated [GitHub page](https://github.com/CNR-STIIMA-IRAS/cnr_param) for more information.

4. Compile and install `cnr_logger`:
    ```bash
    mkdir -p build/cnr_logger
    cmake -S cnr_logger -B build/cnr_logger -DCMAKE_INSTALL_PREFIX=$path_to_install_folder -DUSE_ROS1=False -DCOMPILE_EXAMPLE=True -DENABLE_TESTING=True
    make -C build/cnr_logger install
    ```

5. Compile and install `cnr_class_loader`:
    ```bash
    mkdir -p build/cnr_class_loader
    cmake -S cnr_class_loader -B build/cnr_class_loader -DCMAKE_INSTALL_PREFIX=$path_to_install_folder
    make -C build/cnr_class_loader install
    ```

6. Compile and install `graph_core`:
    ```bash
    mkdir -p build/graph_core
    cmake -S cari_motion_planning/graph_core -B build/graph_core -DCMAKE_INSTALL_PREFIX=$path_to_install_folder
    make -C build/graph_core install
    ```

`graph_core` can also be compiled in both ROS1 and ROS2 workspaces. For a ROS1 workspace, ensure you have set `catkin config --install`.