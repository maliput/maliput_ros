[![GCC](https://github.com/maliput/ros2_maliput/actions/workflows/build.yml/badge.svg)](https://github.com/maliput/ros2_maliput/actions/workflows/build.yml)

# Maliput ROS Interfaces

## Description

This package contains a ROS2 messages and services definitions to interface with to and from
maliput types and the ROS2 world.

maliput offers many pointers in its API, and to signal errors those pointers usually become `nullptr`.
Given that it is not possible to represent natively `nullptr` in the ROS2 IDL, we decided to use the
respective `maliput::api::TypeSpecificIdentifier<T>` string representation to identify entities. Please,
note that these will become _empty strings_ when they try to signal a `nullptr`.

## API Documentation

Please, refer to each type. There are comments in most if not all the types that refer to the
analogous type or procedure to acquire that information.

## Installation

### Supported platforms

Ubuntu Focal Fossa 20.04 LTS.

### Source Installation on Ubuntu

#### Prerequisites

```
sudo apt install python3-rosdep python3-colcon-common-extensions
```

#### Build

1. Create colcon workspace if you don't have one yet.
    ```sh
    mkdir colcon_ws/src -p
    ```

2. Clone this repository in the `src` folder
    ```sh
    cd colcon_ws/src
    git clone https://github.com/maliput/ros2_maliput.git
    ```

3. Install package dependencies via `rosdep`
    ```
    export ROS_DISTRO=foxy
    ```
    ```sh
    rosdep update
    rosdep install -i -y --rosdistro $ROS_DISTRO --from-paths src
    ```

4. Build the package
    ```sh
    colcon build --packages-up-to maliput_ros_interfaces
    ```

For further info refer to [Source Installation on Ubuntu](https://maliput.readthedocs.io/en/latest/installation.html#source-installation-on-ubuntu)

#### For development

It is recommended to follow the guidelines for setting up a development workspace as described [here](https://maliput.readthedocs.io/en/latest/developer_setup.html).

## Contributing

Please see [CONTRIBUTING](https://maliput.readthedocs.io/en/latest/contributing.html) page.

## License

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://github.com/maliput/ros2_maliput/blob/main/LICENSE)
