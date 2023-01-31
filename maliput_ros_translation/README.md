[![GCC](https://github.com/maliput/ros2_maliput/actions/workflows/build.yml/badge.svg)](https://github.com/maliput/ros2_maliput/actions/workflows/build.yml)

# Maliput ROS Translation

## Description

This package contains convert functions to translate ROS2 messages definitions to and from maliput types.
Not all conversions are provided, just those that are needed in the development of the required ROS2 interfaces
and to assist testing. The purpose of this package is to help with that, not to provide full coverage
of type translation.

## API Documentation

There are two types of functions, one is `RosMessageType ToRosMessage(MaliputType)` and the other is
`MaliputType ToRosMessage(RosMessageType)`. The translation is obvious, and details are exposed in doxygen.

TODO(https://github.com/maliput/ros2_maliput/issues/23): publish documentation.

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
    colcon build --packages-up-to maliput_ros_translation
    ```

For further info refer to [Source Installation on Ubuntu](https://maliput.readthedocs.io/en/latest/installation.html#source-installation-on-ubuntu)

#### For development

It is recommended to follow the guidelines for setting up a development workspace as described [here](https://maliput.readthedocs.io/en/latest/developer_setup.html).

## Contributing

Please see [CONTRIBUTING](https://maliput.readthedocs.io/en/latest/contributing.html) page.

## License

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://github.com/maliput/ros2_maliput/blob/main/LICENSE)
