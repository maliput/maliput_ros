[![GCC](https://github.com/maliput/ros2_maliput/actions/workflows/build.yml/badge.svg)](https://github.com/maliput/ros2_maliput/actions/workflows/build.yml)

# Maliput ROS

## Description

This package contains a ROS2 node query server to answer information about the `RoadNetwork`. It relies on the
maliput plugin infrastructure to load all the backend information so specific parameters are parsed from a YAML
file and its path is passed as a runtime parameter (`maliput_yaml_path`).

For a full overview of Maliput capabilities please visit https://maliput.readthedocs.io/en/latest/maliput_overview.html.

The purpose of this package and node in particular is to assist agents in their route planning as well
as assiting with the kinematics of the road. At the moment, the package offers only geometry related
queries, once the rule API is also exposed agents will be able to access more sophisticated queries.

## API Documentation

The ROS2 node exposes many services and topics that belong to the `LifeCycleNode`. Please refer to the
[documentation](https://design.ros2.org/articles/node_lifecycle.html) to properly handle those. The following,
just focuses on the available service calls which are listed below:

- `/branch_point`: looks for a `maliput::api::BranchPoint` by its ID.
- `/derive_lane_s_routes`: derives all paths from a `maliput::api::RoadPosition` to another, filtering Lanes whose
length is bigger than a maximum threshold.
- `/eval_motion_derivatives`: evaluates the motion derivatives at `maliput::api::RoadPosition` and scales it by a
certain `maliput::api::IsoLaneVelocity`.
- `/find_road_positions`: finds all `maliput::api::RoadPositionResult` in radius distance from a
`maliput::api::InertialPosition`.
- `/junction`: looks for a `maliput::api::Junction` by its ID.
- `/lane`: looks for a `maliput::api::Lane` by its ID.
- `/lane_boundaries`: computes the `maliput::api::Lane` boundaries at a given `maliput::api::RoadPosition`.
- `/road_geometry`: responds the `maliput::api::RoadGeometry` configuration.
- `/segment`: looks for a `maliput::api::Segment` by its ID.
- `/to_road_position`: maps a `maliput::api::InertialPosition` into a `maliput::api::RoadPosition`.
- `/to_inertial_pose`: maps a `maliput::api::RoadPosition` into a `maliput::api::InertialPosition` and the
`maliput::api::Rotation` there.

## Examples

[Getting Started](https://maliput.readthedocs.io/en/latest/getting_started.html) page is a good place for
starting to see the Maliput's capabilities.

For this package in particular, simply execute the following after successfully building the package and 
sourcing the environment.


```sh
ros2 launch maliput_ros maliput_ros.launch.py --show-args
```

It should display all the available arguments. Then, make sure you have
[maliput_malidrive](https://github.com/maliput/maliput_malidrive) installed in your system and you can launch
the sample as follows:

```sh
ros2 launch maliput_ros maliput_ros.launch.py maliput_yaml_path:=/home/$USER/maliput_ws_focal/src/ros2_maliput/maliput_ros/resources/maliput_malidrive_plugin.yml
```

It should launch the node which loads the plugin and builds a RoadNetwork for
[TShapeRoad.xord](https://github.com/maliput/maliput_malidrive/blob/main/resources/TShapeRoad.xodr).

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
    colcon build --packages-up-to maliput_ros
    ```

    **Note**: To build documentation a `-BUILD_DOCS` cmake flag is required:
    ```sh
    colcon build --packages-select maliput_ros --cmake-args " -DBUILD_DOCS=On"
    ```

For further info refer to [Source Installation on Ubuntu](https://maliput.readthedocs.io/en/latest/installation.html#source-installation-on-ubuntu)

#### For development

It is recommended to follow the guidelines for setting up a development workspace as described [here](https://maliput.readthedocs.io/en/latest/developer_setup.html).

## Contributing

Please see [CONTRIBUTING](https://maliput.readthedocs.io/en/latest/contributing.html) page.

## License

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://github.com/maliput/ros2_maliput/blob/main/LICENSE)
