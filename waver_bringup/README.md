<h1 align = "center">waver_bringup package</h1>

## Overview

The `waver_bringup` package provides the main profile-driven launcher for the Wave Rover robot. It composes simulation, state estimation, mapping, localization, navigation and visualization launch files from YAML profiles, allowing a complete robot configuration to be started with a single command.

Each profile defines its global launch arguments and an ordered set of stages. Stages can be enabled, disabled or modified from the command line without changing the profile file. The launcher validates the selected profile, resolves package paths through the ROS2 ament index and includes every enabled stage in profile order.

## Dependencies

**Required ROS2 packages**

- [`waver_description`](https://github.com/GGomezMorales/waver/tree/humble/waver_description)
- [`waver_gazebo`](https://github.com/GGomezMorales/waver/tree/humble/waver_gazebo)
- [`waver_localization`](https://github.com/GGomezMorales/waver/tree/humble/waver_localization)
- [`waver_mapping`](https://github.com/GGomezMorales/waver/tree/humble/waver_mapping)
- [`waver_navigation`](https://github.com/GGomezMorales/waver/tree/humble/waver_navigation)
- [`waver_viz`](https://github.com/GGomezMorales/waver/tree/humble/waver_viz)
- `ament_index_python`
- `launch`
- `launch_ros`
- `ros2launch`
- `python3-yaml`

The packages required at runtime depend on the stages enabled by the selected profile.

## Usage

This package can be launched inside the project's Docker container or via standard ROS2 launch commands.

### Docker container environment (Recommended)

If you are working within the provided Docker environment, a helper function `waver` is defined in `autostart.sh` to simplify the build, source and launch process.

To launch the default simulation navigation profile:

```bash
waver bringup
```

The optional second argument can be an installed profile name or the path to a custom `.yaml` or `.yml` profile:

```bash
waver bringup sim_mapping
waver bringup /absolute/path/to/custom_profile.yaml
```

### Standard ROS2 environment

If you are not using the Docker container or prefer standard ROS2 commands, ensure your workspace is built and sourced, then launch the package manually using `ros2 launch`:

```bash
ros2 launch waver_bringup bringup.launch.py
```

To select another installed profile:

```bash
ros2 launch waver_bringup bringup.launch.py profile:=sim_localization
```

#### Launch arguments

The main entry point is `bringup.launch.py`. It loads a YAML profile, validates its configuration and includes each enabled stage in the order defined by the profile.

**Available arguments**

- `profile` _(string)_: Installed profile name or path to a custom `.yaml` or `.yml` profile file.
  Default: `sim_navigation`

#### Available profiles

- `sim_navigation`: Starts Gazebo, EKF state estimation, the map server, AMCL, Nav2 and RViz2 for saved-map navigation.
- `sim_mapping`: Starts Gazebo, EKF state estimation, SLAM Toolbox and RViz2 for online mapping.
- `sim_localization`: Starts Gazebo, EKF state estimation, the map server, AMCL and RViz2 without autonomous navigation.

#### Stage overrides

Every profile stage supports command-line overrides using the `<stage>.<field>:=<value>` format.

**Available structural overrides**

- `<stage>.enabled` _(bool)_: Enables or disables a stage.
- `<stage>.package` _(string)_: Replaces the ROS2 package that provides a stage.
- `<stage>.launch_file` _(string)_: Replaces the launch file used by a stage.
- `<stage>.<argument>` _(string)_: Replaces or adds a launch argument for one stage.

For example, launch mapping without the Gazebo graphical interface:

```bash
ros2 launch waver_bringup bringup.launch.py \
  profile:=sim_mapping \
  simulation.gui:=false
```

To use a different saved map in the navigation profile:

```bash
ros2 launch waver_bringup bringup.launch.py \
  profile:=sim_navigation \
  map_provider.map_file:=/absolute/path/to/map.yaml
```

#### Custom profiles

A profile contains optional global launch arguments and a non-empty `stages` mapping. Each stage requires a `package` and `launch_file`, and can define `enabled` and `arguments` values.

```yaml
arguments:
  use_sim_time: "true"

stages:
  simulation:
    enabled: true
    package: waver_gazebo
    launch_file: gazebo.launch.xml
    arguments:
      gui: "true"
```

Profile values support the following substitutions:

- `$(find-pkg-share <package>)`: Resolves the package share directory through the ament index.
- `$(env <variable> [default])`: Resolves an environment variable with an optional default value.
