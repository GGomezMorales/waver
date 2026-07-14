<h1 align="center">waver_rviz package</h1>

## Overview

The `waver_rviz` package provides visualization for the Wave Rover robot within the RViz2 environment. This package includes the necessary configuration files and a launch file to visualize the robot's sensors, state, and environment in real-time.

## Dependencies

**Required ROS2 packages**

- [`waver_description`](https://github.com/GGomezMorales/waver/tree/humble/waver_description)
- `rviz`

## Usage

This package can be launched using the project's helper aliases (inside the Docker container) or via standard ROS2 launch commands.

### Docker container environment (Recommended)

If you are working within the provided Docker environment, a helper function `waver` is defined in `autostart.sh` to simplify the build, source, and launch process.

To visualize the robot in RViz2:

```bash
waver rviz
```

### Standard ROS2 environment

If you are not using the Docker container or prefer standard ROS2 commands, ensure your workspace is built and sourced, then launch the package manually using `ros2 launch`:

```bash
ros2 launch waver_viz rviz.launch.xml
```

#### Launch arguments

The main entry point is `rviz.launch.xml`. It loads the robot model into `robot_description` (via `xacro`), includes `waver_description/description.launch.xml` (to start state publishers), and launches RViz2 using a configurable `.rviz` layout.

**Available arguments**

- `use_sim_time` _(bool)_: If `true`, the nodes will subscribe to the `/clock` topic for time synchronization. This is required when running the robot in simulators like Rviz2.Default: `true`
- `rviz_config` _(string)_: RViz config file to load at startup.
  Default: `$(find-pkg-share waver_viz)/rviz/waver.rviz`
