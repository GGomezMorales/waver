<h1 align="center">waver_rviz package</h1>

## Overview

The `waver_rviz` package provides visualization for the Wave Rover robot within the RViz environment. This package includes the necessary configuration files and a launch file to visualize the robot's sensors, state, and environment in real-time.

## Dependencies

**Required ROS packages**

- [`waver_description`](https://github.com/GGomezMorales/waver/tree/noetic/waver_description)
- `catkin`
- `rviz`

## Usage

This package can be launched using the project's helper aliases (inside the Docker container) or via standard ROS launch commands.

### Docker container environment (Recommended)

If you are working within the provided Docker environment, a helper function `waver` is defined in `autostart.sh` to simplify the build, source, and launch process.

To visualize the robot in RViz:

```bash
waver rviz
```

### Standard ROS environment

If you are not using the Docker container or prefer standard ROS commands, ensure your workspace is built and sourced, then launch the package manually using `roslaunch`:

```bash
roslaunch waver_viz rviz.launch
```

#### Launch arguments

The main entry point is `rviz.launch`. It loads the robot model into `robot_description` (via `xacro`), includes `waver_description/description.launch` (to start state publishers), and launches RViz using a configurable `.rviz` layout.

**Available arguments**

- `model` _(string)_: Path to the robot model (URDF/Xacro) used for spawning.  
  Default: `$(find waver_description)/urdf/waver.xacro`

- `gui` _(bool)_: Enable the Joint State Publisher GUI (useful to move joints manually in RViz).  
  Default: `true`

- `rviz_config` _(string)_: RViz config file to load at startup.  
  Default: `$(find waver_viz)/rviz/waver.rviz`
