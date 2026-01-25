<h1 align = "center">waver_gazebo package</h1>

## Overview

The `waver_gazebo` package is designed to integrate the Wave Rover robot with the Gazebo simulation environment. This package includes the necessary launch files and configurations to simulate the Wave Rover in a realistic world. It allows to test and validate robot behaviors, algorithms, and interactions in a controlled and reproducible environment before deploying them on real hardware.

## Dependencies

**Required ROS packages**

- [`waver_description`](https://github.com/GGomezMorales/waver/tree/noetic/waver_description)
- `gazebo_msgs`
- `gazebo_plugins`
- `gazebo_ros`
- `gazebo_ros_control`

## Usage

This package can be launched using the project's helper aliases (inside the Docker container) or via standard ROS launch commands.

### Docker container environment (Recommended)

If you are working within the provided Docker environment, a helper function `waver` is defined in `autostart.sh` to simplify the build, source, and launch process.

To launch the Gazebo simulation with the Wave Rover:

```bash
waver gazebo
```

To control the robot using teleoperation tools in a separate terminal, use the bash helper and run:

```bash
./scripts/bash.sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Standard ROS environment

If you are not using the Docker container or prefer standard ROS commands, ensure your workspace is built and sourced, then launch the package manually using `roslaunch`:

```bash
roslaunch waver_gazebo gazebo.launch
```

#### Launch arguments

The main entry point is `gazebo.launch`. It starts Gazebo, loads a world, and spawns the Wave Rover using the URDF/Xacro from `waver_description`. You can customize the simulation (GUI, paused state, world file, model path, etc.) by passing launch arguments.

**Available arguments**

- `model` _(string)_: Path to the robot model (URDF/Xacro) used for spawning.  
  Default: `$(find waver_description)/urdf/waver.xacro`

- `world_name` _(string)_: Path to the Gazebo world file to load.  
  Default: `$(find waver_gazebo)/world/coworking.world`

- `paused` _(bool)_: Start the simulation paused.  
  Default: `false`

- `use_sim_time` _(bool)_: Use Gazebo’s `/clock` as ROS time (`/use_sim_time`).  
  Default: `true`

- `gui` _(bool)_: Launch Gazebo with the GUI client.  
  Default: `true`

- `headless` _(bool)_: Run without rendering (useful for CI or remote machines).  
  Default: `false`

- `debug` _(bool)_: Enable debug mode for Gazebo/ROS nodes (when supported).  
  Default: `false`
