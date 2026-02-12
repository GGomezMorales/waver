<h1 align = "center">waver_gazebo package</h1>

## Overview

The `waver_gazebo` package is designed to integrate the Wave Rover robot with the Gazebo simulation environment. This package includes the necessary launch files and configurations to simulate the Wave Rover in a realistic world. It allows to test and validate robot behaviors, algorithms, and interactions in a controlled and reproducible environment before deploying them on real hardware.

## Dependencies

**Required ROS2 packages**

- [`waver_description`](https://github.com/GGomezMorales/waver/tree/humble/waver_description)
- `ros_gz_sim`
- `ros_gz_bridge`
- `ros_gz_image`
- `ros_gz_interfaces`

## Usage

This package can be launched using the project's helper aliases (inside the Docker container) or via standard ROS2 launch commands.

### Docker container environment (Recommended)

If you are working within the provided Docker environment, a helper function `waver` is defined in `autostart.sh` to simplify the build, source, and launch process.

To launch the Gazebo simulation with the Wave Rover:

```bash
waver gazebo
```

To control the robot using teleoperation tools in a separate terminal, use the bash helper and run:

```bash
./scripts/bash.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Standard ROS2 environment

If you are not using the Docker container or prefer standard ROS2 commands, ensure your workspace is built and sourced, then launch the package manually using `ros2 launch`:

```bash
ros2 launch waver_gazebo gazebo.launch.xml
```

#### Launch arguments

The main entry point is `gazebo.launch.xml`. It starts Gazebo, loads a world, and spawns the Wave Rover using the URDF/Xacro from `waver_description`. You can customize the simulation (GUI, paused state, world file, model path, etc.) by passing launch arguments.

**Available arguments**

- `use_sim_time` _(bool)_: Use Gazebo’s `/clock` as ROS2 time (`/use_sim_time`).  
  Default: `true`

- `model` _(string)_: Path to the robot model (URDF/Xacro) used for spawning.  
  Default: `$(find-pkg-share waver_description)/urdf/waver.xacro`

- `world_name` _(string)_: Path to the Gazebo world file to load.  
  Default: `$(find-pkg-share waver_gazebo)/worlds/room.sdf`
