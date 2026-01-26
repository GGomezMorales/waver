<h1 align="center">waver_nav package</h1>

## Overview

The `waver_nav` package provides a navigation ready bringup for the Wave Rover robot. It connects the robot model from `waver_description` with the ROS navigation stack (localization + planning), so you can send goals (e.g., from RViz) and have the robot plan and navigate.

Responsibilities of this package include:

- Loading the Wave Rover model into `robot_description` (via `waver_description`)
- Starting TF publication (`robot_state_publisher`) and joint states
- Starting localization (`map_server` + `amcl`, or alternatives)
- Starting path planning and control (`move_base`)
- Loading navigation parameters (costmaps, planners, inflation, footprint, etc.)

> Navigation requires a consistent TF tree and sensor/odometry inputs. In most setups you need:
> `map -> odom -> base_footprint -> base_link`

## Dependencies

**Required ROS packages**

- `catkin`
- `gmapping`
- `amcl`
- `map_server`
- `move_base`
- `global_planner`
- `teb_local_planner`

## Usage

This package can be launched using the project's helper aliases (inside the Docker container) or via standard ROS launch commands.

### Docker container environment (Recommended)

If you are working within the provided Docker environment, a helper function `waver` is defined in `autostart.sh` to simplify the build, source, and launch process.

To start navigation mapping:

```bash
waver nav gmapping
```

To bring up navigation:

```bash
waver nav navigation
```

### Standard ROS environment

If you are not using the Docker container or prefer standard ROS commands, ensure your workspace is built and sourced, then launch the package manually using `roslaunch`:

To start navigation mapping:

```bash
roslaunch waver_nav gmapping.launch
```

To bring up navigation:

```bash
roslaunch waver_nav waver_nav.launch
```

#### Launch arguments

The main entry point is `waver_nav.launch`. This launch file centralizes the navigation bringup. It loads the robot model via `waver_description` to populate `robot_description`, starts the required navigation components (e.g., map_server, localization and move_base), and applies the configured costmap/planner parameters, leaving the system ready to accept navigation goals on RViz.

##### Available arguments

###### Map

- `map_file` _(string)_: Path to the YAML map file loaded by `map_server`.  
  Default: `$(find waver_nav)/maps/coworking_map.yaml`

###### Robot model and RViz

- `model` _(string)_: Path to the robot model (URDF/Xacro). Forwarded to `waver_viz/launch/rviz.launch` and `waver_description`.  
  Default: `$(find waver_description)/urdf/waver.xacro`

- `rviz_config` _(string)_: RViz configuration file to load. Forwarded to `waver_viz/launch/rviz.launch`.  
  Default: `$(find waver_nav)/rviz/waver.rviz`

- `gui` _(bool)_: Enables the Joint State Publisher (used for visualization when you don’t have real joint states). Forwarded to `waver_viz/launch/rviz.launch`.  
  Default: `true`

###### Planners and parameter presets (move_base)

- `base_global_planner` _(string)_: Global planner plugin used by `move_base`.  
  Default: `global_planner/GlobalPlanner`

- `base_local_planner` _(string)_: Local planner plugin used by `move_base`.  
  Default: `teb_local_planner/TebLocalPlannerROS`

- `global` _(string)_: Selects which global planner parameter preset to load.  
  Loaded file: `$(find waver_nav)/param/global_planner_$(arg global).yaml`  
  Default: `base`  
  Available presets in the package: `base`, `navfn`

- `local` _(string)_: Selects which local planner parameter preset to load.  
  Loaded file: `$(find waver_nav)/param/local_planner_$(arg local).yaml`  
  Default: `teb`  
  Available presets in the package: `base`, `dwa`, `teb`

### What `move_base.launch` loads

In addition to the planner preset files above, `move_base.launch` loads:

- `param/costmap_common.yaml` (into both `local_costmap` and `global_costmap`)
- `param/costmap_global.yaml`
- `param/costmap_local.yaml`
- `param/move_base.yaml`
