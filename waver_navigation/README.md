<h1 align="center">waver_navigation package</h1>

## Overview

The `waver_navigation` package provides the Nav2 navigation stack for the Wave Rover robot. It contains the launch file that brings up the seven Nav2 lifecycle servers, the Waver-specific parameter files, and the behavior trees that orchestrate them.

The stack is configured as a **multi-plugin** setup: four local planners (controllers) and two global planners are loaded into their servers **at the same time**, and the active one is chosen at runtime through the standard Nav2 behavior tree mechanism. No relaunch, lifecycle transition, or parameter rewrite is needed to switch between them.

The parameters are tuned for the physical dimensions of the Wave Rover (a 0.183 x 0.174 m four-wheel skid-steer base) with an emphasis on navigating narrow paths and tight alleys. Velocity, acceleration and inflation limits are derived from the hardware limits declared in `waver_description/config/waver_controllers.yaml`.

The package includes:

- **Multi-plugin controller server:** MPPI, DWB, TEB and Graceful controllers loaded simultaneously, plus two selectable goal checkers.
- **Multi-plugin planner server:** NavFn and Smac 2D planners loaded simultaneously.
- **Behavior trees:** A default tree that honours the runtime selector topics, and dedicated trees that pin a specific planner/controller pair per goal.
- **Split parameter files:** One YAML per Nav2 server, so a change to one server never risks the others.

## Dependencies

**Required ROS2 packages**

- `navigation2`
- `nav2_bringup`
- `nav2_common`
- `nav2_graceful_controller`
- `teb_local_planner`

**Third-party packages (Git submodules)**

The `teb_local_planner` controller is not distributed as a binary package for ROS2 Humble, so it is vendored into this repository as a Git submodule together with its `costmap_converter` dependency. Both live in the `third_party` folder at the repository root.

| Submodule                       | Upstream                                                                                  | Branch         |
| :------------------------------ | :---------------------------------------------------------------------------------------- | :------------- |
| `third_party/teb_local_planner` | [rst-tu-dortmund/teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner) | `humble-devel` |
| `third_party/costmap_converter` | [rst-tu-dortmund/costmap_converter](https://github.com/rst-tu-dortmund/costmap_converter) | `humble`       |

Initialize the submodules after cloning the repository:

```bash
git submodule update --init --recursive
```

The submodules are ordinary ROS2 packages inside the workspace source tree. Install their system dependencies and build them with the rest of the workspace:

```bash
cd /waver_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

After the submodules are built and the workspace is sourced, `TebFollowPath` is available alongside the other controller plugins.

## Usage

This package can be launched using the project's helper aliases (inside the Docker container) or via standard ROS2 launch commands.

### Docker container environment (Recommended)

If you are working within the provided Docker environment, a helper function `waver` is defined in `autostart.sh` to simplify the build, source, and launch process.

To launch the full simulation navigation stack (Gazebo, EKF, map server, AMCL, Nav2 and RViz2):

```bash
waver bringup sim_navigation
```

### Standard ROS2 environment

If you are not using the Docker container or prefer standard ROS2 commands, ensure your workspace is built and sourced, then launch the package manually using `ros2 launch`:

```bash
ros2 launch waver_navigation nav2.launch.py
```

#### Launch arguments

The main entry point is `nav2.launch.py`. It resolves the parameter files from the package share directory and starts the Nav2 servers together with the lifecycle manager that brings them up in order.

**Available arguments**

- `use_sim_time` _(bool)_: If `true`, the nodes will subscribe to the `/clock` topic for time synchronization. This is required when running the robot in simulators like Gazebo.
  Default: `true`
- `autostart` _(bool)_: If `true`, the lifecycle manager automatically configures and activates every Nav2 server at startup.
  Default: `true`

#### Started nodes

- `controller_server`: Local planning and the local costmap.
- `smoother_server`: Post-processes the global plan.
- `planner_server`: Global planning and the global costmap.
- `behavior_server`: Recovery behaviors.
- `bt_navigator`: Behavior tree orchestrator.
- `waypoint_follower`: Multi-goal follower.
- `velocity_smoother`: Final acceleration-limiting stage.
- `lifecycle_manager_navigation`: Brings all of the above up in order.

#### Parameter files

Each Nav2 server owns one file under `param/nav2`. They are loaded independently by `nav2.launch.py`.

- `controller_server.yaml`: Controller plugins, progress checker and goal checkers.
- `planner_server.yaml`: Global planner plugins.
- `costmaps.yaml`: Local and global costmaps, shared by the controller and planner servers.
- `behavior_server.yaml`: Recovery behaviors.
- `bt_navigator.yaml`: Behavior tree engine and the behavior tree node library list.
- `smoother_server.yaml`: Global path smoother.
- `velocity_smoother.yaml`: Final acceleration limits applied to every command.
- `waypoint_follower.yaml`: Multi-goal follower.

#### Available plugins

All plugins listed below are loaded simultaneously. The **plugin ID** is the name used to select a plugin at runtime; it is defined by the `controller_plugins`, `planner_plugins` and `goal_checker_plugins` lists in the parameter files.

**Controllers** (`controller_server`)

- `DwbFollowPath` _(default)_: `dwb_core::DWBLocalPlanner`. Trajectory-scoring controller tuned for reliable general navigation.
- `MPPIFollowPath`: `nav2_mppi_controller::MPPIController`. Predictive sampling controller tuned for forward progress and smooth path tracking.
- `GracefulFollowPath`: `nav2_graceful_controller::GracefulController`. Pose-following control law using a longer lookahead and softened heading gains for smooth final approaches.
- `TebFollowPath`: `teb_local_planner::TebLocalPlannerROS`. Timed Elastic Band controller for tight passages and dynamic obstacle avoidance.

**Planners** (`planner_server`)

- `GridBased` _(default)_: `nav2_navfn_planner/NavfnPlanner`. Navigation-function planner. Fast, and its potential field naturally follows the costmap inflation gradient down the middle of free space.
- `Smac2D`: `nav2_smac_planner/SmacPlanner2D`. Cost-aware A\*. Its `cost_travel_multiplier` parameter controls how strongly the path is pulled toward the centre of an aisle, which is preferable in genuinely tight corridors.

**Goal checkers** (`controller_server`)

- `general_goal_checker` _(default)_: 0.08 m position tolerance, 0.15 rad yaw tolerance.
- `precise_goal_checker`: 0.03 m position tolerance, 0.07 rad yaw tolerance. Intended to be paired with `GracefulFollowPath`.

#### Standardized topics

Topic names are fixed in `nav2.launch.py`; they are not launch arguments. Switching a planner or controller therefore never changes the RViz subscriptions or requires caller-provided remappings. The authoritative visualization topics are `/plan` for the global path and `/local_plan` for controller-local path visualization:

```
                     /compute_path_to_pose (action)
                                |
  /global_costmap/costmap --> planner_server --> nav_msgs/Path --> /plan
                                |  { GridBased | Smac2D }
                                v
                      /follow_path (action, carries the Path)
                                |
  /local_costmap/costmap  --> controller_server --> TwistStamped
                                |  { DWB | MPPI | Graceful | TEB }
                                v
                          /cmd_vel_nav
                                |
                        velocity_smoother
                                |
                            /cmd_vel
```

A global planner returns its result to `planner_server`, which publishes every selected planner's result on `/plan`. The controller server owns the single velocity-command output, hardcoded to `/cmd_vel_nav`, while controller-specific diagnostic path publishers are kept separate from the authoritative global path. Controllers that publish a predicted local path use `/local_plan`.

- **RViz paths are stable.** The navigation RViz profile always subscribes to `/plan` and `/local_plan`. It never depends on plugin-specific topics such as `/received_global_plan`, `/global_plan`, or `/transformed_global_plan`.
- **Goal tolerance is centralized.** The controllers take their goal tolerance from the active goal checker rather than from their own parameters, so the goal checkers are the single source of truth for all four.
- **Acceleration limiting is centralized.** Every controller's output passes through `velocity_smoother`, so the acceleration ramp of the robot stays identical across a switch even though the controllers have different internal motion models.

#### Switching planners and controllers

Nav2 selects the planner and the controller through the `ComputePathToPose` and `FollowPath` behavior tree nodes, which carry a `planner_id`, a `controller_id` and a `goal_checker_id` port. Whatever writes those ports decides which plugin runs. There are two standard ways to drive them, and both are native to Nav2.

##### 1. Selector topics (switch globally, at runtime)

The default behavior trees start with the `PlannerSelector`, `ControllerSelector` and `GoalCheckerSelector` nodes. Each subscribes to a `std_msgs/String` topic and writes the received name into a blackboard variable that feeds the corresponding port. Publishing a name changes the plugin used from the next planning or control cycle onward, and the choice persists across goals until it is changed again.

```bash
ros2 topic pub --once /controller_selector std_msgs/msg/String "{data: 'DwbFollowPath'}"
ros2 topic pub --once /controller_selector std_msgs/msg/String "{data: 'MPPIFollowPath'}"
ros2 topic pub --once /controller_selector std_msgs/msg/String "{data: 'GracefulFollowPath'}"
ros2 topic pub --once /controller_selector std_msgs/msg/String "{data: 'TebFollowPath'}"

ros2 topic pub --once /planner_selector std_msgs/msg/String "{data: 'Smac2D'}"
ros2 topic pub --once /planner_selector std_msgs/msg/String "{data: 'GridBased'}"

ros2 topic pub --once /goal_checker_selector std_msgs/msg/String "{data: 'precise_goal_checker'}"
ros2 topic pub --once /goal_checker_selector std_msgs/msg/String "{data: 'general_goal_checker'}"
```

The selector nodes subscribe with a transient local, reliable QoS profile so that a name published before the behavior tree is created is still delivered. This matches the default QoS of `ros2 topic pub`, so no additional flags are needed. If a selector topic is never published to, the tree falls back to the `default_planner`, `default_controller` and `default_goal_checker` attributes declared in the XML.

To confirm which plugin IDs are actually loaded:

```bash
ros2 param get /controller_server controller_plugins
ros2 param get /controller_server goal_checker_plugins
ros2 param get /planner_server planner_plugins
```

##### 2. Dedicated behavior trees (switch per goal)

The `NavigateToPose` and `NavigateThroughPoses` actions accept a `behavior_tree` field. Supplying a tree whose `planner_id`, `controller_id` and `goal_checker_id` ports are written literally pins that combination for that goal only, without disturbing the global selector state. This is the appropriate mechanism when a specific goal has different requirements from the rest of the mission, such as a docking approach at the end of a traverse.

| Behavior tree                                                      | Planner        | Controller     | Goal checker   |
| :----------------------------------------------------------------- | :------------- | :------------- | :------------- |
| `navigate_to_pose_w_replanning_and_recovery.xml` _(default)_       | selector topic | selector topic | selector topic |
| `navigate_through_poses_w_replanning_and_recovery.xml` _(default)_ | selector topic | selector topic | selector topic |

##### Adding a new plugin

The two mechanisms above are agnostic to which plugins exist. To make a new controller or planner selectable, add it to the relevant parameter file under a new ID and rebuild; it becomes immediately available to both mechanisms with no change to the behavior trees, the launch file or any topic.

1. Add the plugin ID to `controller_plugins` or `planner_plugins` in the corresponding file under `param/nav2`.
2. Add a block with the same name declaring the `plugin` class and its parameters.
3. Rebuild and relaunch, then select it by publishing its ID or by referencing it in a dedicated behavior tree.
