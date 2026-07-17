<h1 align="center">waver_bringup package</h1>

## Overview

`waver_bringup` composes the complete Waver system from a YAML **profile**. A
profile is an ordered map of **stages** — `simulation`, `mapping`,
`map_provider`, `localization`, `navigation`, `visualization` (any names are
allowed) — and each stage simply names the package, launch file, and launch
arguments that implement it.

The generic launcher (`launch/bringup.launch.py`) resolves the configured
packages through the ament index at launch time. It contains **no backend
names and no conditionals** such as `if backend == "nav2"`: swapping an
implementation is a profile edit or a command-line override, never a code
change. Backend dependencies (SLAM Toolbox, Nav2, Gazebo, ...) belong to the
capability packages referenced by profiles, not to this package.

## Profiles

Shipped profiles (in `profiles/`):

| Profile                  | Stages                                                                                | Purpose                                   |
| ------------------------ | ------------------------------------------------------------------------------------- | ----------------------------------------- |
| `sim_mapping`          | Gazebo, SLAM Toolbox, RViz                                                            | Drive around and build a map              |
| `sim_localization`     | Gazebo, map server, AMCL, RViz                                                        | Localize on a saved map (no autonomy)     |
| `sim_navigation`       | Gazebo, map server, AMCL, Nav2, RViz                                                  | Full autonomous navigation on a saved map |
| `sim_mapping_upstream` | Same as`sim_mapping`, mapping stage swapped to the upstream `slam_toolbox` launch | Stage-swap example                        |

### Profile format

```yaml
# Optional: arguments merged into every stage (stage arguments win).
arguments:
  use_sim_time: "true"

stages:
  simulation:                     # stage name: free-form, order preserved
    enabled: true                 # optional, default true
    package: waver_gazebo         # resolved through the ament index
    launch_file: gazebo.launch.xml  # looked up in share/<pkg>/ and share/<pkg>/launch/
    arguments:                    # optional launch arguments for this stage
      world_file: $(find-pkg-share waver_gazebo)/worlds/room.sdf
```

Argument values support two substitutions so profiles keep working from an
installed colcon workspace:

- `$(find-pkg-share <package>)` — share directory of a package
- `$(env <VAR> [default])` — environment variable

Malformed profiles, unknown keys, missing packages, and missing launch files
all fail immediately with a `[waver_bringup] ...` error explaining what to fix.

## Usage

```bash
ros2 launch waver_bringup bringup.launch.py profile:=<name|/path/to/profile.yaml> [overrides...]
```

### Global arguments

| Argument         | Meaning                                                                                   |
| ---------------- | ----------------------------------------------------------------------------------------- |
| `profile`      | Profile name from this package, or a path to any profile YAML (default`sim_navigation`) |
| `namespace`    | ROS namespace forwarded to every stage that declares one                                  |
| `use_sim_time` | Forwarded to every stage when non-empty (otherwise profile values apply)                  |
| `map`          | Map yaml path forwarded to every stage that accepts a`map` argument                     |

Stages that do not declare a forwarded argument simply ignore it. Note that
the default simulation and visualization stages use global topic names, so
running the whole graph under a namespace additionally requires a
namespace-aware platform stage.

### Per-stage overrides

Any `<stage>.<field>:=<value>` pair overrides the profile without editing it:

| Override                        | Effect                                            |
| ------------------------------- | ------------------------------------------------- |
| `<stage>.enabled:=false`      | Skip the stage                                    |
| `<stage>.package:=<pkg>`      | Take the stage's launch file from another package |
| `<stage>.launch_file:=<file>` | Use a different launch file                       |
| `<stage>.<argument>:=<value>` | Override/add a launch argument of that stage      |

Precedence (low → high): profile `arguments` → stage `arguments` → global CLI
(`namespace`, `use_sim_time`, `map`) → per-stage CLI overrides.

### Examples

```bash
# 1. Simulation + mapping
ros2 launch waver_bringup bringup.launch.py profile:=sim_mapping

# 2. Simulation + saved-map localization (no autonomy)
ros2 launch waver_bringup bringup.launch.py profile:=sim_localization

# 3. Simulation + saved-map navigation (full stack)
ros2 launch waver_bringup bringup.launch.py profile:=sim_navigation

# 4. Stage swap without touching generic code: profile-only change
#    (mapping stage runs the upstream slam_toolbox launch file)
ros2 launch waver_bringup bringup.launch.py profile:=sim_mapping_upstream
#    ...or the same swap from the command line:
ros2 launch waver_bringup bringup.launch.py profile:=sim_mapping \
    mapping.package:=slam_toolbox \
    mapping.launch_file:=online_async_launch.py \
    mapping.slam_params_file:=$HOME/tuning/slam_custom.yaml

# 5. CLI overrides: namespace, sim time, map path, params path,
#    stage package and stage launch file
ros2 launch waver_bringup bringup.launch.py profile:=sim_navigation \
    namespace:=waver1 \
    use_sim_time:=true \
    map:=$HOME/waver_maps/office.yaml \
    navigation.params_file:=$HOME/tuning/nav2_custom.yaml \
    visualization.enabled:=false
```

A profile path outside the package works too:

```bash
ros2 launch waver_bringup bringup.launch.py profile:=$HOME/profiles/my_robot.yaml
```

## Adding a new stage implementation

No generic bringup code changes are ever required:

1. Create (or install) a package with a launch file for the new backend, e.g.
   `my_mapping_pkg/launch/cartographer.launch.xml`. Follow the stage contract
   below.
2. Point a profile at it — either copy an existing profile and change the
   stage:

   ```yaml
   mapping:
     package: my_mapping_pkg
     launch_file: cartographer.launch.xml
     arguments:
       params_file: $(find-pkg-share my_mapping_pkg)/param/cartographer.yaml
   ```

   or override on the command line:

   ```bash
   ros2 launch waver_bringup bringup.launch.py profile:=sim_mapping \
       mapping.package:=my_mapping_pkg mapping.launch_file:=cartographer.launch.xml
   ```

### Stage contract

A stage launch file should:

- accept `use_sim_time` (and, when applicable, `namespace`) arguments;
- use the shared graph conventions: TF `map -> odom -> base_footprint -> base_link`, topics `scan`, `odom`, `cmd_vel`, `map`, `/clock` for sim time;
- ensure at most one component publishes `map -> odom` (mapping or
  localization — never both in one profile); the simulation/platform stage
  owns `odom -> base_footprint`.

## Files

```
waver_bringup/
├── CMakeLists.txt
├── launch/
│   └── bringup.launch.py     # generic, data-driven launcher
├── package.xml
├── profiles/
│   ├── sim_localization.yaml
│   ├── sim_mapping.yaml
│   ├── sim_mapping_upstream.yaml
│   └── sim_navigation.yaml
└── README.md
```
