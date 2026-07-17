<h1 align = "center">Waver Robot</h1>

## Overview

This repository contains a collection of ROS2 (Humble) packages designed to be executed both in simulation and on a real robot, with a focus on the [Wave Rover](https://www.waveshare.com/wave-rover.htm) by [Waveshare](https://www.waveshare.com/) as the primary hardware platform. These packages include launch files, configuration files, and resources necessary for running the ROS nodes. This project also includes Docker implementations to simplify running simulations on different operation systems or deploying the software on a real robot. This flexibility allows users to simulate the [Wave Rover](https://www.waveshare.com/wave-rover.htm) in a virtual environment or directly implement the software on the robot. As an open source project, encouraging contributions and replication.

## Motivation

This project aims to create a low-cost robotic platform that serves as a base for implementing and testing complex packages and algorithms. It's designed to be accessible to both hobbyists and professionals, providing a flexible and expandable system for a wide range of robotics applications.

## Packages

ROS packages for Wave Rover, supporting both simulation and real robot use:

- **`waver_bringup`**: Generic, profile-driven bringup that composes the whole system from YAML profiles. [See](https://github.com/GGomezMorales/waver/tree/humble/waver_bringup)
- **`waver_description`**: URDF files for robot description. [See](https://github.com/GGomezMorales/waver/tree/humble/waver_description)
- **`waver_gazebo`**: Gazebo simulation integration. [See](https://github.com/GGomezMorales/waver/tree/humble/waver_gazebo)
- **`waver_localization`**: Map provider and localization launch files, parameters, and maps. [See](https://github.com/GGomezMorales/waver/tree/humble/waver_localization)
- **`waver_mapping`**: Mapping launch files, parameters, and map saving tools. [See](https://github.com/GGomezMorales/waver/tree/humble/waver_mapping)
- **`waver_navigation`**: Navigation launch files and Waver-specific parameters. [See](https://github.com/GGomezMorales/waver/tree/humble/waver_navigation)
- **`waver_viz`**: RViz2 visualization tools. [See](https://github.com/GGomezMorales/waver/tree/humble/waver_viz)

## Key features

- **Low cost implementation:** The project is designed to be an affordable solution (< $300 USD), providing a foundation for implementing and testing complex packages and algorithms.
- **Extensive documentation:** Each package includes documentation to help users understand and implement the provided features.
- **Cross platform compatibility (Simulation):** The project supports Linux natively and Windows via Docker, specifically for simulation purposes. Efforts are ongoing to extend compatibility to macOS using Docker.

## Hardware components

|             Component              |                                                        Link                                                         |
| :--------------------------------: | :-----------------------------------------------------------------------------------------------------------------: |
|         `DTOF LiDAR LD19`          |                                [See](https://www.waveshare.com/dtof-lidar-ld19.htm)                                 |
| `Raspberry Pi 4 Model B (4GB RAM)` | [See](https://www.waveshare.com/product/raspberry-pi/boards-kits/raspberry-pi-4/raspberry-pi-4-model-b-4gb-ram.htm) |
|            `Wave Rover`            |                                   [See](https://www.waveshare.com/wave-rover.htm)                                   |

## Docker setup

### Installing Docker on Host machine

To install Docker on your host machine (Linux, Windows or macOS), follow the official Docker installation guide for your specific operating system:

- **Linux:** [Install Docker Engine on Linux](https://docs.docker.com/engine/install/#supported-platforms)
- **Windows:** [Install Docker Desktop on Windows](https://docs.docker.com/desktop/install/windows-install/)
- **macOS:** [Install Docker Desktop on macOS](https://docs.docker.com/desktop/install/mac-install/)

After installing Docker, ensure Docker is running by checking the Docker version:

```bash
docker --version
```

To access graphical interfaces like Gazebo and RViz in Docker on Windows and macOS, you will need to use a VNC viewer. This tool allows you to view and interact with these interfaces through a remote desktop environment. For Linux users, this step is not required as graphical interfaces can be accessed directly from the Docker container without additional tools.

For resources and instructions on how to set up and use VNC viewers on Windows and macOS to run this project, follow the detailed guide provided [here](https://github.com/roboticamed/docker_ros_vnc?tab=readme-ov-file#docker-ros-noetic-vnc)

### Installing Docker on Raspberry PI

To install Docker on a Raspberry Pi, please follow the detailed guide provided [here](https://roboticamed.github.io/docs/guides/setup/rpi4_setup/#docker)

## How to use

This project includes scripts in the [scripts](https://github.com/GGomezMorales/waver/tree/humble/scripts) folder to make Docker easier to use. These scripts handle tasks such as building the Docker image, running the container, and accessing the container's terminal.

1. **Building the Docker image:**

   - Use the `build.sh` script to build the Docker image.
     ```bash
     ./scripts/build.sh
     ```

2. **Running the Docker container:**

   - There are three ways to run a container, depending on your host machine:
     - **`run_docker.sh`:** Uses the standard Docker command to run the container.
       ```bash
       ./scripts/run_docker.sh
       ```
     - **`run_cpu.sh`:** Uses [rocker](https://github.com/osrf/rocker) to run the container on a CPU environment.
       ```bash
       ./scripts/run_cpu.sh
       ```
     - **`run_nvidia.sh`:** Uses [rocker](https://github.com/osrf/rocker) to run the container with NVIDIA GPU support.
       ```bash
       ./scripts/run_nvidia.sh
       ```

3. **Accessing the container's terminal:**

   - If you need to access the terminal of the running Docker container, use the `bash.sh` script.
     ```bash
     ./scripts/bash.sh
     ```

## Helper aliases & commands

This project includes a set of preconfigured aliases and a unified command utility to optimize the development workflow within the Docker container. These are automatically loaded when entering the container.

**Core aliases:**

- bros (**B**uild **ROS**): Compiles the entire workspace using colcon build system.

  ```bash
  bros
  ```

  Equivalent to: `cd ${WS} && colcon build`

- dros (**D**ependencies **ROS**): Installs missing system dependencies for all packages in the source folder.

  ```bash
  dros
  ```

  Equivalent to: `cd ${WS} && rosdep update && rosdep install --from-paths src --ignore-src -r -y`

- sros (**S**ource **ROS**): Sources the ROS installation and your workspace overlay. Always run this command in every new terminal you open inside the container before running other ROS commands.

  ```bash
  sros
  ```

  Equivalent to: `source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WS}/install/setup.bash`

<!-- ### Real Implementation
#### Hardware Setup
### Demos -->
