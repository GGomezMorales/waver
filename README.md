<h1 align = "center">Waver Robot</h1>

## Overview

This repository contains a collection of ROS (Noetic) packages designed to be executed both in simulation and on a real robot, with a focus on the [Wave Rover](https://www.waveshare.com/wave-rover.htm) by [Waveshare](https://www.waveshare.com/) as the primary hardware platform. These packages include launch files, configuration files, and resources necessary for running the ROS nodes. This project also includes Docker implementations to simplify running simulations on different operation systems or deploying the software on a real robot. This flexibility allows users to simulate the [Wave Rover](https://www.waveshare.com/wave-rover.htm) in a virtual environment or directly implement the software on the robot. As an open source project, encouraging contributions and replication.

## Motivation

This project aims to create a low-cost robotic platform that serves as a base for implementing and testing complex packages and algorithms. It's designed to be accessible to both hobbyists and professionals, providing a flexible and expandable system for a wide range of robotics applications.

## Packages

ROS packages for Wave Rover, supporting both simulation and real robot use:

- **`waver_description`**: URDF files for robot description. [See]()
- **`waver_gazebo`**: Gazebo simulation integration. [See]()
- **`waver_nav`**: Navigation capabilities. [See]()
- **`waver_viz`**: RViz visualization tools. [See]()

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

## [Docker Setup](https://docs.docker.com/get-docker/)

### Installing Docker on Host Machine

To install Docker on your host machine (Linux, Windows or macOS), follow the official Docker installation guide for your specific operating system:

- **Linux:** [Install Docker Engine on Linux](https://docs.docker.com/desktop/install/linux-install/)
- **Windows:** [Install Docker Desktop on Windows](https://docs.docker.com/desktop/install/windows-install/)
- **macOS:** [Install Docker Desktop on macOS](https://docs.docker.com/desktop/install/mac-install/)

After installing Docker, ensure Docker is running by checking the Docker version:

```bash
docker --version
```

### Installing Docker on Raspberry PI

To install Docker on a Raspberry Pi, please follow the detailed guide provided [here](https://roboticamed.github.io/docs/guides/setup/rpi4_setup/#docker)

### How to use

This project includes scripts in the [scripts](https://github.com/GGomezMorales/waver/tree/noetic/scripts) folder to make Docker easier to use. These scripts handle tasks such as building the Docker image, running the container, and accessing the container's terminal.

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

<!-- ### Real Implementation
#### Hardware Setup
### Demos -->
