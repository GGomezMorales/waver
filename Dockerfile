FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-vcstool \
    git \
    libbullet-dev 

RUN apt-get update && apt-get install ros-${ROS_DISTRO}-xacro -y
RUN apt-get update && apt-get install ros-${ROS_DISTRO}-joint-state-publisher -y

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

COPY ./autostart /
RUN chmod +x /autostart
