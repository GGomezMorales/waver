ARG IMAGE="osrf/ros:humble-desktop-full"

FROM ${IMAGE}

ARG OS
ARG WS_ROS
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV USER=root
ENV WS=/${WS_ROS}
WORKDIR ${WS}

RUN apt update && apt install -y \
    python3-pip \
    python3-opencv \
    git \
    nano \
    iputils-ping \
    net-tools

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-joint-state-publisher

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-graceful-controller \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-teleop-twist-keyboard

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz-* \
    ros-${ROS_DISTRO}-gz-ros2-control* \
    ros-${ROS_DISTRO}-ros2-control*

ENV GZ_VERSION=fortress

COPY ./autostart.sh /${WS_ROS}
RUN chmod +x /${WS_ROS}/autostart.sh
RUN ./autostart.sh
