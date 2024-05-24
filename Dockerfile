FROM osrf/ros:noetic-desktop-full

RUN mkdir -p /catkin_ws/src

RUN apt update && apt install -y \
    python3-catkin-tools \
    python3-rosinstall \
    git \
    nano \
    graphviz \
    tmux wget curl \
    iputils-ping \
    net-tools

RUN apt-get update && apt-get install -y \
    ros-noetic-joy \
    ros-noetic-teleop-twist-joy \
    ros-noetic-catkin \
    ros-noetic-urdf \
    ros-noetic-urdf-sim-tutorial \
    ros-noetic-xacro \
    ros-noetic-tf2-tools

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "~/waver_ws/devel/setup.bash" >> ~/.bashrc

COPY ./autostart.sh /
RUN chmod +x /autostart.sh
ENTRYPOINT /autostart.sh

