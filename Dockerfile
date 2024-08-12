FROM osrf/ros:noetic-desktop-full

ENV WS=/waver_ws
WORKDIR ${WS}

RUN apt update && apt install -y \
    python3-catkin-tools \
    python3-rosinstall \
    git \
    nano \
    graphviz \
    iputils-ping \
    net-tools

RUN apt-get update && apt-get install -y \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-catkin \
    ros-noetic-urdf \
    ros-noetic-xacro \
    ros-noetic-tf2-tools

RUN apt-get update && apt-get install -y \
    ros-noetic-navigation \
    ros-noetic-slam-gmapping \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-teb-local-planner

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${WS}/devel/setup.bash" >> ~/.bashrc
RUN echo "alias sros='source /opt/ros/${ROS_DISTRO}/setup.bash ; catkin build ; source ${WS}/devel/setup.bash'" >> ~/.bashrc
