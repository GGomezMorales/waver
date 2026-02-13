#!/bin/bash

echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
echo "alias bros='cd ${WS} && colcon build'" >> /root/.bashrc
echo "alias dros='cd ${WS} && rosdep update && rosdep install --from-paths src --ignore-src -r -y'" >> /root/.bashrc
echo "alias sros='source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WS}/install/setup.bash'" >> /root/.bashrc

echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "alias bros='cd ${WS} && colcon build'" >> ~/.bashrc
echo "alias dros='cd ${WS} && rosdep update && rosdep install --from-paths src --ignore-src -r -y'" >> ~/.bashrc
echo "alias sros='source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WS}/install/setup.bash'" >> ~/.bashrc

echo 'waver() { \
    if [[ "$1" == "description" && -z "$2" ]]; then \
        bros && sros && ros2 launch waver_description description.launch.xml; \
    elif [[ "$1" == "gazebo" && -z "$2" ]]; then \
        bros && sros && ros2 launch waver_gazebo gazebo.launch.xml; \
    elif [[ "$1" == "nav" && -z "$2" ]]; then \
        bros && sros && ros2 launch waver_nav waver_nav.launch.xml; \
    elif [[ "$1" == "rviz" && -z "$2" ]]; then \
        bros && sros && ros2 launch waver_viz rviz.launch.xml; \
    elif [[ "$1" == "nav" && "$2" == "gmapping" ]]; then \
        bros && sros && ros2 launch waver_nav gmapping.launch.xml; \
    elif [[ "$1" == "nav" && "$2" == "navigation" ]]; then \
        bros && sros && ros2 launch waver_nav waver_nav.launch.xml; \
    elif [[ "$1" == "teleop" && -z "$2" ]]; then \
        sros && ros2 run teleop_twist_keyboard teleop_twist_keyboard; \
    else \
        echo "Use: waver [description|gazebo|nav|rviz]"; \
    fi \
}' >> ~/.bashrc

echo '_waver_completion() { \
    local cur=${COMP_WORDS[COMP_CWORD]} \
    local prev=${COMP_WORDS[COMP_CWORD-1]} \
    
    if [[ $COMP_CWORD -eq 1 ]]; then \
        COMPREPLY=( $(compgen -W "description gazebo nav rviz teleop" -- "$cur") ); \
    elif [[ $COMP_CWORD -eq 2 && "$prev" == "nav" ]]; then \
        COMPREPLY=( $(compgen -W "gmapping navigation" -- "$cur") ); \
    fi \
}; \
complete -F _waver_completion waver' >> ~/.bashrc

echo "source ~/.bashrc" >> ~/.bash_profile

exec "$@"