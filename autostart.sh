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
    elif [[ "$1" == "localization" && -z "$2" ]]; then \
        bros && sros && ros2 launch waver_localization localization.launch.xml; \
    elif [[ "$1" == "localization" && ( "$2" == "amcl" || "$2" == "beluga" ) && -z "$3" ]]; then \
        bros && sros && ros2 launch waver_localization localization.launch.xml backend:="$2"; \
    elif [[ "$1" == "mapping" && -z "$2" ]]; then \
        bros && sros && ros2 launch waver_mapping mapping.launch.xml; \
    elif [[ "$1" == "mapping" && ( "$2" == "slam_toolbox" || "$2" == "cartographer" ) && -z "$3" ]]; then \
        bros && sros && ros2 launch waver_mapping mapping.launch.xml backend:="$2"; \
    elif [[ "$1" == "navigation" && -z "$2" ]]; then \
        bros && sros && ros2 launch waver_navigation navigation.launch.xml; \
    elif [[ "$1" == "navigation" && "$2" == "nav2" && -z "$3" ]]; then \
        bros && sros && ros2 launch waver_navigation navigation.launch.xml backend:=nav2; \
    elif [[ "$1" == "rviz" && -z "$2" ]]; then \
        bros && sros && ros2 launch waver_viz rviz.launch.xml; \
    elif [[ "$1" == "teleop" && -z "$2" ]]; then \
        sros && ros2 run teleop_twist_keyboard teleop_twist_keyboard; \
    else \
        echo "Use: waver [description|gazebo|localization|mapping|navigation|rviz|teleop]"; \
    fi \
}' >> ~/.bashrc

echo '_waver_completion() { \
    local cur=${COMP_WORDS[COMP_CWORD]} \
    local prev=${COMP_WORDS[COMP_CWORD-1]} \
    
    if [[ $COMP_CWORD -eq 1 ]]; then \
        COMPREPLY=( $(compgen -W "description gazebo localization mapping navigation rviz teleop" -- "$cur") ); \
    elif [[ $COMP_CWORD -eq 2 && "$prev" == "localization" ]]; then \
        COMPREPLY=( $(compgen -W "amcl beluga" -- "$cur") ); \
    elif [[ $COMP_CWORD -eq 2 && "$prev" == "mapping" ]]; then \
        COMPREPLY=( $(compgen -W "slam_toolbox cartographer" -- "$cur") ); \
    elif [[ $COMP_CWORD -eq 2 && "$prev" == "navigation" ]]; then \
        COMPREPLY=( $(compgen -W "nav2" -- "$cur") ); \
    fi \
}; \
complete -F _waver_completion waver' >> ~/.bashrc

echo "source ~/.bashrc" >> ~/.bash_profile

exec "$@"
