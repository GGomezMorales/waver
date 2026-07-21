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
    if [[ "$1" == "bringup" && -z "$2" ]]; then \
        bros && sros && ros2 launch waver_bringup bringup.launch.py; \
    elif [[ "$1" == "bringup" && -n "$2" && -z "$3" ]]; then \
        bros && sros && ros2 launch waver_bringup bringup.launch.py profile:="$2"; \
    elif [[ "$1" == "description" && -z "$2" ]]; then \
        bros && sros && ros2 launch waver_description description.launch.xml use_sim_time:=false; \
    elif [[ "$1" == "gazebo" && -z "$2" ]]; then \
        bros && sros && ros2 launch waver_gazebo gazebo.launch.xml; \
    elif [[ "$1" == "rviz" && -z "$2" ]]; then \
        bros && sros && ros2 launch waver_viz rviz.launch.xml use_sim_time:=false; \
    elif [[ "$1" == "teleop" && -z "$2" ]]; then \
        sros && ros2 run teleop_twist_keyboard teleop_twist_keyboard; \
    else \
        echo "Use: waver [bringup|description|gazebo|localization|mapping|navigation|rviz|teleop]"; \
        echo "     waver bringup [profile-name|profile-path]"; \
    fi \
}' >> ~/.bashrc

echo '_waver_completion() { \
    local cur=${COMP_WORDS[COMP_CWORD]} \
    local prev=${COMP_WORDS[COMP_CWORD-1]} \
    
    if [[ $COMP_CWORD -eq 1 ]]; then \
        COMPREPLY=( $(compgen -W "bringup description gazebo localization mapping navigation rviz teleop" -- "$cur") ); \
    elif [[ $COMP_CWORD -eq 2 && "$prev" == "bringup" ]]; then \
        COMPREPLY=( $(compgen -W "sim_navigation sim_mapping sim_localization" -- "$cur") ); \
    fi \
}; \
complete -F _waver_completion waver' >> ~/.bashrc

echo "source ~/.bashrc" >> ~/.bash_profile

exec "$@"
