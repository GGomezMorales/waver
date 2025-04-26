#!/bin/bash

echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
echo "alias bros='cd ${WS} && catkin build'" >> /root/.bashrc
echo "alias dros='cd ${WS} && rosdep update && rosdep install --from-paths src --ignore-src -r -y'" >> /root/.bashrc
echo "alias sros='source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WS}/devel/setup.bash'" >> /root/.bashrc

echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "alias bros='cd ${WS} && catkin build'" >> ~/.bashrc
echo "alias dros='cd ${WS} && rosdep update && rosdep install --from-paths src --ignore-src -r -y'" >> ~/.bashrc
echo "alias sros='source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WS}/devel/setup.bash'" >> ~/.bashrc

echo 'waver() { \
    if [[ "$1" == "description" && -z "$2" ]]; then \
        bros && sros && roslaunch waver_description description.launch; \
    elif [[ "$1" == "gazebo" && -z "$2" ]]; then \
        bros && sros && roslaunch waver_gazebo gazebo.launch; \
    elif [[ "$1" == "nav" && -z "$2" ]]; then \
        bros && sros && roslaunch waver_nav waver_nav.launch; \
    elif [[ "$1" == "rviz" && -z "$2" ]]; then \
        bros && sros && roslaunch waver_viz rviz.launch; \
    elif [[ "$1" == "nav" && "$2" == "gmapping" ]]; then \
        bros && sros && roslaunch waver_nav gmapping.launch; \
    elif [[ "$1" == "nav" && "$2" == "navigation" ]]; then \
        bros && sros && roslaunch waver_nav waver_nav.launch; \
    else \
        echo "Use: waver [rviz|gazebo|nav]"; \
    fi \
}' >> ~/.bashrc

echo '_waver_completion() { \
    local cur=${COMP_WORDS[COMP_CWORD]} \
    local prev=${COMP_WORDS[COMP_CWORD-1]} \
    
    if [[ $COMP_CWORD -eq 1 ]]; then \
        COMPREPLY=( $(compgen -W "description gazebo nav rviz" -- "$cur") ); \
    elif [[ $COMP_CWORD -eq 2 && "$prev" == "nav" ]]; then \
        COMPREPLY=( $(compgen -W "gmapping navigation" -- "$cur") ); \
    fi \
}; \
complete -F _waver_completion waver' >> ~/.bashrc

echo "source ~/.bashrc" >> ~/.bash_profile

exec "$@"
