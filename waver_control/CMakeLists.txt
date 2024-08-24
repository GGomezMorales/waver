cmake_minimum_required(VERSION 3.0.2)
project(waver_control)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch param
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
