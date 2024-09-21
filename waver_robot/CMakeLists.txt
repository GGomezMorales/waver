cmake_minimum_required(VERSION 3.0.2)
project(waver_robot)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY launch nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
