cmake_minimum_required(VERSION 3.0.2)
project(basketball_motion_controller_ros_bridge)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED rtmbuild2)

rtmbuild2_init()

catkin_package(
)

add_subdirectory(rtc)
