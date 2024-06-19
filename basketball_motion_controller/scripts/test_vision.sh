#!/bin/bash

source $(rospack find jaxon_ros_bridge)/scripts/upstart/byobu-utils.bash
create-session test_vision
SOURCE_SCRIPT="ast_ws-source"

new-window roscore "${SOURCE_SCRIPT} && roscore"
new-window nameserver "rm -f /tmp/omninames-* && omniNames -start 15005 -datadir /tmp"
sleep 0.5 # wait for roscore
new-window choreonoid "rossetlocal && rossetip && ${SOURCE_SCRIPT} && roslaunch auto_stabilizer_config choreonoid_JAXON_RED_WITH_MSLHAND.launch"
sleep 1.0 # wait for clock and spawning models
new-window hrpsys "rossetlocal && rossetip && ${SOURCE_SCRIPT} && roslaunch auto_stabilizer_config hrpsys_JAXON_RED_WITH_MSLHAND_for_basketball.launch"

new-window workspace1
new-window workspace2

new-window rviz "${SOURCE_SCRIPT} && rviz -d $(rospack find auto_stabilizer_config)/rviz_config/JAXON_RED_WITH_MSLHAND.rviz"
new-window rosbag "${SOURCE_SCRIPT} && roscd basketball_motion/rosbag "
sleep 1.0
new-window hsi_color_filter "${SOURCE_SCRIPT} && roslaunch jsk_pcl_ros hsi_color_filter.launch basketball_color:=orange ball_detect:=true"
new-window calc-pointcloud-centroid "${SOURCE_SCRIPT} && roslaunch jsk_pcl_ros_utils sample_centroid_publisher.launch INPUT:=/orange/hsi_output"
new-window object_trajectory_estimator "${SOURCE_SCRIPT} && roslaunch object_trajectory_estimator object_trajectory_estimator.launch"
new-window ros-topic "${SOURCE_SCRIPT} && rostopic list"
new-window plot "${SOURCE_SCRIPT} && rosrun rqt_plot rqt_plot"



