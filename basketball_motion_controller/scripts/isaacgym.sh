#!/bin/bash

source $(rospack find jaxon_ros_bridge)/scripts/upstart/byobu-utils.bash
SESSION_NAME=jaxon-sim
create-session

#SOURCE_SCRIPT="source ~/catkin_ws/ast_ws/devel/setup.bash"
SOURCE_SCRIPT="ast_ws-source"

new-window roscore "rossetlocal && rossetip && roscore"
new-window nameserver "rm -f /tmp/omninames-* && omniNames -start 15005 -datadir /tmp"
sleep 0.5 # wait for roscore
new-window hrpsys "rossetlocal && rossetip && ${SOURCE_SCRIPT} && roslaunch auto_stabilizer_config hrpsys_JAXON_RED_WITH_MSLHAND.launch"
sleep 1.0 # wait for robot_description
new-window roseus "${SOURCE_SCRIPT} && cd ~/prog/jaxon-prog/euslisp/ && emacs -f shell"
