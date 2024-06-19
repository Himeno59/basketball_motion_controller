#!/bin/bash

source $(rospack find jaxon_ros_bridge)/scripts/upstart/byobu-utils.bash
SESSION_NAME=jaxon
create-session

#SOURCE_SCRIPT="source ~/catkin_ws/ast_ws/devel/setup.bash"
SOURCE_SCRIPT="ast_ws-source"

new-window roseus "${SOURCE_SCRIPT} && rossetjaxon_red && cd ~/prog/jaxon-prog/ && emacs -f shell"
new-window rviz "${SOURCE_SCRIPT} && rossetjaxon_red && rviz -d $(rospack find auto_stabilizer_config)/rviz_config/JAXON_RED_WITH_MSLHAND.rviz"

#sleep 0.5 # wait for roscore
#sleep 1.0 # wait for clock and spawning models
#sleep 1.0 # wait for robot_description
