#!/bin/bash

source $(rospack find jaxon_ros_bridge)/scripts/upstart/byobu-utils.bash
# source ${HOME}/prog/scripts/byobu-utils.bash
SESSION_NAME=vision_check
create-session

#SOURCE_SCRIPT="source ~/catkin_ws/ast_ws/devel/setup.bash"
SOURCE_SCRIPT="ast_ws-source"

# new-window d455 "${SOURCE_SCRIPT} && roslaunch realsense2_camera rs_d435_and_t265.launch color_height:=480 color_width:=640"
# sleep 2.0
new-window rqt_image_view " ${SOURCE_SCRIPT} && rossetjaxon_red && rqt_image_view "
new-window rviz "${SOURCE_SCRIPT} && rossetjaxon_red && rviz -d /home/himeno/catkin_ws/ast_ws/src/auto_stabilizer_config/auto_stabilizer_config/rviz_config/JAXON_RED_WITH_MSLHAND.rviz "
