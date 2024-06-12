#!/bin/bash

source $(rospack find jaxon_ros_bridge)/scripts/upstart/byobu-utils.bash
# source ${HOME}/prog/scripts/byobu-utils.bash
SESSION_NAME=test_rls
create-session

#SOURCE_SCRIPT="source ~/catkin_ws/ast_ws/devel/setup.bash"
SOURCE_SCRIPT="ast_ws-source"

new-window roscore "${SOURCE_SCRIPT} && roscore"
# new-window d455 "${SOURCE_SCRIPT} && roslaunch realsense2_camera rs_d435_and_t265.launch color_height:=480 color_width:=640"
# new-window ball_detect "${SOURCE_SCRIPT} && roslaunch basketball_motion ball_detect.launch"
new-window dummy_pos "${SOURCE_SCRIPT} && rosrun basketball_motion dummy_pos.py"
new-window rls "${SOURCE_SCRIPT} && roslaunch basketball_motion object_trajectory_estimator.launch"
new-window ros-topic "${SOURCE_SCRIPT} && rostopic echo /ObjectTrajectoryEstimator/pred_ball_state"
# new-window rqt_image_view " ${SOURCE_SCRIPT} && rqt_image_view "
# new-window rviz "ast_ws-source && rviz -d /home/himeno/catkin_ws/ast_ws/src/auto_stabilizer_config/auto_stabilizer_config/rviz_config/JAXON_RED_WITH_MSLHAND.rviz "
new-window rviz "ast_ws-source && rviz -d /home/himeno/catkin_ws/ast_ws/src/basketball_motion/config/ball_path_config.rviz"
new-window plot "ast_ws-source && rosrun rqt_plot rqt_plot"
