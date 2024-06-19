(ros::roseus "basketball")
(ros::advertise "basketball_motion_joint_states" sensor_msgs::JointState 1)

(setq *joint-name-list* (send *robot* :joint-list :name))

(ros::rate 1)
(while (ros::ok)
  (setq joint_state (instance sensor_msgs::JointState :init))
  (send joint_state :name *joint-name-list*)
  (send joint_state :position (send *robot* :angle-vector))
  
  (ros::ros-info "JointState [~A]" (send joint_state :position))

  (ros::publish "basketball_motion_joint_states" joint_state)
  
  (ros::sleep)
  )
