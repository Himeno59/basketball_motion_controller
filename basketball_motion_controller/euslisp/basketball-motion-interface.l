(require"package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(ros::load-ros-manifest "basketball_motion_controller")
(instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods :ros-pkg-name "basketball_motion_controller")

(def-set-get-param-method
  'basketball_motion_controller::Openhrp_BasketballMotionControllerService_BasketballMotionControllerParam
  :raw-set-basketball-motion-param
  :get-basketball-motion-param
  :get-basketball-motion-param-arguments
  :basketballmotioncontrollerservice_setbasketballmotioncontrollerparam
  :basketballmotioncontrollerservice_getbasketballmotioncontrollerparam
  )

(defmethod rtm-ros-robot-interface
  (:start-dribble-mode
   ()
   (send *robot* :reset-pose)
   (send self :angle-vector (send *robot* :reset-pose) 5000)
   (send self :wait-interpolation)
   (send self :basketballmotioncontrollerservice_startDribbleMode))
  (:stop-dribble-mode
   ()
   (send self :basketballmotioncontrollerservice_stopDribbleMode))
  (:start-dribble-motion
   ()
   (send self :basketballmotioncontrollerservice_startDribbleMotion))
  (:stop-dribble-motion
   ()
   (send self :basketballmotioncontrollerservice_stopDribbleMotion))

  (:set-basketball-motion-param
   (&rest args
    &keys target-contact-ball-state hand-offset rarm-range larm-range ff-motion-time
    &allow-other-keys)
   (let ((prm (send self :get-basketball-motion-param)))
     (send* self :raw-set-basketball-motion-param
	    (append
	     (if (and (memq :target-contact-ball-state args) target-contact-ball-state)
		 (let ((tmp (send prm :target-contact-ball-state)))
		   (setq (tmp . ros::_data) (apply #'concatenate float-vector target-contact-ball-state))
		   (list :target-contact-ball-state tmp)))
	     (if (and (memq :hand-offset args) hand-offset)
		 (let ((tmp (send prm :hand-offset)))
		   (setq (tmp . ros::_data) (apply #'concatenate float-vector hand-offset))
		   (list :hand-offset tmp)))
	     (if (and (memq :rarm-range args) rarm-range)
		 (let ((tmp (send prm :rarm-range)))
		   (setq (tmp . ros::_data) (apply #'concatenate float-vector rarm-range))
		   (list :rarm-range tmp)))
	     (if (and (memq :larm-range args) larm-range)
		 (let ((tmp (send prm :larm-range)))
		   (setq (tmp . ros::_data) (apply #'concatenate float-vector larm-range))
		   (list :larm-range tmp)))
	     )	 
	    )
     )
   )
  )

