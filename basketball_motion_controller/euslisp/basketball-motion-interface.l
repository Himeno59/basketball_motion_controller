(require"package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(ros::load-ros-manifest "basketball_motion_controller")
(instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods :ros-pkg-name "basketball_motion_controller")

;; (def-set-get-param-method
;;   'basketball_motion_controller::Openhrp_BasketballMotionControllerService_BasketballMotionControllerParam
;;   )

(defmethod rtm-ros-robot-interface
  (:start-dribble-mode
   ()
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
  )
