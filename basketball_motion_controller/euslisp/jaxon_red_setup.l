;; for ast_ws
(load "package://msl_hand_controller/euslisp/jaxon_red_mslhand-interface.l")
(load "package://auto_stabilizer/euslisp/auto-stabilizer-interface.l")
(load "package://basketball_motion_controller/euslisp/basketball-motion-interface.l")
(load "package://control_tools/play-hrpsysfile.l")

;; initialize
(unless (boundp '*jaxon_red*)
  (jaxon_red-init)
  (setq *robot* *jaxon_red*))

(objects (list *robot*))
(send *irtviewer* :change-background #f(0.5 0.5 0.5))
(send *irtviewer* :draw-objects)

;; for log
(send *ri* :set-log-maxlength (* 500 60))

(load "package://basketball_motion_controller/euslisp/dribble-motion_setup.l")

(format t "=============================================================~%")
(format t "<-- setup -->~%")
(format t "(send *ri* :set-auto-stabilizer-param :is-emergency-step-mode nil)~%")
(format t "(start-ab)~%")
(format t "[ロボットを下ろす]~%")
(format t "(start-st)~%")
(format t "=============================================================~%")
(format t "<-- for basketball_motion_controller -->~%")
(format t "(send *ri* :set-auto-stabilizer-param :dq-weight #f(1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 100.0 100.0 100.0 1.0 1.0 10.0 1.0 1.0 1.0 3.5 1.0 3.5 1.0 10.0 1.0 1.0 1.0 3.5 1.0 3.5 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0))")
(format t "start-dribble-mode: (start-d-mode)~%")
(format t "start-dribble-motion: (start-d-motion)~%")
(format t "stop-dribble-motion: (stop-d-motion)~%")
(format t "stop-dribble-mode: (stop-d-mode)~%")
(format t "=============================================================~%")
