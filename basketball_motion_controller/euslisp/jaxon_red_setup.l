;; for ast_ws
(load "package://msl_hand_controller/euslisp/jaxon_red_mslhand-interface.l")
(load "package://auto_stabilizer/euslisp/auto-stabilizer-interface.l")
(load "package://basketball_motion/euslisp/basketball-motion-interface.l")
(load "package://control_tools/play-hrpsysfile.l")

(format t "=============================================================~%")
(format t "<-- setup -->~%")
(format t "(send *ri* :set-auto-stabilizer-param :is-emergency-step-mode nil)~%")
(format t "(start-ab)~%")
(format t "[ロボットを下ろす]~%")
(format t "(start-st)~%")
(format t "=============================================================~%")
(format t "<-- for basketball_motion_controller -->~%")
(format t "start-dribble-mode: (start-d-mode)~%")
(format t "start-dribble-motion: (start-d-motion)~%")
(format t "stop-dribble-motion: (stop-d-motion)~%")
(format t "stop-dribble-mode: (stop-d-mode)~%")
(format t "=============================================================~%")