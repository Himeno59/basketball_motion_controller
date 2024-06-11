;; for ros-workspace
;; (load "package://msl_hand_controller/euslisp/jaxon_red-interface.l")
;; (load "package://auto_stabilizer/euslisp/auto-stabilizer-interface.l")

;; load file
;; (load "~/prog/jaxon-prog/euslisp/utils/utils.l")

;; initialize
(unless (boundp '*jaxon_red*)
  (jaxon_red-init)
  (setq *robot* *jaxon_red*))

(objects (list *robot*))
(change-bg #f(0.5 0.5 0.5))
(send *robot* :fix-leg-to-coords (make-coords))
(send *irtviewer* :draw-object)

;; log
;; (load "package://control_tools/play-hrpsysfile.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(format t "====================================================================================~%")
(format t "(send *ri* :start-auto-balancer)~%")
(format t "ロボットを下ろす~%")
(format t "(send *ri* :start-st)~%")
(format t "====================================================================================~%")
