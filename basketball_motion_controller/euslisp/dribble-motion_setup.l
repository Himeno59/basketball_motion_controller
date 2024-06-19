;; utils funtion
(defun start-ab () (send *ri* :start-auto-balancer))
(defun stop-ab () (send *ri* :stop-auto-balancer))
(defun start-st () (send *ri* :start-st))
(defun stop-st () (send *ri* :stop-st))
(defun start-ee () (send *ri* :start-whole-body-master-slave))
(defun stop-ee () (send *ri* :stop-whole-body-master-slave))
(defun start-d-mode () (send *ri* :start-dribble-mode))
(defun stop-d-mode () (send *ri* :stop-dribble-mode))
(defun start-d-motion () (send *ri* :start-dribble-motion))
(defun stop-d-motion () (send *ri* :stop-dribble-motion))

;; set dq_weight
;; collar-yの重みを上げる + 肘pitchと手首rollの重みを上げる
(send *ri* :set-auto-stabilizer-param :dq-weight #f(1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 100.0 100.0 100.0 1.0 1.0 10.0 1.0 1.0 1.0 3.5 1.0 3.5 1.0 10.0 1.0 1.0 1.0 3.5 1.0 3.5 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0))
;; (send (send *ri* :get-auto-stabilizer-param) :dq_weight) ;; getのときはidlと同じで_でつなぐ
;; (send *ri* :set-auto-stabilizer-param :dq-weight #f(1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 100.0 100.0 100.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0))

;; 順番
;; hand(12) or leg(12)
;; CHEST_JOINT0 CHEST_JOINT1 CHEST_JOINT2
;; HEAD_JOINT0 HEAD_JOINT1
;; rarm
;; larm
;; hand(12) or leg(12)
;; defaultではchestにかなり重みがかかっている
