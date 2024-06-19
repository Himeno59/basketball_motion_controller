;;;; JAXON dribble-motion ;;;;

;; gc print
(defun __gc_print (l1 l2) (print '!!!!!!!!!!gc-called!!!!!!!!!!))
(setq system:*gc-hook* '__gc_print)

(defmacro do-until-key-w/o-gc
    (&rest forms)
  `(let* ((str (list *standard-input*)))
     (while (null (select-stream str 0.0000001)) ,@forms)
     (let ((strm (car (select-stream str 0.1))))
       (if strm (read-line strm nil nil)))))

;; !!コンパイルするので使う関数は全てこのファイルに集約すること!!
;;;; setup ;;;;
;; initialize
(unless (boundp '*jaxon_red*)
  (jaxon_red-init)
  (setq *robot* *jaxon_red*))

(objects (list *robot*))
(send *irtviewer* :change-background #f(0.5 0.5 0.5))
(send *irtviewer* :draw-objects)

;; for log
(send *ri* :set-log-maxlength (* 500 60))

(format t "=======================================================================~%")
(format t "(send *ri* :set-auto-stabilizer-param :is-emergency-step-mode nil)~%")
(format t "(start-ab)~%")
(format t "ロボットを下ろす~%")
(format t "(start-st)~%")
(format t "=======================================================================~%")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; joint-state用
(setq *joint-name-list* (send *robot* :joint-list :name))

;; ros msg
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "sensor_msgs")

;; gloval変数
;; センサ値
(setq *object-state* nil)                        ;; 対称物の(予測)位置、速度
(setq *tactile-state* nil)                       ;; 接触力センサの値
(setq *joint-states* (instance sensor_msgs::JointState :init)) ;; hrpsysに送る関節角度(+関節名)
(send *joint-states* :name *joint-name-list*)
;; plot用
(setq *generated-hand-pos-list* nil)             ;; IKを解いた後のhandのpos
(setq *generated-hand-rpy-list* nil)             ;; IKを解いた後のhandのrpy
(setq *angle-vector-list* nil)             ;; IKを解いた後のhandのrpy

(defclass HandTrajectoryController
  :super propertied-object
  :slots (
	  object-state tactile-state joint-states
	  ref-pose target-z-start target-z-goal target-pitch-start target-pitch-goal
	  motion-period scale-period
	  dt target-hand-pos target-hand-rpy target-hand-coords target-angle-vector
	  dt-list target-hand-pos-list target-hand-rpy-list target-hand-coords-list target-angle-vector-list
	  additional-weight-list
	  )
  )

(defmethod HandTrajectoryController  
  (:init ()
   (setq
    ;; 可変長リスト(float-vector, listなど)はここで一度宣言し、あとでsetfで代入し直すようにする
    ;; ros msg	  
    object-state *object-state*      ;; 対象物の位置、速度
    tactile-state *tactile-state*    ;; 接触力センサの値
    joint-states *joint-states*        ;; hrpsysに送る関節角度(+関節名)
    ;; 軌道生成に関するパラメタ
    ref-pose nil                     ;; IKを解く際の参照関節角度
    target-z-start nil               ;; モーション中の最高ハンド高さ
    target-z-goal 0                  ;; モーション中の最低ハンド目標高さ
    target-pitch-start nil           ;; モーション中の最低pitch角
    target-pitch-goal 0.25           ;; モーション中の最高ハンド高さ
    motion-period 0.80               ;; 周期[sec]
    scale-period 1.0                 ;; 1周期のうちどのくらいの割合で軌道を生成するか
    dt 0.0020                           ;; hrpsys制御周期500[Hz]->2.0[msec]
    time 0.0                         ;; 1モーションの開始から何[sec]進んだか
    ;; 軌道生成用
    ;; JointState用
    target-hand-pos (float-vector 0.0 0.0 0.0)      ;; 目標ハンド位置
    target-hand-rpy (float-vector 0.0 0.0 0.0)      ;; 目標ハンド姿勢
    target-hand-coords (make-coords)                ;; 目標ハンド座標
    target-angle-vector nil                         ;; 目標関節角度
    ;; angle-vector-sequence-full用
    dt-list nil                      ;; 時間リスト
    target-hand-pos-list nil         ;; 目標ハンド位置軌道
    target-hand-rpy-list nil         ;; 目標ハンド姿勢軌道
    target-hand-coords-list nil      ;; 目標ハンド座標軌道
    target-angle-vector-list nil     ;; 目標関節角度軌道

    ;; IKの重みに関するリスト
    additional-weight-list (list  (list (send *robot* :rarm-collar-y :child-link) 0.1)
				  (list (send *robot* :rarm-shoulder-r :child-link) 0.01)
				  (list (send *robot* :rarm-shoulder-y :child-link) 0.1))
    )
   self)

  ;; accessor
  ;; 外から見る必要がない変数のものは削除する
  (:object-state (&optional(new-object-state nil)) (when new-object-state (setq object-state new-object-state)) object-state)
  (:tactile-state (&optional(new-tactile-state nil)) (when new-tactile-state (setq tactile-state new-tactile-state)) tactile-state)
  (:joint-states (&optional(new-joint-states nil)) (when new-joint-states (setq joint-states new-joint-states)) joint-states)
  (:ref-pose (&optional(new-ref-pose nil)) (when new-ref-pose (setq ref-pose new-ref-pose)) ref-pose)
  (:target-z-start (&optional(new-target-z-start nil)) (when new-target-z-start (setq target-z-start new-target-z-start)) target-z-start)
  (:target-z-goal (&optional(new-target-z-goal nil)) (when new-target-z-goal (setq target-z-goal new-target-z-goal)) target-z-goal)
  (:target-pitch-start (&optional(new-target-pitch-start nil)) (when new-target-pitch-start (setq target-pitch-start new-target-pitch-start)) target-pitch-start)
  (:target-pitch-goal (&optional(new-target-pitch-goal nil)) (when new-target-pitch-goal (setq target-pitch-goal new-target-pitch-goal)) target-pitch-goal)
  (:motion-period (&optional(new-motion-period nil)) (when new-motion-period (setq motion-period new-motion-period)) motion-period)
  (:scale-period (&optional(new-scale-period nil)) (when new-scale-period (setq scale-period new-scale-period)) scale-period)
  (:dt (&optional(new-dt nil)) (when new-dt (setq dt new-dt)) dt)
  (:time (&optional(new-time nil)) (when new-time (setq time new-time)) time)
  (:dt-list (&optional(new-dt-list nil)) (when new-dt-list (setq dt-list new-dt-list)) dt-list)
  (:target-hand-pos (&optional(new-target-hand-pos nil)) (when new-target-hand-pos (setq target-hand-pos new-target-hand-pos)) target-hand-pos)
  (:target-hand-rpy (&optional(new-target-hand-rpy nil)) (when new-target-hand-rpy (setq target-hand-rpy new-target-hand-rpy)) target-hand-rpy)
  (:target-hand-coords (&optional(new-target-hand-coords nil)) (when new-target-hand-coords (setq target-hand-coords new-target-hand-coords)) target-hand-coords)
  (:target-angle-vector (&optional(new-target-angle-vector nil)) (when new-target-angle-vector (setq target-angle-vector new-target-angle-vector)) target-angle-vector)
  (:target-hand-pos-list (&optional(new-target-hand-pos-list nil)) (when new-target-hand-pos-list (setq target-hand-pos-list new-target-hand-pos-list)) target-hand-pos-list)
  (:target-hand-rpy-list (&optional(new-target-hand-rpy-list nil)) (when new-target-hand-rpy-list (setq target-hand-rpy-list new-target-hand-rpy-list)) target-hand-rpy-list)
  (:target-hand-coords-list (&optional(new-target-hand-coords-list nil)) (when new-target-hand-coords-list (setq target-hand-coords-list new-target-hand-coords-list)) target-hand-coords-list)
  (:target-angle-vector-list (&optional(new-target-angle-vector-list nil)) (when new-target-angle-vector-list (setq target-angle-vector-list new-target-angle-vector-list)) target-angle-vector-list)
  
  ;;;; ros ;;;;
  (:ros-init ()
   ;; node init
   (ros::roseus "basketball_motion")
   ;; subscriber
   ;; (ros::subscribe "/ObjectTrajectoryEstimator/pred_ball_state_pos"
		   ;; geometry_msgs::PointStamped
		   ;; #'send self :object-state_cb)
   ;; (ros::subscribe "/dummy_pos" geometry_msgs::Point #'send self :object-state_cb)
   ;; (ros::subscribe [接触力センサ] geometry_msgs::Point #'send self :tactile-state_cb)
   ;; publisher 	     
   (ros::advertise "basketball_joint_states" sensor_msgs::JointState 1)
   )

  ;; call-back function
  (:object-state_cb (msg) (when msg (setq object-state msg)))
  ;; (:tactile-state_cb (msg (when msg (setq tactile-state msg))))

  ;; init-pose
  (:dribble-pose
   (&key(restart nil))
   
   (unless restart
     (send *robot* :reset-pose)
     ;; 脚の調整
     ;; lower-waist
     (format t "PRESS ENTER -> lower-waist~%")
     (read-line)
     (lower-waist :dz 100 :move t) ;; reset-poseから100[mm]膝を曲げる
     
     ;; set-foot-steps
     ;; auto-balancerを入れていないと動かないので注意
     ;; 右足相対で左足を400[mm]離す
     (format t "PRESS ENTER -> set-foot-steps~%")
     (read-line)
     (send *ri* :set-foot-steps
	 (list
	  (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
	  (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 600 400 0)) :name :lleg)
	  (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 0 0 0)) :name :rleg)
	  ))
     (send *ri* :wait-interpolation)

     ;; ;; 股関節pitchの調整
     ;; (send *robot* :rleg-crotch-p :joint-angle -45)
     ;; (send *robot* :lleg-crotch-p :joint-angle -45)
     ;; (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
     ;; (send *ri* :wait-interpolation)
     )

   ;; cnoidの初期設定の誤差を回避
   ;; (send *robot* :torso-waist-p :joint-angle 2)
  
   ;; 右腕の調整
   (send *robot* :rarm-shoulder-p :joint-angle 0)
   (send *robot* :rarm-shoulder-r :joint-angle -45)
   (send *robot* :rarm-shoulder-y :joint-angle 0)
   (send *robot* :rarm-elbow-p :joint-angle -100)
   (send *robot* :rarm-wrist-y :joint-angle 45) ;; wrist-y = 90 + shoulder-r
   (send *robot* :rarm-wrist-p :joint-angle 0)
   (send *robot* :rarm-wrist-r :joint-angle -25)
   (send *robot* :rarm :move-end-pos #f(50 0 0) :world) ;; 特異点を回避するための調整 
   ;; 左腕の調整
   (send *robot* :larm-shoulder-r :joint-angle 20)
   (send *robot* :larm-shoulder-p :joint-angle 0)
   (send *robot* :larm-elbow-p :joint-angle 0)

   (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
   (send *ri* :wait-interpolation)
   )

  ;; パラメタの更新
  (:update-param
   (&key(new-target-z-goal nil) (new-target-pitch-goal nil) (new-motion-period nil) (new-scale-period nil))
   
   (send self :target-z-start (elt (send (send *robot* :rarm :end-coords :copy-worldcoords) :worldpos) 2))
   (send self :target-pitch-start
	 (elt (coerce (car (send (send *robot* :rarm :end-coords :copy-worldcoords) :rpy-angle)) float-vector) 1))
   (send self :target-z-goal new-target-z-goal)
   (send self :target-pitch-goal new-target-pitch-goal)
   (send self :motion-period new-motion-period)
   (send self :scale-period new-scale-period)
   nil
   )

  (:make-target-hand-coords
   (&key (fb nil)) ;; fb:nil -> フィードフォワードに軌道を生成し続ける 
   (send self :make-target-hand-pos :fb fb)
   (send self :make-target-hand-rpy :fb fb)
   (send self :target-hand-coords (make-coords :pos (send self :target-hand-pos)
					       :rpy (send self :target-hand-rpy)))
   )
  
  (:make-target-hand-pos
   (&key (fb nil))
   ;; z = a*cos(bt + c) + d
   ;; z = 0.5*(target-z-start - target-z-goal) * cos ((2pi/motion-period)*t + 0.0)
   ;;     + (target-z-start + target-z-goal) / 2.0)
   (let* ((a (* 0.50 (- (send self :target-z-start) (send self :target-z-goal))))
	  (b (/ (* 2 pi) (send self :motion-period)))
	  (c 0.0)
	  (d (/ (+ (send self :target-z-start) (send self :target-z-goal)) 2.0))
	  (rarm-end-coords (send *robot* :rarm :end-coords :copy-worldcoords))
	  )
     ;; (send self :target-hand-pos hand-pos)
     (setf (aref target-hand-pos 0) (elt (send rarm-end-coords :pos) 0)
     	   (aref target-hand-pos 1) (elt (send rarm-end-coords :pos) 1)
     	   (aref target-hand-pos 2) (+ (* a (cos (+ (* b time) c))) d))
     )
   )
   
  
  (:make-target-hand-rpy
   (&key (fb nil))
   ;; pitch = a*cos(bt + c) + d
   ;; pitch = 0.5*(target-pitch-goal - target-pitch-start) * cos ((2pi/motion-period)*t + pi)
   ;;         + (target-pitch-start + target-pitch-goal)) / 2.0
   (let* ((a (* 0.50 (- (send self :target-pitch-goal) (send self :target-pitch-start))))
	  (b (/ (* 2 pi) (send self :motion-period)))
	  (c pi)
	  (d (/ (+ (send self :target-pitch-start) (send self :target-pitch-goal)) 2.0))
	  (rarm-end-coords (send *robot* :rarm :end-coords :copy-worldcoords))
	  )
     ;; (send self :target-hand-rpy hand-rpy)
     (setf (aref target-hand-rpy 0) (elt (car (send rarm-end-coords :rpy-angle)) 0)
     	   (aref target-hand-rpy 1) (+ (* a (cos (+ (* b time) c))) d)
     	   (aref target-hand-rpy 2) (elt (car (send rarm-end-coords :rpy-angle)) 2)
     	   )
     )
   )
  
  (:solve-ik
   (&key(translation-axis t) (rotation-axis t))
   (when (eq (send self :target-hand-coords) nil) (send self :make-target-hand-coords))
   (send *robot* :rarm :inverse-kinematics (send self :target-hand-coords)
	 :thre 0.01 ;; 位置誤差のthreshold(デフォルトは1[mm])
	 :rthre 0.01 ;; 回転誤差のthreshold(デフォルトは0.017453[rad]=1[deg])
	 ;; null spaceを減らす
	 :additional-weight-list additional-weight-list
	 ;; :translation-axis translation-axis
	 ;; :rotation-axis rotation-axis
	 :translation-axis t
	 :rotation-axis t
	 )
   )

  (:dribble-mode
   (&key (restart nil)) ;; 最初の一回目はnil、以降はt
   ;; (send self :dribble-pose :restart restart)
   (send self :update-param)
   (send self :time 0.0)
   (setq joint-states (instance sensor_msgs::JointState :init))
   (send joint-states :name *joint-name-list*)
   (format t "dribble-mode~%")
   )

  ;; mainの部分
  (:motion-cb (event)
   ;; パラメタの更新
   ;; (when update (send self :update-param))
	      
   ;; stampのセット
   ;; (send joint-states :header :stamp (ros::time-now))

   ;; angle-vectorのセット
   ;; deg2radする必要がある <-- rtc側でやる
   ;; (send joint-states :position (send *robot* :angle-vector))
   ;; debug用
   ;; (ros::ros-info "JointAngle [~A]" (send joint-states :position))

   ;; publish
   (ros::publish "basketball_joint_states" *joint-states*)
	     
   ;; 時間の更新
   ;; (send self :time (+ time dt))
   ;; 目標ハンド座標の生成
   ;; (send self :make-target-hand-coords)
   ;; 目標関節角度の生成
   ;; (send self :solve-ik)
   )

  (:start-motion nil
   (ros::rate 500)
   (while (ros::ok)
   ;; (do-until-key-w/o-gc
     ;; パラメタの更新
     ;; (when update (send self :update-param))
     
     ;; stampのセット
     ;; (send joint-states :header :stamp (ros::time-now))

     ;; angle-vectorのセット
     ;; deg2radする必要がある <-- rtc側でやる
     ;; (send joint-states :position (send *robot* :angle-vector))
     ;; (send joint-states :position
     ;; 	   (map float-vector #'(lambda (x) (deg2rad x)) (send *robot* :angle-vector)))
     
     ;; debug用
     ;; (ros::ros-info "JointAngle [~A]" (send joint-states :position))

     ;; publish
     (ros::publish "basketball_joint_states" *joint-states*)

     ;; 時間の更新
     ;; (send self :time (+ time dt))
     ;; 目標ハンド座標の生成
     ;; (send self :make-target-hand-coords)
     ;; 目標関節角度の生成
     ;; (send self :solve-ik)
     (ros::sleep)
     )     
   )

  (:start-motion2 nil
   (ros::create-timer 0.0020 #'send self :motion-cb)
   (ros::rate 500)		 
   (ros::spin)
   )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  (:update-param2
   (&key(new-target-z-goal nil) (new-target-pitch-goal nil) (new-motion-period nil) (new-scale-period nil))
   
   (send *robot* :angle-vector (car (last (send self :target-angle-vector-list))))
   (send self :ref-pose (send *robot* :angle-vector))
   
   (send self :target-z-start (elt (send (send *robot* :rarm :end-coords :copy-worldcoords) :worldpos) 2))
   (send self :target-pitch-start
	 (elt (coerce (car (send (send *robot* :rarm :end-coords :copy-worldcoords) :rpy-angle)) float-vector) 1))
   (send self :target-z-goal new-target-z-goal)
   (send self :target-pitch-goal new-target-pitch-goal)
   (send self :motion-period new-motion-period)
   (send self :scale-period new-scale-period)
   nil
   )
 
  ;; angle-vector-sequence-full用timeリストの作成
  (:make-dt-list ()
   (send self :dt-list (make-list (round (/ (* (send self :motion-period) (send self :scale-period)) 2.0)) :initial-element 2.0))
   )
  
  ;; 目標ハンド座標軌道の作成
  (:make-target-hand-coords-list ()
   ;; posとrpyの作成			 
   (send self :make-target-hand-pos-list)
   (send self :make-target-hand-rpy-list)
   
   (let* ((hand-coords nil) (hand-coords-list nil))
     (dotimes (i (length (send self :dt-list)))
       (setq hand-coords (make-coords :pos (elt (send self :target-hand-pos-list) i)
				      :rpy (elt (send self :target-hand-rpy-list) i)))
       (push hand-coords hand-coords-list)
       )
     (nreverse hand-coords-list)
     (send self :target-hand-coords-list hand-coords-list)
     )
   nil
   )

  ;; 目標ハンド位置軌道の作成
  (:make-target-hand-pos-list ()
   ;; dt-listの作成
   (when (eq (send self :dt-list) nil) (send self :make-dt-list))
   ;; z = a*cos(bt + c) + d
   ;; z = 0.5*(target-z-start - target-z-goal) * cos ((2pi/motion-period)*t + 0.0)
   ;;     + (target-z-start + target-z-goal) / 2.0)
   (let* ((a (* 0.50 (- (send self :target-z-start) (send self :target-z-goal))))
	  (b (/ (* 2 pi) (send self :motion-period)))
	  (c 0.0) (d (/ (+ (send self :target-z-start) (send self :target-z-goal)) 2.0))
	  (hand-pos nil) (hand-pos-list nil)
	  (rarm-end-coords (send *robot* :rarm :end-coords :copy-worldcoords)))
     (dotimes (i (length (send self :dt-list)))
       ;; todo: x,yをどう与えるかを考える
       ;; とりあえず基準姿勢と同じにしておく
       (setq hand-pos (float-vector (elt (send rarm-end-coords :pos) 0)
				    (elt (send rarm-end-coords :pos) 1)
				    (+ (* a (cos (+ (* b (* dt i)) c))) d)))
       (push hand-pos hand-pos-list)
       )
     (nreverse hand-pos-list)
     (send self :target-hand-pos-list hand-pos-list)
     )
   )
  
  (:make-target-hand-rpy-list ()
   ;; dt-listの作成
   (when (eq (send self :dt-list) nil) (send self :make-dt-list))   
   ;; pitch = a*cos(bt + c) + d
   ;; pitch = 0.5*(target-pitch-goal - target-pitch-start) * cos ((2pi/motion-period)*t + pi)
   ;;         + (target-pitch-start + target-pitch-goal)) / 2.0
   (let* ((a (* 0.50 (- (send self :target-pitch-goal) (send self :target-pitch-start))))
	  (b (/ (* 2 pi) (send self :motion-period)))
	  (c pi) (d (/ (+ (send self :target-pitch-start) (send self :target-pitch-goal)) 2.0))
	  (hand-rpy nil) (hand-rpy-list nil)
	  (rarm-end-coords (send *robot* :rarm :end-coords :copy-worldcoords)))
     (dotimes (i (length (send self :dt-list)))
       (setq hand-rpy (float-vector (elt (car (send rarm-end-coords :rpy-angle)) 0)
				    (+ (* a (cos (+ (* b (* dt i)) c))) d)
				    (elt (car (send rarm-end-coords :rpy-angle)) 2))
	     )
       (push hand-rpy hand-rpy-list)
       )
     (nreverse hand-rpy-list)
     (send self :target-hand-rpy-list hand-rpy-list)
     )
   )
  
  (:solve-ik2
   (&key(translation-axis t) (rotation-axis t))
   ;; hand-coordsの作成
   (when (eq (send self :target-hand-coords-list) nil) (send self :make-target-hand-coords-list))
   
   (let* ((angle-vector-list nil) (hand-pos-list nil) (hand-rpy-list nil))
     (dotimes (i (length (send self :dt-list)))
       (send *robot* :rarm :inverse-kinematics (elt (send self :target-hand-coords-list) i)
	     ;; :thre 1 ;; 位置誤差のthreshold(デフォルトは1[mm])
	     :thre 0.5 ;; 位置誤差のthreshold(デフォルトは1[mm])
	     :rthre 0.01 ;; 回転誤差のthreshold(デフォルトは0.017453[rad]=1[deg])
	     ;; null spaceを減らす
	     :additional-weight-list (list
				      (list (send *robot* :rarm-collar-y :child-link) 0.1)
				      (list (send *robot* :rarm-shoulder-r :child-link) 0.01)
				      (list (send *robot* :rarm-shoulder-y :child-link) 0.1)
				      ;; (list (send *robot* :rarm-wrist-y :child-link) 0.1)
				      )
	     :translation-axis translation-axis
	     :rotation-axis rotation-axis) 
       (push (send *robot* :angle-vector) angle-vector-list)
       (push (send (send *robot* :rarm :end-coords :copy-worldcoords) :worldpos) hand-pos-list)
       (push (car (send (send *robot* :rarm :end-coords :copy-worldcoords) :rpy-angle)) hand-rpy-list)
       ;; (objects (list *robot* (send *robot* :rarm :end-coords :copy-worldcoords)))
       ;; (read-line)
       )
     (nreverse angle-vector-list)
     (nreverse hand-pos-list)
     (nreverse hand-rpy-list)
     (send self :target-angle-vector-list angle-vector-list)

     ;; plot用
     (setq *generated-hand-pos-list* hand-pos-list
	   *generated-hand-rpy-list* hand-rpy-list)
     )
   nil
   )

  ;; (:add-interpolation-angle-vector
  ;;  (&key)
  ;;  (unless *target-angle-vector-list* (format t "no angle-vector-list~%") (return-from :add-interpolation-angle-vector))
   
  ;;  (let* ((av-list (nreverse (progn (setq tmp nil)
  ;; 				    (dotimes (i *interpolation-num*) (push (elt *target-angle-vector-list* i) tmp))
  ;; 				    (nreverse tmp)))) ;; 本当はn番目とn-1番目だけ取れればいい
  ;; 	  (add-av-num (- (* 2 *interpolation-num*) 1))
  ;; 	  (angle_0 *ref-pose*)
  ;; 	  (angle_n (elt av-list (- *interpolation-num* 1)))
  ;; 	  (angle_n-1 (elt av-list (- *interpolation-num* 2)))
  ;; 	  (delta-angle (v- angle_n angle_n-1))
  ;; 	  (delta-angle-sign
  ;; 	   (map float-vector #'(lambda (delta-angle) (cond ((plusp delta-angle) -1) ((minusp delta-angle) 1) (t 0))) delta-angle))
  ;; 	  (add-av-list nil)
  ;; 	  (dt 2.0)
  ;; 	  )

  ;;    (dotimes (i add-av-num)
  ;;      (setq a (map float-vector #'(lambda (x) (cond ((eq x 0.0) 0.0) (t (expt (/ 1.0 x) 2)))) (scale (/ 1.0 dt) delta-angle)))
  ;;      (setq b (map float-vector #'(lambda (x) (- (* (expt *interpolation-num* 2) (* 4 (+ 1.0 x))) (expt (+ i 1) 2))) a))
  ;;      (setq c (map float-vector #'(lambda (x y z) (* (* x y) z)) dt delta-angle-sign b))
  ;;      (format t "delta-angle: ~A~%" (scale (/ 1.0 dt) delta-angle))
  ;;      (format t "a: ~A~%" c)
  ;;      (format t "b: ~A~%" c)
  ;;      (format t "c: ~A~%" c)
  ;;      (format t "dt: ~A~%" dt)
  ;;      (format t "delta-angle-sign: ~A~%" delta-angle-sign)
  ;;      ;; (push (v+ angle_0 c) add-av-list)
  ;;      )
  ;;    (nreverse add-av-list)
  ;;    ;; (format t "angle-vector-list: ~A~%" add-av-list)
  ;;    (setq *target-angle-vector-list* (append add-av-list (cdr *target-angle-vector-list*)))
  ;;    (setq *dt-list* (append (make-list *interpolation-num* :initial-element 2.0) *dt-list*))
  ;;    ) 
  ;;  )

  ;; 軌道チェック
  (:plot
   (&key(idx 21)) ;; 22(=21+1)->rarm-elbow-p
   (format t "==========================================================================~%")
   (let* ((t-z-list (mapcar #'(lambda (x) (elt x 2)) (send self :target-hand-pos-list)))
	  (t-val (car t-z-list))
	  (g-z-list (mapcar #'(lambda (x) (elt x 2)) *generated-hand-pos-list*))
	  (g-val (car g-z-list))
	  (t-r-list (mapcar #'(lambda (x) (elt x 0)) (send self :target-hand-rpy-list)))
	  (t-p-list (mapcar #'(lambda (x) (elt x 1)) (send self :target-hand-rpy-list)))
	  (t-y-list (mapcar #'(lambda (x) (elt x 2)) (send self :target-hand-rpy-list)))
	  (g-r-list (mapcar #'(lambda (x) (elt x 0)) *generated-hand-rpy-list*))
	  (g-p-list (mapcar #'(lambda (x) (elt x 1)) *generated-hand-rpy-list*))
	  (g-y-list (mapcar #'(lambda (x) (elt x 2)) *generated-hand-rpy-list*))
	  (id idx)
	  (angle-val (elt (car (send self :target-angle-vector-list)) idx)) ;; 一番最初のtarget-angle
	  (angle) (vel)
	  )
     ;; hand-pos-z
     ;; 位置
     (unless (boundp '*gp-z*) (setq *gp-z* (gnuplot)))
     (send *gp-z* :draw t-z-list g-z-list)
     ;; 速度(微分)
     (unless (boundp '*gp-dz*) (setq *gp-dz* (gnuplot)))
     (send *gp-dz* :draw
	   (mapcar #'(lambda (cur) (let ((dx)) (setq dx (- cur t-val) t-val cur) dx)) t-z-list)
	   (mapcar #'(lambda (cur) (let ((dx)) (setq dx (- cur g-val) g-val cur) dx)) g-z-list))
     (format t "first-z-vel: ~A~%" (elt (mapcar #'(lambda (cur) (let ((dx)) (setq dx (- cur t-val) t-val cur) dx)) t-z-list) 0))
     ;; hand-rpy
     (unless (boundp '*gp-rpy*) (setq *gp-rpy* (gnuplot)))
     (send *gp-rpy* :draw t-r-list t-p-list t-y-list g-r-list g-p-list g-y-list)
     ;; angle
     (format t "angle-val: ~A~%" angle-val) ;; first-angleと同じになるはず
     (unless (boundp '*gp-dot-theta*) (setq *gp-dot-theta* (gnuplot)))
     (unless (boundp '*gp-theta*) (setq *gp-theta* (gnuplot)))
     (setq angle (mapcar #'(lambda (x) (elt x idx)) (send self :target-angle-vector-list)))
     (setq vel (mapcar #'(lambda (av) (let ((cur (elt av idx)) (dx)) (setq dx (- cur angle-val) angle-val cur) dx)) (send self :target-angle-vector-list)))
     (send *gp-theta* :draw angle)
     (send *gp-dot-theta* :draw vel)
     (format t "first-angle: ~A~%" (elt angle 0))
     (format t "second-angle: ~A~%" (elt angle 1))
     (format t "first-angle-vel: ~A~%" (elt vel 0))
     (format t "second-angle-vel: ~A~%" (elt vel 1))
     )
   (format t "==========================================================================~%")
   )
   

  (:dribble-mode2 ()
   ;; 初期姿勢に遷移
   (send self :dribble-pose :move t :time 5000)
   (send *ri* :wait-interpolation)
   (unix::usleep 1000000)
   
   ;; パラメタの更新
   (send self :update-param)

   ;; チェック
   (format t "==========================================================================~%")
   (format t "dribble-pose-angle-vector: ~A~%" (send *robot* :angle-vector))
   (format t "dribble-pose-end-coords: ~A~%" (send *robot* :rarm :end-coords :copy-worldcoords))
   (setq a (send *robot* :right_hand :angle-vector))
   (setq now-angle-vector (send *robot* :angle-vector))
   (format t "dribble-pose-rhand-angle-vector: ~A~%" (send *robot* :right_hand :angle-vector))
   (format t "target-z-start: ~A~%" (send self :target-z-start))
   (format t "target-pitch-start: ~A~%" (send self :target-pitch-start))
   (format t "==========================================================================~%")

   ;; 関節角度軌道の生成
   (send self :make-dt-list)
   (send self :make-target-hand-coords-list)
   (send self :solve-ik :translation-axis t :rotation-axis t)
   (format t "num: ~A~%" (length (send self :target-angle-vector-list)))
   (ri2robot)

   ;; チェック
   (format t "==========================================================================~%")
   (format t "first-target-angle-vector: ~A~%" (elt (send self :target-angle-vector-list) 0))
   (format t "first-target-hand-coords: ~A~%" (elt (send self :target-hand-coords-list) 0))
   (send *robot* :angle-vector (elt (send self :target-angle-vector-list) 0))
   (setq b (send *robot* :right_hand :angle-vector))
   (setq target-angle-vector (send *robot* :angle-vector))
   (format t "first-target-pose-rhand-angle-vector: ~A~%" (send *robot* :right_hand :angle-vector))
   (format t "diff hand-angle-vector: ~A~%" (v- b a))
   (format t "diff angle-vector1: ~A~%" (v- (elt (send self :target-angle-vector-list) 0)
					    now-angle-vector))
   (format t "diff angle-vector2: ~A~%" (v- (elt (send self :target-angle-vector-list) 1)
					    (elt (send self :target-angle-vector-list) 0)))
   (format t "target-z-start: ~A~%" (send self :target-z-start))
   (format t "target-pitch-start: ~A~%" (send self :target-pitch-start))
   (format t "==========================================================================~%")
       
   ;; plot
   (send self :plot)

   ;; 補間モードの設定
   ;; dtを小さくするとどっちにしてもあまり変わらない
   ;; (send *ri* :set-interpolation-mode :linear)
   ;; (send *ri* :set-interpolation-mode :hoffarbib)
   
   (format t "status: dribble-mode~%")
   )
   
  ;; (:start-motion3
  ;;  (&key(n 3) (log t) (check nil))
  ;;  (unless (send self :target-angle-vector-list)
  ;;    (format t "no angle-vector-list~%")
  ;;    (return-from :start-motion3))

  ;;  (when log (send *ri* :start-log))
   
  ;;  ;; ドリブル開始
  ;;  (format t "PRESS ENTER~%")
  ;;  (read-line)
   
  ;;  (dotimes (num n)
  ;;    ;; 目標関節角度を送る
  ;;    ;; ;; interpolationのほうが先になるように書くこと
  ;;    (send *ri* :wait-interpolation-seq)
  ;;    (bench (send *ri* :angle-vector-sequence-full (send self :target-angle-vector-list) (send self :dt-list)))

     
  ;;    ;; ボール予測位置の取得
  ;;    ;; (ros:spin-once)
  ;;    ;; (send self :target-z-start (send (send object-state :point) :z)) ;; とりあえず最高点で叩くようにする

  ;;    ;; パラメタの更新
  ;;    (bench (send self :update-param))
     
  ;;    ;; ;; チェック
  ;;    ;; (when check
  ;;    ;;   (format t "last-target-angle-vector: ~A~%" (elt (send self :target-angle-vector-list)
  ;;    ;; 						       (- (length (send self :target-angle-vector-list)) 1)))
  ;;    ;;   (format t "now-angle-vector: ~A~%" (send *robot* :angle-vector))
  ;;    ;;   (format t "diff: ~A~%" (v- (elt (send self :target-angle-vector-list) (- (length (send self :target-angle-vector-list)) 1))
  ;;    ;; 				  (send *robot* :angle-vector)))
  ;;    ;;   (format t "now-hand-coords: ~A~%" (send *robot* :rarm :end-coords :copy-worldcoords))
  ;;    ;;   (format t "target-z-start: ~A~%" (send self :target-z-start))
  ;;    ;;   (format t "target-pitch-start: ~A~%" (send self :target-pitch-start))
  ;;    ;;   )
      
  ;;    ;; 次の軌道の生成
  ;;    (bench (send self :make-dt-list))
  ;;    (bench (send self :make-target-hand-coords-list))
  ;;    (bench (send self :solve-ik :translation-axis t :rotation-axis t))
     
  ;;    ;; ;; チェック
  ;;    ;; (when check
  ;;    ;;   (format t "first-target-angle-vector: ~A~%" (elt (send self :target-angle-vector-list) 0))
  ;;    ;;   (format t "first-target-hand-coords: ~A~%" (elt (send self :target-hand-coords-list) 0))
  ;;    ;;   (send *robot* :angle-vector (elt (send self :target-angle-vector-list) 0))
  ;;    ;;   (format t "first-target-pose-rhand-angle-vector: ~A~%" (send *robot* :right_hand :angle-vector))
  ;;    ;;   (format t "target-z-start: ~A~%" (send self :target-z-start))
  ;;    ;;   (format t "target-pitch-start: ~A~%" (send self :target-pitch-start))

  ;;    ;;   ;; plot
  ;;    ;;   (send self :plot)
  ;;    ;;   (read-line)
  ;;    ;;   )
  ;;    (when check (send self :plot) (read-line))
  ;;    )

  ;;  (when log (save-log))

  ;;  (unix::usleep 100000)
  ;;  (send *ri* :set-interpolation-mode :hoffarbib)
  ;;  (unix::usleep 500000)
  ;;  (send self :dribble-pose :move t :time 5000)

  ;;  (send self :init)
  ;;  )
  )
 
(setq *hand* (instance HandTrajectoryController :init))


;;;; utilis function ;;;;
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

;; (send (send *ri* :get-auto-stabilizer-param) :dq_weight) ;; getのときはidlと同じで_でつなぐ
;; #f(1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 100.0 100.0 100.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0)
;; (send *ri* :set-auto-stabilizer-param :dq-weight #f(1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 100.0 100.0 100.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0)) ;; setのときは-でつなぐ

;; バスケ用の設定にする場合
;; collar-yの重みを上げる + 肘pitchと手首rollの重みを上げる
;; (send *ri* :set-auto-stabilizer-param :dq-weight #f(1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 100.0 100.0 100.0 1.0 1.0 10.0 1.0 1.0 1.0 3.5 1.0 3.5 1.0 10.0 1.0 1.0 1.0 3.5 1.0 3.5 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0))

;; 順番
;; hand(12) or leg(12)
;; CHEST_JOINT0 CHEST_JOINT1 CHEST_JOINT2
;; HEAD_JOINT0 HEAD_JOINT1
;; rarm
;; larm
;; hand(12) or leg(12)
;; defaultではchestにかなり重みがかかっている




(defun start-dribble-mode ()
  (start-ab)
  (start-st)
  (unix::usleep 1000000)
  (start-d-mode)
  (unix::usleep 1000000)
  (start-ee)
  )

(defun stop-dribble-mode ()
  (stop-d-mode)
  (stop-ee)
  )


;; emergency-step-modeをoffにする
;; (send *ri* :set-auto-stabilizer-param :is-emergency-step-mode nil)

(defun move (&key (time 5000) (interpolation t))
  (send *ri* :angle-vector (send *robot* :angle-vector) time nil 0 :min-time (/ time 1000))
  (if interpolation
      (send *ri* :wait-interpolation)
      )
  (send *robot* :angle-vector)
  )

(defun lower-waist (&key (dz 120) (move nil)) ;;単位は[mm]
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :legs :move-end-pos (float-vector 0 0 dz))
  (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (when move 
    (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
    (send *ri* :wait-interpolation))
  (send *robot* :newcoords (make-coords))
  )
