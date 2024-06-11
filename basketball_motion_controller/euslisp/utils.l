;; jaxon用個人ファイル

;; methodの仕様 = robot-interface.l
;; msl_hand_controller/euslisp/jaxon_red_mslhand-interface.l
;; -> hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l 
;; -> /opt/ros/melodic/share/pr2eus/robot-interface.l

#|

 1. auto-balancer/stabilizer
 2. angle-vector関連
 3. 基本姿勢
 4. 基本動作
 5. 制御・パラメタ設定
 6. 未整理

|#
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;このファイルのロード
(defun load-utils () (load "~/prog/jaxon-prog/euslisp/utils.l"))

;; 1. auto-balancer/stabilizer ;;
;; auto-balancer
(defun start-ab () (send *ri* :start-auto-balancer))
(defun stop-ab () (send *ri* :stop-auto-balancer))

;; stabilizer
(defun start-st () (send *ri* :start-st))
(defun stop-st () (send *ri* :stop-st))

;; emergency-step-modeをoffにする
;; (send *ri* :set-auto-stabilizer-param :is-emergency-step-mode nil)

;; 2. angle-vector関連
;; モデル(*robot*)の状態を実機に反映
(defun move (&key (time 5000) (interpolation t))
  (send *ri* :angle-vector (send *robot* :angle-vector) time nil 0 :min-time (/ time 1000))
  (if interpolation
      (send *ri* :wait-interpolation)
      )
  (send *robot* :angle-vector)
  )

;; ;; ハンドのみを動かす
;; ;; これをやると今後full-bodyで送った時にハンドが動かなくなるので基本的にはやらない
;; (defun move-rhand (&key(time 5000))
;;   (send *ri* :angle-vector (send *robot* :angle-vector) time :right_hand-controller)
;;   (send *ri* :wait-interpolation)
;;   )

;; (defun move-lhand (&key(time 5000))
;;   (send *ri* :angle-vector (send *robot* :angle-vector) time :left_hand-controller)
;;   (send *ri* :wait-interpolation)
;;   )

;; 3,基本姿勢
;; reset-pose
(defun reset-pose (&key(move nil) (time 10000))
  (send *robot* :reset-pose)
  (when move (move :time time))
  (send *robot* :angle-vector)
  )

;; reset-manip-pose
(defun reset-manip-pose (&key(move nil) (time 10000))
  (send *robot* :reset-manip-pose)
  (when move (move :time time))
  (send *robot* :angle-vector)
  )

;; 4,基本動作
;; 歩行
;; (send *ri* :go-pos x y th) ;; thは[deg]

;; 5,制御・パラメタ設定
;; 補間の設定 SequencePlayerのサービスを呼ぶ
(defun set-linear () (send *ri* :set-interpolation-mode :linear)) ;; 線形補間
(defun set-hoffarbib () (send *ri* :set-interpolation-mode :hoffarbib)) ;; hoffarbib

;; 位置制御ゲインの調整
#|
<R/LARM_JOINT>
0: collar-y, 1: shoulder-p, 2: shoulder-r, 3: shoulder-y, 4: elbow-p
5: wrist-y,  6: wrist-r,    7: wrist-p
|#

;; 関数の引数にrarmとかを入れてメソッドの引数に与えたい場合は頭に'をつけてシンボルとして渡す
;; :~~を関数の引数として入れたかったら'~~と書けばいい
(defun set-servoPgain (j_name p tm) ;; p = percentage, t[msec] ;; 'RARM_JOINT6 100 1000
  (send *ri* :robothardwareservice_setservopgainpercentagewithtime
	:name j_name :percentage p :time (/ tm 1000.0))
  )

(defun set-servoDgain (j_name p tm)
  (send *ri* :robothardwareservice_setservodgainpercentagewithtime
	:name j_name :percentage p :time (/ tm 1000.0))
  )

(defun set-limb-servoPDgain (limb_name p tm) ;; "rarm" 100 1000 
  (let ((j-list nil)
  	(j_name nil))

    ;; set joint list
    (cond ((equal limb_name "rarm")
	   (setq j-list (send *robot* :rarm :joint-list)))
	  ((equal limb_name "larm")
	   (setq j-list (send *robot* :larm :joint-list)))
	  ((equal limb_name "rleg")
	   (setq j-list (send *robot* :rleg :joint-list)))
	  ((equal limb_name "lleg")
	   (setq j-list (send *robot* :lleg :joint-list)))
	  )
	  
    (dolist (j j-list)
      ;; set joint name
      (setq j_name (send j :name))
      
      ;; set p/d-gain
      (send *ri* :robothardwareservice_setservopgainpercentagewithtime
  	    :name j_name :percentage p :time (/ tm 1000.0))
      (send *ri* :robothardwareservice_setservodgainpercentagewithtime
  	    :name j_name :percentage p :time (/ tm 1000.0))
      )
    )
  )


;; 6,未整理
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;制御モード
;;ImpedanceControllerとTorqueControllerがある
;;rtm-ros-robotics/rtmmros_common/hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.lに詳細あり

;;オフセット
(defun rmfo-arms nil
  (send *ri* :remove-force-sensor-offset-rmfo-arms)
  )

;; インピーダンス制御
;; インピーダンス制御の設定。K(バネ)D(ダンパ)M(慣性)、r(回転)、p(並進)
(defun set-impedance-params ()
  ;; (send *ri* :set-impedance-controller-param :arms :K-r 200.0 :D-r 300.0  :M-r 1
  ;;                                                  :K-p 500.0 :D-p 1000.0 :M-p 5)
  (send *ri* :set-impedance-controller-param :arms :K-r 200.0 :D-r 300.0  :M-r 1
                                                   :K-p 150.0 :D-p 100.0 :M-p 5)
  )

;; start/stop
(defun start-impedance () (send *ri* :start-impedance :arms))
(defun stop-impedance () (send *ri* :stop-impedance :arms))

;; ;; かかる参照力の設定
;; ;; インピーダンス制御を入れている場合、設定した反力を受けるまで力をかけるようになる
;; ;; まずは挟む力をかける
;; (send *ri* :set-ref-forces
;;       (list
;;        (float-vector 0 0 0) ;;rleg
;;        (float-vector 0 0 0) ;;lleg
;;        (float-vector 0 -100  0) ;;rarm (x, y, z) 単位はニュートン
;;        (float-vector 0 100   0) ;;larm
;;        )
;;        3000)
;; (unix:sleep 5) ;;指定した時間(秒)処理を止める

;; ;; トルク制御
;; (defun set-torque-param () (send *ri* :set-reference-torque ))
;; (defun start-toruque () (send *robot* :start-toruque-control :arms))
;; (defun stop-toruque () (send *robot* :stop-toruque-control :arms))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Other

;; print文デバッグ
(defun debug-point ()
  (format t "checkpoint~%")
  (read-line)
  )

;; [msec]でのストップ
(defun msec-sleep (msec) (unix:usleep (* msec 1000)))

;; irtviewer
;; 背景の色を変える
;; (send *irtviewer* :change-background #f(0 0 0))


;; 座標関連
;; ルートリンクに座標を合わせる
;; (send *robot* :newcoords (make-coords))

;; 両足裏中心に座標を合わせる
;; (send *robot* :fix-leg-to-coords (make-coords)))

;; 実機(*ri*)の状態をモデル(*robot*)に反映
(defun ri2robot ()
  (let ((av (send *ri* :state :potentio-vector)))
    (send *robot* :angle-vector av)
    ;; (send *robot* :newcoords (make-coords))
    )
  )

;; 重心を両足の中心に持ってくる
;; (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))

;; coordsの確認
(defun coords-check (pos rpy)
  ;; 引数は
  ;; (float-vector 0 0 0) (float-vector (deg2rad 0) (deg2rad 0) (deg2rad 0))
  ;; のように入れる
  (setq *crds* (make-coords :pos pos :rpy rpy))
  (objects (append (objects) *crds*))
  )

;; 両腕のik
(defun solve-ik
    (&key
       (rarm-target (send *robot* :rarm :end-coords :copy-worldcoords))
       (larm-target (send *robot* :larm :end-coords :copy-worldcoords))
       (translation-axis (list t t))
       (rotation-axis (list nil nil))
       )
		   
  ;; target-coordsを表示
  (setq *rarm-target* (send (send rarm-target :copy-worldcoords) :scale 5))
  (setq *larm-target* (send (send larm-target :copy-worldcoords) :scale 5))
  (objects (list *robot* *rarm-target* *larm-target*))
  (send *irtviewer* :draw-objects :flush t)
  
  (let* ((rarm-coords (send *robot* :rarm :end-coords :copy-worldcoords))
	 (larm-coords (send *robot* :larm :end-coords :copy-worldcoords))
	 (new-angle-vector nil)
	 )
    (setq new-angle-vector
	  (send *robot* :inverse-kinematics
		(list rarm-target larm-target)
		:move-target (list (send *robot* :rarm :end-coords)
				   (send *robot* :larm :end-coords))
		:link-list (list (send *robot* :link-list
				       (send *robot* :rarm :end-coords :parent))
				 (send *robot* :link-list
				       (send *robot* :larm :end-coords :parent)))
		:translation-axis translation-axis
		:rotation-axis rotation-axis
		)
	  )  
    )
  )

;; 世界座標系での対象物の位置・姿勢を計算
(defun world-to-target (target)
  (let target-coords
    ;; ikを解く前に*robot*に実機の姿勢を反映
    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (send *robot* :newcoords (make-coords))
    (setq target-coords
	  (send (send *robot* :rs_l515_depth_optical_frame :copy-worldcoords) :transform target))
    target-coords)
  )

;; 屋外で実験・デモする時に空中で浮かせておく時の姿勢
(defun rmfo-pose (&key (dz 380) (move nil) (time 10000))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :legs :move-end-pos (float-vector 0 0 dz))
  (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (when move (move :time time))
  (send *robot* :newcoords (make-coords))
  )

;; 紐を握った状態の歩行姿勢
(defun walking-pose (&key (dz 120) (move nil) (time 10000))
  (send *robot* :reset-manip-pose)
  (lower-waist :dz dz)
  (grasp-r)
  (grasp-l)
  (when move (move :time time))
  (send *robot* :newcoords (make-coords))
  )

;; しゃがむ
(defun lower-waist (&key (dz 120) (move nil)) ;;単位は[mm]
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :legs :move-end-pos (float-vector 0 0 dz))
  (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (when move 
    (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
    (send *ri* :wait-interpolation))
  (send *robot* :newcoords (make-coords))
  )

;; y->t n->nil と評価される関数
;; (y-or-n-p "[表示させたい文章]")
;; ex.) (unless (y-or-n-p "OK? -> ") (retrun-from [function-name]))

;; リストの要素全てに対して同じ処理を刷る
;; (mapcar '(lambda (x) ([行う処理])) [list])
;; ex.) (setq target-coords (mapcar '(lambda (x) (world-to-target (ros::tf-pose->coords (send x :pose)))) [list]))

;; ;; target-coordsの描画
;; (setq *rarm-target*
;;       (send (send *robot* :rarm :end-coords :copy-worldcoords) :scale 5))
;; (setq *larm-target*
;;       (send (send *robot* :larm :end-coords :copy-worldcoords) :scale 5))
;; (objects (list *robot* *rarm-target* *larm-target*))
