;; roseus subscriber sample code

;; (ros::subscrive ~)のところを変更して使う

(ros::roseus "sub") ;; nodeのinit

;; call-back関数
(defun test-cb (msg) (print msg)) ;; (ros::spin-once)の際に呼ばれる

(ros::subscribe *topic-name* ;; subscribeするtopicの名前を指定
		jsk_recognition_msgs::BoundingBoxArray ;; msgの型
		#'test-cb ;; call-back関数を指定
		)

(ros::rate 500) ;; 周期[Hz]
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep) ;; ros::rateで指定した周期になるように待つ
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; クラスで定義する場合

(defclass Subscriber
  :super propertied-object
  :slots (ros-msg)
  )

(defmethod Subscriber
  (:init () (setq ros-msg nil) self)
  
  (:ros-init ()
    (ros::roseus "sub")
    (ros::subscribe *topic-name* ;; subscribeするtopicの名前を指定
		    jsk_recognition_msgs::BoundingBoxArray ;; msgの型
		    #'send self :cb-function ;; call-backメソッドを指定
		    )
   )
  
  (:cb-function (msg)
    (when msg
      (setq ros-msg)
      ;; (print ros-msg)
      )
    )

  (:ros-motion ()
    (while (ros::ok)
      (ros:spin-once)
      ;; ここにsubscribeした結果を元にした動作メソッドとかを入れたりする
      (ros::sleep)
      )
    )
  )

