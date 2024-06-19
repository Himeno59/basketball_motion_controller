;; roseus publisher sample code

(ros::roseus "pub") ;; nodeのinit
(ros::advertise "chatter" std_msgs::string 1) ;; topicとmsg型

(ros::rate 500) ;; 周期[Hz] 
(while (ros::ok)
  ;; std_msge::string型のmsgをinit
  (setq msg (instance std_msgs::string :init))
  ;; msgが持つdataという変数に代入
  (send msg :data (format nil "hello world ~a" (send (ros::time-now) :sec-nsec)))

  ;; 出力&ログファイルへの書き込み&rosoutへのpublish
  ;; デバッグに便利
  (ros::ros-info "msg [~A]" (send msg :data))

  ;; topicにmsgをpublish
  (ros::publish "chatter" msg)
  
  (ros::sleep)
  )
