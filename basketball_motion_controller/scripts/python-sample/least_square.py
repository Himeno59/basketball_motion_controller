#!/usr/bin/env python
# -*- coding: utf-8 -*-

# subscribe: 点群の重心
# publish: ボールの重心、ボールの予測重心軌道
# 速度の方はVector3にしたほうがいい?

import time
import rospy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, PoseArray, Vector3, Transform, TransformStamped

from tf2_msgs.msg import TFMessage
import numpy as np

class LeastSquare:
    def __init__(self):
        # node
        rospy.init_node('least_square_node', anonymous=True)
        # subscriber
        self.point_sub = rospy.Subscriber('/dummy_pos', PointStamped, self.calc_cb)
        
        # publisher
        # 現在の位置・速度
        self.now_pos_pub = rospy.Publisher('/ball_detection/now_ball_centroid_pos', PointStamped, queue_size=1)
        self.now_vel_pub = rospy.Publisher('/ball_detection/now_ball_centroid_vel', PointStamped, queue_size=1)
        # n[msec]後の推定した位置・速度
        self.pred_pos_pub = rospy.Publisher('/ball_detection/pred_ball_centroid_pos', PointStamped, queue_size=1)
        self.pred_vel_pub = rospy.Publisher('/ball_detection/pred_ball_centroid_vel', PointStamped, queue_size=1)
        # rviz用
        self.real_trajectory_pub = rospy.Publisher('/ball_detection/real_trajectory_visualization', TFMessage, queue_size=1)
        self.pred_trajectory_pub = rospy.Publisher('/ball_detection/pred_trajectory_visualization', TFMessage, queue_size=1)

        # 初期化
        self.now_pos = PointStamped()
        self.now_vel = PointStamped()
        self.pred_pos = PointStamped()
        self.pred_vel = PointStamped()
        self.real_trajectory = TFMessage()
        self.pred_trajectory = TFMessage()
        
        self.now_pos.header.frame_id = "map"
        self.pred_pos.header.frame_id = "map"

        self.target_time = 0.30 # 地面から跳ね返ってから何秒後に手と接触させたいか

        # 最小二乗法を計算するために地面から跳ね返った後の値を保持
        self.time = np.array([])
        self.X = np.array([])
        self.Y = np.array([])
        self.Z = np.array([])
        self.VX = np.array([])
        self.VY = np.array([])
        self.VZ = np.array([])
        self.n = 3 # 最小二乗法を計算する時に何個値を使うか
        self.hz = 80 # D455の色付き点群のHz

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0
        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_vz = 0.0

    def reset_param(self):
        # 地面に跳ね返ったタイミングでリストを空にする
        self.time = np.array([])
        self.X = np.array([])
        self.Y = np.array([])
        self.Z = np.array([])
        self.VX = np.array([])
        self.VY = np.array([])
        self.VZ = np.array([])
        
    def update_stamp(self, msg):
        # self.current_time = rospy.Time.now() # 遅れがでてくるからダメ
        self.current_time = msg.header.stamp
        self.now_pos.header.stamp = self.current_time
        self.now_vel.header.stamp = self.current_time
        self.pred_pos.header.stamp = self.current_time
        self.pred_vel.header.stamp = self.current_time

    def calc_cb(self, msg):
        self.update_stamp(msg)

        dt = max(1.0/self.hz, (self.current_time - self.last_time).to_sec())
                                
        # 重心位置の計算
        # todo: カメラ相対のままなので、ここでbase_link相対に変換する
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        D = (x**2 + y**2 + z**2)**0.5       
        k = 0.5 * 0.1225 # r=0.1225[m]
        
        self.now_pos.point.x = x + k * x/D
        self.now_pos.point.y = y + k * y/D
        self.now_pos.point.z = z + k * z/D

        # 重心速度の計算
        self.now_vel.point.x = (self.now_pos.point.x - self.prev_x) / dt
        self.now_vel.point.y = (self.now_pos.point.y - self.prev_y) / dt
        self.now_vel.point.z = (self.now_pos.point.z - self.prev_z) / dt
       
        # 地面から跳ね返ったタイミングでリセット
        if self.now_vel.point.z > 0 and self.prev_vz < 0:
            # rospy.loginfo("<------- reset ------->")
            self.reset_param()

        new_time = (self.time[-1] + dt) if len(self.time) > 0 else 0
        self.time = np.append(self.time, new_time)
        self.X = np.append(self.X, self.now_pos.point.x)
        self.Y = np.append(self.Y, self.now_pos.point.y)
        self.Z = np.append(self.Z, self.now_pos.point.z)        
        self.VX = np.append(self.VX, self.now_vel.point.x)
        self.VY = np.append(self.VY, self.now_vel.point.y)
        self.VZ = np.append(self.VZ, self.now_vel.point.z)
        
        # 最小二乗法
        if len(self.time) > self.n:
            # rospy.loginfo("time: %s", new_time)
            # 現在からn個前までの値のみを使って計算
            time = self.time[-self.n:]
            X = self.X[-self.n:]
            Y = self.Y[-self.n:]
            Z = self.Z[-self.n:]
            VX = self.VX[-self.n:]
            VY = self.VY[-self.n:]
            VZ = self.VZ[-self.n:]
        
            # 係数の計算
            coef_x = np.polyfit(time, X, 1)
            coef_y = np.polyfit(time, Y, 1)
            coef_z = np.polyfit(time, Z, 2)
            # coef_vx = np.polyfit(time, VX, 1)
            # coef_vy = np.polyfit(time, VY, 1)
            # coef_vz = np.polyfit(time, VZ, 2)
            
            # 予測位置を計算
            # 今の時間からtarget_time後
            self.pred_pos.point.x = max(0, np.poly1d(coef_x)(self.time[-1]+self.target_time))
            self.pred_pos.point.y = max(0, np.poly1d(coef_y)(self.time[-1]+self.target_time))
            self.pred_pos.point.z = max(0, np.poly1d(coef_z)(self.time[-1]+self.target_time))
            # self.pred_pos.point.x = np.poly1d(coef_x)(self.time[-1])
            # self.pred_pos.point.y = np.poly1d(coef_y)(self.time[-1])
            # self.pred_pos.point.z = np.poly1d(coef_z)(self.time[-1])
            # self.pred_vel.point.x = np.poly1d(coef_vx)(self.target_time)
            # self.pred_vel.point.y = np.poly1d(coef_vy)(self.target_time)
            # self.pred_vel.point.z = np.poly1d(coef_vz)(self.target_time)
             
            # TFMessage用
            time_list = np.arange(0.0, self.target_time, 0.015)
            pred_x = np.poly1d(coef_x)(time_list)
            pred_y = np.poly1d(coef_y)(time_list)
            pred_z = np.poly1d(coef_z)(time_list)
            for i in range(19):
                new_tf = TransformStamped()
                new_tf.header.frame_id = "map"
                # new_tf.header.stamp = 
                new_tf.transform.translation.x = pred_x[i]
                new_tf.transform.translation.y = pred_y[i]
                new_tf.transform.translation.z = pred_z[i]
                self.pred_trajectory.transforms = np.append(self.pred_trajectory.transforms, new_tf)
            # new_tf = np.empty(len(time_list), dtype=TransformStamped())
            # new_tf['transform']['translation']['x'] = pred_x
            # new_tf['transform']['translation']['y'] = pred_y
            # new_tf['transform']['translation']['z'] = pred_z
            # self.pred_trajectory.transforms = new_tf.tolist()
            
        # 値の更新
        self.prev_x = self.X[-1]
        self.prev_y = self.Y[-1]
        self.prev_z = self.Z[-1]
        self.prev_vx = self.VX[-1]
        self.prev_vy = self.VY[-1]
        self.prev_vz = self.VZ[-1]
        self.last_time = self.current_time
        
        # publish
        self.publish()
        
    def publish(self):
        self.now_pos_pub.publish(self.now_pos)
        self.now_vel_pub.publish(self.now_vel)
        self.pred_pos_pub.publish(self.pred_pos)
        self.pred_vel_pub.publish(self.pred_vel)
        self.real_trajectory_pub.publish(self.real_trajectory)
        self.pred_trajectory_pub.publish(self.pred_trajectory)
        
if __name__ == '__main__':
    least_square_node = LeastSquare()
    time.sleep(1.0)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
