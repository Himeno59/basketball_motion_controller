#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np

class DummyPos():
    def __init__(self):
        rospy.init_node('dummy_pos_node', anonymous=True)
        self.pos_pub = rospy.Publisher('/dummy_pos', PointStamped, queue_size=1)
        self.pos_pred_pub = rospy.Publisher('/dummy_pos_pred', PointStamped, queue_size=1)

        self.t = 0.0
        
        self.x = 1.0
        self.y = 1.0
        self.z = 0.0
        self.z_pred = 0.0

        tmp_time = rospy.Time.now()
        
        self.pos = PointStamped()
        self.pos.header.frame_id = "map"
        self.pos.header.stamp = tmp_time

        self.pos_pred = PointStamped()
        self.pos_pred.header.frame_id = "map"
        self.pos_pred.header.stamp = tmp_time

        self.last_time = tmp_time
        self.pred_time = 0.25
        self.rate = rospy.Rate(80)
        
    def pub(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            self.pos.header.stamp = current_time
            self.pos_pred.header.stamp = current_time
            
            self.pos.point.x = self.x
            self.pos.point.y = self.y
            self.pos.point.z = self.z
            
            self.pos_pred.point.x = self.x
            self.pos_pred.point.y = self.y
            self.pos_pred.point.z = self.z_pred
            
            self.pos_pub.publish(self.pos)
            self.pos_pred_pub.publish(self.pos_pred)
            
            if self.z < 0:
                self.t = 0.0
            
            self.t += dt
            noise = np.random.normal(0, 0.01);
            # self.z = - 4.9 * self.t * (self.t - 0.8) + noise
            # self.z_pred = -4.9 * (self.t + self.pred_time) * (self.t - 0.8 + self.pred_time) + noise
            self.z = - 4.9 * self.t * (self.t - 0.8)
            self.z_pred = -4.9 * (self.t + self.pred_time) * (self.t - 0.8 + self.pred_time)
            self.last_time = current_time
            self.rate.sleep()
        
if __name__ == '__main__':
    try:
        dummy_pos_node = DummyPos()
        dummy_pos_node.pub()
    except rospy.ROSInterruptException:
        pass
