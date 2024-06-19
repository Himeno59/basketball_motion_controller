#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PointStamped
import numpy as np

class SampleNode:
    def __init__(self):
        rospy.init_node('sample_node', anonymous=True)
        # subscriber
        self.topic_sub = rospy.Subscriber('[topic-name]', PointStamped, self.callback)
        # publisher
        self.topic_pub = rospy.Publisher('[topic-name]', PointStamped, queue_size=1)
        
    def callback(self, msg):
        rospy.loginfo("msg: %s", msg)

        # subscribeしたmsgを処理してpublishするパターンの場合は、callback関数の中にself.topic_pub.publish(~)を入れれば良い
        

if __name__ == '__main__':
    try:
        sample_node = SampleNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
