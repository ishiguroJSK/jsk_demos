#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import PointCloud2

pub = None

def cb(msg):
  msg.is_dense = True
  pub.publish(msg)


if __name__ == '__main__':
  rospy.init_node('debug')
  pub  = rospy.Publisher("/extract_indices/output2", PointCloud2, queue_size=1)
  sub  = rospy.Subscriber("/extract_indices/output", PointCloud2, cb)
  rospy.spin()
