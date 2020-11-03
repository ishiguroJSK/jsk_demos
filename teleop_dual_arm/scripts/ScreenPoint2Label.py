#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
# import tf
# import copy
# import math
from std_msgs.msg import Int32
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from struct import *

class ScreenPoint2Label():

  def __init__(self):
    rospy.init_node('ScreenPoint2Label', anonymous=False)
    self.l_sub = rospy.Subscriber("~input_label", Image, self.label_cb)
    self.p_sub = rospy.Subscriber("~input_point", PointStamped, self.point_cb)
    self.pub = rospy.Publisher("~output_label", Int32, queue_size=1)
    rospy.spin()

  def label_cb(self, msg):
    self.label_img = msg

  def point_cb(self, msg):
    if self.label_img:
      STEP_BYTE = 4 # 32C1 = 4Byte
      data_id = int(self.label_img.width * msg.point.y + msg.point.x) * STEP_BYTE
      label_int = unpack('i', self.label_img.data[data_id : data_id + STEP_BYTE])[0] # unpack returns (123, )
      self.pub.publish(label_int)
      rospy.loginfo("The pointed label is " + str(label_int))
    else:
      rospy.logwarn("input_point is given, but input_label has not been received...")


if __name__ == '__main__':
  inst = ScreenPoint2Label()
