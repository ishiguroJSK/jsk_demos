#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

class Label2BoundingBox():

  def __init__(self):
    rospy.init_node('Label2BoundingBox', anonymous=False)
    self.l_sub = rospy.Subscriber("~input_label", Int32, self.label_cb)
    self.b_sub = rospy.Subscriber("~input_boxes", BoundingBoxArray, self.boxes_cb)
    self.b_pub = rospy.Publisher("~output_box", BoundingBox, queue_size=1)
    self.p_pub = rospy.Publisher("~output_pose", PoseStamped, queue_size=1)
    rospy.spin()

  def boxes_cb(self, msg):
    self.bba = msg

  def label_cb(self, msg):
    if self.bba:
      for bb in self.bba.boxes:
        if bb.label == msg.data - 1:
          self.b_pub.publish(bb)
          self.p_pub.publish(PoseStamped(header=self.bba.header, pose=bb.pose))
          # rospy.loginfo("The pointed label is " + str(label_int))
          break
    else:
      rospy.logwarn("input_point is given, but input_label has not been received...")


if __name__ == '__main__':
  inst = Label2BoundingBox()
