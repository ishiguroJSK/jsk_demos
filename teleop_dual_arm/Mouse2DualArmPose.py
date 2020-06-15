#!/usr/bin/python
# -*- coding: utf-8 -*-
# import signal
import rospy
import copy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

keys = ["larm", "rarm"]
sign = [1,-1]
vals = [PoseStamped() for k in keys]
center_pose = PoseStamped()
go_pick = False
  
def joy_cb(data):
  global center_pose, go_pick
  center_pose.pose.position.x += data.axes[1] * 0.001
  center_pose.pose.position.y -= data.axes[0] * 0.001
  if data.buttons[0] == 1:
    go_pick = True
  else:
    go_pick = False

if __name__ == '__main__':
  # signal.signal(signal.SIGINT, signal.SIG_DFL)
  sub = rospy.Subscriber("/mouse/joy", Joy, joy_cb)
  pubs = [rospy.Publisher('/master_'+k+'_pose', PoseStamped, queue_size=1) for k in keys]
  rospy.init_node('Joy2PoseStamped', anonymous=True)
  print "start ROS pub loop"
  r = rospy.Rate(10)
  open_width = 0.0
  pick_z = 0.0
  pick_wait_count = 0
  PICK_W = -0.03
  RELEASE_W = 0.2
  PICK_H = -0.055
  RELEASE_H = 0.1
  while not rospy.is_shutdown():

    if go_pick: ### go down
      pick_z = PICK_H
      open_width -= 0.01
      if open_width < PICK_W: ### wait pick complete
        pick_wait_count += 1
        open_width = PICK_W
        if pick_wait_count > 10: ### pick up
          pick_z = RELEASE_H
    else:
      pick_z      = RELEASE_H
      open_width  = RELEASE_W
      pick_wait_count = 0


    for s, v, p in zip(sign, vals, pubs):
      v.pose = copy.deepcopy(center_pose.pose)
      v.pose.position.y += s * open_width / 2.0
      v.pose.position.z += pick_z
      p.publish(v)

    rospy.loginfo_throttle(2,"Open width : "+str(open_width)+" Pick height : "+str(pick_z))
    r.sleep()