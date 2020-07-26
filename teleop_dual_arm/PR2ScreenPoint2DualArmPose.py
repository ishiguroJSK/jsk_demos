#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import tf
import copy
from geometry_msgs.msg import PoseStamped

keys = ["larm", "rarm"]
sgns = [1, -1]
vals = [PoseStamped() for k in keys]
vals_old = [PoseStamped() for k in keys]
vals_not_move = [0 for k in keys]

if __name__ == '__main__':
  rospy.init_node('ScreenPoint2DualArmPose', anonymous=True)
  pubs  = [rospy.Publisher('/master_'+k+'_pose', PoseStamped, queue_size=1) for k in keys]
  tfl = tf.TransformListener()
  print "start ROS pub loop"
  HZ = 10
  r = rospy.Rate(HZ)

  # init
  for s, v in zip(sgns, vals):
    v.header.frame_id = "base_link"
    v.pose.position.x = 0.5
    v.pose.position.y = s * 0.3
    v.pose.position.z = 1.0
    v.pose.orientation.x = 0
    v.pose.orientation.y = 0
    v.pose.orientation.z = 0
    v.pose.orientation.w = 1
  
  
  while not rospy.is_shutdown():

    stamp = rospy.Time.now()
    try:
      tfl.waitForTransform("/base_link", "/ray_target", stamp, timeout=rospy.Duration(1))
      (pos, rot) = tfl.lookupTransform("/base_link", "/ray_target", rospy.Time(0))
    except Exception, e:
      rospy.logerr(e)
      continue

    tgtid = 0 if (pos[1]>0) else 1 ## divide center pose into L R
    vals[tgtid].header.frame_id = "base_link"
    vals[tgtid].header.stamp = stamp
    vals[tgtid].pose.position.x = pos[0]
    vals[tgtid].pose.position.y = pos[1]
    vals[tgtid].pose.position.z = pos[2]
    vals[tgtid].pose.orientation.x = 0
    vals[tgtid].pose.orientation.y = 0
    vals[tgtid].pose.orientation.z = 0
    vals[tgtid].pose.orientation.w = 1


    # for k, v, v_old, v_not_move, p in zip(keys, vals, vals_old, vals_not_move, pubs):
    for i, (k, v, v_old, p) in enumerate(zip(keys, vals, vals_old, pubs)):
      if     abs(v.pose.position.x - v_old.pose.position.x) < 0.01 \
         and abs(v.pose.position.y - v_old.pose.position.y) < 0.01\
         and abs(v.pose.position.z - v_old.pose.position.z) < 0.01 :
        vals_not_move[i] += 1
      else :
        vals_not_move[i] = 0
        rospy.loginfo(k+" set as :\n"+str(v.pose.position))

    vals_old = copy.deepcopy(vals)

    for v, v_not_move, p in zip(vals, vals_not_move, pubs):
      if v_not_move < 1*HZ or 2*HZ < v_not_move :
        # hover
        v_mod = copy.deepcopy(v)
        v_mod.pose.position.z += 0.2
        p.publish(v_mod)
      else :
        # touch
        p.publish(v)

    r.sleep()
