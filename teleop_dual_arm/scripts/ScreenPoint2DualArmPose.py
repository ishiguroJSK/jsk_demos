#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import tf
import copy
from geometry_msgs.msg import PoseStamped

keys = ["larm", "rarm"]
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

  while not rospy.is_shutdown():

    stamp = rospy.Time.now()
    try:
      # tfl.waitForTransform("/iiwa_torso_link", "/ray_target", stamp, timeout=rospy.Duration(1))
      (pos, rot) = tfl.lookupTransform("/iiwa_torso_link", "/ray_target", rospy.Time(0))
    except Exception, e:
      rospy.logerr(e)
      continue

    tgtid = 0 if (pos[1]>0) else 1 ## divide center pose into L R
    vals[tgtid].header.frame_id = "iiwa_torso_link"
    vals[tgtid].header.stamp = stamp
    vals[tgtid].pose.position.x = pos[0] + 0.02 ## push obj 0.02m
    vals[tgtid].pose.position.y = pos[1]
    vals[tgtid].pose.position.z = pos[2]
    vals[tgtid].pose.orientation.x = 0
    vals[tgtid].pose.orientation.y = 0
    vals[tgtid].pose.orientation.z = 0
    vals[tgtid].pose.orientation.w = 1


    # for k, v, v_old, v_not_move, p in zip(keys, vals, vals_old, vals_not_move, pubs):
    for i, (k, v, v_old, p) in enumerate(zip(keys, vals, vals_old, pubs)):
      if v.pose.position.x == v_old.pose.position.x and v.pose.position.y == v_old.pose.position.y and v.pose.position.z == v_old.pose.position.z :
        vals_not_move[i] += 1
      else :
        vals_not_move[i] = 0
        rospy.loginfo(k+" set as :\n"+str(v.pose.position))

    vals_old = copy.deepcopy(vals)

    for v, v_not_move, p in zip(vals, vals_not_move, pubs):
      if v_not_move < 0.5*HZ or 1*HZ < v_not_move :
        v_mod = copy.deepcopy(v)
        v_mod.pose.position.x -= 0.1
        p.publish(v_mod)
      else :
        p.publish(v)

    r.sleep()
