#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import tf
import copy
from geometry_msgs.msg import PoseStamped
from pr2_controllers_msgs.msg import Pr2GripperCommandActionGoal

LR = ["l", "r"]
sgns = [1, -1]
vals = [PoseStamped() for lr in LR]
vals_old = [PoseStamped() for lr in LR]
vals_not_move = [0 for lr in LR]
g_vals = [Pr2GripperCommandActionGoal() for lr in LR]
g_flags = [False, False]# is pickup phase

if __name__ == '__main__':
  rospy.init_node('ScreenPoint2DualArmPose', anonymous=True)
  pubs  = [rospy.Publisher('/master_'+lr+'arm_pose', PoseStamped, queue_size=1) for lr in LR]
  g_pubs  = [rospy.Publisher('/'+lr+'_gripper_controller/gripper_action/goal', Pr2GripperCommandActionGoal, queue_size=1) for lr in LR]
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
    vals[tgtid].pose.position.x = pos[0] + 0.02
    vals[tgtid].pose.position.y = pos[1]
    vals[tgtid].pose.position.z = pos[2]
    vals[tgtid].pose.orientation.x = 0
    vals[tgtid].pose.orientation.y = 0
    vals[tgtid].pose.orientation.z = 0
    vals[tgtid].pose.orientation.w = 1


    for i, (lr, v, v_old, p) in enumerate(zip(LR, vals, vals_old, pubs)):
      if     abs(v.pose.position.x - v_old.pose.position.x) < 0.01 \
         and abs(v.pose.position.y - v_old.pose.position.y) < 0.01\
         and abs(v.pose.position.z - v_old.pose.position.z) < 0.01 :
        vals_not_move[i] += 1
      else :
        vals_not_move[i] = 0
        rospy.loginfo(lr+"arm set as :\n"+str(v.pose.position))

    vals_old = copy.deepcopy(vals)

    for i, (v, v_not_move, p, gp) in enumerate(zip(vals, vals_not_move, pubs, g_pubs)):
      if v_not_move < 1.5*HZ or 4*HZ < v_not_move :
        # hover
        v_mod = copy.deepcopy(v)
        v_mod.pose.position.z += 0.2
        p.publish(v_mod)
        
        if g_flags[i] == True and g_vals[i].goal.command.position == 0:
          g_flags[i] = False
          
        if g_flags[i] == False and g_vals[i].goal.command.position != 0:
          g_flags[i] = True
          
      else :
        # touch
        p.publish(v)
        
        if 2*HZ < v_not_move:
          g_vals[i].goal.command.position = ( 0 if g_flags[i] else 0.1)
          g_vals[i].goal.command.max_effort = 25;
          gp.publish(g_vals[i])



    r.sleep()