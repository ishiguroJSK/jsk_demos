#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import tf
import copy
import math
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import Pr2GripperCommandActionGoal

LR = ["l", "r"]
sgns = [1, -1]
vals = [PoseStamped() for lr in LR]
# vals_old = [PoseStamped() for lr in LR]
# vals_not_move = [0 for lr in LR]
g_vals = [Pr2GripperCommandActionGoal() for lr in LR]
g_flags = [False, False]# is pickup phase
clicked_pos = [PoseStamped() for lr in LR]

def click_cb(msg):
  print msg
  stamp = rospy.Time.now()
  try:
    tfl.waitForTransform("/base_link", "/ray_target", stamp, timeout=rospy.Duration(1))
    (pos, rot) = tfl.lookupTransform("/base_link", "/ray_target", rospy.Time(0))
  except Exception, e:
    rospy.logerr(e)
    return

  tgtid = 0 if (pos[1]>0.0) else 1 ## divide center pose into L R
  clicked_pos[tgtid].header.frame_id = "base_link"
  clicked_pos[tgtid].header.stamp = stamp
  clicked_pos[tgtid].pose.position.x = pos[0]
  clicked_pos[tgtid].pose.position.y = pos[1]
  clicked_pos[tgtid].pose.position.z = pos[2]
  clicked_pos[tgtid].pose.orientation.x = 0
  clicked_pos[tgtid].pose.orientation.y = 0
  clicked_pos[tgtid].pose.orientation.z = 0
  clicked_pos[tgtid].pose.orientation.w = 1


  

if __name__ == '__main__':
  rospy.init_node('ScreenPoint2DualArmPose', anonymous=True)
  pubs  = [rospy.Publisher('/master_'+lr+'arm_pose', PoseStamped, queue_size=1) for lr in LR]
  # sub = rospy.Subscriber("/kinect_head/rgb/image_raw/screenpoint", PointStamped, click_cb)
  sub = rospy.Subscriber("/pointcloud_screenpoint_nodelet/output_point", PointStamped, click_cb)
  g_pubs  = [rospy.Publisher('/'+lr+'_gripper_controller/gripper_action/goal', Pr2GripperCommandActionGoal, queue_size=1) for lr in LR]
  tfl = tf.TransformListener()
  print "start ROS pub loop"
  HZ = 10

  # init
  for i in range(len(lr)):
    vals[i].header.frame_id = "base_link"
    vals[i].pose.position.x = 0.5
    vals[i].pose.position.y = sgns[i] * 0.3
    vals[i].pose.position.z = 1.0
    vals[i].pose.orientation.x = 0
    vals[i].pose.orientation.y = 0
    vals[i].pose.orientation.z = 0
    vals[i].pose.orientation.w = 1
  
  
  t_after_click = [0,0]
  while not rospy.is_shutdown():
    rospy.loginfo("sadf")
    for p, v, cp, lr in zip(pubs, vals, clicked_pos, LR):
      rospy.loginfo(cp.header.stamp)
      t_after_click = (rospy.Time.now() - cp.header.stamp).to_sec()
      if t_after_click > 9:
        t = t_after_click - 9
        theta = math.pi * (t / 3.0)
        theta = min(max(math.radians(0),theta),math.radians(90)) # 0~90
        r = 0.2
        v = copy.deepcopy(cp)
        v.pose.position.x -= r * math.sin(theta)
        v.pose.position.z -= r * (1-math.cos(theta))
      else:
        v = copy.deepcopy(cp)
      
      if lr == "l" :
        p.publish(v)

    

    # for i, (lr, v, v_old, p) in enumerate(zip(LR, vals, vals_old, pubs)):
    #   if     abs(v.pose.position.x - v_old.pose.position.x) < 0.01 \
    #      and abs(v.pose.position.y - v_old.pose.position.y) < 0.01\
    #      and abs(v.pose.position.z - v_old.pose.position.z) < 0.01 :
    #     vals_not_move[i] += 1
    #   else :
    #     vals_not_move[i] = 0
    #     rospy.loginfo(lr+"arm set as :\n"+str(v.pose.position))

    # vals_old = copy.deepcopy(vals)



    
    # for i, (v, v_not_move, p, gp) in enumerate(zip(vals, vals_not_move, pubs, g_pubs)):
    #   if v_not_move < 1.5*HZ or 4*HZ < v_not_move :
    #     # hover
    #     v_mod = copy.deepcopy(v)
    #     v_mod.pose.position.z += 0.2
    #     p.publish(v_mod)
        
    #     if g_flags[i] == True and g_vals[i].goal.command.position == 0:
    #       g_flags[i] = False
          
    #     if g_flags[i] == False and g_vals[i].goal.command.position != 0:
    #       g_flags[i] = True
          
    #   else :
    #     # touch
    #     p.publish(v)
        
    #     if 2*HZ < v_not_move:
    #       g_vals[i].goal.command.position = ( 0 if g_flags[i] else 0.1)
    #       g_vals[i].goal.command.max_effort = 25;
    #       gp.publish(g_vals[i])



    rospy.Rate(HZ).sleep()
