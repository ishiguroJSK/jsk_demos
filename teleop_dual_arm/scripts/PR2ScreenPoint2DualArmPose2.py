#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import tf
import copy
import math
from std_msgs.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import Pr2GripperCommandActionGoal

class MouseCmd2ArmPose():
  LR          = ["l", "r"]
  sgns        = { "l" : 1, "r" : -1 }

  def __init__(self):
    rospy.init_node('MouseCmd2ArmPose', anonymous=False)
    self.HZ = 10
    self.ee_pose = {}
    self.ep_pubs = {}
    self.g_pubs = {}
    self.clk_tgt_pos = {}
    self.t_cmd = {}
    self.is_pulling = {}
    self.current_lr_mode = "l"
    for lr in self.LR:
      self.ee_pose[lr] = PoseStamped()
      self.ee_pose[lr].header.frame_id  = "base_link"
      self.ee_pose[lr].pose.position    = Point(0.5, self.sgns[lr] * 0.3, 1.0)
      self.ee_pose[lr].pose.orientation = Quaternion(0,0,0,1)
      self.clk_tgt_pos[lr] = PoseStamped()
      self.t_cmd[lr] = Time()
      self.is_pulling[lr] = False
      self.ep_pubs[lr]  = rospy.Publisher('/master_'+lr+'arm_pose', PoseStamped, queue_size=1)
      self.g_pubs[lr]   = rospy.Publisher('/'+lr+'_gripper_controller/gripper_action/goal', Pr2GripperCommandActionGoal, queue_size=1)
    
    self.head_pub       = rospy.Publisher('/master_head_pose', PoseStamped, queue_size=1)
    self.head_pose = PoseStamped()

    self.cmd_str_sub = rospy.Subscriber("/rwt_command_string", String, self.cmd_str_cb)
    self.sub = rospy.Subscriber("/pointcloud_screenpoint_nodelet/output_point", PointStamped, self.click_cb)
    # self.bbp_sub = rospy.Subscriber("/PickNearestBoundingBox/output_pose", PoseStamped, self.bbp_cb)


    self.tfl = tf.TransformListener()
    print "start ROS pub loop"
  
    while not rospy.is_shutdown():
      # for p, v, cp, lr in zip(pubs, ee_pose, clk_tgt_pos, LR):
        # rospy.loginfo(cp.header.stamp)

      for lr in self.LR:
        t_after_click = (rospy.Time.now() - self.clk_tgt_pos[lr].header.stamp).to_sec()

        if self.is_pulling[lr]:
          t_after_pull_cmd = (rospy.Time.now() - self.t_cmd[lr]).to_sec()
          theta = math.pi / 2 * (t_after_pull_cmd / 3.0)
          # theta = min(max(math.radians(0),theta),math.radians(90)) # 0~90
          r = 0.2
          # v = copy.deepcopy(cp)
          # v.pose.position.x -= r * math.sin(theta)
          # v.pose.position.z -= r * (1-math.cos(theta))
          self.ee_pose[lr].pose.position.x = self.clk_tgt_pos[lr].pose.position.x - r * math.sin(theta)
          self.ee_pose[lr].pose.position.y = self.clk_tgt_pos[lr].pose.position.y
          self.ee_pose[lr].pose.position.z = self.clk_tgt_pos[lr].pose.position.z - r * (1-math.cos(theta))
          if t_after_pull_cmd > 3.0:
            self.is_pulling[lr] = False
        # else:
        #   v = copy.deepcopy(cp)
        
        # if lr == "l" :
        #   p.publish(v)

        self.ep_pubs[lr].publish(self.ee_pose[lr])
      
      # for i, (v, v_not_move, p, gp) in enumerate(zip(ee_pose, ee_pose_not_move, pubs, g_pubs)):
      #   if v_not_move < 1.5*HZ or 4*HZ < v_not_move :
      #     # hover
      #     v_mod = copy.deepcopy(v)
      #     v_mod.pose.position.z += 0.2
      #     p.publish(v_mod)
          
      #     if g_flags[i] == True and g_ee_pose[i].goal.command.position == 0:
      #       g_flags[i] = False
      #     if g_flags[i] == False and g_ee_pose[i].goal.command.position != 0:
      #       g_flags[i] = True
            
      #   else :
      #     # touch
      #     p.publish(v)
          
      #     if 2*HZ < v_not_move:
      #       g_ee_pose[i].goal.command.position = ( 0 if g_flags[i] else 0.1)
      #       g_ee_pose[i].goal.command.max_effort = 25;
      #       gp.publish(g_ee_pose[i])



      rospy.Rate(self.HZ).sleep()

  def bbp_cb(self, msg):
    self.clk_tgt_pos[self.current_lr_mode].header.frame_id = "base_link"
    self.clk_tgt_pos[self.current_lr_mode].header.stamp = rospy.Time.now()
    self.ee_pose[self.current_lr_mode] = copy.deepcopy(self.clk_tgt_pos[self.current_lr_mode])
    


  def click_cb(self, msg):
    stamp = rospy.Time.now()
    try:
      self.tfl.waitForTransform("/base_link", "/ray_target", stamp, timeout=rospy.Duration(1))
      (pos, rot) = self.tfl.lookupTransform("/base_link", "/ray_target", rospy.Time(0))
    except Exception, e:
      rospy.logerr(e)
      return

    self.clk_tgt_pos[self.current_lr_mode].header.frame_id = "base_link"
    self.clk_tgt_pos[self.current_lr_mode].header.stamp = stamp
    self.clk_tgt_pos[self.current_lr_mode].pose.position = Point( pos[0], pos[1], pos[2])
    # self.clk_tgt_pos[self.current_lr_mode].pose.orientation = Quaternion(0,0,0,1)
    self.ee_pose[self.current_lr_mode].header = self.clk_tgt_pos[self.current_lr_mode].header
    self.ee_pose[self.current_lr_mode].pose.position = self.clk_tgt_pos[self.current_lr_mode].pose.position

  def cmd_str_cb(self, msg):
    print msg
    str_l = msg.data.split("_")
    if len(str_l) == 2:
      lr, cmd = str_l # l_open
    elif len(str_l) == 3:
      lr, cmd, val = str_l # l_turn_90
    else:
      print "something wrong"
      return

    if cmd == "look":
      if lr == "l":
        self.head_pose.pose.position.z += 0.03
      if lr == "r":
        self.head_pose.pose.position.z -= 0.03
      if lr == "u":
        self.head_pose.pose.position.y -= 0.03
      if lr == "d":
        self.head_pose.pose.position.y += 0.03
      if lr == "c":
        self.head_pose = PoseStamped()

      self.head_pub.publish(self.head_pose)
      
    # if lr not in ["l", "r"]:
    #   print "something wrong"
    #   return

    if cmd == "mode":
      self.current_lr_mode = lr

    if cmd in ["open", "close"]:
      gripper_cmd = Pr2GripperCommandActionGoal()
      gripper_cmd.goal.command.position = ( 0.0 if cmd == "close" else 0.1)
      gripper_cmd.goal.command.max_effort = 75
      self.g_pubs[lr].publish(gripper_cmd)

    if cmd == "pull":
      self.is_pulling[lr] = True
      self.t_cmd[lr] = rospy.Time.now()

    if cmd == "turn":
      q_org = self.ee_pose[lr].pose.orientation
      q_rel = tf.transformations.quaternion_from_euler(math.pi/2,0,0)
      q_new = tf.transformations.quaternion_multiply(q_rel, [q_org.x, q_org.y, q_org.z, q_org.w])
      self.ee_pose[lr].pose.orientation = Quaternion(q_new[0],q_new[1],q_new[2],q_new[3])


  

if __name__ == '__main__':
  inst = MouseCmd2ArmPose()
