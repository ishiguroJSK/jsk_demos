#!/usr/bin/python
# -*- coding: utf-8 -*-
import math
import rospy
import copy
import numpy as np
from geometry_msgs.msg import *
#from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import CameraInfo
import tf

keys = ["larm", "rarm"]
sign = [1,-1]
vals = [PoseStamped() for k in keys]
go_pick = False
mouse_on_cam = np.zeros(3)
mouse_on_table = np.zeros(3)

cam_origin = np.array([0.424264, 0.03, 1.6])
R_cam2robot = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
TABLE_HEIGHT = 0.397 + 0.0

K = np.identity(3)
R = tf.transformations.euler_matrix(0, math.radians(45),0, 'rxyz')[0:3, 0:3]

def camerainfo_cb(data):
  global K
  K = np.array(data.K).reshape(3,3)  

def point_cb(data):
  global go_pick, mouse_on_cam, mouse_on_table

  go_pick = (data.point.z > 0)

  K_inv = np.linalg.inv(K)
  screen_px   = np.array([data.point.x,data.point.y,1])
  pt_cam  = K_inv.dot(screen_px)
  pt_cam2  = R_cam2robot.dot(pt_cam)
  mouse_on_cam = R.dot(pt_cam2) + cam_origin

  ray_vec = mouse_on_cam - cam_origin
  if ray_vec[2] != 0.0:
    a = (TABLE_HEIGHT - cam_origin[2]) / ray_vec[2]  ### (ray_vec * a + cam_origin)[2] = TABLE_HEIGHT
    mouse_on_table = cam_origin + a * ray_vec

   
if __name__ == '__main__':
  # sub_p = rospy.Subscriber("/iiwa/camera/left/image_raw/movepoint", PointStamped, point_cb)
  # sub_c = rospy.Subscriber("/iiwa/camera/left/camera_info", CameraInfo, camerainfo_cb)
  sub_p = rospy.Subscriber("/xtion/rgb/image_raw/movepoint", PointStamped, point_cb)
  sub_c = rospy.Subscriber("/xtion/rgb/camera_info", CameraInfo, camerainfo_cb)
  pub_m = rospy.Publisher("target_marker", MarkerArray, queue_size=1)
  pubs  = [rospy.Publisher('/master_'+k+'_pose', PoseStamped, queue_size=1) for k in keys]
  rospy.init_node('ScreenPoint2PoseStamped', anonymous=True)
  print "start ROS pub loop"
  HZ = 100
  r = rospy.Rate(HZ)
  open_w = 0.0
  move_z = 0.0
  pick_wait_count = 0
  PICK_W = -0.04
  PICK_H = TABLE_HEIGHT - 0.001
  RELEASE_W = 0.2
  RELEASE_H = PICK_H + 0.2
  while not rospy.is_shutdown():

    if go_pick: ### go down
      move_z = PICK_H
      open_w -= 0.1 / HZ
      if open_w < PICK_W: ### wait pick complete
        pick_wait_count += 1
        open_w = PICK_W
        if pick_wait_count > 1*HZ: ### pick up
          move_z = RELEASE_H
    else:
      move_z      = RELEASE_H
      open_w      = RELEASE_W
      pick_wait_count = 0

    for s, v, p in zip(sign, vals, pubs):
      v.pose.position.x = mouse_on_table[0]
      v.pose.position.y = mouse_on_table[1] + s * open_w / 2.0
      v.pose.position.z = move_z
      v.pose.orientation.x, v.pose.orientation.y, v.pose.orientation.z, v.pose.orientation.w = \
         tf.transformations.quaternion_from_euler(math.radians(-90), math.radians(90 - s*30), math.radians(-90))
      p.publish(v)

    ### rviz marker
    marker = Marker()
    marker.header.frame_id = "iiwa_torso_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "my_namespace"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    ma = MarkerArray()

    marker.id = 0
    marker.color.a = 0.2
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = cam_origin
    ma.markers.append(copy.deepcopy(marker))

    marker.id = 1
    marker.color.a = 0.6
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = mouse_on_cam
    ma.markers.append(copy.deepcopy(marker))

    marker.id = 2
    marker.color.a = 1.0
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = mouse_on_table
    ma.markers.append(copy.deepcopy(marker))
    
    pub_m.publish(ma)

    rospy.loginfo_throttle(2,"Open width : "+str(open_w)+" Pick height : "+str(move_z))
    r.sleep()
