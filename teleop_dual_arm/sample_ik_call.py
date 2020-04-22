#!/usr/bin/env python  

import time
import math
import random
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import hrpsys_ros_bridge
from hrpsys_ros_bridge.srv import *

if __name__ == '__main__':
    rospy.init_node('sample_ik_call_node')
    pub = rospy.Publisher("ik_target_marker", Marker, queue_size=1)
    marker = Marker()
    rate = rospy.Rate(0.5)
    rospy.loginfo("Wait for service available")
    rospy.wait_for_service("/SequencePlayerServiceROSBridge/setTargetPose")
    rospy.loginfo("Service ready")
    
    while not rospy.is_shutdown():
        x, y, z = random.uniform(0.8, 1.2), random.uniform(-0.2, 0.2), random.uniform(0.6,1.4)
        ### rviz marker
        marker.header.frame_id = "iiwa_torso_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        pub.publish(marker)
        ### ik call
        rospy.loginfo("random ik call "+str(x)+", "+str(y)+", "+str(z))
        try:
            ik_call_handler = rospy.ServiceProxy("/SequencePlayerServiceROSBridge/setTargetPose", OpenHRP_SequencePlayerService_setTargetPose)
            request = OpenHRP_SequencePlayerService_setTargetPoseRequest()
            request.name = 'larm'
            request.xyz = [x, y+0.1, z]
            request.rpy = [math.pi/2, 0, 0]
            request.tm = 1
            resp = ik_call_handler(request)
            rospy.loginfo("larm setTargetPose call = "+str(resp))
            request.name = 'rarm'
            request.xyz = [x, y-0.1, z]
            request.rpy = [-math.pi/2, 0, 0]
            request.tm = 1
            resp = ik_call_handler(request)
            rospy.loginfo("rarm setTargetPose call = "+str(resp))
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s"%e)

        rate.sleep()
