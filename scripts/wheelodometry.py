#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
# import for ros function
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

from jsk_rviz_plugins.msg import *
from std_msgs.msg import ColorRGBA, Float32, Float64
from nav_msgs.msg import Odometry

class wheelodom():
    def __init__(self):
        # Parameter
        self.R=0.03
        # Initialization
        rospy.init_node('wheelodometry', anonymous=True)
        rospy.loginfo("R:%f",self.R)
        self.r = rospy.Rate(50)  # 50hzself.odself.odom_broadcaster = tf.TransformBroadcaster()om_broadcaster = tf.TransformBroadcaster()
        #initialize publisher

        self.odom = rospy.Publisher("/odom", Odometry, queue_size=5)
        self.odom_wheel = rospy.Publisher("/OmuniRobot/odom", Odometry, queue_size=5)
        #initialize subscriber
        #self.cmd_sub = rospy.Subscriber("/cmd_vel",Twist, self.get_cmd)
        self.gazebo_states_sub = rospy.Subscriber("/Encoder_odom",Odometry, self.get_whoeelodom)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.x=0
        self.y=0

    def get_whoeelodom(self,value):
        current_time = rospy.Time.now()
        #rospy.loginfo("state3:%f",value.process_value)
        #print(value.pose.pose.position.x,value.pose.pose.position.y)
        #math.radians(angle)
        q=[0,0,0,0]
        e = tf.transformations.euler_from_quaternion((q[0],q[1],q[2],q[3]))
        w0=value.twist.twist.linear.x
        w1=value.twist.twist.linear.y
        rx=self.R*0.5*(w0+w1)
        ry=self.R*0.8660254039*(w0-w1)
        self.x+=rx*math.cos(e[2])-ry*math.sin(e[2])
        self.y+=rx*math.sin(e[2])+ry*math.cos(e[2])
        value.pose.pose.position.x=self.x
        value.pose.pose.position.y=self.y
        value.twist.twist.linear.x=rx
        value.twist.twist.linear.y=ry

        q = tf.transformations.quaternion_from_euler(0,0,e[2])
        value.pose.pose.orientation.x=q[0]
        value.pose.pose.orientation.y=q[1]
        value.pose.pose.orientation.z=q[2]
        value.pose.pose.orientation.w=q[3]

        print(value.pose.pose.position.x,value.pose.pose.position.y,",",value.twist.twist.linear.x,value.twist.twist.linear.y)
        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (value.pose.pose.position.x,value.pose.pose.position.y, 0.),
            (value.pose.pose.orientation.x, value.pose.pose.orientation.y, value.pose.pose.orientation.z, value.pose.pose.orientation.w),
            current_time,
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        # set the position
        odom.pose.pose = value.pose.pose

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = value.twist.twist

        # publish the message
        self.odom.publish(odom)

if __name__ == '__main__':
    rospy.loginfo('wheel odometry start!')
    wheel=wheelodom()
    try:
        while not rospy.is_shutdown():
            pass
    except KeyboardInterrupt:
        print("finished!")