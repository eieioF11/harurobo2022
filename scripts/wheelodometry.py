#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
import time

# import for ros function
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3Stamped
from sensor_msgs.msg import Imu

from jsk_rviz_plugins.msg import *
from std_msgs.msg import ColorRGBA, Float32, Float64 ,String
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
        self.imu_sub = rospy.Subscriber("/imu/rpy",Vector3Stamped, self.get_rpy)
        self.enc_odom_sub = rospy.Subscriber("/Encoder_odom",Odometry, self.get_whoeelodom)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.x=0
        self.y=0
        self.yaw=0.0
        self.oldtime=time.time()
        rospy.spin()

    def reset(self):
        self.x=0
        self.y=0

    def get_rpy(self,value):
        #print(value)
        self.yaw=value.vector.z

    def get_whoeelodom(self,value):
        current_time = rospy.Time.now()
        nowtime = time.time()
        #rospy.loginfo("state3:%f",value.process_value)
        #print(value.pose.pose.position.x,value.pose.pose.position.y)
        #math.radians(angle)
        q=[0,0,0,0]
        w0=value.twist.twist.linear.x*-1
        w1=value.twist.twist.linear.y*-1
        ry=self.R*0.7071067812*(w0+w1)
        rx=self.R*0.7071067812*(w0-w1)
        dt=nowtime-self.oldtime
        #ry=self.R*w1
        #rx=self.R*w0
        self.x+=(rx*math.cos(self.yaw)-ry*math.sin(self.yaw))*dt
        self.y+=(rx*math.sin(self.yaw)+ry*math.cos(self.yaw))*dt
        value.pose.pose.position.x=self.x
        value.pose.pose.position.y=self.y
        value.twist.twist.linear.x=rx
        value.twist.twist.linear.y=ry

        q = tf.transformations.quaternion_from_euler(0,0,self.yaw)
        value.pose.pose.orientation.x=q[0]
        value.pose.pose.orientation.y=q[1]
        value.pose.pose.orientation.z=q[2]
        value.pose.pose.orientation.w=q[3]
        print("({:.2f},{:.2f}),r({:.2f},{:.2f})w({:.2f},{:.2f}){:.2f}[deg]{:.2f}".format(value.pose.pose.position.x,value.pose.pose.position.y,value.twist.twist.linear.x,value.twist.twist.linear.y,w0,w1,math.degrees(self.yaw),dt))
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

        self.oldtime=nowtime

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