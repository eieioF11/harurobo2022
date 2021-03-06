#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from math import sin, cos, pi,sqrt
# import for ros function
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from control_msgs.msg import JointControllerState

from jsk_rviz_plugins.msg import *
from std_msgs.msg import ColorRGBA, Float32, Float64
from nav_msgs.msg import Odometry

class omuni3():
    def __init__(self):
        # Parameter
        self.R=0.03
        self.L=0.1*sqrt(2) #0.1414...
        self.Adjust=16.7 #1.0
        # Initialization
        rospy.init_node('Omuni3Controller', anonymous=True)
        rospy.loginfo("R:%f,L:%f",self.R,self.L)
        self.r = rospy.Rate(50)  # 50hz
        #initialize publisher
        #self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=50)#実機使用時
        self.value_pub1 = rospy.Publisher("CMD_Vx", Float32, queue_size=1)
        self.value_pub2 = rospy.Publisher("CMD_Az", Float32, queue_size=1)
        self.value_pub3 = rospy.Publisher("Curvature_val", Float32, queue_size=1)

        self.c0 = rospy.Publisher("/OmuniRobot/joint_controller0/command", Float64, queue_size=5)
        self.c1 = rospy.Publisher("/OmuniRobot/joint_controller1/command", Float64, queue_size=5)
        self.c2 = rospy.Publisher("/OmuniRobot/joint_controller2/command", Float64, queue_size=5)

        self.odom = rospy.Publisher("/odom", Odometry, queue_size=5)
        self.odom_wheel = rospy.Publisher("/OmuniRobot/odom", Odometry, queue_size=5)
        #initialize subscriber
        self.cmd_sub = rospy.Subscriber("/cmd_vel",Twist, self.get_cmd)
        #self.state0_sub = rospy.Subscriber("/OmuniRobot/joint_controller0/state",JointControllerState, self.get_state0)
        #self.state1_sub = rospy.Subscriber("/OmuniRobot/joint_controller1/state",JointControllerState, self.get_state1)
        #self.state2_sub = rospy.Subscriber("/OmuniRobot/joint_controller2/state",JointControllerState, self.get_state2)
        self.gazebo_states_sub = rospy.Subscriber("/tracker",Odometry, self.get_gazebo_states)

        self.w0=0.0
        self.w1=0.0
        self.w2=0.0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.last_time=rospy.Time.now()
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.move(0,0,0)

    def move(self,Vx,Vy,Angular):
        Angular=Angular/4.8
        w0=(-Vy+Angular)*self.Adjust
        w1=(0.5*Vy+0.866*Vx+Angular)*self.Adjust
        w2=(0.5*Vy-0.866*Vx+Angular)*self.Adjust
        #rospy.loginfo("w(%f,%f,%f,%f)",w0,w1,w2,w3)
        self.c0.publish(w0)
        self.c1.publish(w1)
        self.c2.publish(w2)

    def get_cmd(self,value):
        #rospy.loginfo("LinerX:%from std_msgs.msg import ColorRGBA, Float32, Float64f",value.linear.x)
        #rospy.loginfo("LinerY:%f",value.linear.y)
        #rospy.loginfo("Angular:%f",value.angular.z)
        self.move(value.linear.x,value.linear.y,value.angular.z)

    def get_state0(self,value):
        #rospy.loginfo("state0:%f",value.process_value)
        self.w0=value.process_value
    def get_state1(self,value):
        #rospy.loginfo("state1:%f",value.process_value)
        self.w1=value.process_value
    def get_state2(self,value):
        #rospy.loginfo("state2:%f",value.process_value)
        self.w2=value.process_value

    def get_gazebo_states(self,value):
        current_time = rospy.Time.now()
        #rospy.loginfo("state3:%f",value.process_value)
        self.tracker=value
        #print(value.pose.pose.position.x,value.pose.pose.position.y)
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

    def update(self):
        pass



if __name__ == '__main__':
    rospy.loginfo('Omuni3 controller start!')
    wheel=omuni3()
    try:
        while not rospy.is_shutdown():
            wheel.update()
    except KeyboardInterrupt:
        print("finished!")
