#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import glob
import os
import sys
# import for ros function
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
from std_msgs.msg import Bool

#########################
# Simple Path publisher #
#########################
class Simple_path_simulator():

    ##################
    # Initialization #
    ##################
    def __init__(self,field):

        rospy.init_node('Path_Publisher', anonymous=True)
        self.r = rospy.Rate(50)  # 50hz
        #Initialize odometry header
        self.path_header = Header()
        self.path_header.seq = 0
        self.path_header.stamp = rospy.Time.now()
        self.path_header.frame_id = "map"

        #initialize publisher
        self.pathn_pub = rospy.Publisher("/path_now", Path, queue_size=50)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=50)
        self.odom_sub = rospy.Subscriber('move_end',Bool, self.end)
        self.pathn = Path()
        self.pathn.header = self.path_header
        self.field = field

    def end(self,value):
        print(value)
        if bool(value)==True:
            self.n+=1
            self.pathn=self.path

    #############################################
    # Update odometry form User request cmd_vel #
    #############################################
    def get_poses_from_csvdata(self):
        #Get poses from csv data
        poses_list = []
        for indx in range(len(self.csv_path_data)):
            temp_pose = PoseStamped()
            temp_pose.pose.position.x = self.csv_path_data["x"][indx]
            temp_pose.pose.position.y = self.csv_path_data["y"][indx]
            temp_pose.pose.orientation.x = self.csv_path_data["w0"][indx]
            temp_pose.pose.orientation.y = self.csv_path_data["w1"][indx]
            temp_pose.pose.orientation.z = self.csv_path_data["w2"][indx]
            temp_pose.pose.orientation.w = self.csv_path_data["w3"][indx]
            temp_pose.header = self.path_header
            temp_pose.header.seq = indx
            poses_list.append(temp_pose)
        return poses_list

    def publish_path_topic(self):
        self.n=0
        fpath=os.environ['HOME']+"/catkin_ws/src/harurobo2022/scripts/csv/"+self.field+"/"
        fname=[]
        for f in glob.glob(fpath+'*.csv'):
            fname.append(int(os.path.splitext(os.path.basename(f))[0]))
        fname.sort()
        while not rospy.is_shutdown():
            if max(fname)<self.n:
                return
            self.csv_path_data = pd.read_csv(fpath+str(self.n)+".csv")
            #print self.csv_path_data
            self.path = Path()
            self.path.header = self.path_header
            #get pose data from csv
            pose_list = self.get_poses_from_csvdata()
            self.path.poses =pose_list
            print self.n
            self.pathn_pub.publish(self.pathn)
            self.path_pub.publish(self.path)
            self.r.sleep()


if __name__ == '__main__':
    print('Path Publisher is Started...')
    args = sys.argv
    print(args[1])
    test = Simple_path_simulator(args[1])
    try:
        test.publish_path_topic()
    except KeyboardInterrupt:
        print("finished!")