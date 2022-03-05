#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
from scipy.special import comb
import pandas as pd
import matplotlib.pyplot as plt

import glob
import os

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

import tf


class path_creator():
    def __init__(self,path_pub,index_ox,index_oy,resolution,start,field):
        self.path_pub=path_pub
        self.index_ox=index_ox
        self.index_oy=index_oy
        self.resolution=resolution
        self.dot=[]
        self.dot_goal=None
        self.end=False
        self.start=start
        self.field=field
        ROBOTSIZE=0.5
        self.boxsize=ROBOTSIZE//self.resolution
        print ("ロボットサイズ",ROBOTSIZE,"boxsize:",self.boxsize)
        self.line=None
        self.t1=None
        self.endx=0
        self.endy=0
        self.angle=0.0
        self.inputflag=False
        self.inputendflag=False
        self.strnum=""
        self.POINT=[]
        self.goal=[]
        self.sign=1

        self.handmode=False
        self.hand=[0,0]
        self.arm=0

    def euler_to_quaternion(self,angle):
        q = tf.transformations.quaternion_from_euler(0,0,math.radians(angle))
        return q

    def read_csv(self):
        n=0
        fpath=os.environ['HOME']+"/catkin_ws/src/harurobo2022/scripts/csv/"+self.field+"/"
        fname=[]
        for f in glob.glob(fpath+'*.csv'):
            fname.append(int(os.path.splitext(os.path.basename(f))[0]))
        #print(fname)
        n=0
        if len(fname):
            n=max(fname)
        print(str(n)+".csv")
        self.csv_path_data = pd.read_csv(fpath+str(n)+".csv")
        pathend=[0,0]
        for indx in range(len(self.csv_path_data)):
            pathend[0] = self.csv_path_data["x"][indx]
            pathend[1] = self.csv_path_data["y"][indx]
        pathend=self.point_generation(pathend,self.index_ox,self.index_oy,self.resolution)
        return pathend

    def save_csv(self):
        # Save CSV path file
        cols = ["x", "y"]
        df = pd.DataFrame(self.ros_path,columns=cols)
        print(self.angle)
        q=self.euler_to_quaternion(self.angle)
        df.insert(len(df.columns),"w0",q[0])
        df.insert(len(df.columns),"w1",q[1])
        df.insert(len(df.columns),"w2",q[2])
        df.insert(len(df.columns),"w3",q[3])
        df.insert(len(df.columns),"arm",self.arm)
        df.insert(len(df.columns),"hand1",self.hand[0])
        df.insert(len(df.columns),"hand2",self.hand[1])
        print(df)
        fpath=os.environ['HOME']+"/catkin_ws/src/harurobo2022/scripts/csv/"+self.field+"/"
        fname=[]
        for f in glob.glob(fpath+'*.csv'):
            fname.append(int(os.path.splitext(os.path.basename(f))[0]))
        print(fname)
        n=0
        if len(fname):
            n=max(fname)+1
        df.to_csv(fpath+str(n)+".csv",index=False)

    def motion(self,event):
        x = event.xdata
        y = event.ydata
        try:
            self.ln_v.set_xdata(x)
            self.ln_h.set_ydata(y)
            self.rects.set_x(x-self.boxsize//2)
            self.rects.set_y(y-self.boxsize//2)
            #print(x-self.boxsize//2,y-self.boxsize//2)

            self.t1.set_x(x+10)
            self.t1.set_y(y+10)
            self.t1.set_text("")
            if self.line!=None:
                self.line.remove()
                self.line=None

            if event.button == 1:
                pass
            if event.button == 2:
                self.line,=plt.plot([x,self.endx],[y,self.endy],c="green")
                self.angle=int(math.degrees(math.atan2(self.endx-x,self.endy-y))+180)
                if self.angle>180:
                    self.angle-=360
                self.t1.set_x(x+10)
                self.t1.set_y(y+10)
                self.t1.set_text(str(self.angle)+"[deg]")
        except:
            pass
        plt.draw()

    def release(self,event):
        x = int(event.xdata)
        y = int(event.ydata)

        if event.button == 1 and  not self.handmode and not self.inputflag:
            if not self.map[y,x] and not self.end:
                plt.title("Initial position addition with i key")
                if self.path==[]:
                    self.path=np.array([y,x])
                    dot,=plt.plot(self.path[1],self.path[0],"o",c="red")
                    self.endx=self.path[1]
                    self.endy=self.path[0]
                    self.dot.append(dot)
                else:
                    self.path=np.vstack((self.path,[y,x]))
                    dot,=plt.plot(self.path[:,1],self.path[:,0],"o",c="red")
                    self.endx=self.path[-1,1]
                    self.endy=self.path[-1,0]
                    self.dot.append(dot)
            else:
                plt.title("Error")
            print(self.dot,self.path)
        if event.button == 3:
            pass
            print(x,y)

        plt.draw()

    def onkey(self,event):
        print('you pressed', event.key, event.xdata, event.ydata)
        if not self.end:
            if not self.inputflag:
                if not self.handmode:
                    if event.key == 'p':
                        if len(self.goal)>1:
                            self.path=np.vstack((self.path,self.goal))
                        self.line1,=plt.plot(self.path[:,1],self.path[:,0],c="red")
                        self.path=self.pathconverter(self.path)
                        self.line2,=plt.plot(self.path[:,1],self.path[:,0],c="Cyan")
                        #経路配信
                        self.ros_path=self.path_generation(self.path,self.index_ox,self.index_oy,self.resolution)
                        self.end=True
                        self.save_csv()
                        plt.title("Path generation is completed!")
                    if event.key == '@':
                        if len(self.goal)>1:
                            self.path=np.vstack((self.path,self.goal))
                        self.line1,=plt.plot(self.path[:,1],self.path[:,0],c="red")
                        self.path=self.pathconverter(self.path)
                        self.line2,=plt.plot(self.path[:,1],self.path[:,0],c="Cyan")
                        plt.title("Path show!")
                    if event.key == 'i':
                        if self.path==[]:
                            self.path=np.array(self.start)
                            dot,=plt.plot(self.path[1],self.path[0],"o",c="red")
                            self.endx=self.path[1]
                            self.endy=self.path[0]
                            self.dot.append(dot)
                        else:
                            self.path=np.vstack((self.path,self.start))
                            dot,=plt.plot(self.path[:,1],self.path[:,0],"o",c="red")
                            self.endx=self.path[-1,1]
                            self.endy=self.path[-1,0]
                            self.dot.append(dot)
                    if event.key == 't':#入力
                        self.inputflag=True
                        self.strnum=""
                        self.sign=1
                        self.POINT=[]
                        plt.title("")
                        self.inputendflag=False
                    if event.key == 'h':#キャッチ
                        plt.title("hand mode (1,2)")
                        self.handmode=True
                    if event.key == 'd':
                        try:
                            self.line1.remove()
                            self.line2.remove()
                        except:
                            pass
                        if len(self.path)!=0:
                            self.dot[-1].remove()
                            del self.dot[-1]
                            self.path=np.delete(self.path,-1, 0)
                            print(self.dot,self.path)
                        else:
                            plt.title("Error")
                    if event.key == '1':
                        self.arm=1
                        plt.title("arm mode1")
                    if event.key == '2':
                        self.arm=2
                        plt.title("arm mode2")
                    if event.key == '3':
                        self.arm=3
                        plt.title("arm mode3")
                else:
                    if event.key == 'e':
                        self.handmode=False
                        plt.title("")
                    elif event.key == '1':
                        self.hand[0]=1
                        plt.title("hand1 on")
                    elif event.key == '2':
                        self.hand[1]=1
                        plt.title("hand2 on")
                    elif event.key == 'r':
                        self.hand=[0,0]
                        plt.title("hand all off")

            else:
                if event.key == 'e':
                    self.inputflag=False
                    self.inputendflag=False
                    plt.title("")
                elif event.key == 'r':
                    self.strnum=""
                    self.POINT=[]
                    plt.title("")
                    self.sign=1
                    if self.dot_goal!=None:
                        self.dot_goal.remove()
                    self.inputendflag=False
                elif event.key == 'd':
                    self.strnum=""
                    self.inputendflag=False
                    plt.title("")
                elif event.key==u'enter' and not self.inputendflag:
                    if len(self.POINT)!=2:
                        self.POINT.append(int(self.strnum)*0.001*self.sign)
                    else:
                        self.angle = self.sign*int(self.strnum)
                        self.inputendflag=True
                        plt.title(str(self.POINT)+str(self.angle))
                        self.goal=self.point_generation(self.POINT,self.index_ox,self.index_oy,self.resolution)
                        print(self.goal)
                        self.dot_goal,=plt.plot(self.goal[1],self.goal[0],"o",c="blue")
                    self.strnum=""
                    self.sign=1
                    plt.title("")
                elif not self.inputendflag:
                    try:
                        if event.key == '-':
                            self.sign=-1
                        else:
                            num=int(event.key)
                            self.strnum+=event.key
                        if len(self.POINT)!=2:
                            if self.sign<0:
                                plt.title("-"+self.strnum+"[mm]")
                            else:
                                plt.title(self.strnum+"[mm]")
                        else:
                            if self.sign<0:
                                plt.title("-"+self.strnum+"[deg]")
                            else:
                                plt.title(self.strnum+"[deg]")
                    except:
                        plt.title("Error")
                        pass
                else:
                    plt.title(str(self.POINT)+str(self.angle))
                    self.goal=self.point_generation(self.POINT,self.index_ox,self.index_oy,self.resolution)
                    print(self.goal)
                    self.dot_goal,=plt.plot(self.goal[1],self.goal[0],"o",c="blue")

        if event.key == 'n':
            if self.end:
                self.start=self.read_csv()
                self.path=[self.path[-1,0],self.path[-1,1]]
                self.line1.remove()
                self.line2.remove()
                if self.dot_goal!=None:
                    self.dot_goal.remove()
                for d in self.dot:
                    d.remove()
                self.dot=[]
                self.goal=[]
                self.dot_goal=None
                self.end=False
                dot,=plt.plot(self.path[1],self.path[0],"o",c="Blue")
                self.dot.append(dot)
                plt.title("Initial position addition with i key")
            else:
                plt.title("Press P key")

        plt.draw()

    #path conversion

    def bernstein_poly(self,i, n, t):
        return comb(n, i) * ( t**(n-i) ) * (1 - t)**i

    def bezier_curve(self,points, nTimes=1000):
        nPoints = len(points)
        xPoints = np.array([p[0] for p in points])
        yPoints = np.array([p[1] for p in points])

        t = np.linspace(0.0, 1.0, nTimes)

        polynomial_array = np.array([ self.bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

        xvals = np.dot(xPoints, polynomial_array)
        yvals = np.dot(yPoints, polynomial_array)

        return xvals, yvals

    def pathconverter(self,path):
        xvals, yvals = self.bezier_curve(path, nTimes=1000)#ベジェ曲線で経路を滑らかにする
        cpath=np.flipud(np.array(list(map(list, zip(xvals,yvals)))))#xvalsとyvalsの結合と反転
        #結果表示
        print path
        print cpath
        return cpath

    def point_generation(self,point,ox,oy,resolution):
        px = (point[0]/resolution)+ox
        py = (point[1]/resolution)+oy
        return [int(px),int(py)]
    #path generation
    def path_generation(self,sPath,ox,oy,resolution):#ROSにpath形式のデータを配信
        #Initialize odometry header
        global path_pub
        global head
        path = Path()
        path_header = Header()
        path_header.seq = 0
        path_header.stamp = rospy.Time.now()
        path_header.frame_id = "map"
        ros_path=[]
        for i in range(0,len(sPath)):
            temp_pose = PoseStamped()
            temp_pose.pose.position.x = (sPath[i][0]-ox)*resolution
            temp_pose.pose.position.y = (sPath[i][1]-oy)*resolution
            temp_pose.pose.position.z = 0
            temp_pose.header = path_header
            temp_pose.header.seq = i
            path.poses.append(temp_pose)
            ros_path.append([(sPath[i][0]-ox)*resolution,(sPath[i][1]-oy)*resolution])
        #print path.poses
        path.header = path_header
        self.path_pub.publish(path)
        rospy.loginfo('End path generation')
        return ros_path


    def create(self,map):
        self.map=map
        fig=plt.figure(figsize=(8,8))
        ax = fig.add_subplot(111)
        self.ln_v = plt.axvline(0)
        self.ln_h = plt.axhline(0)
        self.rects = plt.Rectangle((0,0),self.boxsize,self.boxsize,color='m',fill=False)
        ax.add_artist(self.rects)
        self.path=[]
        x=y=0
        if self.field=="r":
            x=970
            y=970
            w=270
            h=150
            plt.xlim(x,x+w)
            plt.ylim(y+h,y)
        elif self.field=="b":
            x=760
            y=970
            w=270
            h=150
            plt.xlim(x,x+w)
            plt.ylim(y+h,y)
        self.t1 = ax.text(x,y, str(""))
        try:
            self.start=self.read_csv()
        except:
            pass
        plt.text(x+10,y+10, "d:delete p:path generation n:create new path i:initial position" )
        #plt.imshow(self.map[y:y+h,x:x+w])
        plt.imshow(self.map)
        plt.title("Initial position addition with i key")
        plt.connect('motion_notify_event', self.motion)
        plt.connect('button_release_event', self.release)
        plt.connect('key_press_event', self.onkey)
        plt.show()
        print("end path create!")
        return


#Pcreat=path_creator()
#map = np.zeros((1000,1000))
#for i in range(100):
#    map[500+i,:]=255
#path=Pcreat.create(map)
#plt.imshow(map)
#plt.plot(path[:,1],path[:,0],c="red")
#plt.show()
