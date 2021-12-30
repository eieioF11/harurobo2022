#!/usr/bin/env python
# -*- coding: utf-8 -*-
class PID():
    def __init__(self,Kp,Ki,Kd,Min,Max):
        self.reset()
        self.KP=Kp
        self.KI=Ki
        self.KD=Kd
        self.Min=Min
        self.Max=Max

    def reset(self):
        self.integral=0.0
        self.past = 0.0
        self.deviation = 0.0

    def set_gain(self,Kp,Ki,Kd):
        self.reset()
        self.KP=Kp
        self.KI=Ki
        self.KD=Kd

    def output(self,r,y,dt):#目標値 r 出力値 y
        self.past = self.deviation
        self.deviation = r - y                                   #偏差
        self.integral += ((self.past + self.deviation) / 2) * dt #積分
        differential = (self.deviation - self.past) / dt         #微分
        pid = self.KP * self.deviation + self.KI * self.integral + self.KD * differential
        if pid > self.Max:
            pid = self.Max
        if pid < self.Min:
            pid = self.Min
        return pid