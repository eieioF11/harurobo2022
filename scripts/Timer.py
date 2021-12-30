#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time

class Timer:
    def __init__(self):
        self.first=True
        self.start=time.time()
    def reset(self):
        self.first=True
    def stand_by(self,t):#[s]
        if self.first:
            self.start=time.time()
            self.first=False
        else:
            if (time.time()-self.start)>=t:
                self.first=True
                return True
        return False