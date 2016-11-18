#!/usr/bin/env python

'''
Discretized trajectory for circle.

'''

#import rospy
import numpy as np

xc = 1.0
yc = -0.25
r = 1.5

class Path():
    def __init__(self):
        self.points = []

    def get_points(self):
        for i in range(0,360):
            x = xc + r*np.cos(np.radians(i))
            y = yc + r*np.sin(np.radians(i))
            theta = -np.arctan2(x - xc, y-yc)

            self.points.append((x,y,theta))

        return self.points
