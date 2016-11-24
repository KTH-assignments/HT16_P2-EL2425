#!/usr/bin/env python

'''
Discretized trajectory for circle.

'''

#import rospy
import numpy as np

#xc = 1.0
#yc = -0.25
#r = 1.5
#vel = 12

xc = 0.8
yc = 1.6
r = 1.2
vel = 12

class Path():
    def __init__(self):
        self.points = []

    def get_points(self):
        for i in range(0,360):
            x = xc + r*np.cos(np.radians(i))
            y = yc + r*np.sin(np.radians(i))
            v = vel
            theta = np.pi - np.arctan2(x - xc, y - yc)

            if i < 271:
                self.points.append(list((x, y, vel, theta)))
            else:
                self.points.append(list((x, y, vel, theta + 2*np.pi)))

        return self.points
