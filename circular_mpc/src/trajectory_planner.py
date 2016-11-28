#!/usr/bin/env python

'''
Discretized circular trajectory

'''

import numpy as np

xc = 0.95
yc = -0.1
r = 1.5
vel = 1.7

#xc = 1.0
#yc = -0.25
#r = 1.5
#vel = 1.7



class Path():
    def __init__(self):
        self.points = []

    def get_points(self):
        for i in range(0,180):
            x = xc + r*np.sin(np.radians(i))
            y = yc - r*np.cos(np.radians(i))
            v = vel
            theta = np.radians(i)

            self.points.append(list((x, y, vel, theta)))

        for i in range(0,180):
            x = xc + r*np.sin(np.pi + np.radians(i))
            y = yc - r*np.cos(np.pi + np.radians(i))
            v = vel
            theta = -np.pi + np.radians(i)

            self.points.append(list((x, y, vel, theta)))

        return self.points
