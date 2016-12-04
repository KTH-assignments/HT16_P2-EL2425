#!/usr/bin/env python

'''
Discretized circular trajectory

'''

import numpy as np



#xc = 1.0
#yc = -0.25
#r = 1.5
#vel = 1.7



class Path():
    def __init__(self):
        self.xc = 0.95
        self.yc = -0.1
        self.r = 1.5

        self.points = []

    def get_points(self):
        for i in range(0,181):
            x = self.xc + self.r*np.sin(np.radians(i))
            y = self.yc - self.r*np.cos(np.radians(i))
            theta = np.radians(i)

            self.points.append(list((x, y, theta)))

        for i in range(1,180):
            x = self.xc + self.r*np.sin(np.pi + np.radians(i))
            y = self.yc - self.r*np.cos(np.pi + np.radians(i))
            theta = -np.pi + np.radians(i)

            self.points.append(list((x, y, theta)))

        return self.points

    def get_center_and_radius(self):

        ret_list = []

        ret_list.append(self.xc)
        ret_list.append(self.yc)
        ret_list.append(self.r)

        return ret_list
