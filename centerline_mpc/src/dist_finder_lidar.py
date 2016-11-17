#!/usr/bin/env python
'''
Node to measure distance to obstacles.

'''

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from slip_control_communications.msg import input_pid

# The desired velocity of the car.
# TODO make it an argument and include it in the launcher
vel = 30

pub = rospy.Publisher('error_topic', input_pid, queue_size=10)


#-------------------------------------------------------------------------------
#   Input:  data: Lidar scan data
#           beam_index: The index of the angle at which the distance is requried

#   OUTPUT: distance of scan at angle theta whose index is beam_index
#-------------------------------------------------------------------------------
def getRange(data, beam_index):

    # Find the index of the array that corresponds to angle theta.
    # Return the lidar scan value at that index
    distance = data.ranges[beam_index]

    # Do some error checking for NaN and ubsurd values
    if math.isnan(distance) or distance < range_min:
        distance = 0
    if distance > range_max:
        distance = 50

    return distance


#-------------------------------------------------------------------------------
# Input:
#   data: Lidar scan data
#
# Output:
#   a two-element list. The first is the distance of the object detected at the
#   lidar's "three o' clock" and the second at its "nine o'clock"
#-------------------------------------------------------------------------------
def getLateralRanges(data):

    # The laser has a 270 degree detection angle, and an angular resolution of
    # 0.25 degrees. This means that in total there are 1080+1 beams.
    # What we need here is the distance at 45, 45+90=135 and 45+180=225 degrees
    # from the start of the detection range. These are the lateral and straight
    # on ranges at both lateral ends of the scan. The index of the beam at an
    # angle t is given by t / 0.25 hence the first index will be 45*4=180, the
    # second (45+90)*4 =540 and the third 225*4 = 900.
    # Instead of taking only one measurement, take 2 on either side of the main
    # range beam and average them.
    # Consult https://www.hokuyo-aut.jp/02sensor/07scanner/ust_10lx_20lx.html

    # Range at 45 degrees (0)
    range_right = getRange(data, 178) +
        getRange(data, 179) +
        getRange(data, 180) +
        getRange(data, 181) +
        getRange(data, 182)

    range_right = range_right / 5


    # Range at 135 degrees (90)
    range_face = getRange(data, 538) +
        getRange(data, 539) +
        getRange(data, 540) +
        getRange(data, 541) +
        getRange(data, 542)

    range_face = range_face / 5

    # Range at 225 degrees (180)
    range_left = getRange(data, 898) +
        getRange(data, 899) +
        getRange(data, 900) +
        getRange(data, 901) +
        getRange(data, 902)

    range_left = range_left / 5

    distance = []
    distance.append(range_right)
    distance.append(range_face)
    distance.append(range_left)


#-------------------------------------------------------------------------------
# callback
#-------------------------------------------------------------------------------
def callback(data):
    #swing = math.radians(theta)

    # The list where the lateral ranges is stored
    ranges_list = getLateralRanges(data)

    # The disparity between the two ranges.
    # This difference is expressed between the right and left lateral ranges.
    # Regardless of the car's orientation, R - L < 0 means that the car is at
    # the right half of the road and needs to turn left, which means that the
    # signal going to the motor will be negative (the sign of the difference).
    # The opposite case is analogous to this one.
    R = ranges_list(0)
    F = ranges_list(1)
    L = ranges_list(2)

    # The overall angular error is: see
    # https://gits-15.sys.kth.se/alefil/HT16_P2_EL2425_resources/blob/master/various/Progress%20reports/Agendas/2016.11.16/main.pdf
    # The scaling factor R+L is there to make the range disparity invariant to
    # the width of the lane

    CCp = 5
    tan_arg_1 = float(L-R) / ((2 * CCp) * (L+R))
    tan_arg_2 = float(F) / R
    error =  -(np.arctan(tan_arg_1) + np.pi/2 - np.arctan(tan_arg_2))

    # Check for angular overflow
    while error > np.pi:
        error -= 2*np.pi
    while angle_error < -np.pi:
        error += 2*np.pi


    # Create the message that is to be sent to the pid controller,
    # pack all relevant information (error and default velocity)
    # and publish it
    msg = input_pid()
    msg.pid_error = error
    msg.pid_vel = vel
    pub.publish(msg)



#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':

    rospy.init_node('dist_finder_lidar_node', anonymous = True)
    print("[Node] dist_finder_lidar started")

    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()
