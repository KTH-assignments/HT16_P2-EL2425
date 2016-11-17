#!/usr/bin/env python
'''
Node to measure distance to obstacles.

'''

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from slip_control_communications.msg import pose

# The desired velocity of the car.
# TODO make it an argument and include it in the launcher
vel = 30

pub = rospy.Publisher('pose_topic', pose, queue_size=1)


#-------------------------------------------------------------------------------
#   Input:  data: Lidar scan data
#           beam_index: The index of the angle at which the distance is required
#
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
#   Input:  data: Lidar scan data
#           beam_index: The index of the angle at which the distance is required
#           degree_offset: The offset from beam_index, expressed in *degrees*
#
#   OUTPUT: distance of scan at angle theta whose index is
#           (beam_index + 4*degree_offset)
#-------------------------------------------------------------------------------
def getRange(data, beam_index, degree_offset):

    return getRange(data, beam_index + 4*degree_offset)



#-------------------------------------------------------------------------------
#   Input:  data: Lidar scan data
#           beam_index: The index of the angle at which the distance is required
#           degree_offset: The offset from beam_index, expressed in *degrees*
#
#   OUTPUT: the difference between two scans, one at (beam_index - 4*degree_offset)
#           and one at (beam_index + 4*degree_offset). Their difference is
#           taken in an anti-clockwise fashion.
#-------------------------------------------------------------------------------
def getRangeDifference(data, beam_index, degree_offset):

    first = getRange(data, beam_index, -degree_offset)
    second = getRange(data, beam_index, degree_offset)

    return first-second


#-------------------------------------------------------------------------------
#   Input:  data: Lidar scan data
#
#           beam_index: The index of the angle at which the distance is required
#
#           num_aux_scans_halfed: The number of auxiliary scans around
#           beam_index required to take the average around it, halved. This
#           means that if you want to take the average of 10 scans around
#           beam_index, num_aux_scans_halfed should be 10/2 = 5
#
#   OUTPUT: average distance of scan at angle theta whose index is beam_index
#-------------------------------------------------------------------------------
def getAverageRange(data, beam_index, num_aux_scans_halfed):

    dist = 0

    for i in range(beam_index - num_aux_scans_halfed, beam_index + num_aux_scans_halfed + 1):
        dist = dist + getRange(data, i)

    dist = float (dist) / (2*num_aux_scans_halfed + 1)

    return dist




#-------------------------------------------------------------------------------
# Input:
#   data: Lidar scan data
#
# Output:
#   a two-element list. The first is the distance of the object detected at the
#   lidar's "three o' clock" and the second at its "nine o'clock"
#-------------------------------------------------------------------------------
def getRanges(data):

    # The laser has a 270 degree detection angle, and an angular resolution of
    # 0.25 degrees. This means that in total there are 1080+1 beams.
    # What we need here is the distance at 45, 45+90=135 and 45+180=225 degrees
    # from the start of the detection range. The index of the beam at an
    # angle t is given by t / 0.25 hence the first index will be (45+0)*4=180,
    # the second (45+90)*4 =540 and the third (45+180)*4 = 225*4 = 900.
    # Instead of taking only one measurement, take 2 on either side of the main
    # range beam and average them.
    # Consult https://www.hokuyo-aut.jp/02sensor/07scanner/ust_10lx_20lx.html

    # Range at 45 degrees (0)
    range_right = getAverageRange(data, 180, 2)

    # Range at 135 degrees (90)
    range_face = getAverageRange(data, 540, 2)

    # Range at 225 degrees (180)
    range_left = getAverageRange(data, 900, 2)

    ranges_list = []
    ranges_list.append(range_right)
    ranges_list.append(range_face)
    ranges_list.append(range_left)

    return ranges_list



#-------------------------------------------------------------------------------
# callback
#-------------------------------------------------------------------------------
def callback(data):
    #swing = math.radians(theta)

    # The list where the front and lateral ranges are stored
    ranges_list = getRanges(data)

    R = ranges_list(0)
    F = ranges_list(1)
    L = ranges_list(2)

    # Take a range scan difference between a scan at -8 degrees relative to the
    # main beam at 0 degrees and one at +8 degrees. If the number returned is
    # positive then the vehicle is facing the left lane boundary. If not,
    # it's facing the right.
    if getRangeDifference(data, 540, 8) > 0:
        tan_arg_2 = float(L) / F
    else:
        tan_arg_2 = -float(R) / F

    # Check for angular overflow
    while tan_arg_2 > np.pi:
        tan_arg_2 -= 2*np.pi
    while tan_arg_2 < -np.pi:
        tan_arg_2  += 2*np.pi

    # The vehicle's orientation with respect to the lane
    psi = np.arctan(tan_arg_2)

    # The vehicle's displacement from the centerline
    y = -0.5 * (L-R)cos(psi)

    # Create the message that is to be sent to the mpc controller,
    # pack all relevant information and publish it
    msg = pose()
    msg.x = 0
    msg.y = y
    msg.psi = psi
    msg.v = vel # NOT ACTUALLY
    pub.publish(msg)



#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':

    rospy.init_node('dist_finder_lidar_node', anonymous = True)
    print("[Node] dist_finder_lidar started")

    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()
