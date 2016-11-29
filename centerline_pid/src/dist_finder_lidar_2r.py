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
vel = 12

pub = rospy.Publisher('error_topic', input_pid, queue_size=1)

timestamp_last_message = None

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
    if math.isnan(distance) or distance < data.range_min:
        distance = 0
    if math.isinf(distance) or distance > data.range_max:
        distance = 50

    return distance


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
def getLateralRanges(data):

    # The laser has a 270 degree detection angle, and an angular resolution of
    # 0.25 degrees. This means that in total there are 1080+1 beams.
    # What we need here is the distance at 45 and 45+180=225 degrees
    # from the start of the detection range. The index of the beam at an
    # angle t is given by t / 0.25 hence the first index will be (45+0)*4=180,
    # and the second (45+180)*4 = 225*4 = 900.
    # Instead of taking only one measurement, take n on either side of the main
    # range beam and average them.
    # Consult https://www.hokuyo-aut.jp/02sensor/07scanner/ust_10lx_20lx.html

    # Range at 45 degrees (0)
    range_right = getAverageRange(data, 180, 10)

    # Range at 225 degrees (180)
    range_left = getAverageRange(data, 900, 10)

    ranges_list = []
    ranges_list.append(range_right)
    ranges_list.append(range_left)

    return ranges_list


# INPUT: a message of type LaserScan
# OUTPUT: a list containing the range and the index of the minimum range
#         contained in the message
def get_minimum_range(data):

    ranges_scan = data.ranges
    min_range = 1000.0

    # Find the minimum range
    for r in ranges_scan:
        if r < min_range:
            min_range = r

    # The index of the reference point in the circle list
    min_index = ranges_scan.index(min_range)

    ret_list = []
    ret_list.append(min_range)
    ret_list.append(min_index)

    return ret_list


#-------------------------------------------------------------------------------
# callback
# finds and expresses the orientation and distance from the centerline
# deviations in angular terms only
#-------------------------------------------------------------------------------
def callback(data):
    global timestamp_last_message

    # Update the (not necessarily constant) sampling time
    if timestamp_last_message == None:
        ts = 0.01
    else:
        ts =  rospy.Time.now() - timestamp_last_message
        ts = ts.to_sec()


    timestamp_last_message = rospy.Time.now()

    # The list where the front and lateral ranges are stored
    ranges_list = getLateralRanges(data)

    R = ranges_list[0]
    L = ranges_list[1]

    # Get the range and the index of the minimum range
    min_list = get_minimum_range(data)
    min_range = min_list[0]
    min_index = min_list[1]

    # phi is the orientation of the vehicle with respect to that of the lane
    if L >= R:
        phi = float(540-min_index) / 4 - 90
    else:
        phi = float(540-min_index) / 4 + 90

    phi = np.radians(phi)

    # Check for angular overflow in the orientation deviation
    # (unnecessary, used as a precautionary measure)
    while phi > np.pi:
        phi -= 2*np.pi
    while phi < -np.pi:
        phi += 2*np.pi


    CCp = 5
    tan_arg = float(L-R) / ((2 * CCp) * (L+R))

    # Check for angular overflow
    while tan_arg > np.pi:
        tan_arg -= 2*np.pi
    while tan_arg < -np.pi:
        tan_arg  += 2*np.pi


    # If the steering angle to the left requires a negative sign and the
    # steering angle to the right requires a positive sign, then the vehicle
    # should turn left if it is facing the right wall and right if it is
    # facing the left wall. If the vehicle lies at the leftmost half of the
    # lane, then it should turn right, and if it lies at the rightmost half,
    # it should turn left.
    error = -np.arctan(tan_arg) + phi

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

    rospy.Subscriber("scan", LaserScan, callback, queue_size=1)
    rospy.spin()
