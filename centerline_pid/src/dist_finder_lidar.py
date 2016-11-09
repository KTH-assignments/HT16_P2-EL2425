#!/usr/bin/env python
'''
Node to measure distance to obstacles.

'''

import rospy
import math
from sensor_msgs.msg import LaserScan
from slip_control_communications.msg import input_pid

# The desired velocity of the car.
# TODO make it an argument and include it in the launcher
vel = 30

pub = rospy.Publisher('error_topic', input_pid, queue_size=10)


#-------------------------------------------------------------------------------
#   Input:  data: Lidar scan data
#           theta: The angle at which the distance is requried

#   OUTPUT: distance of scan at angle theta
#-------------------------------------------------------------------------------
def getRange(data, beam_index):

    # Find the index of the array that corresponds to angle theta.
    # Return the lidar scan value at that index
    distance = data.ranges[beam_index]

    # Do some error checking for NaN and ubsurd values
    if math.isnan(distance) or distance > range_max:
        distance = 50
    if distance < range_min:
        distance = 0

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

    distance = []

    # The laser has a 270 degree detection angle, and an angular resolution of
    # 0.25 degrees. This means that in total there are 1080+1 beams.
    # What we need here is the distance at 45 and 45+180=225 degrees from the
    # start of the detection range. These are the lateral ranges at both lateral
    # ends of the scan. The index of the beam at an angle t is given by t / 0.25
    # hence the first index will be 45*4=180 and the second 225*4 = 900.
    # Consult https://www.hokuyo-aut.jp/02sensor/07scanner/ust_10lx_20lx.html
    range_right = getRange(data, 180)
    range_left = getRange(data, 900)

    distance.append(range_right)
    distance.append(range_left)


#-------------------------------------------------------------------------------
# callback
#-------------------------------------------------------------------------------
def callback(data):
    swing = math.radians(theta)

    # The list where the lateral ranges is stored
    ranges_left_right_list = getLateralRanges(data)

    # The disparity between the two ranges.
    # This difference is expressed between the right and left lateral ranges.
    # Regardless of the car's orientation, R - L < 0 means that the car is at
    # the right half of the road and needs to turn left, which means that the
    # signal going to the motor will be negative (the sign of the difference).
    # The opposite case is analogous to this one.
    # The scaling factor R+L is there to make the disparity invariant to the
    # width of the lane
    R = ranges_left_right_list(0)
    L = ranges_left_right_list(1)
    error = (R - L) / (R + L)


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
