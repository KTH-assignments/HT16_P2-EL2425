#!/usr/bin/env python
'''
Node to measure distance to obstacles.

'''

import rospy
import math
from sensor_msgs.msg import LaserScan
from slip_control_communications.msg import input_pid

desired_trajectory = 1
vel = 30

pub = rospy.Publisher('error_topic', input_pid, queue_size=10)


#-------------------------------------------------------------------------------
#   Input:  data: Lidar scan data
#           theta: The angle at which the distance is requried

#   OUTPUT: distance of scan at angle theta
#-------------------------------------------------------------------------------
def getRange(data, theta):
    # Find the index of the arary that corresponds to angle theta.
    # Return the lidar scan value at that index
    # Do some error checking for NaN and ubsurd values

    # Your code goes here

    return

#-------------------------------------------------------------------------------
# callback
#-------------------------------------------------------------------------------
def callback(data):
    theta = 50;
    a = getRange(data,theta)
    b = getRange(data,0)
    swing = math.radians(theta)

    ## Your code goes here




    ## END

    msg = input_pid()
    msg.pid_error = error
    msg.pid_vel = vel
    pub.publish(msg)



#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':

    rospy.init_node('dist_finder_lidar_node', anonymous = True)
    print("Laser node started")

    rospy.Subscriber("scan_topic", LaserScan, callback)
    rospy.spin()
