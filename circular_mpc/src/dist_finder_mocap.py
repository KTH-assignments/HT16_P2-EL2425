#!/usr/bin/env python
'''
Node to determine deviation from trajectory.

'''

import rospy
import numpy as np
from trajectory_planner import Path
from slip_control_communications.msg import pose
from slip_control_communications.msg import mocap_data

pub = rospy.Publisher('pose_topic', pose, queue_size=1)
path = Path()

# The reference trajectory
traj = path.get_points()

vel = 20.0

#-------------------------------------------------------------------------------
# callback
#
# INPUT:
#   state: measurements derived from mocap
#-------------------------------------------------------------------------------
def callback(state):
    min_dist = 100.0
    min_point = None

    # Find the closest trajectory point
    for point in traj:
        dist = np.sqrt((state.x - point[0])**2 + (state.y - point[1])**2)
        if dist < min_dist:
            min_dist = dist
            min_point = point


    # x-wise error
    x_error = min_dist * sin(state.yaw)

    # y-wise error
    y_error = -min_dist * cos(state.yaw)

    # Angle error to reference point R
    angle_error = min_point[2] - np.radians(state.yaw)

    while angle_error > np.pi:
        angle_error -= 2*np.pi
    while angle_error < -np.pi:
        angle_error += 2*np.pi


    # Create the message that is to be sent to the mpc controller,
    # pack all relevant information and publish it
    h = pose.Header()
    h.stamp = rospy.Time.now()

    msg = pose()
    msg.header = h
    msg.x = x_error
    msg.y = y_error
    msg.psi = angle_error
    msg.v = vel
    pub.publish(msg)


#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('dist_finder_mocap_node', anonymous=True)
    print("Mocap node started")

    rospy.Subscriber("car_state_topic", mocap_data, callback)
    rospy.spin()
