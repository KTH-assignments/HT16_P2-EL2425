#!/usr/bin/env python
'''
Node to determine deviation from trajectory.

'''

import rospy
import numpy as np
from trajectory_planner import Path
from slip_control_communications.msg import pose_and_reference
from slip_control_communications.msg import mocap_data

pub = rospy.Publisher('pose_and_reference_topic', pose_and_reference, queue_size=1)
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

    # Ref as a future point in trajectory
    if min_point != None:
        ref_point = traj[(traj.index(min_point) + 15)%360]


    # x-wise error
    ref_x = ref_point[0]

    # y-wise error
    ref_y = ref_point[1]

    # Angle error to reference point
    ref_psi = ref_point[2]


    while angle_error > np.pi:
        angle_error -= 2*np.pi
    while angle_error < -np.pi:
        angle_error += 2*np.pi


    # Create the message that is to be sent to the mpc controller,
    # pack all relevant information and publish it
    h = pose_and_reference.Header()
    h.stamp = rospy.Time.now()

    msg = pose_and_reference()
    msg.header = h

    msg.x = state.x
    msg.y = state.y
    msg.psi = state.psi
    msg.v = vel

    msg.ref_x = ref_x
    msg.ref_y = ref_y
    msg.ref_psi = ref_psi
    msg.ref_vel = vel

    pub.publish(msg)


#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('dist_finder_mocap_node', anonymous=True)
    print("Mocap node started")

    rospy.Subscriber("car_state_topic", mocap_data, callback)
    rospy.spin()
