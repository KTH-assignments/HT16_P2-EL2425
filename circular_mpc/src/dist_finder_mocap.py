#!/usr/bin/env python
'''
Node to determine deviation from trajectory.

'''

import rospy
import numpy as np
import std_msgs
from trajectory_planner import Path
from slip_control_communications.msg import pose_and_reference
from slip_control_communications.msg import mocap_data

pub = rospy.Publisher('pose_and_reference_topic', pose_and_reference, queue_size=1)
path = Path()

# The reference trajectory
circle = path.get_points()

vel = 12.0

# Previous reference point on the circular trajectory
previous_ref_point = None

#-------------------------------------------------------------------------------
# callback
#
# INPUT:
#   state: measurements derived from mocap
#-------------------------------------------------------------------------------
def callback(state):
    min_dist = 100.0
    ref_point = None
    min_index = None

    global circle
    global previous_ref_point

    # Find the closest trajectory point
    for point in circle:
        dist = np.sqrt((state.x - point[0])**2 + (state.y - point[1])**2)
        if dist < min_dist:
            min_dist = dist
            ref_point = point


    # The index of the reference point in the circle list
    min_index = circle.index(ref_point)

    if previous_ref_point is not None:
        if ref_point[3] < previous_ref_point[3]:
            circle[min_index][3] = circle[min_index][3] + 2*np.pi
            ref_point[3] = ref_point[3] + 2*np.pi

    # x coordinate of the reference point
    ref_x = ref_point[0]

    # y coordinate of the reference point
    ref_y = ref_point[1]

    # velocity of the reference point
    ref_v = ref_point[2]

    # orientation of the reference point
    ref_psi = ref_point[3]


    # Create the message that is to be sent to the mpc controller,
    # pack all relevant information and publish it
    #h = pose_and_reference.msg.Header()
    #h.stamp = rospy.Time.now()

    msg = pose_and_reference()
    #msg.header = h

    msg.x = state.x
    msg.y = state.y
    msg.v = vel # NOT MEASURED
    msg.psi = state.yaw

    msg.ref_x = ref_x
    msg.ref_y = ref_y
    msg.ref_v = vel
    msg.ref_psi = ref_psi

    pub.publish(msg)

    if previous_ref_point == None:
        previous_ref_point = ref_point


#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('dist_finder_mocap_node', anonymous=True)
    print("Mocap node started")

    rospy.Subscriber("car_state_topic", mocap_data, callback)
    rospy.spin()
