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

# The reference velocity. MAKE SURE THIS IS THE SAME IN trajectory_planner.py
vel = 12.0

# Previous reference point on the circular trajectory
previous_ref_point = None

# The coordinates of the vehicle at the previous sampling time
previous_x = None
previous_y = None

# The timestamp of the last message received. Will be needed in calculating
# the actual sampling time, that is, the time between two consecutive
# callbacks
timestamp_last_message = None

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
    global previous_x
    global previous_y
    global timestamp_last_message

    # Update the (not necessarily constant) sampling time
    if timestamp_last_message == None:
        ts = 0.01
    else:
        ts =  rospy.Time.now() - timestamp_last_message
        ts = ts.to_sec()


    timestamp_last_message = rospy.Time.now()

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
    msg = pose_and_reference()

    msg.x = state.x
    msg.y = state.y
    msg.psi = state.yaw

    if previous_x is not None:
        msg.v = np.sqrt((state.x - previous_x)**2 + (state.y - previous_y)**2) / ts

    msg.ref_x = ref_x
    msg.ref_y = ref_y
    msg.ref_v = vel
    msg.ref_psi = ref_psi

    msg.ts = ts

    pub.publish(msg)

    previous_ref_point = ref_point
    previous_x = state.x
    previous_y = state.y


#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('dist_finder_mocap_node', anonymous=True)
    print("Mocap node started")

    rospy.Subscriber("car_state_topic", mocap_data, callback)
    rospy.spin()
