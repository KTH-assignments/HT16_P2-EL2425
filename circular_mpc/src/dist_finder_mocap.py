#!/usr/bin/env python
'''
Node to determine deviation from trajectory.

'''

import rospy
import numpy as np
import std_msgs
from trajectory_planner import Path
from slip_control_communications.msg import pose_and_references
from slip_control_communications.msg import mocap_data

pub = rospy.Publisher('pose_and_reference_topic', pose_and_references, queue_size=1)
path = Path()

# The reference trajectory
circle = path.get_points()

# And the coordinates of its center, and its radius as well.
circle_center_and_radius = path.get_center_and_radius()

circle_x_0 = circle_center_and_radius[0]
circle_y_0 = circle_center_and_radius[1]
circle_r = circle_center_and_radius[2]

# Previous reference point on the circular trajectory
previous_ref_point = None

# The coordinates of the vehicle at the previous sampling time
previous_x = None
previous_y = None

# The timestamp of the last message received. Will be needed in calculating
# the actual sampling time, that is, the time between two consecutive
# callbacks
timestamp_last_message = None

# The horizon. Make sure that the same number is in the predictive
# controller script
N = 20

#-------------------------------------------------------------------------------
# callback
#
# INPUT:
#   state: measurements derived from mocap
#-------------------------------------------------------------------------------
def callback(state):
    min_dist = 1000.0
    ref_point = None
    min_index = None

    global circle
    global previous_ref_point
    global previous_x
    global previous_y
    global timestamp_last_message

    # Update the (not necessarily constant) sampling time
    if timestamp_last_message == None:
        ts = 0.1
    else:
        ts =  rospy.Time.now() - timestamp_last_message
        ts = ts.to_sec()


    timestamp_last_message = rospy.Time.now()

    # Find the point that connects the vehicle to the circle.
    # This point is tangent to the circle, and this tangent passes through
    # the position of the vehicle.
    # For visualization see http://jsfiddle.net/zxqCw/1/
    dx = circle_x_0 - state.x
    dy = circle_y_0 - state.y
    dd = np.sqrt(dx**2 + dy**2)


    # The vehicle is outside the circle
    if dd >= circle_r:

        a = np.arcsin(circle_r / dd)
        b = np.arctan2(dy, dx)

        # first quarter
        if state.x > circle_x_0 and state.y > circle_y_0:
            t = b + np.pi/2 - a + np.pi
            tan_point_x = circle_x_0 + circle_r * np.cos(t)
            tan_point_y = circle_y_0 + circle_r * np.sin(t)

        # second quarter
        if state.x < circle_x_0 and state.y > circle_y_0:
            t = np.pi -a -b
            tan_point_x = circle_x_0 + circle_r * np.cos(t)
            tan_point_y = circle_y_0 - circle_r * np.sin(t)

        # third quarter
        if state.x < circle_x_0 and state.y < circle_y_0:
            t = b + np.pi/2 - a + np.pi
            tan_point_x = circle_x_0 + circle_r * np.cos(t)
            tan_point_y = circle_y_0 + circle_r * np.sin(t)

        # fourth quarter
        if state.x > circle_x_0 and state.y < circle_y_0:
            t = np.pi -a -b
            tan_point_x = circle_x_0 + circle_r * np.cos(t)
            tan_point_y = circle_y_0 - circle_r * np.sin(t)

    # The vehicle is inside the circle
    else:

        tan_point_x = state.x
        tan_point_y = state.y



    # Find the closest trajectory point
    for point in circle:
        dist = np.sqrt((tan_point_x - point[0])**2 + (tan_point_y - point[1])**2)
        if dist < min_dist:
            min_dist = dist
            ref_point = point


    # The index of the reference point in the circle list
    min_index = circle.index(ref_point)

    #if previous_ref_point is not None:
        #if ref_point[3] < previous_ref_point[3]:
            #circle[min_index][3] = circle[min_index][3] + 2*np.pi
            #ref_point[3] = ref_point[3] + 2*np.pi

    # MEASURE the velocity of the vehicle. Ideally this would come from a
    # Kalman filter.
    if previous_x is not None:
        vel = np.sqrt((state.x - previous_x)**2 + (state.y - previous_y)**2) / ts
    else:
        vel = 0.0

    # Create the message that is to be sent to the mpc controller,
    # pack all relevant information and publish it
    msg = pose_and_references()

    msg.x = state.x
    msg.y = state.y
    msg.psi = np.radians(state.yaw)
    msg.v = vel


    # The array of references. Each is a separate point on the circle,
    # each ahead from the previous, starting from ref_point.
    # These will be the points the vehicle will have to plan to pass while
    # running the prediction equations
    refs_x = []
    refs_y = []
    refs_v = []
    refs_psi = []

    for i in range(1,N+2):
        t = ref_point[2] + vel / circle_r * ts * i
        x = circle_x_0 + circle_r * np.cos(t - np.pi/2)
        y = circle_y_0 + circle_r * np.sin(t - np.pi/2)

        refs_x.append(x)
        refs_y.append(y)
        refs_v.append(vel)
        refs_psi.append(t)

    msg.refs_x = refs_x
    msg.refs_y = refs_y
    msg.refs_v = refs_v
    msg.refs_psi = refs_psi

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

    rospy.Subscriber("car_state_topic", mocap_data, callback, queue_size=1)
    rospy.spin()
