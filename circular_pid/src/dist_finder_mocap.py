#!/usr/bin/env python
'''
Node to determine deviation from trajectory.

'''

import rospy
import numpy as np
from trajectory_planner import Path
from slip_control_communications.msg import input_pid
from slip_control_communications.msg import mocap_data

pub = rospy.Publisher('error_topic', input_pid, queue_size=10)
path = Path()
traj = path.get_points()
vel = 20.0

def callback(state):
    min_dist = 100.0
    min_point = ref_point = None

    # Find the closest trajectory point
    for point in traj:
        dist = np.sqrt((state.x - point[0])**2 + (state.y - point[1])**2)
        if dist < min_dist:
            min_dist = dist
            min_point = point

    # Ref as a future point in trajectory
    if min_point != None:
        ref_point = traj[(traj.index(min_point) + 15)%360]

    # Angle error to reference point
    angle_error = np.arctan2(ref_point[1] - state.y, ref_point[0] - state.x) - np.radians(state.yaw)

    while angle_error > np.pi:
        angle_error -= 2*np.pi
    while angle_error < -np.pi:
        angle_error += 2*np.pi

    # Distance to reference point
    vel_error = np.sqrt((ref_point[0] - state.x)**2 + (ref_point[1] - state.y)**2)

    msg = input_pid()
    msg.pid_error = angle_error
    msg.pid_vel = vel
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('dist_finder_mocap_node', anonymous=True)
    print("Mocap node started")

    rospy.Subscriber("car_state_topic", mocap_data, callback)
    rospy.spin()
