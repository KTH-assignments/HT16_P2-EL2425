#!/usr/bin/env python
'''
Node for PID control.

'''

import rospy
import numpy as np
from slip_control_communications.msg import input_model
from slip_control_communications.msg import input_pid

sim_rate = 10

# 1
#kp = 2.1
#ki = 0.05
#kd = 0.1

# 2
#kp = 0.8
#ki = 0.01
#kd = 0.1

kp = 0.7
ki = 0.003
kd = 0.1



prev_angle_error = None
integral_error = 0.0
angle = 0.0

pub = rospy.Publisher('drive_parameters_topic', input_model, queue_size=1)


#-------------------------------------------------------------------------------
# control
#
# INPUTS:
#   data: a input_pid type message: velocity and angle error with respect to
#         the reference the vehicle is to track
#-------------------------------------------------------------------------------
def control(data):
    global prev_angle_error
    global integral_error
    global kp
    global ki
    global kd
    global angle

    ## Your code goes here
    # 1. Scale the error
    # 2. Apply the PID equation on error
    # 3. Make sure the error is within bounds

    angle_error = data.pid_error

    if prev_angle_error == None:
        delta_error = 0
    else:
        delta_error = angle_error - prev_angle_error

    prev_angle_error = angle_error
    integral_error += angle_error

    if abs(angle_error) < 0.01:
        integral_error = 0.

    angle = kp*angle_error + kd*delta_error*sim_rate + ki*integral_error/sim_rate #- kd*delta_error*sim_rate

    # Limit angle command between steering angle limits
    while angle > np.pi/3:
        angle = np.pi/3
    while angle < -np.pi/3:
        angle = -np.pi/3

    msg = input_model();
    msg.velocity = data.pid_vel
    msg.angle = angle
    pub.publish(msg)


#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('pid_controller_node', anonymous=True)
    print("[Node] pid_controller started: listening to error from lidar")

    rospy.Subscriber("error_topic", input_pid, control)
    rospy.spin()
