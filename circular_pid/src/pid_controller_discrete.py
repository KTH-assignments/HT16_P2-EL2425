#!/usr/bin/env python
'''
Node for PID control.

'''

import rospy
import numpy as np
from slip_control_communications.msg import input_model
from slip_control_communications.msg import input_pid
from slip_control_communications.msg import mocap_data

sim_rate = 10
kp = 1.1
ki = 0.01
kd = 0.5

error_t_minus_one = 0.0
error_t_minus_two = 0.0
input_t_minus_one = 0.0

integral_error = 0.0
angle = 0.0

pub = rospy.Publisher('drive_parameters_topic', input_model, queue_size=1)


#-------------------------------------------------------------------------------
# control
#-------------------------------------------------------------------------------
def control(data):
    global prev_error
    global integral_error
    global kp
    global ki
    global kd
    global angle

    error = data.pid_error

    error_t_minus_two = error_t_minus_one
    error_t_minus_one = error
    input_t_minus_one = angle

    integral_error += error


    angle = input_t_minus_one
            + (kp + ki * sim_rate + kd / sim_rate) * error
            + (-kp -2 * kd / sim_rate) * error_t_minus_one
            + kd / sim_rate * error_t_minus_two

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
    print("Listening to error")

    rospy.Subscriber("error_topic", input_pid, control)
    rospy.spin()
