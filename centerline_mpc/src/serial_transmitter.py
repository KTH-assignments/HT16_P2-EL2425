#!/usr/bin/env python
'''
Node to receive commands and map them to PWM values.

'''

import rospy
import numpy as np
from slip_control_communications.msg import input_drive
from slip_control_communications.msg import input_model
from std_msgs.msg import Bool

pub = rospy.Publisher('drive_pwm', input_drive , queue_size=1)
em_pub = rospy.Publisher('eStop', Bool, queue_size=1)



#-------------------------------------------------------------------------------
# function to map from one range to another, similar to arduino
#-------------------------------------------------------------------------------
def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min



#-------------------------------------------------------------------------------
# callback function on occurance of drive parameters(angle & velocity)
#
# INPUTS:
# data: message of type input_model
#-------------------------------------------------------------------------------
def callback(data):

    velocity = data.velocity
    angle = data.angle
    print("Velocity: ", velocity, "Angle: ", angle)

    angle = angle - np.radians(6)

    # Do the computation
    pwm1 = arduino_map(velocity,-100,100,6554,13108);
    pwm2 = arduino_map(angle,-np.pi/3,np.pi/3,13108,6554);

    msg = input_drive()
    msg.pwm_drive = pwm1
    msg.pwm_angle = pwm2
    pub.publish(msg)



#-------------------------------------------------------------------------------
# talker
#-------------------------------------------------------------------------------
def talker():
    rospy.init_node('serial_transmitter_node', anonymous=True)
    print("[Node] Serial talker initialized")

    em_pub.publish(False)
    rospy.Subscriber("drive_parameters_topic", input_model, callback, queue_size=1)

    rospy.spin()



#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':
    talker()
