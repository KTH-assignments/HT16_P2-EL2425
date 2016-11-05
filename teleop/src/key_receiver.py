#!/usr/bin/env python

import rospy
from slip_control_communications.msg import input_drive
from slip_control_communications.msg import input_model
from std_msgs.msg import Bool

"""
 1. Subscribe to the keyboard messages (If you use the default keyboard.py, you must subcribe to "drive_paramters" which is publishing messages of "drive_param")
 2. Map the incoming values to the needed PWM values
 3. Publish the calculated PWM values on topic "drive_pwm" using custom message "drive_values"
"""

velocity = 0.
angle = 0.

#-------------------------------------------------------------------------------
# get_commands
#-------------------------------------------------------------------------------
def get_commands(data):
    global velocity, angle

    velocity = data.velocity
    angle = data.angle


#-------------------------------------------------------------------------------
# Receiver
#-------------------------------------------------------------------------------
def receiver():

    global velocity, angle

    rospy.init_node('key_receiver_node', anonymous=True)
    print("key receiver node started")

    rospy.Subscriber("input_model_topic", input_model, get_commands)
    pub = rospy.Publisher('drive_pwm', input_drive, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    pwm = input_drive()

    while not rospy.is_shutdown():

        pwm.pwm_drive = int(round((3277./100.)*velocity) + 9831.)
        pwm.pwm_angle = int(round((3277./100.)*angle) + 9831.)

        rospy.loginfo('throttle : ' + str(pwm.pwm_drive) + '; steer : ' + str(pwm.pwm_angle))

        pub.publish(pwm)
        rate.sleep()


#-------------------------------------------------------------------------------
# Main
#-------------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        receiver()
    except rospy.ROSInterruptException:
        pass
