#!/usr/bin/env python
'''
Node to record the state of the vehicle

'''

import rospy
import csv
import numpy as np
from slip_control_communications.msg import mocap_data


#-------------------------------------------------------------------------------
# callback
#
# INPUT:
#   data: a message containing the pose of the vehicle and its references
#-------------------------------------------------------------------------------
def callback(data):

    store_file = open(u'/home/li9i/circular_pid_states_log.csv','ab')

    writer = csv.writer(store_file)
    writer.writerow([data.x, data.y, data.v, np.radians(data.yaw)])

    store_file.close()


#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':

    rospy.init_node('logger_states_node', anonymous=True)
    rospy.loginfo("[Node] logger_states_node started")

    rospy.Subscriber("car_state_topic", mocap_data, callback, queue_size=1)
    rospy.spin()
