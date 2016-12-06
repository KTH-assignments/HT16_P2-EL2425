#!/usr/bin/env python
'''
Node to record the state of the vehicle

'''

import rospy
import csv
from slip_control_communications.msg import input_pid


#-------------------------------------------------------------------------------
# callback
#
# INPUT:
#   data: a message containing the pose of the vehicle
#-------------------------------------------------------------------------------
def callback(data):

    store_file = open(u'/home/li9i/centerline_pid_states_log.csv','ab')

    writer = csv.writer(store_file)
    writer.writerow([data.pid_error, data.pid_vel])

    store_file.close()


#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':

    rospy.init_node('logger_states_node', anonymous=True)
    rospy.loginfo("[Node] logger_states_node started")

    rospy.Subscriber("error_topic", input_pid, callback, queue_size=1)
    rospy.spin()
