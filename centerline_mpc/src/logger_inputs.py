#!/usr/bin/env python
'''
Node to record the inputs to the vehicle

'''

import rospy
import csv
from slip_control_communications.msg import input_model


#-------------------------------------------------------------------------------
# callback
#
# INPUT:
#   data: a message containing the inputs to the vehicle
#-------------------------------------------------------------------------------
def callback(data):

    store_file = open(u'/home/li9i/centerline_mpc_inputs_log.csv','ab')

    writer = csv.writer(store_file)
    writer.writerow([data.angle])

    store_file.close()


#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':

    rospy.init_node('logger_inputs_node', anonymous=True)
    rospy.loginfo("[Node] logger_inputs_node started")

    rospy.Subscriber("drive_parameters_topic", input_model, callback, queue_size=1)
    rospy.spin()
