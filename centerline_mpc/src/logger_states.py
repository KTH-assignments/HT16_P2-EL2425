#!/usr/bin/env python
'''
Node to record the state of the vehicle

'''

import rospy
import csv
from slip_control_communications.msg import pose


#-------------------------------------------------------------------------------
# callback
#
# INPUT:
#   data: a message containing the pose of the vehicle
#-------------------------------------------------------------------------------
def callback(data):

    store_file = open(u'/home/li9i/centerline_mpc_states_log.csv','ab')

    writer = csv.writer(store_file)
    writer.writerow([data.y, data.psi])

    store_file.close()


#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':

    rospy.init_node('logger_states_node', anonymous=True)
    rospy.loginfo("[Node] logger_states_node started")

    rospy.Subscriber("pose_topic", pose, callback, queue_size=1)
    rospy.spin()
