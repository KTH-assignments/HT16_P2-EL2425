#!/usr/bin/env python
'''
Node to solve a mpc optimization problem

'''

import rospy
from dynamic_reconfigure.server import Server
from circular_mpc.cfg import circular_mpcConfig


import math
import numpy as np
from numpy import tan
from cvxpy import *
import scipy.linalg
from slip_control_communications.msg import pose_and_references
from slip_control_communications.msg import input_model



# Publish the desired velocity and angle to the serial transmitter
pub = rospy.Publisher('drive_parameters_topic', input_model, queue_size=1)

# Length of front axle to center of gravity
l_f = 0.17

# Length of rear axle to center of gravity
l_r = 0.16

# A relevant ratio
l_q = l_r / (l_r + l_f)

# the velocity's time constant
tau = 1.0122

previous_input = 0


#-------------------------------------------------------------------------------
# Extracts the A, B matrices of a reduced order kinematic model
#-------------------------------------------------------------------------------
def get_model_matrices(psi, v, ts):

    beta = np.arctan(l_q * np.tan(previous_input));
    p = l_q / (l_q**2 * np.sin(previous_input)**2 + np.cos(previous_input)**2);

    matrices = []

    # A
    A_11 = 1
    A_12 = 0
    A_13 = -ts * v * np.sin(psi + beta)

    A_21 = 0
    A_22 = 1
    A_23 = ts * v * np.cos(psi + beta)

    A_31 = 0
    A_32 = 0
    A_33 = 1

    A = np.matrix([[A_11, A_12, A_13], [A_21, A_22, A_23], [A_31, A_32, A_33]])

    # B
    B_11 = -ts * v * np.sin(psi + beta) * p
    B_21 = ts * v * np.cos(psi + beta) * p
    B_31 = ts * v / l_r * np.cos(beta) * p

    B = np.matrix([[B_11], [B_21], [B_31]])

    matrices.append(A)
    matrices.append(B)

    return matrices


#-------------------------------------------------------------------------------
# Finds the solution to the Lyapunov equation
#-------------------------------------------------------------------------------
def terminal_cost_penalty(A, B, Q ,R):

    return scipy.linalg.solve_discrete_are(A, B, Q, R)



#-------------------------------------------------------------------------------
# Solves the optimization problem.
# Returns the optimum input.
#-------------------------------------------------------------------------------
def solve_optimization_problem(num_states, num_inputs, horizon, A, B, Q, R, s_0, s_ref):

    s = Variable(num_states, horizon + 1)
    u = Variable(num_inputs, horizon)

    states = []
    for t in range(horizon):
        cost = quad_form(s[:,t] - s_ref[:,t], Q) + quad_form(u[t], R)

        constr = [s[:,t+1] == A*s[:,t] + B*u[t],
                u[t] >= -np.pi / 3,
                u[t] <= np.pi / 3]

        states.append(Problem(Minimize(cost), constr))


    # Add terminal cost
    #Q_f = terminal_cost_penalty(A, B, Q, R)
    #cost = quad_form(s[:,t+1] - s_ref[:,t+1], Q_f)
    #states.append(Problem(Minimize(cost), constr))

    # sum problem objectives and concatenate constraints.
    prob = sum(states)

    # Terminal constraint with slack variables
    #prob.constraints += [s[:,horizon] <= s_ref[:,horizon] + np.matrix([[1],[1],[np.pi/2]])]
    #prob.constraints += [s[:,horizon] >= s_ref[:,horizon] - np.matrix([[0.5],[0.5],[1]])]

    # Initial conditions constraint
    prob.constraints += [s[:,0] == s_0]

    prob.solve(solver=CVXOPT)
    #prob.solve()

    ret_value = u[0].value

    return ret_value



#-------------------------------------------------------------------------------
# callback
#-------------------------------------------------------------------------------
def callback(data):

    global N, Q_x, Q_y, Q_v, Q_psi, R_v, R_delta, previous_input

    # The horizon
    #N = 4


    # Unpack message
    x = data.x
    y = data.y
    v = data.v
    psi = data.psi

    refs_x = data.refs_x
    refs_y = data.refs_y
    refs_v = data.refs_v
    refs_psi = data.refs_psi

    ts = data.ts

    rospy.loginfo('--')
    rospy.loginfo('s_0(x): ' + str(x))
    rospy.loginfo('s_ref(x): ' + str(refs_x[0]))
    rospy.loginfo('--')
    rospy.loginfo('s_0(y): ' + str(y))
    rospy.loginfo('s_ref(y): ' + str(refs_y[0]))
    rospy.loginfo('--')
    rospy.loginfo('s_0(psi): ' + str(psi * 180 / np.pi))
    rospy.loginfo('s_ref(psi): ' + str(refs_psi[0] * 180 / np.pi))
    rospy.loginfo('--')
    rospy.loginfo('ts: ' + str(ts))
    rospy.loginfo('--')


    # The model's matrices are time-variant. Calculate them.
    matrices = get_model_matrices(psi, v, ts)

    A = matrices[0]
    B = matrices[1]


    # Penalty matrices
    #Q = np.matrix([[100, 0, 0], [0, 100, 0], [0, 0, 150]])
    #R = np.matrix([[400]])

    # Defaults
    #Q = np.matrix([[100, 0, 0], [0, 100, 0], [0, 0, 150]])
    #R = np.matrix([[500]])

    Q = np.matrix([[Q_x, 0, 0], [0, Q_y, 0], [0, 0, Q_psi]])
    R = np.matrix([[R_delta]])

    # Initial conditions
    s_0 = np.matrix([[x], [y], [psi]])

    # References. Consider the refs_XXX lists as columns and append them
    # in the appropriate order
    refs_x_matrix = np.matrix(refs_x)
    refs_y_matrix = np.matrix(refs_y)
    refs_psi_matrix = np.matrix(refs_psi)

    s_ref = np.matrix(refs_x_matrix)
    s_ref = np.append(s_ref, refs_y_matrix, axis=0)
    s_ref = np.append(s_ref, refs_psi_matrix, axis=0)

    rospy.loginfo(str(s_ref))


    # Solve the optimization problem
    optimum_input = solve_optimization_problem(3, 1, N, A, B, Q, R, s_0, s_ref)

    if optimum_input is None:
        optimum_input = 0
        rospy.loginfo('INVALID STEERING')

    optimum_input = optimum_input


    # Pack the message to be sent to the serial_transmitter node
    msg = input_model()
    msg.velocity = data.v
    msg.angle = optimum_input
    pub.publish(msg)

    rospy.loginfo('-------------')
    rospy.loginfo('opt angle: ' + str(np.degrees(optimum_input)))

    # Store the input for the next timestep (needed in calculation of beta)
    previous_input = optimum_input



#-------------------------------------------------------------------------------
# Callback for dynamically reconfigurable parameters
#-------------------------------------------------------------------------------
def dynamic_reconfigure_callback(config, level):

    global N, Q_x, Q_y, Q_v, Q_psi, R_v, R_delta

    N = config.N

    Q_x = config.Q_x
    Q_y = config.Q_y
    Q_v = config.Q_v
    Q_psi = config.Q_psi

    R_v = config.R_v
    R_delta = config.R_delta

    return config


#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':

    rospy.init_node('predictive_controller_node', anonymous = True)
    rospy.loginfo("[Node] predictive_controller_node started")

    rospy.Subscriber("pose_and_references_topic", pose_and_references, callback, queue_size=1)
    dynamic_reconfigure_server = Server(circular_mpcConfig, dynamic_reconfigure_callback)
    rospy.spin()
