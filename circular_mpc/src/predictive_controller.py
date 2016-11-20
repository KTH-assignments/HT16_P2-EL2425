#!/usr/bin/env python
'''
Node to solve a mpc optimization problem

'''

import rospy
import math
import numpy as np
from cvxpy import *
import scipy.linalg
from slip_control_communications.msg import pose_and_reference
from slip_control_communications.msg import input_model

# Publish the desired velocity and angle to the serial transmitter
pub = rospy.Publisher('drive_parameters_topic', input_model, queue_size=10)

# Length of front axle to center of gravity
lf = 0.17

# Length of rear axle to center of gravity
lr = 0.16

# A relevant ratio
l_q = l_r / (l_r + l_f)

timestamp_last_message = None
previous_input = 0.0

#-------------------------------------------------------------------------------
# Extracts the A, B matrices of a reduced order kinematic model
#-------------------------------------------------------------------------------
def get_model_matrices(ref_psi, v, ts):

    beta = np.arctan(l_q * np.tan(previous_input)));
    p = l_q / (l_q**2 * np.sin(previous_input)**2 + np.cos(previous_input)**2);

    matrices = []

    # A
    A_11 = 1
    A_12 = 0
    A_12 = -ts * v * np.sin(ref_psi + beta)

    A_21 = 0
    A_22 = 1
    A_23 = ts * v * np.cos(ref_psi + beta)

    A_31 = 0
    A_32 = 0
    A_33 = 1

    A = np.matrix([A_11, A_12, A_13], [A_21, A_22, A_23], [A_31, A_32, A_33]])

    # B
    B_11 = -ts * v * np.sin(ref_psi + beta) * p
    B_12 = ts * v * np.cos(ref_psi + beta) * p
    B_13 = ts * v / l_r * np.cos(beta) * p

    B = np.matrix([[B_11], [B_12], [B_13]])

    matrices.append(A)
    matrices.append(B)

    return matrices


#-------------------------------------------------------------------------------
# Finds the solution to the Lyapunov equation
#-------------------------------------------------------------------------------
def terminal_cost_penalty(A, B, Q ,R)

    return np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))



#-------------------------------------------------------------------------------
# Solves the optimization problem.
# Returns the optimum input.
#-------------------------------------------------------------------------------
def solve_optimization_problem(num_states, num_inputs, horizon, A, B, Q, R, s_0, s_ref)

    s = Variable(num_states, horizon + 1)
    u = Variable(num_inputs)

    states = []
    for t in range(horizon):
        cost = quad_form(s[:,t] - s_ref, Q) + quad_form(u[:,t], R)

        constr = [s[:,t+1] == A*s[:,t] + B*u[:,t],
                norm(u[:,t], 'inf') <= np.pi / 3,
                norm(u[:,t], 'inf') => -np.pi / 3 ]
        states.append(Problem(Minimize(cost), constr))


        # Add terminal cost
        Q_f = terminal_cost_penalty(A, B, Q, R)

        cost = cost + quad_form(s[:,t+1] - s_ref, Q_f)

        # add terminal constraints
        constr = [norm(u[:,t], 'inf') <= np.pi / 3,
                norm(u[:,t], 'inf') => -np.pi / 3 ]
        states.append(Problem(Minimize(cost), constr))

    # sum problem objectives and concatenate constraints.
    prob = sum(states)
    prob.constraints += [s[:,horizon] == 0, s[:,0] == s_0]

    return prob.solve()



#-------------------------------------------------------------------------------
# callback
#-------------------------------------------------------------------------------
def callback(data):

    global timestamp_last_message

    # Update the (not necessarily constant) sampling time
    if timestamp_last_message == None:
        ts = 0
    else:
       ts = data.header.stamp - timestamp_last_message

    timestamp_last_message = data.header.stamp

    # Unpack message
    ref_x = data.ref_x
    ref_y = data.ref_y
    ref_psi = data.ref_psi
    ref_v = data.ref_v

    x = data.x
    y = data.y
    psi = data.psi
    v = data.v

    # The model's matrices are time-variant. Calculate them.
    matrices = get_model_matrices(ref_psi, v, ts)

    A = matrices(0)
    B = matrices(1)

    # The horizon
    N = 50

    # Penalty matrices
    Q = np.matrix([[100, 0, 0], [0, 100, 0], [0, 0, 100]])
    R = 0.01

    # Initial conditions
    s_0 = np.matrix([[x], [y], [psi]])

    # Reference
    s_ref = np.matrix([[x_ref], [y_ref], [psi_ref]])


    # Solve the optimization problem
    optimal_input = solve_optimization_problem(3, 1, N, A, B, Q, R, s_0, s_ref)

    # Store the input for the next timestep (needed in calculation of beta)
    global previous_input = optimal_input


    # Pack the message to be sent to the serial_transmitter node
    msg = input_model()
    msg.velocity = v
    msg.angle = optimal_input
    pub.publish(msg)



#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':

    rospy.init_node('centerline_predictive_controller_node', anonymous = True)
    print("[Node] predictive_controller started")

    rospy.Subscriber("pose_and_reference_topic", pose_and_reference, callback)
    rospy.spin()
