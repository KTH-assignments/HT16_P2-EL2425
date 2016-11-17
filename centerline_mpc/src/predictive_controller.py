#!/usr/bin/env python
'''
Node to solve a mpc optimization problem

'''

import rospy
import math
import numpy as np
from cvxpy import *
import scipy.linalg
from slip_control_communications.msg import pose
from slip_control_communications.msg import input_model

# Publish the desired velocity and angle to the serial transmitter
pub = rospy.Publisher('drive_parameters_topic', input_model, queue_size=10)

# Length of front axle to center of gravity
lf = 0.17

# Length of rear axle to center of gravity
lr = 0.16

global timestamp_last_message = None

#-------------------------------------------------------------------------------
# Extracts the A, B matrices of a reduced order kinematic model whose states
# are y and psi, and whose only input is delta
#-------------------------------------------------------------------------------
def get_model_matrices(psi, v, ts):

    matrices = []

    A_11 = 1
    A_12 = ts * v * cos(psi)
    A_21 = 0
    A_22 = 1
    A = np.matrix([A_11, A_12],[A_21, A_22]])

    B_11 = ts * float(l_r) * v / (l_r + l_f) * cos(phi)
    B_12 = ts * float(v) / (l_r + l_f)
    B = np.matrix([[B_11], [B_12]])

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
def solve_optimization_problem(num_states, num_inputs, horizon, A, B, Q, R, s_0)
    s = Variable(num_states, horizon + 1)
    u = Variable(num_inputs)

    states = []
    for t in range(horizon):
        cost = quad_form(s[:,t+1], Q) + quad_form(u[:,t], R)
        constr = [s[:,t+1] == A*s[:,t] + B*u[:,t],
                norm(u[:,t], 'inf') <= np.pi / 3,
                norm(u[:,t], 'inf') => -np.pi / 3 ]
        states.append(Problem(Minimize(cost), constr))


        # Add terminal cost
        Q_f = terminal_cost_penalty(A, B, Q, R)

        cost = cost + quad_form(s[:,t+1], Q_f)

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

    # Update the (not necessarily constant) sampling time
    if timestamp_last_message == None:
            ts = 0
    else:
            ts = data.header.stamp - timestamp_last_message


    # Unpack message
    y = data.y
    psi = data.psi
    v = data.v

    # The model's matrices are time-variant. Grab them.
    matrices = get_model_matrices(psi, v, ts)

    A = matrices(0)
    B = matrices(1)

    # The horizon
    N = 50

    # Penalty matrices
    Q = np.matrix([[0.001, 0], [0 1]])
    R = 0.1

    # Initial conditions
    x_0 = np.matrix([[psi], [v]])


    # Solve the optimization problem
    optimal_input = solve_optimization_problem(2, 1, N, A, B, Q, R, x_0)


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

    rospy.Subscriber("pose_topic", pose, callback)
    rospy.spin()
