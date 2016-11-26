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
l_f = 0.17

# Length of rear axle to center of gravity
l_r = 0.16

# A relevant ratio
l_q = l_r / (l_r + l_f)

# the velocity's time constant
tau = 1.34

previous_input = [0, 0]

#-------------------------------------------------------------------------------
# Extracts the A, B matrices of a reduced order kinematic model
#-------------------------------------------------------------------------------
def get_model_matrices(psi, v, ts):

    beta = np.arctan(l_q * np.tan(previous_input[1]));
    p = l_q / (l_q**2 * np.sin(previous_input[1])**2 + np.cos(previous_input[1])**2);

    matrices = []

    # A
    A_11 = 1
    A_12 = ts * v * np.cos(psi + beta)

    A_21 = 0
    A_22 = 1

    A = np.matrix([[A_11, A_12], [A_21, A_22]])

    # B
    B_11 = 0
    B_12 = ts * v * np.cos(psi + beta) * p

    B_21 = 0
    B_22 = ts * v / l_r * np.cos(beta) * p

    B = np.matrix([[B_11, B_12], [B_21, B_22]])

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
        cost = quad_form(s[:,t] - s_ref, Q) + quad_form(u[:,t], R)

        constr = [s[:,t+1] == A*s[:,t] + B*u[:,t],
                u[0,t] >= 0,
                u[0,t] <= 14,
                u[1,t] <= np.pi / 3,
                u[1,t] >= -np.pi / 3]

        states.append(Problem(Minimize(cost), constr))


        # Add terminal cost
        #Q_f = terminal_cost_penalty(A, B, Q, R)

        #cost = cost + quad_form(s[:,t+1] - s_ref, Q_f)

        # input constraints
        states.append(Problem(Minimize(cost), constr))

    # sum problem objectives and concatenate constraints.
    prob = sum(states)
    #prob.constraints += [s[:,horizon] == s_ref, s[:,0] == s_0]
    prob.constraints += [s[:,0] == s_0]

    prob.solve()

    ret_list = [u[0,0].value, u[1,0].value]

    return ret_list



#-------------------------------------------------------------------------------
# callback
#-------------------------------------------------------------------------------
def callback(data):

    global previous_input

    # Unpack message
    x = data.x
    y = data.y
    v = data.v
    psi = data.psi

    ts = data.ts

    # The model's matrices are time-variant. Calculate them.
    matrices = get_model_matrices(psi, v, ts)

    A = matrices[0]
    B = matrices[1]

    # The horizon
    N = 20

    # Penalty matrices
    Q = np.matrix([[1, 0], [0, 1]])
    R = np.matrix([[0.01, 0], [0, 1]])

    # Initial conditions
    s_0 = np.matrix([[y], [psi]])

    # Reference
    s_ref = np.matrix([[0], [0]])


    # Solve the optimization problem
    optimum_input = solve_optimization_problem(2, 2, N, A, B, Q, R, s_0, s_ref)


    # Pack the message to be sent to the serial_transmitter node
    msg = input_model()
    msg.velocity = optimum_input[0]
    msg.angle = optimum_input[1]
    pub.publish(msg)

    rospy.loginfo('-------------')
    rospy.loginfo('opt velocity: ' + str(optimum_input[0]))
    rospy.loginfo('opt angle: ' + str(optimum_input[1]))

    # Store the input for the next timestep (needed in calculation of beta)
    previous_input = optimum_input



#-------------------------------------------------------------------------------
# main
#-------------------------------------------------------------------------------
if __name__ == '__main__':

    rospy.init_node('centerline_predictive_controller_node', anonymous = True)
    print("[Node] predictive_controller started")

    rospy.Subscriber("pose_topic", pose, callback)
    rospy.spin()
