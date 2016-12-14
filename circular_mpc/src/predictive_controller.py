#!/usr/bin/env python
'''
Node to solve a mpc optimization problem

'''

import rospy
from dynamic_reconfigure.server import Server
from circular_mpc.cfg import predictive_controllerConfig

import math
import numpy as np
from numpy import tan

from cvxpy import *
import scipy.linalg
import matplotlib.pyplot as plt

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

# Given that t is now, this is the input given at t-1
previous_input = 0.0

# The timestamp of the last message received. Will be needed in calculating
# the execution time, that is, the time between two consecutive
# callbacks
timestamp_last_message = None

# Linearize around state or reference?
linearize_around_state = True

# Plot trajectory, references etc.
plot = False

# Show debut information
debug = False


#-------------------------------------------------------------------------------
# Extracts the A, B matrices of a reduced order kinematic model
#-------------------------------------------------------------------------------
def get_model_matrices(psi, v, ts):

    beta = np.arctan(l_q * np.tan(previous_input))

    p = l_q / (l_q**2 * np.sin(previous_input)**2 + np.cos(previous_input)**2)

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

    matrices = []
    matrices.append(A)
    matrices.append(B)

    return matrices


#-------------------------------------------------------------------------------
# Finds the solution to the Lyapunov equation
#-------------------------------------------------------------------------------
def terminal_cost_penalty(A, B, Q ,R):

    return scipy.linalg.solve_discrete_are(A, B, Q, R)



#-------------------------------------------------------------------------------
# Solves the optimization problem for invariant A and B matrices.
# Returns the optimum input sequence.
#-------------------------------------------------------------------------------
def solve_optimization_problem_invariant(num_states, num_inputs, horizon, A, B, Q, R, s_0, s_ref):

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
#    Q_f = terminal_cost_penalty(A, B, Q, R)
    #cost = quad_form(s[:,t+1] - s_ref[:,t+1], Q_f)
    #states.append(Problem(Minimize(cost), constr))

    # sum problem objectives and concatenate constraints.
    prob = sum(states)

    # Terminal constraint with slack variables
    #slack = np.matrix([[1],[1],[np.pi/2]])
    #prob.constraints += [s[:,horizon] <= s_ref[:,horizon] + slack]
    #prob.constraints += [s[:,horizon] >= s_ref[:,horizon] - slack]

    # Initial conditions constraint
    prob.constraints += [s[:,0] == s_0]

    prob.solve(solver=CVXOPT)

    # Return the sequence of optimal inputs and according predicted states.
    ret_list = []
    list_u = []
    list_s = []
    for i in range(0, horizon):
        list_u.append(u[i].value)
        list_s.append(s[:,i])

    list_s.append(s[:,horizon])

    ret_list.append(list_u)
    ret_list.append(list_s)


    return ret_list


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

        constr = [s[:,t+1] == A[t]*s[:,t] + B[t]*u[t],
                u[t] >= -np.pi / 3,
                u[t] <= np.pi / 3]

        states.append(Problem(Minimize(cost), constr))


    # Add terminal cost
#    Q_f = terminal_cost_penalty(A, B, Q, R)
    #cost = quad_form(s[:,t+1] - s_ref[:,t+1], Q_f)
    #states.append(Problem(Minimize(cost), constr))

    # sum problem objectives and concatenate constraints.
    prob = sum(states)

    # Terminal constraint with slack variables
    #slack = np.matrix([[1],[1],[np.pi/2]])
    #prob.constraints += [s[:,horizon] <= s_ref[:,horizon] + slack]
    #prob.constraints += [s[:,horizon] >= s_ref[:,horizon] - slack]

    # Initial conditions constraint
    prob.constraints += [s[:,0] == s_0]

    prob.solve(solver=CVXOPT)

    # Return the sequence of optimal inputs and according predicted states.
    ret_list = []
    list_u = []
    list_s = []
    for i in range(0, horizon):
        list_u.append(u[i].value)
        list_s.append(s[:,i])

    list_s.append(s[:,horizon])

    ret_list.append(list_u)
    ret_list.append(list_s)


    return ret_list


#-------------------------------------------------------------------------------
# callback
#-------------------------------------------------------------------------------
def callback(data):

    global previous_input, timestamp_last_message

    # Update the execution time
    if timestamp_last_message == None:
        exec_time = 0.1
    else:
        exec_time =  rospy.Time.now() - timestamp_last_message
        exec_time = exec_time.to_sec()

        if debug:
            rospy.loginfo('Execution time = ' + str(exec_time) + ' sec')

    timestamp_last_message = rospy.Time.now()


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

    if debug:
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
        rospy.loginfo('s_0(v): ' + str(refs_v[0]))
        rospy.loginfo('--')
        rospy.loginfo('ts: ' + str(ts))
        rospy.loginfo('--')


        rospy.loginfo('-- REFERENCES --')
        rospy.loginfo(str(refs_x))
        rospy.loginfo(str(refs_y))
        rospy.loginfo(str(refs_psi))


    # Penalty matrices
    Q = np.matrix([[Q_xy, 0, 0], [0, Q_xy, 0], [0, 0, Q_psi]])

    R = np.matrix([[R_delta]])

    # Initial conditions
    s_0 = np.matrix([[x], [y], [psi]])

    # Linearize around the current state
    if linearize_around_state == True:

        # References. Consider the refs_XXX lists as columns and append them
        # in the appropriate order
        refs_x_matrix = np.matrix(refs_x)
        refs_y_matrix = np.matrix(refs_y)
        refs_psi_matrix = np.matrix(refs_psi)

        s_ref = np.matrix(refs_x_matrix)
        s_ref = np.append(s_ref, refs_y_matrix, axis=0)
        s_ref = np.append(s_ref, refs_psi_matrix, axis=0)

        # The model's matrices linearized around the current orientation
        # of the vehicle
        matrices = get_model_matrices(psi, v, ts)

        A_0 = matrices[0]
        B_0 = matrices[1]


        # Solve the optimization problem once, to obtain the predicted inputs.
        # These will give us access to the predicted states.
        predicted_inputs_and_states_invariant = solve_optimization_problem_invariant(3, 1, N, A_0, B_0, Q, R, s_0, s_ref)

        # Obtain the sequence of optimal inputs
        predicted_inputs_invariant = predicted_inputs_and_states_invariant[0]

        # Obtain the predicted states, given the sequence of optimal inputs
        predicted_states_invariant = predicted_inputs_and_states_invariant[1]

        # Obtain the A and B matrices that correspond to each predicted state
        # of the vehicle. These are needed while iterating through the prediction
        # equations in the optimization problem.
        A = [A_0]
        B = [B_0]

        for i in range(0, len(predicted_states_invariant)):

            # or maybe refs_psi[i]: linearize around the reference orientation
            ab_list = get_model_matrices(predicted_states_invariant[i].value[2], v, ts)

            A.append(ab_list[0])
            B.append(ab_list[1])


        # Solve the optimization problem with the list of time-variant A's and B's
        optimum_inputs_and_states = solve_optimization_problem(3, 1, N, A, B, Q, R, s_0, s_ref)

        # The final optimal inputs
        optimum_input = optimum_inputs_and_states[0][0]

        # The final predicted states
        optimum_states = optimum_inputs_and_states[1]

    # Linearize around the reference
    elif linearize_around_state == False:

        A = []
        B = []
        for i in range(0, len(refs_psi)):

            # linearize around the reference orientation
            ab_list = get_model_matrices(refs_psi[i], v, ts)

            A.append(ab_list[0])
            B.append(ab_list[1])


#        # The reference is now zero
        #s_ref = np.matrix([[0], [0], [0]])
        #for i in range(1, N+1):
            #s_ref = np.append(s_ref, s_ref, axis=1)

        # References. Consider the refs_XXX lists as columns and append them
        # in the appropriate order
        refs_x_matrix = np.matrix(refs_x)
        refs_y_matrix = np.matrix(refs_y)
        refs_psi_matrix = np.matrix(refs_psi)

        s_ref = np.matrix(refs_x_matrix)
        s_ref = np.append(s_ref, refs_y_matrix, axis=0)
        s_ref = np.append(s_ref, refs_psi_matrix, axis=0)

        # Solve the optimization problem with the list of time-variant A's and B's
        optimum_inputs_and_states = solve_optimization_problem(3, 1, N, A, B, Q, R, s_0, s_ref)

        # The final optimal inputs
        optimum_input = optimum_inputs_and_states[0][0]

        # The final predicted states
        optimum_states = optimum_inputs_and_states[1]


    # MOCAP blind spot: when mocap loses the car, its steering goes to 0.0,
    # therefore, if this happens, keep momentarily the same steering angle.
    # In practice this makes the difference between a circular and an
    # elliptic trajectory
    if optimum_input is None or optimum_input == 0.0:
        optimum_input = previous_input
        rospy.logwarn('INVALID STEERING')


    # Pack the message to be sent to the serial_transmitter node
    msg = input_model()
    msg.velocity = data.v
    msg.angle = optimum_input
    pub.publish(msg)

    if debug:
        rospy.loginfo('-------------')
        rospy.loginfo('opt angle: ' + str(np.degrees(optimum_input)))

    # Store the input for the next timestep (needed in calculation of beta)
    previous_input = optimum_input


    # Plot trajectory, references, predicted states
    if plot:
        plt.ion()
        plt.plot(x, y, '*')

        for i in range(0, len(refs_x)):
            plt.plot(refs_x[i], refs_y[i], 'o')

        for i in range(0, len(optimum_states)):
            plt.plot(optimum_states[i].value[0], optimum_states[i].value[1], '.')
            #plt.plot(optimum_states[i].value[0] + 4 * np.cos(optimum_states[i].value[2]), optimum_states[i].value[1] + 4 * np.sin(optimum_states[i].value[2]), 'd')

        plt.axis('equal')
        plt.draw()
        plt.pause(0.0001)



#-------------------------------------------------------------------------------
# Callback for dynamically reconfigurable parameters
#-------------------------------------------------------------------------------
def dynamic_reconfigure_callback(config, level):

    global N, Q_xy, Q_v, Q_psi, R_v, R_delta

    N = config.N

    Q_xy = config.Q_xy
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
    dynamic_reconfigure_server = Server(predictive_controllerConfig, dynamic_reconfigure_callback)
    rospy.spin()
