# -*- coding: utf-8 -*-
"""
Created on Thu May 25 14:29:19 2023

@author: Mitchel
"""
import casadi as ca
import numpy as np

# Define problem parameters
T = 0.2  # Time step size [s]
N = 80  # Prediction horizon
n_states = 5  # Number of states
n_controls = 2  # Number of control inputs

xs = np.array([1, 1, 0, 0, 0])  # Reference state

# Define the symbolic optimization parameters
states = ca.SX.sym('x', n_states)  # States: x, y, theta, psi, phi
controls = ca.SX.sym('u', n_controls)  # Control inputs: u1, u2

# Define system dynamics and function
u1 = controls[0]
u2 = controls[1]
x = states[0]
y = states[1]
theta = states[2]
psi = states[3]
phi = states[4]

L1 = 5
L2 = 12
M = 0.1

rhs = ca.vertcat(
    u1 * ca.cos(theta),
    u1 * ca.sin(theta),
    (u1 * ca.tan(phi)) / L1,
    -((u1 * ca.sin(psi)) / L2) - ((((M * ca.cos(psi)) / L2) + 1) * (u1 * ca.tan(phi))) / L1,
    u2
)

f = ca.Function('f', [states, controls], [rhs])

# Define the cost function matrices
Q = ca.diag([1000, 1, 0.1, 0.1, 0.1])  # State weights
R = ca.diag([0.1, 0.1])  # Control input weights

# Optimization problem setup
# Define the optimization variables and parameters
X = ca.SX.sym('X', n_states, N+1)  # State decision variables over the prediction horizon
U = ca.SX.sym('U', n_controls, N)  # Control input decision variables over the prediction horizon
P = ca.SX.sym('P', n_states)  # Additional parameters (initial state)

# Define the initial constraint
g = [X[:, 0] - P]  # Equality constraint for initial state

# Define the objective function
obj = 0  # Initialize objective function
for k in range(N):
    obj += ca.mtimes([(U[:, k]).T, R, (U[:, k])])
    obj += ca.mtimes([(X[:, N] - xs).T, Q, (X[:, N] - xs)])

# Define the constraints
# Define the system dynamics constraints over the prediction horizon
for k in range(N):
    st = X[:, k]
    con = U[:, k]
    st_next = X[:, k+1]
    f_value = f(st, con)
    g.append(st_next - (st + T * f_value))
    
g = ca.vertcat(*g)  # Concatenate all constraints

# Create the NLP solver
# Create dict with NLP problem formulation
nlp_prob = {'f': obj, 'x': ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1)), 'g': g, 'p': P}

# Define the solver options
opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.tol': 1e-3, 'ipopt.acceptable_tol': 1e-2, 'ipopt.max_iter': 100}

# Create the solver
solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

# MPC Execution
x0 = np.array([0, 0, 0, 0, 0])  # Initial state
xs = np.array([1, 1, 0, 0, 0])  # Reference state

XX = np.zeros((n_states, N+1))  # Array to store the predicted state trajectory
UU = np.zeros((n_controls, N))  # Array to store the predicted control input trajectory
t0 = 0  # Initial time
x_current = x0  # Current state
x0 = np.zeros((5+(N*7), 1))  # Initial guess
x0[:5] = x_current.reshape(5, 1)

for i in range(N):
    # Set the parameter values
    args = {'x0': x0, 'p': x_current}

    # Solve the NLP problem
    sol = solver(**args)

    # Extract the optimal solution
    u_opt = sol['x'][-n_controls:]
    x_next = f(x_current, u_opt)

    # Store the predicted trajectory
    XX[:, i+1] = np.array(x_next).flatten()
    UU[:, i] = np.array(u_opt).flatten()
    
    # Update the initial guess for the next iteration
    x0[:5] = XX[:, i+1].reshape(5, 1)
    for j in range(N):
        idx = 5 + j * 7
        x0[idx:idx+5] = XX[:, i+1].reshape(5, 1)
        x0[idx+5:idx+7] = UU[:, i].reshape(2, 1)
    
    # Update the current state
    x_current = np.array(x_next).flatten()

    # Print the values of XX and UU
    print("Iteration:", i+1)
    print("XX:", XX)
    print("UU:", UU)
    print("==============================")






u_opt = UU[:, 0]





