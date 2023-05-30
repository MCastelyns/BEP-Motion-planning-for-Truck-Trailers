# -*- coding: utf-8 -*-
"""
Created on Thu May 25 21:36:10 2023

@author: Mitchel
"""
import numpy as np
import casadi as ca
import matplotlib.pyplot as plt

# Constants
N = 40  # MPC horizon length (timesteps)
T = 5  # [s] used for generating reference path
n_states = 4  # Number of states
n_controls = 2  # Number of control inputs
L = 2.5  # Length from truck to trailer hitch
dt = 0.1  # Time step

v_ref = 2.0  # Reference speed
look_ahead_distance = 2.0  # Look-ahead distance for reference point selection

# System dynamics
def system_model(x, u):
    x_dot = u[0] * ca.cos(x[2])
    y_dot = u[0] * ca.sin(x[2])
    theta_dot = u[0] * ca.tan(u[1]) / L
    psi_dot = u[0] * ca.sin(x[2] - x[3]) / L

    x_next = x[0] + x_dot * dt
    y_next = x[1] + y_dot * dt
    theta_next = x[2] + theta_dot * dt
    psi_next = x[3] + psi_dot * dt

    return ca.vertcat(x_next, y_next, theta_next, psi_next)

# Cost function
def cost_function(x, u, x_ref):
    Q = np.diag([1000, 1000, 1, 1])  # State error weights
    R = np.diag([10, 0.01])  # Control effort weights
    S = 10000.0  # Speed error weight

    state_error = x - x_ref
    control_effort = u
    speed_error = u[0] - v_ref

    return ca.mtimes([state_error.T, Q, state_error]) + ca.mtimes([control_effort.T, R, control_effort]) + S * speed_error**2

# Reference trajectory
def generate_reference_trajectory(N, T):
    t = np.linspace(0, T, N+1)
    x_ref = t
    y_ref = t**2/10
    theta_ref = np.arctan(2*t/10)
    return np.vstack((x_ref, y_ref, theta_ref, np.zeros_like(t)))



def look_ahead_reference_point(x, ref_traj, look_ahead_distance):
    distances = ca.sqrt((x[0] - ref_traj[0, :]) ** 2 + (x[1] - ref_traj[1, :]) ** 2)
    look_ahead_mask = distances > look_ahead_distance
    distances_masked = ca.if_else(look_ahead_mask, distances, np.inf)  # assign infinity if within look-ahead distance

    min_distance = ca.mmin(distances_masked)

    min_indices = ca.find(ca.eq(distances_masked, min_distance))

    if min_indices.numel() == 0:
        raise ValueError("Minimum distance not found.")

    min_index = min_indices[0].get_element()

    return ref_traj[:, min_index]


# Create bounds
def create_bounds(lbx_state, ubx_state, lbx_control, ubx_control):
    lbx = lbx_state * (N+1)  # lower bounds on x
    ubx = ubx_state * (N+1)  # upper bounds on x

    lbx += lbx_control * N  # lower bounds on u
    ubx += ubx_control * N  # upper bounds on u

    lbg = [0]*n_states*N  # lower bounds for g
    ubg = [0]*n_states*N  # upper bounds for g

    return lbx, ubx, lbg, ubg

# Optimization problem
X = ca.SX.sym('X', n_states, N+1)  # Predicted states
U = ca.SX.sym('U', n_controls, N)  # Control inputs

# Initialize cost and model
cost = 0
g = []  # Model constraints
ref_traj = generate_reference_trajectory(N, T)

for k in range(N):
    look_ahead_ref = look_ahead_reference_point(X[:,k], ref_traj, look_ahead_distance)
    cost += cost_function(X[:,k], U[:,k], look_ahead_ref)
    x_next = system_model(X[:,k], U[:,k])
    g.append(X[:,k+1] - x_next)

# Optimization problem definition
nlp = {'x': ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1)),
       'f': cost,
       'g': ca.vertcat(*g)}

opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.tol': 1e-3}

solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

# MPC Controller
def mpc_controller(x0, lbx, ubx, lbg, ubg):
    lbx[:n_states] = ubx[:n_states] = x0  # fix the initial state to x0

    res = solver(lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)

    x_opt = np.array(res['x']).reshape(-1, 1)
    u_opt = x_opt[n_states*(N+1):]

    return u_opt[:n_controls].reshape(-1)

# Simulation Loop
lbx_state = [-np.inf, -np.inf, -np.inf, -1]
ubx_state = [np.inf, np.inf, np.inf, 1]
lbx_control = [-15, -np.pi/3]
ubx_control = [15, np.pi/3]

lbx, ubx, lbg, ubg = create_bounds(lbx_state, ubx_state, lbx_control, ubx_control)

x0 = np.array([0, 0, np.pi/4, 0])  # Initial state

x_hist = [x0]
u_hist = []
cost_hist = []
sim_data = []

plt.figure()

for k in range(N):
    u = mpc_controller(x0, lbx, ubx, lbg, ubg)
    x0 = system_model(x0, u)
    x_hist.append(x0)
    u_hist.append(u)

    look_ahead_ref = look_ahead_reference_point(x0, ref_traj, look_ahead_distance)
    cost_value = cost_function(x0, u, look_ahead_ref)
    cost_hist.append(cost_value)

    sim_data.append(np.hstack([x0, u, cost_value]))

    plt.cla()
    plt.plot(ref_traj[0, :], ref_traj[1, :], 'r--')  # Reference trajectory
    plt.plot(np.array(x_hist)[:, 0], np.array(x_hist)[:, 1], 'b')  # Actual trajectory
    plt.draw()
    plt.pause(0.01)

x_hist = np.array(x_hist)
u_hist = np.array(u_hist)
cost_hist = np.array(cost_hist)
sim_data = np.array(sim_data)

plt.show()

print("State history:", x_hist)
print("Control history:", u_hist)
print("Cost history:", cost_hist)
print("Simulation data (state, control, cost):", sim_data)




