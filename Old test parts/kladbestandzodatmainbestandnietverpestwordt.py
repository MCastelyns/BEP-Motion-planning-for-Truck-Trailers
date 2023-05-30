# -*- coding: utf-8 -*-
"""
Created on Fri May 26 00:43:13 2023

@author: Mitchel
"""
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

T = 1  # [s] Time step size was 0.5
N = 10  # Prediction horizon was (Matlab = 80)
sim_tim = 60  # Simulation time was (Matlab = 160)

# Symbolic optimization variables
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
psi = ca.SX.sym('psi')
phi = ca.SX.sym('phi')
states = ca.vertcat(x, y, theta, psi, phi)
n_states = states.numel()

u1 = ca.SX.sym('u1')
u2 = ca.SX.sym('u2')
controls = ca.vertcat(u1, u2)
n_controls = controls.numel()

M = 0.1
L1 = 5
L2 = 12
d = 1

rhs = ca.vertcat(u1 * ca.cos(theta), u1 * ca.sin(theta), (u1 * ca.tan(phi)) / L1,
                 -((u1 * ca.sin(psi)) / L2) - ((((M * ca.cos(psi)) / L2) + 1) * (u1 * ca.tan(phi))) / L1, u2)

f = ca.Function('f', [states, controls], [rhs])

U = ca.SX.sym('U', n_controls, N)
P = ca.SX.sym('P', n_states + N * (n_states + n_controls))
X = ca.SX.sym('X', n_states, N + 1)

obj = 0
g = []

Q = ca.DM.zeros(5, 5)
Q[0, 0] = 1000  # x position cost
Q[1, 1] = 1000  # y position cost
Q[2, 2] = 0  # truck heading cost
Q[3, 3] = 10  # hitch angle cost
Q[4, 4] = 0  # steering angle cost
R = ca.DM.zeros(2, 2)
R[0, 0] = 0  # velocity cost
R[1, 1] = 0  # steering speed cost

st = X[:, 0]
g.append(st - P[0:5])

for k in range(1, N + 1):
    st = X[:, k - 1]
    con = U[:, k - 1]
    obj = obj + (st - P[7 * k - 2:7 * k + 3]).T @ Q @ (st - P[7 * k - 2:7 * k + 3])
    obj = obj + (con - P[7 * k + 3:7 * k + 5]).T @ R @ (con - P[7 * k + 3:7 * k + 5])
    st_next = X[:, k]
    f_value = f(st, con)
    st_next_euler = st + (T * f_value)
    g.append(st_next - st_next_euler)

OPT_variables = ca.vertcat(ca.reshape(X, 5 * (N + 1), 1), ca.reshape(U, 2 * N, 1))

nlp_prob = {'f': obj, 'x': OPT_variables, 'g': ca.vertcat(*g), 'p': P}

opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.tol': 1e-6, 'ipopt.acceptable_tol': 1e-8, 'ipopt.max_iter': 2000}

solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

lbg = ca.DM.zeros((n_states * (N + 1), 1))
ubg = ca.DM.zeros((n_states * (N + 1), 1))

lbx = ca.DM.zeros((n_states * (N + 1) + n_controls * N, 1))
ubx = ca.DM.zeros((n_states * (N + 1) + n_controls * N, 1))

lbx[0: n_states * (N + 1): n_states] = -ca.inf  # X lower bound
lbx[1: n_states * (N + 1): n_states] = -ca.inf  # Y lower bound
lbx[2: n_states * (N + 1): n_states] = -ca.inf  # theta lower bound heading truck
lbx[3: n_states * (N + 1): n_states] = -1  # psi lower bound hitch angle
lbx[4: n_states * (N + 1): n_states] = -np.pi / 3  # steering angle lower bound

ubx[0: n_states * (N + 1): n_states] = ca.inf  # X lower bound
ubx[1: n_states * (N + 1): n_states] = ca.inf  # Y lower bound
ubx[2: n_states * (N + 1): n_states] = ca.inf  # theta lower bound heading truck
ubx[3: n_states * (N + 1): n_states] = 1  # psi lower bound hitch angle
ubx[4: n_states * (N + 1): n_states] = np.pi / 3  # steering angle lower bound

lbx[n_states * (N + 1): 5 + (n_states + n_controls) * (N): n_controls] = -0.8  # u lower bound for u1 velocity
lbx[1 + n_states * (N + 1): 5 + (n_states + n_controls) * (N): n_controls] = -0.15  # lower bound for u2 steering change

ubx[n_states * (N + 1): 5 + (n_states + n_controls) * (N): n_controls] = 0.8  # u upper bound for u1
ubx[1 + n_states * (N + 1): 5 + (n_states + n_controls) * (N): n_controls] = 0.15  # upper bound for u2 steering change

args = {'lbg': lbg, 'ubg': ubg, 'lbx': lbx, 'ubx': ubx}

t0 = 0
x0 = np.array([[60], [20], [0], [0], [0]])  # initial condition 

xx = np.zeros((int(sim_tim / T) + 1, 5))
xx[0, :] = x0.flatten()

u0 = np.zeros((N, 2))  # two control inputs for each robot
X0 = np.tile(x0.T, (N + 1, 1))  # initialization of the states decision variables

# Start MPC
mpciter = 0
xx1 = []
u_cl = []
x_ref_traj = []
y_ref_traj = []

# the main simulation loop
while mpciter < sim_tim / T:
    current_time = mpciter * T

    args['p'] = np.zeros(7 * N + 5)
    args['p'][0:5] = x0.flatten()

    for k in range(1, N + 1):
        t_predict = current_time + k * T
        x_ref = -0.5 * t_predict + 60
        y_ref = 20 * np.cos(0.05 * t_predict)
        x_ref_traj.append(x_ref)
        y_ref_traj.append(y_ref)
        theta_ref = 0
        Psi_ref = 0
        Phi_ref = 0
        u1_ref = 0.5
        u2_ref = 0
        if x_ref <= -30:
            x_ref = -30
            y_ref = 1
            theta_ref = 0
            u1_ref = 0
            u2_ref = 0

        args['p'][7 * k - 1:7 * k + 4] = [x_ref, y_ref, theta_ref, Psi_ref, Phi_ref]
        args['p'][7 * k + 3:7 * k + 5] = [u1_ref, u2_ref]

    args['x0'] = ca.vertcat(ca.reshape(X0, n_states * (N + 1), 1), ca.reshape(u0, n_controls * N, 1))
    sol = solver(x0=args['x0'], lbx=args['lbx'], ubx=args['ubx'], lbg=args['lbg'], ubg=args['ubg'], p=args['p'])
    u = np.reshape(sol['x'][5 * (N + 1):], (N, 2))
    xx1.append(np.reshape(sol['x'][:5 * (N + 1)], (N + 1, 5)))
    u_cl.extend(u[0])

    def shift(T, t0, x0, u, f):
        st = x0
        con = u[0, :]
        f_value = f(st, con)
        st = st + (T * f_value)
        x0 = st.full()

        t0 = t0 + T
        u0 = np.concatenate((u[1:], [u[-1]]), axis=0)

        return t0, x0, u0

    t0, x0, u0 = shift(T, t0, x0, u, f)
    xx[mpciter + 1, :] = x0.flatten()
    X0 = np.concatenate((X0[:, 1:], X0[:, -1:]), axis=1)
    mpciter += 1
    print(mpciter)

# Plotting the trajectories
plt.plot(xx[:, 0], xx[:, 1], 'b-', label='Trajectory')
plt.plot(x_ref_traj, y_ref_traj, 'r--', label='Reference Trajectory')
plt.xlabel('x')
plt.ylabel('y')
plt.title('MPC Trajectory')
plt.grid(True)
plt.legend()
plt.show()

