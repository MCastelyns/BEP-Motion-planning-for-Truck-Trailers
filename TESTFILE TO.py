# -*- coding: utf-8 -*-
"""
Created on Thu Jun  1 22:42:47 2023

@author: Mitchel
"""
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import json

# Make new class to store the variables. We can easily send all this data later to a JSON
class TrajectoryPoint:
    def __init__(self, x, y, heading, hitch_angle, steering_angle):
        self.x = x
        self.y = y
        self.heading = heading
        self.hitch_angle = hitch_angle
        self.steering_angle = steering_angle

# Import Data from JSON's
# Import Obstacles
with open('obstacles.json', 'r') as f:
    data1 = json.load(f)
x_obstacles = []
y_obstacles = []
for i, obstacle in enumerate(data1):
    # Retrieve corner coordinates (this has values still under keys 'X' and 'Y', so can't directly use this have to extract the raw values first)
    FL = obstacle['FL']
    FR = obstacle['FR']
    BL = obstacle['BL']
    BR = obstacle['BR']
    
    # Get the raw coordinate values in a list for plotting
    x_obstacles.append([FL['X'], FR['X'], BR['X'], BL['X']])
    y_obstacles.append([FL['Y'], FR['Y'], BR['Y'], BL['Y']])
    # Convert them to np arrays for better handling
    x_obstacles_np = np.array(x_obstacles) # First row is x coordinates of FL1,FR1,BR1,BL1 second row FL2,FR2,BR2,BL2 etc 
    y_obstacles_np = np.array(y_obstacles) # First row is y coordinates of FL1,FR1,BR1,BL1 second row FL2,FR2,BR2,BL2 etc 

# Import Hybrid A* path (used as initial guess for warmstarting the solver)
with open('initialize.json', 'r') as f:
    data2 = json.load(f)

# Now we make arrays for each variable. We should reconstruct the path from this later, still missing some values
# Have to make sure those are added to JSON and also properly imported here, for now we just set those to 0
positions = np.array(data2['Positions'])
headings = np.array(data2['Headings'])
hitch_angles = np.array(data2['HitchAngles'])


# Solver variables
N = 120 #Steps

# For now force N to be equal to length of positions -1, but have to fix this difference in nodes
N = len(positions)
T = 60 #Simulation time
dt = T/N #Step size

# Symbolic optimization variables
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
psi = ca.SX.sym('psi')
phi = ca.SX.sym('phi')
states = ca.vertcat(x, y, theta, psi, phi)

u1 = ca.SX.sym('u1')
u2 = ca.SX.sym('u2')
controls = ca.vertcat(u1, u2)

# Constants
M = 0.1
L1 = 5
L2 = 12

# Dynamics
rhs = ca.vertcat(u1 * ca.cos(theta), u1 * ca.sin(theta), (u1 * ca.tan(phi)) / L1, -((u1 * ca.sin(psi)) / L2) - ((((M * ca.cos(psi)) / L2) + 1) * (u1 * ca.tan(phi))) / L1, u2)
f = ca.Function('f', [states, controls], [rhs])

# Obstacles HAVE TO REPLACE WITH PROPER OBSTACLES CONSTRUCTED FROM THE CORNERPOINTS STORED IN x_obstacles_np and y_obstacles_np
obstacles = []

# Variables for full trajectory (states+controls and a OPT_variables variable to use in the solver)
X = ca.MX.sym('X', states.shape[0], N+1)
U = ca.MX.sym('U', controls.shape[0], N)
OPT_variables = ca.vertcat(ca.reshape(X, 5 * (N + 1), 1), ca.reshape(U, 2 * N, 1))

# Cost variables
x_cost = 500
y_cost = 500
theta_cost = 0
psi_cost = 2500  # Hitch angle cost
phi_cost = 250   # Steering angle cost
velocity_cost = 0
steeringspeed_cost = 250
direction_change_cost = 25  #Is nu meer soort verandering van snelheid cost
backward_driving_cost = 100  #Iets hoger dan forward, is niet zo dat backward helemaal niet mag of per se slecht is

# Cost matrices
Q = np.diag([x_cost,y_cost,theta_cost,psi_cost,phi_cost])
R = np.diag([velocity_cost, steeringspeed_cost])

# Initial and final conditions
x0 = np.array([positions[0][0], positions[0][1], headings[0], hitch_angles[0], 0]) # Initializing with values from Hybrid A*
xf = np.array([positions[-1][0], positions[-1][1], headings[-1], 0, 0]) # Final state we want hitch to be 0

# Cost function
J = 0

# Path constraints
g = []

# Shitty collision detection function, have to rewrite to use geometrics of truck and trailer
def check_collision(pos, obstacle):
    dist_to_obstacle = pos - obstacle
    return ca.mtimes([dist_to_obstacle.T, dist_to_obstacle])  # Return the square of the distance to the obstacle

# Large penalty for collisions
penalty = 1e7

# Cost and constraints over full trajectory
for k in range(N-1):  # Loop till N-1 because we are now taking k+1 for the direction change cost
    J += ca.mtimes([U[:, k].T, R, U[:, k]])  # Control cost
    J += ca.mtimes([(X[:, k] - xf).T, Q, (X[:, k] - xf)])  # State cost
    x_next = X[:, k] + dt*f(X[:, k], U[:, k])  # Euler estimate for next step

    # Add cost for change in direction of movement 
    # (weet niet hoe ik dit goed kan doen voor nu is het gewoon cost op hoge verandering in snelheid)
    J += direction_change_cost * ca.fabs(U[0, k+1] - U[0, k])
    
    # Add cost for backward driving, so the truck prefers moving forward, shouldn't be too high obviously
    J += backward_driving_cost * ca.fmax(-U[0, k], 0)
    #print(ca.fmax(-U[0, k], 0))

    # Obstacle collision
    for obstacle in obstacles:
        obstacle_distance = check_collision(X[:2, k+1], obstacle)
        J += penalty * ca.fmax(1 - obstacle_distance, 0)  # Only add penalty if within a radius of 1 from the center of the obstacle

    # Add path constraint
    g += [X[:, k+1] - x_next]

# Last step cost, extra cost if final state is different than xf, can add scaling factor to make this heavier
# But the other solution to enforcing final state is probably better (Adding another constraint, as this is a hard enforced constraint)
J += ca.mtimes([U[:, N-1].T, R, U[:, N-1]])
J += backward_driving_cost * ca.fmax(-U[0, N-1], 0)

terminal_scaling = 100 
J += ca.mtimes([(X[:, N] - xf).T, Q, (X[:, N] - xf)]) * terminal_scaling # Terminal cost


# Add path constraint for the last step
g += [X[:, N] - (X[:, N-1] + dt*f(X[:, N-1], U[:, N-1]))]


# Add boundary conditions to constraints
g += [X[:, 0] - x0]

# Hard constraint for final state (TEST, to see if this will force the final state to be as we wanted ALWAYS)
g += [X[:, N] - xf]

# Concatenate constraints
g = ca.vertcat(*g)

# Lower and upper bounds for states, done per line so it's easier to adjust the bounds for bugfixing/testing
lbx_x = -ca.inf  # Lower bound for state 'x'
lbx_y = -ca.inf  # Lower bound for state 'y'
lbx_theta =-ca.inf  # Lower bound for state 'theta'
lbx_psi = -np.pi/3  # Lower bound for state 'psi'
lbx_phi = -np.pi/3  # Lower bound for state 'phi'

ubx_x =  ca.inf  # Upper bound for state 'x'
ubx_y = ca.inf  # Upper bound for state 'y'
ubx_theta = ca.inf  # Upper bound for state 'theta'
ubx_psi =  np.pi/3  # Upper bound for state 'psi'
ubx_phi =  np.pi/3  # Upper bound for state 'phi'

# Similarly for controls:
lbx_u1 =  -4  # Lower bound for control 'u1' (velocity) m/s  4 is ongeveer 15km/u zoals in unity
lbx_u2 = -0.7  # Lower bound for control 'u2' (steering speed)

ubx_u1 =  4  # Upper bound for control 'u1' (velocity) m/s   4 is ongeveer 15km/u zoals in unity
ubx_u2 =  0.7  # Upper bound for control 'u2' (steering speed)

# Concatenate to form lbx and ubx:
lbx_states = np.tile([lbx_x, lbx_y, lbx_theta, lbx_psi, lbx_phi], (1, N+1)).flatten()
ubx_states = np.tile([ubx_x, ubx_y, ubx_theta, ubx_psi, ubx_phi], (1, N+1)).flatten()
lbx_controls = np.tile([lbx_u1, lbx_u2], (1, N)).flatten()
ubx_controls = np.tile([ubx_u1, ubx_u2], (1, N)).flatten()

lbx = np.concatenate((lbx_states, lbx_controls))
ubx = np.concatenate((ubx_states, ubx_controls))

# constraint bounds  
lbg = np.zeros(states.shape[0]*N + states.shape[0])
ubg = np.zeros(states.shape[0]*N + states.shape[0])



# Have to also add constraint bounds for the hard final state constraint (TEST)
# These define how much the states can deviate from the desired final state, so should probably not be 0, as this is way too strict
lbg = np.append(lbg, [-0.5, -0.5, -0.2, -0.2, -0.2])
ubg = np.append(ubg, [0.5, 0.5, 0.2, 0.2, 0.2])


# NLP problem
nlp = {'x': OPT_variables, 'f': J, 'g': g}

# Solver options
opts ={'ipopt.print_level': 0, 'print_time': 1, 'ipopt.acceptable_obj_change_tol': 1e-6, 'ipopt.acceptable_tol': 1e-8, 'ipopt.max_iter': 2000, }

# Create solver
solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

# Path from Hybrid A* goes here
# Test pad met onnodige 'waypoints/nodes', om te kijken of hij deze eruit kan halen (Lastigere test)
x_path = positions[:,0]
x_path = np.append(x_path, x_path[-1])

y_path = positions[:,1]
y_path = np.append(y_path, y_path[-1])

theta_path = headings
theta_path = np.append(theta_path, theta_path[-1])

psi_path = np.zeros(N+1)


phi_path = np.zeros(N+1)


path = np.array([x_path, y_path, theta_path, psi_path, phi_path])

# Initialize guess for states and controls, for now just a random example, in real implementation these will be provided by C#
init_X = path
init_U = np.zeros((controls.shape[0], N))

# Initial guess for solver
init = np.concatenate((init_X.flatten(), init_U.flatten()))

# Solve the NLP with initial guess
res = solver(x0=init, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)

# Solution
solution = res['x'].full().flatten()
X_sol = solution[:states.shape[0]*(N+1)].reshape((N + 1, 5)).T #Zal vast op een nettere manier geschreven kunnen worden, maar het werkt
U_sol = solution[states.shape[0]*(N+1):].reshape((N, 2)).T


# NEW: Make a list of TrajectoryPoints named 'trajectory' that contains all the steps in the path
trajectory = [TrajectoryPoint(X_sol[0, i], X_sol[1, i], X_sol[2, i], X_sol[3, i], X_sol[4, i]) for i in range(N+1)]

# Extract optimized path
X_opt = X_sol[0,:]
Y_opt = X_sol[1,:]
Theta_opt = X_sol[2,:]
Psi_opt = X_sol[3,:]
Phi_opt = X_sol[4,:]

# Extract reference path
X_ref = path[0,:]
Y_ref = path[1,:]
Theta_ref = path[2,:]
Psi_ref = path[3,:]
Phi_ref = path[4,:]

# Plotting
fig, ax = plt.subplots()

#'quiver' plot, hiermee kan je pijltjes voor richting laten zien. Hier geeft dit aan in welke richting de truck staat
# hij kan nog steeds achteruit rijden, dus is niet helemaal altijd even duidelijk
U, V = np.cos(Theta_opt), np.sin(Theta_opt)
ax.quiver(X_opt, Y_opt, U, V, color='b', scale=20, label='Path')

# Plot reference path, the path that was given as initial guess/warmstart
ax.plot(X_ref, Y_ref, 'g--', label='Reference Path')

# Plot the start and goal points
ax.plot(x0[0], x0[1], 'mo', label='Start')
ax.plot(xf[0], xf[1], 'ro', label='Goal')

# Loop over the obstacles
for obstacle in data1:
    # Extract the coordinates for the corners of each rectangle
    FLo = [obstacle['FL']['X'], obstacle['FL']['Y']]
    FRo = [obstacle['FR']['X'], obstacle['FR']['Y']]
    BLo = [obstacle['BL']['X'], obstacle['BL']['Y']]
    BRo = [obstacle['BR']['X'], obstacle['BR']['Y']]

    # Create a rectangle patch for the obstacle
    rectangle = patches.Polygon([FLo, FRo, BRo, BLo], closed=True, fill=False, edgecolor='r')

    # Add the obstacle to the plot
    ax.add_patch(rectangle)

# Equalize the axis scales
ax.axis('equal')


plt.grid()
plt.show()