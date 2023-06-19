import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, tan, pi
from draw import draw_truck_trailer
from matplotlib.patches import Rectangle
import math
from truck_trailer_model import TruckTrailerModel
from trajectory_optimization import TrajectoryOptimization
from get_obstacles import get_obstacles
from get_initial_goal_states import get_initial_goal_states
from estimate_horizon import estimate_horizon
import casadi as ca
import copy
import os
import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches

do_planning = True

def f_dyn(q, u, params):
    L1 = params['L1']
    L2 = params['L2']
    M  = params['M']
    x, y, theta, psi, phi, v = q
    a, omega = u
    q_dot = np.zeros(q.shape)
    q_dot[0] = v * cos(theta)
    q_dot[1] = v * sin(theta)
    q_dot[2] = v * tan(phi) / L1
    q_dot[3] = - v * tan(phi) / L1 * (1 + M/L2 * cos(psi)) - v * sin(psi) / L2
    q_dot[4] = omega
    q_dot[5] = a
    
    return q_dot

def update(q, u, params):
    q_ = q + f_dyn(q, u, params)*params["dt"]
    return q_
    
if __name__ == "__main__":
    dt = 0.1
    #horizon_s = estimate_horizon()
    #horizon = int(horizon_s / dt) #Now we convert the time in seconds to amount of timesteps using dt, and force it to be an int
    # Params: 1 = truck, 2 = trailer, M = hitch offset/length
    horizon = 200
    params = {"M": 0.15, 
              "L1": 7.05, "L2": 12.45, 
              "W1": 3.05, "W2": 2.95,
              "dt": dt,
              "horizon": horizon}
    #params = {"M": 0.1, 
    #          "L1": 5., "L2": 12., 
    #          "W1": 2.5, "W2": 2.5,
    #          "dt": dt,
    #          "horizon": 200}        
    model = TruckTrailerModel(params)
    

    # Either import obstacles from JSON (using get_obstacles()) or use pre-set obstacles for testing, just uncomment one line out of the following 3:
    obstacle_list = get_obstacles()
    #obstacle_list = [{"center": (20, -25), "width": 14, "height": 20},{"center": (40, -25), "width": 14, "height": 20}] # Working case
    #obstacle_list = [{"center": (5, -5), "width": 10, "height": 10}] # Not working case

    Q = np.eye(6)
    Q[0, 0] = 1.  # x position 
    Q[1, 1] = 1.  # y position 
    Q[2, 2] = 1.  # truck heading 
    Q[3, 3] = 1.  # hitch angle 
    Q[4, 4] = 1.  # steering angle 
    Q[5, 5] = 1.  # speed 
    R = np.eye(2)
    R[0, 0] = 10.  # acceleration 
    R[1, 1] = 10.  # steering speed 
    # (x, y, theta, psi, phi, v) lower and upper bounds 
    state_bound = {"lb": ca.DM([-ca.inf, -ca.inf, -ca.inf, -ca.pi/3., -ca.pi/4., -5.]), #heading constraint was -pi,pi
                   "ub": ca.DM([ ca.inf,  ca.inf,  ca.inf,  ca.pi/3.,  ca.pi/4.,  10.])}      
    input_bound = {"lb": ca.DM([-5, -ca.pi/2]),
                   "ub": ca.DM([ 5,  ca.pi/2])}
    planner = TrajectoryOptimization(model, params,
                                     Q, R, 
                                     state_bound, input_bound, obstacle_list)
    
    # Import initial state and goal state from JSON (using get_initial_goal_states()), or use pre-set ones for testing, 
    initial_state, goal_state = get_initial_goal_states()
    # These imported states don't have speed as state, but initial and final state speed should be 0 anyways.
    initial_state.append(0)
    initial_state.append(0)
    goal_state.append(0)
    goal_state.append(0)

    initial_state = np.array(initial_state)
    goal_state = np.array(goal_state)

    # Manually setting goal state for heading and hitch_angle to exact values
    # goal_state[2] = pi/2 
    # goal_state[3] = 0
    # initial_state = np.array([0., 3., 0., 0., 0., 0.])
    # goal_state = np.array([30, -20., pi/2, 0., 0., 0.])
    
    dir_path = os.path.dirname(os.path.realpath(__file__))
    if do_planning == True:
        states, inputs =  planner.plan(initial_state, goal_state)  
        np.savetxt(dir_path+'/data/state_traj.txt', states)
        np.savetxt(dir_path+'/data/input_traj.txt', inputs)
    else:
        states = np.loadtxt(dir_path+'/data/state_traj.txt')
        inputs = np.loadtxt(dir_path+'/data/input_traj.txt')
    
    state = copy.deepcopy(initial_state)
    x =  [state[0]]
    y = [state[1]]
    theta = [state[2]]
    psi = [state[3]]
    phi = [state[4]]
    v = [state[5]]
    
    k = 0



    while k < params["horizon"]:
        u_con = inputs[:,k]
        state = update(state, u_con, params)
        
        x.append(state[0])
        y.append(state[1])
        theta.append(state[2])
        psi.append(state[3])
        phi.append(state[4])
        v.append(state[5])
        
        
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
        
        # Add plot of hybrid A* (reference trajectory)
        # Get the states from the json
        with open('python-files\initialize.json', 'r') as f:
            data = json.load(f)

        
        positions = np.array(data['Positions'])
        plt.plot(positions[:,0], positions[:,1], "-g", label="initial guess")

        # Plot the trajectory from the optimization
        plt.plot(states[0,:], states[1,:], "-r", label="course")
        plt.plot(x, y, "ob", label="trajectory")
        for obstacle in obstacle_list:
            w = obstacle["width"]
            h = obstacle["height"]
            c = obstacle["center"]
            xy = (c[0]-w/2, c[1]-h/2)
            plt.gca().add_patch(Rectangle(xy, w, h))
        draw_truck_trailer(pose=state[:4], params=params)

        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.pause(dt)
        # plt.pause(0.1)

        k += 1

    plt.show()

