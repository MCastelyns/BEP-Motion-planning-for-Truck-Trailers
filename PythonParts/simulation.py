import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from math import cos, sin, tan, pi
from draw import draw_truck_trailer
import math
from truck_trailer_model import TruckTrailerModel
from mpc_control import MPCTrackingControl
from get_obstacles import get_obstacles
from get_initial_goal_states import get_initial_goal_states
import casadi as ca
import copy
import os

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

def do_interpolation(state_traj, input_traj, dt_1, dt_2):
    # from dt_1 to dt_2
    # dt_1 > dt_2
    N = input_traj.shape[1]
    num_state = state_traj.shape[0]
    num_input = input_traj.shape[0]
    n = math.floor(dt_1/dt_2)
    N_new = n*N
    state_traj_new = np.zeros((num_state, N_new+1))
    input_traj_new = np.zeros((num_input, N_new))
    for k in range(N):
        for m in range(n):
            t = m/n
            state_traj_new[:,k*n+m] = (1-t)*state_traj[:,k] + t*state_traj[:,k+1]
            input_traj_new[:,k*n+m] = input_traj[:,k] 
    state_traj_new[:,-1] = state_traj[:,-1]
    
    return state_traj_new, input_traj_new 

if __name__ == "__main__":
    dt_to = 0.1
    dt = 0.05
    horizon = 60
    params = {"M": 0.15, 
              "L1": 7.05, "L2": 12.45, 
              "W1": 3.05, "W2": 2.95,
              "dt": dt,
              "horizon": horizon}
            
    model = TruckTrailerModel(params)
    #obstacle_list = [{"center": (5, -5), "width": 10, "height": 10}]
    obstacle_list = get_obstacles()
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
    # (x, y, theta, psi, phi, v)
    state_bound = {"lb": ca.DM([-ca.inf, -ca.inf, -ca.pi, -ca.pi/3., -ca.pi/4., -10.]),
                   "ub": ca.DM([ ca.inf,  ca.inf,  ca.pi,  ca.pi/3.,  ca.pi/4.,  10.])}      
    input_bound = {"lb": ca.DM([-5, -ca.pi/2]),
                   "ub": ca.DM([ 5,  ca.pi/2])}
    controller = MPCTrackingControl(model, params,
                                    Q, R, 
                                    state_bound, input_bound)
    initial_state, goal_state = get_initial_goal_states()
    initial_state.append(0)
    initial_state.append(0)
    initial_state = np.array(initial_state)
    # initial_state = np.array([0., 3., 0., 0., 0., 0.])
   

    dir_path = os.path.dirname(os.path.realpath(__file__))
    ref_state_traj = np.loadtxt(dir_path+'/data/state_traj.txt')
    ref_input_traj = np.loadtxt(dir_path+'/data/input_traj.txt')
    
    ref_state_traj, ref_input_traj = do_interpolation(ref_state_traj, ref_input_traj, dt_to, dt)
    
    T_sim = 20.
    # T_sim = 10. / 30
    time = 0.
    
    state = copy.deepcopy(initial_state)
    x =  [state[0]]
    y = [state[1]]
    theta = [state[2]]
    psi = [state[3]]
    phi = [state[4]]
    v = [state[5]]

    ref_state_traj_ = np.zeros((model.num_state, horizon+1))
    ref_input_traj_ = np.zeros((model.num_input, horizon))
    N = ref_input_traj.shape[1]
    while time <= T_sim:
        k = math.floor(time/dt)
        print(f"step: {k}")
        if k + horizon <= N:
            ref_state_traj_[:,:] = ref_state_traj[:,k:k+horizon+1]
            ref_input_traj_[:,:] = ref_input_traj[:,k:k+horizon]
        elif k < N:
            print(f"horizon: {horizon}")
            print(f"k: {k}")
            print(f"N: {N}")
            ref_state_traj_[:,:N+1-k] = ref_state_traj[:,k:]
            ref_state_traj_[:,N+1-k:] = ca.repmat(ref_state_traj[:,-1], 1, horizon-N+k)
            ref_input_traj_[:,:N-k] = ref_input_traj[:,k:]
            ref_input_traj_[:,N-k:] = ca.repmat(ref_input_traj[:,-1], 1, horizon-N+k)
        else:
            ref_state_traj_[:,:] = ca.repmat(ref_state_traj[:,-1], 1, horizon+1)
            ref_input_traj_[:,:] = ca.repmat(np.zeros((model.num_input,1)), 1, horizon)
        
        states, inputs = controller.solve(state, ref_state_traj_, ref_input_traj_)     
        u_con = inputs[:,0]
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
        plt.plot(ref_state_traj[0,:], ref_state_traj[1,:], "-r", label="course")
        plt.plot(x, y, "ob", label="trajectory")
        for obstacle in obstacle_list:
            w = obstacle["width"]
            h = obstacle["height"]
            c = obstacle["center"]
            xy = (c[0]-w/2, c[1]-h/2)
            plt.gca().add_patch(Rectangle(xy, w, h))
        plt.plot(states[0,:], states[1,:], '-o', color='darkorange')
        # plt.plot(ref_state_traj[0,k:k+horizon+1], ref_state_traj[1,k:k+horizon+1], '-o')
        plt.plot(ref_state_traj_[0,:], ref_state_traj_[1,:], '-o')
        draw_truck_trailer(pose=state[:4], params=params)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.0001)
    
        time += dt
    
    plt.show()
    