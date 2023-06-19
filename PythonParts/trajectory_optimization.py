from trajectory_planning import TrajectoryPlanning
from interpolate_waypoints import interpolate_waypoints
import numpy as np
import json
import casadi as ca
import time

class TrajectoryOptimization(TrajectoryPlanning):
    def __init__(self, 
                  dynamics, params, 
                  Q, R, 
                  state_bound, input_bound, obstacle_list):
        super().__init__(dynamics, params, Q, R, state_bound, input_bound)
        self._goal_state_sm = ca.SX.sym("goal_state", self._num_state, 1)
        
        # Obstacle list
        self.obstacle_list = obstacle_list
        
        # Collision avoidance dual variables, 
        # for now 4*num_obstacles, should be expanded to 8*num_obstacles (for both truck and trailer 4*)
        num_obstacles = len(obstacle_list)
        self._mus_sm =  ca.SX.sym("mus",  8*num_obstacles, self._horizon+1)
        self._lams_sm = ca.SX.sym("lams", 8*num_obstacles, self._horizon+1)
        
        # Build solver
        self._build_solver()

        
    def _obstacle_Hrep(self, obstacle):
        width = obstacle["width"]
        height = obstacle["height"]
        center = ca.DM([
            [obstacle["center"][0]],
            [obstacle["center"][1]],
        ])
        A = ca.DM([
            [ 1.,  0.],
            [ 0.,  1.],
            [-1.,  0.],
            [ 0., -1.],
        ])
        b = ca.DM([
            [width/2.],
            [height/2.],
            [width/2.],
            [height/2.],
        ])
        b += A @ center # Shift center of half-space representation 
                
        return A, b # Returns A matrix (4 by 2) and b vector (4 by 1) this is the obstacle's half-space representation  
    
    def _state_input_constraints(self):
        self._vars = []
        lb_vars = []
        ub_vars = []
        num_obstacles = len(self.obstacle_list)
        for k in range(self._horizon):
            self._vars = ca.vertcat(self._vars,
                                    self._states_sm[:,k],
                                    self._inputs_sm[:,k],
                                    self._mus_sm[:,k],
                                    self._lams_sm[:,k])
            lb_vars = ca.vertcat(lb_vars,
                                 self._state_bound['lb'],
                                 self._input_bound['lb'],
                                 ca.DM.zeros(8*num_obstacles),
                                 ca.DM.zeros(8*num_obstacles))
            ub_vars = ca.vertcat(ub_vars, 
                                 self._state_bound['ub'], 
                                 self._input_bound['ub'],
                                 ca.inf*ca.DM.ones(8*num_obstacles),
                                 ca.inf*ca.DM.ones(8*num_obstacles))
            
        # Constraint for final state (controls not needed)
        self._vars = ca.vertcat(self._vars,
                                self._states_sm[:,-1],
                                self._mus_sm[:,-1],
                                self._lams_sm[:,-1])
        lb_vars = ca.vertcat(lb_vars,
                             self._state_bound['lb'],
                             ca.DM.zeros(8*num_obstacles),
                             ca.DM.zeros(8*num_obstacles))
        ub_vars = ca.vertcat(ub_vars,
                             self._state_bound['ub'],
                             ca.inf*ca.DM.ones(8*num_obstacles),
                             ca.inf*ca.DM.ones(8*num_obstacles))
        
        return lb_vars, ub_vars
    
    def _collision_constraints(self):
        # Rectangles
        d_min = 0.2 # Why does the plot get so weird when we make d_min high, is that supposed to happen?
        Gv, gv = self._dynamics.get_vehicle_Hrep() # Half-space representation of the vehicle (still centered at 0)
        Gt, gt = self._dynamics.get_trailer_Hrep() # Half-space representation of the trailer (still centered at 0)
        g_col = [] # Empty array for collision constraints
        lb_col = [] #lb for g_col
        ub_col = [] #ub for g_col
        for k in range(0, self._horizon+1): # Loop over entire prediction horizon
            x_rear = self._states_sm[0,k] # Get the x position of rear axle from the states vector
            y_rear = self._states_sm[1,k] # Get the y position of rear axle from the states vector   
            heading = self._states_sm[2,k]# Get the heading of the vehicle from the states vector
            hitch_angle = self._states_sm[3,k] # Get the hitch angle from the states vector 
            xv_center, yv_center = self._dynamics.get_vehicle_center(x_rear, y_rear, heading) # Compute the x,y position of the center of the vehicle
            pv_center = ca.vertcat(xv_center, yv_center) # Make a vector out of this x and y component of the center
            
            # Do the same for pt_center, the center point of the trailer
            xt_center, yt_center = self._dynamics.get_trailer_center(x_rear, y_rear, heading, hitch_angle)
            pt_center = ca.vertcat(xt_center, yt_center)
            # Making rotation matrices for vehicle and trailer
            Rv = ca.SX.zeros((2, 2))
            Rv[0,0] =  ca.cos(heading)
            Rv[0,1] = -ca.sin(heading)
            Rv[1,0] =  ca.sin(heading)
            Rv[1,1] =  ca.cos(heading)
            
            Rt = ca.SX.zeros((2, 2))
            Rv[0,0] =  ca.cos(heading+hitch_angle)
            Rv[0,1] = -ca.sin(heading+hitch_angle)
            Rv[1,0] =  ca.sin(heading+hitch_angle)
            Rv[1,1] =  ca.cos(heading+hitch_angle)
            
            for i, obstacle in enumerate(self.obstacle_list):
                Ao, bo = self._obstacle_Hrep(obstacle) # Getting the obstacle half-space representation of current obstacle
                # constraint 1 for Truck: here we use mu and [0,1,2,3] and [8,9,10,11]
                g_col = ca.vertcat(g_col, gv.T @ self._mus_sm[i*8:i*8+4,k] 
                                          - (Ao @ pv_center - bo).T @ self._lams_sm[i*8:i*8+4,k] 
                                          + d_min)
                # Do it again for trailer: here we use mu and lam [4,5,6,7] and [12,13,14,15]
                g_col = ca.vertcat(g_col, gt.T @ self._mus_sm[i*8+4:i*8+8,k] 
                                          - (Ao @ pt_center - bo).T @ self._lams_sm[i*8+4:i*8+8,k] 
                                          + d_min)
                # Now we added 2 constraints
                lb_col = ca.vertcat(lb_col, -ca.inf*ca.DM.ones(1))
                lb_col = ca.vertcat(lb_col, -ca.inf*ca.DM.ones(1))
                ub_col = ca.vertcat(ub_col,  ca.DM.zeros(1))
                ub_col = ca.vertcat(ub_col,  ca.DM.zeros(1))

                # constraint equation 2 for Truck:
                g_col = ca.vertcat(g_col, Gv.T @ self._mus_sm[i*8:i*8+4,k] + Rv.T @ Ao.T @ self._lams_sm[i*8:i*8+4,k]) 
                
                # Do it again for trailer: 
                g_col = ca.vertcat(g_col, Gt.T @ self._mus_sm[i*8+4:i*8+8,k] + Rt.T @ Ao.T @ self._lams_sm[i*8+4:i*8+8,k])
                
                # Now we should add 4 lb and ub, because we added 2x2 lines of constraints
                lb_col = ca.vertcat(lb_col, -1e-5*ca.DM.ones(2)) #was 1e-5
                lb_col = ca.vertcat(lb_col, -1e-5*ca.DM.ones(2))
                ub_col = ca.vertcat(ub_col,  1e-5*ca.DM.ones(2))
                ub_col = ca.vertcat(ub_col,  1e-5*ca.DM.ones(2))
                
                
                # Constraint 3 for Truck:
                g_col = ca.vertcat(g_col, ca.norm_2(Ao.T @ self._lams_sm[i*8:i*8+4,k]) - 1)
                
                # Do it again for trailer:
                g_col = ca.vertcat(g_col, ca.norm_2(Ao.T @ self._lams_sm[i*8+4:i*8+8,k]) - 1)
                
                # Nowe we should add 2 lb and ub, because we added 2x1 lines of constraints
                lb_col = ca.vertcat(lb_col, -ca.inf*ca.DM.ones(1))
                lb_col = ca.vertcat(lb_col, -ca.inf*ca.DM.ones(1))
                ub_col = ca.vertcat(ub_col,  ca.DM.zeros(1))
                ub_col = ca.vertcat(ub_col,  ca.DM.zeros(1))
        
        return g_col, lb_col, ub_col
    
    def _final_state_constraint(self):
        g_fin = ca.vertcat([], self._states_sm[:,-1] - self._goal_state_sm)
        lb_fin = -1e-2*ca.DM.ones(g_fin.shape)
        ub_fin = 1e-2*ca.DM.ones(g_fin.shape)
        
        return g_fin, lb_fin, ub_fin
    
    def _cost_function(self):
        cost = 0.
        for k in range(self._horizon):
            cost += self._inputs_sm[:,k].T @ self._R @ self._inputs_sm[:,k]  + \
                (self._states_sm[:,k] - self._goal_state_sm).T  @ self._Q @ (self._states_sm[:,k] - self._goal_state_sm)  
        Q_f = self._Q * 100
        cost += (self._states_sm[:,-1] - self._goal_state_sm).T  @ Q_f @ (self._states_sm[:,-1] - self._goal_state_sm)
        
        return cost
    
    def _build_solver(self):
        g_dyn, lb_dyn, ub_dyn = self._dynamics_constraints()
        g_col ,lb_col, ub_col = self._collision_constraints()
        g_fin, lb_fin, ub_fin = self._final_state_constraint()
        g = ca.vertcat(g_dyn, g_col,  g_fin)
        self._lb_g = ca.vertcat(lb_dyn, lb_col, lb_fin)
        self._ub_g = ca.vertcat(ub_dyn, ub_col, ub_fin)
        
        self._lb_vars, self._ub_vars = self._state_input_constraints()        
        cost = self._cost_function()
        solver_opts = {
            'ipopt': {'max_iter': 5000, 'print_level': 5}, # max_iter was 5000 TEST: 'honor_original_bounds': 'no', 'constr_viol_tol': 1e-8
            'print_time': True,
        }               
        nlp_prob = {
            'f': cost,
            'x': self._vars,
            'p': ca.vertcat(self._init_state_sm, self._goal_state_sm),
            'g': g
        }
        self._solver = ca.nlpsol('solver','ipopt', nlp_prob, solver_opts)
        
        # self.g_dyn_fun = ca.Function("g_dyn", [self._vars, self._init_state_sm], [g_dyn])
    
    def _generate_initial_trajectory_guess(self, initial_state, goal_state):
        vars_guess = []
        num_obstacle = len(self.obstacle_list)
        for k in range(self._horizon):
            t = k/self._horizon
            state_guess = (1-t)*initial_state + t*goal_state
            input_guess = ca.DM.zeros((self._num_input))
            vars_guess = ca.vertcat(vars_guess, 
                                    state_guess, 
                                    input_guess,
                                    100.*ca.DM.ones(8*num_obstacle),
                                    ca.kron(ca.DM.ones(num_obstacle), ca.DM([100, 105, 110, 115, 100, 105, 110, 115])))
        vars_guess = ca.vertcat(vars_guess, 
                                goal_state,
                                100.*ca.DM.ones(8*num_obstacle),
                                ca.kron(ca.DM.ones(num_obstacle), ca.DM([100, 105, 110, 115, 100, 105, 110, 115])))
        return vars_guess
    
    def _hybrid_a_star_initial_trajectory(self):
        vars_guess = []
        num_obstacle = len(self.obstacle_list)

        # Get the states from the json
        with open('python-files\initialize.json', 'r') as f:
            data = json.load(f)

        # Now we can access the data from the file. For example:
        positions = np.array(data['Positions'])
        headings = np.array(data['Headings']) + np.pi/2 # Have to do this, because of differences in coordinate systems
        hitch_angles = np.array(data['HitchAngles'])
        
        # Interpolate these states using cubic spline interpolation, making as many states as the horizon 
        positions_new = interpolate_waypoints(positions, self._horizon)[0]
        headings_new = interpolate_waypoints(headings, self._horizon)[0]
        hitch_angles_new = interpolate_waypoints(hitch_angles, self._horizon)[0]

        # Now we construct the vars_guess using these interpolated values and our guesses for the dual variables
        vars_guess = []
        for k in range(self._horizon):
            state_guess = positions_new[k]
            state_guess = np.append(state_guess, headings_new[k])
            state_guess = np.append(state_guess, hitch_angles_new[k])
            state_guess = np.append(state_guess, 0) #Guess for steering angle, has to be implemented later
            state_guess = np.append(state_guess, 0) #Guess for velocity, has to be implemented later

            input_guess = ca.DM.zeros((self._num_input)) #Right now guess for input is 0, could also be set to some initial value

            vars_guess = ca.vertcat(vars_guess, 
                                    state_guess, 
                                    input_guess,
                                    100.*ca.DM.ones(8*num_obstacle),
                                    ca.kron(ca.DM.ones(num_obstacle), ca.DM([100, 105, 110, 115, 100, 105, 110, 115])))
        # Goal state, could be done in a different way when we actually implement it, for now it's just the last state of the A* trajectory
        state_guess = positions_new[-1]
        state_guess = np.append(state_guess, headings_new[-1])
        state_guess = np.append(state_guess, hitch_angles_new[-1])
        state_guess = np.append(state_guess, 0) #Guess for steering angle, has to be implemented later
        state_guess = np.append(state_guess, 0) #Guess for velocity, has to be implemented later

        # Final state outside of loop, we don't have input guess for this
        vars_guess = ca.vertcat(vars_guess, 
                                state_guess,
                                100.*ca.DM.ones(8*num_obstacle),
                                ca.kron(ca.DM.ones(num_obstacle), ca.DM([100, 105, 110, 115, 100, 105, 110, 115])))
        return vars_guess

    
    def _split_decision_variables(self, vars):
        states = []
        inputs = []
        mus = []
        lams = []
        
        num_obstacles = len(self.obstacle_list)
        num_state_input = self._num_state + self._num_input
        num_vars_per_step = num_state_input + 8*num_obstacles + 8*num_obstacles
        
        for k in range(self._horizon):
            vars_k = vars[k*num_vars_per_step:(k+1)*num_vars_per_step]
            state_input = vars_k[:num_state_input]
            dual_vars = vars_k[num_state_input:]
            states = ca.vertcat(states, state_input[:self._num_state])
            inputs = ca.vertcat(inputs, state_input[-self._num_input:])
            mus    = ca.vertcat(mus,    dual_vars[:8*num_obstacles])
            lams   = ca.vertcat(lams,   dual_vars[8*num_obstacles:2*8*num_obstacles])
        
        # Adding final step manually
        vars_final = vars[-(num_vars_per_step-self._num_input):]
        states = ca.vertcat(states, vars_final[:self._num_state])
        mus    = ca.vertcat(mus,    vars_final[self._num_state:self._num_state+8*num_obstacles])
        lams   = ca.vertcat(lams,   vars_final[self._num_state+8*num_obstacles:])

        
        states = states.reshape((self._num_state, self._horizon+1))
        inputs = inputs.reshape((self._num_input, self._horizon))
        mus    = mus.reshape((8*num_obstacles, self._horizon+1))
        lams   = lams.reshape((8*num_obstacles, self._horizon+1))
        
        # Return states input mus and lambdas
        return states.full(), inputs.full(), mus.full(), lams.full()    
    
    def plan(self, initial_state, goal_state):
        vars_guess = self._hybrid_a_star_initial_trajectory() # vars_guess = self._generate_initial_trajectory_guess(initial_state, goal_state)
        #vars_guess = self._hybrid_a_star_initial_trajectory()
        
        start = time.time()
        sol = self._solver(
                x0  = vars_guess, 
                lbx = self._lb_vars,
                ubx = self._ub_vars,
                lbg = self._lb_g,
                ubg = self._ub_g,
                p = ca.vertcat(initial_state, goal_state)
            )
        end = time.time()
        print(f"run time: {end-start}")
        vars_opt = sol['x']
        states, inputs, _, _ = self._split_decision_variables(vars_opt)
            
        # print(f"g_dyn: {self.g_dyn_fun(vars_opt, initial_state)[:40]}")

        return states, inputs