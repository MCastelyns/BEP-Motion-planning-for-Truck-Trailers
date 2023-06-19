from trajectory_planning import TrajectoryPlanning 
import casadi as ca
import time

class MPCTrackingControl(TrajectoryPlanning):
    def __init__(self, 
                 dynamics, params, 
                 Q, R, 
                 state_bound, input_bound):
        super().__init__(dynamics, params, Q, R, state_bound, input_bound)
        
        self._ref_states_sm = ca.SX.sym("reference_states", self._num_state, self._horizon+1)
        self._ref_inputs_sm = ca.SX.sym("reference_inputs", self._num_input, self._horizon)        
        
        self._build_solver()
    
    def _cost_function(self):
        cost = 0.
        for k in range(self._horizon):
            cost += (self._inputs_sm[:,k] - self._ref_inputs_sm[:,k]).T @ self._R @ (self._inputs_sm[:,k] - self._ref_inputs_sm[:,k]) + \
                (self._states_sm[:,k] - self._ref_states_sm[:,k]).T  @ self._Q @ (self._states_sm[:,k] - self._ref_states_sm[:,k])
        Q_f = self._Q
        cost += (self._states_sm[:,-1] - self._ref_states_sm[:,-1]).T  @ Q_f @ (self._states_sm[:,-1] - self._ref_states_sm[:,-1])
        
        return cost
        
    def _build_solver(self):
        g_dyn, lb_dyn, ub_dyn = self._dynamics_constraints()
        g = g_dyn
        self._lb_g = lb_dyn
        self._ub_g = ub_dyn
                        
        self._lb_vars, self._ub_vars = self._state_input_constraints()        
        cost = self._cost_function()
        solver_opts = {
            'ipopt': {'max_iter': 5000, 'print_level': 0},
            # 'ipopt': {'max_iter': 0, 'print_level': 5},
            'print_time': False
        }     
        # print(self._ref_inputs_sm[:,0])
        # exit()
        # print(self._ref_states_sm.reshape((-1, 1)))
        # print(self._ref_inputs_sm.reshape((-1, 1)))        
        # exit()  
        nlp_prob = {
            'f': cost,
            'x': self._vars,
            'p': ca.vertcat(self._ref_states_sm.reshape((-1, 1)), 
                            self._ref_inputs_sm.reshape((-1, 1)),
                            self._init_state_sm),
            'g': g
        }
        self._solver = ca.nlpsol('solver','ipopt', nlp_prob, solver_opts)

        self.g_dyn_fun = ca.Function("g_dyn", [self._vars, self._init_state_sm], [g_dyn])
        self.cost_fun = ca.Function("cost", [self._vars, self._ref_states_sm, self._ref_inputs_sm], [cost])
        
    def _get_initial_guess(self, reference_states, reference_inputs):
        vars_guess = []
        for k in range(self._horizon):
            state_guess = reference_states[:,k]
            input_guess = reference_inputs[:,k]
            vars_guess = ca.vertcat(vars_guess, state_guess, input_guess)
        vars_guess = ca.vertcat(vars_guess, reference_states[:,-1])
        return vars_guess

    def solve(self, 
              initial_state, 
              reference_states,
              reference_inputs):   
        reference_states_flat = reference_states.T.reshape((-1, 1))
        reference_inputs_flat = reference_inputs.T.reshape((-1, 1))
        
        vars_guess = self._get_initial_guess(reference_states, reference_inputs)
        start = time.time()
        sol = self._solver(
                x0  = vars_guess, 
                lbx = self._lb_vars,
                ubx = self._ub_vars,
                lbg = self._lb_g,
                ubg = self._ub_g,
                p = ca.vertcat(reference_states_flat, 
                               reference_inputs_flat,
                               initial_state)
            )
        end = time.time()
        print(f"run time: {end-start}")
        vars_opt = sol['x']
        
        # print(f"g_dyn: {self.g_dyn_fun(vars_guess, initial_state)}")
        
        # print(vars_guess[:20])
        # print(reference_inputs_flat)
        # print(reference_states)
        # print(reference_inputs)
        # print(vars_opt[:20])
        
        # print(f"g_dyn: {self.g_dyn_fun(vars_opt, initial_state)}")
        # print(f"cost: {self.cost_fun(vars_opt, reference_states, reference_inputs)}")
        # exit()
        
        if self._solver.stats()['success'] == False:
            print("Cannot find a solution!") 
        states, inputs = self._split_decision_variables(vars_opt)
        
        return states, inputs