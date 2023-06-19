import casadi as ca

class TrajectoryPlanning:
    def __init__(self,
                 dynamics, params, 
                 Q, R, 
                 state_bound, input_bound):
        self._dynamics = dynamics  

        self._horizon = params['horizon']
        self._num_state = dynamics.num_state
        self._num_input = dynamics.num_input

        self._states_sm = ca.SX.sym("states", self._num_state, self._horizon+1)
        self._inputs_sm = ca.SX.sym("inputs", self._num_input, self._horizon)
        
        self._init_state_sm = ca.SX.sym("initial_state", self._num_state, 1)
                     
        # (x, y, theta, psi, phi, v) lower and upper bounds
        self._state_bound = state_bound  
        self._input_bound = input_bound
        
        # Cost matrices
        self._Q = Q
        self._R = R
      
    
    def _dynamics_constraints(self):
        g_dyn = [] 
        g_dyn = ca.vertcat(g_dyn, self._states_sm[:,0] - self._init_state_sm)    
        for k in range(self._horizon):
            g_dyn = ca.vertcat(g_dyn, self._states_sm[:,k+1] - self._dynamics.compute_next_state(self._states_sm[:, k], self._inputs_sm[:, k]))    
        lb_dyn = ca.DM.zeros(g_dyn.shape) # -testvalue*ca.DM.ones(g_dyn.shape)
        ub_dyn = ca.DM.zeros(g_dyn.shape) #  testvalue*ca.DM.ones(g_dyn.shape)
        
        return g_dyn, lb_dyn, ub_dyn
    
    def _state_input_constraints(self):
        self._vars = []
        lb_vars = []
        ub_vars = []
        for k in range(self._horizon):
            self._vars = ca.vertcat(self._vars,
                                    self._states_sm[:,k],
                                    self._inputs_sm[:,k])
            lb_vars = ca.vertcat(lb_vars,
                                 self._state_bound['lb'],
                                 self._input_bound['lb'])
            ub_vars = ca.vertcat(ub_vars, 
                                 self._state_bound['ub'], 
                                 self._input_bound['ub'])
        
        self._vars = ca.vertcat(self._vars,
                                self._states_sm[:,-1])
        lb_vars = ca.vertcat(lb_vars,
                             self._state_bound['lb'])
        ub_vars = ca.vertcat(ub_vars,
                             self._state_bound['ub'])
        
        return lb_vars, ub_vars
    
    def _split_decision_variables(self, vars):
        states = []
        inputs = []
        
        num_state_input = self._num_state + self._num_input
        num_vars_per_step = num_state_input
        
        for k in range(self._horizon):
            vars_k = vars[k*num_vars_per_step:(k+1)*num_vars_per_step]
            state_input = vars_k[:num_state_input]
            states = ca.vertcat(states, state_input[:self._num_state])
            inputs = ca.vertcat(inputs, state_input[-self._num_input:])
        
        # Adding final step manually
        vars_final = vars[-(num_vars_per_step-self._num_input):]
        states = ca.vertcat(states, vars_final[:self._num_state])

        
        states = states.reshape((self._num_state, self._horizon+1))
        inputs = inputs.reshape((self._num_input, self._horizon))
        
        # Return states input mus and lambdas
        return states.full(), inputs.full()
    
    
   