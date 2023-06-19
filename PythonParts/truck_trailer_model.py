import casadi as ca
class TruckTrailerModel:
    def __init__(self, params):
        self.num_state = 6
        self.num_input = 2
        self._params = params
        
    def f(self, q, u):
        L1 = self._params['L1']
        L2 = self._params['L2']
        M  = self._params['M']
        # q := (x, y, theta, psi, phi, v)
        x, y, theta, psi, phi, v = ca.vertsplit(q, 1)
        a, omega = ca.vertsplit(u, 1)
        
        f = ca.SX.zeros(self.num_state)
        f[0] = v * ca.cos(theta)
        f[1] = v * ca.sin(theta)
        f[2] = v * ca.tan(phi) / L1
        f[3] = - v * ca.tan(phi) / L1 * (1 + M/L2 * ca.cos(psi)) - v * ca.sin(psi) / L2
        f[4] = omega
        f[5] = a
        
        return f
      
    def compute_next_state(self, x_k, u_k):
        x_next = x_k + self.f(x_k, u_k) * self._params["dt"]
        
        return x_next
    
    def get_vehicle_Hrep(self):
        Gv = ca.DM([
            [ 1.,  0.],
            [ 0.,  1.],
            [-1.,  0.],
            [ 0., -1.],
        ])
        gv = ca.DM([
            [self._params['L1'] / 2],
            [self._params['W1'] / 2],
            [self._params['L1'] / 2],
            [self._params['W1'] / 2],
        ])
        return Gv, gv
        
    def get_trailer_Hrep(self):
        Gt = ca.DM([
            [ 1.,  0.],
            [ 0.,  1.],
            [-1.,  0.],
            [ 0., -1.],
        ])
        gt = ca.DM([
            [self._params['L2'] / 2],
            [self._params['W2'] / 2],
            [self._params['L2'] / 2],
            [self._params['W2'] / 2],
        ])
        return Gt, gt
    
    def get_vehicle_center(self, x_rear, y_rear, heading):
        xv_center = x_rear + ca.cos(heading) * self._params['L1'] / 2 # 1.975 = distance from pivot (rear axle) to center
        yv_center = y_rear + ca.sin(heading) * self._params['L1'] / 2 # ""
        return(xv_center, yv_center)
    
    def get_trailer_center(self, x_rear, y_rear, heading, psi): 
        # First we determine the hitch point, from here we can determine the trailer center
        x_hitch = x_rear - ca.cos(heading) * self._params['M'] # I think 0.15 = self._params[M]
        y_hitch = y_rear - ca.sin(heading) * self._params['M'] 
        xt_center = x_hitch - ca.cos(heading + psi) * self._params['L2'] / 2 # 
        yt_center = y_hitch - ca.sin(heading + psi) * self._params['L2'] / 2 # 
        return(xt_center, yt_center)