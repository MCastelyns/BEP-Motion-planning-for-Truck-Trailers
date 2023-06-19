import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.patches import Rectangle
import numpy as np
from math import sin, cos, pi

def draw_truck_trailer(pose, params):
    """
    Args:
        pose (array): 
            (x, y): center of the tractor rear axle
            theta: tractor heading
            psi: hitch angle
        params (dict):
            M: 
            L1:
            L2:
            W1:
            W2:
    """
    def Rot(angle):
        R = np.array([
            [cos(angle), -sin(angle)],
            [sin(angle),  cos(angle)]
        ])
        return R    
        
    x, y, theta, psi = pose
    M = params["M"]
    L1 = params["L1"]
    L2 = params["L2"]
    W1 = params["W1"]
    W2 = params["W2"]
    
    rec_tractor = Rectangle((0., 0.), L1, W1, alpha=1)
    rec_trailer = Rectangle((0., 0.), L2, W2, alpha=0.5)
    
    pc_tractor = np.array([x, y]) - Rot(theta) @ np.array([-L1/2, 0.]) #was -L1/2
    T_tractor = mpl.transforms.Affine2D().rotate_around(pc_tractor[0], pc_tractor[1], theta) + plt.gca().transData
    rec_tractor.set_transform(T_tractor)
    rec_tractor.set_xy((pc_tractor[0]-L1/2., pc_tractor[1]-W1/2.))
    
    p_trailer = np.array([x, y]) + Rot(theta) @ np.array([-M, 0.]) # was -M
    pc_trailer = p_trailer + Rot(theta+psi) @ np.array([-L2/2, 0.]) # was L2/2
    T_trailer = mpl.transforms.Affine2D().rotate_around(pc_trailer[0], pc_trailer[1], theta+psi) + plt.gca().transData
    rec_trailer.set_transform(T_trailer)
    rec_trailer.set_xy((pc_trailer[0]-L2/2., pc_trailer[1]-W2/2.))
        
    plt.gca().add_patch(rec_tractor)
    plt.gca().add_patch(rec_trailer)
    
if __name__ == "__main__":
    pose = np.array([1., 1., pi/6, pi/6]) # heading = pi/3 psi = -pi/6
    params = {"M": 0.15, "L1": 7.05, "L2": 12.45, "W1": 3.05, "W2": 2.95}
    #OLD: params = {"M": 0.1, "L1": 5., "L2": 12., "W1": 2.5, "W2": 2.5}
    
    # Seperate the state variables to make the code easier to understand
    x_rear  = pose[0]
    y_rear  = pose[1]
    heading = pose[2]
    psi     = pose[3] #hitch angle
        
    #ADD calculated centerpoints of truck and trailer to plot, to make sure they're calculated correctly
    xv_center = pose[0] + np.cos(pose[2]) * params['L1'] / 2 # 1.975 = distance from pivot (rear axle) to center
    yv_center = pose[1] + np.sin(pose[2]) * params['L1'] / 2 # ""
    
    # First we determine the hitch point, from here we can determine the trailer center
    x_hitch = x_rear - np.cos(heading) * params['M'] # I think 0.15 = self._params[M]
    y_hitch = y_rear - np.sin(heading) * params['M'] 
    xt_center = x_hitch - np.cos(heading + psi) * params['L2'] / 2 # 
    yt_center = y_hitch - np.sin(heading + psi) * params['L2'] / 2 # 
    
    
    draw_truck_trailer(pose, params)
    plt.plot(xv_center, yv_center, "og")
    plt.plot(xt_center, yt_center, "or")
    plt.axis("equal")
    plt.grid(True)
    plt.show()