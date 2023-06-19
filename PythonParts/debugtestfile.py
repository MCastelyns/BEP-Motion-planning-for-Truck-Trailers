import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

def obstacle_Hrep(obstacle):
    width = obstacle["width"]
    height = obstacle["height"]
    center = np.array([
        [obstacle["center"][0]],
        [obstacle["center"][1]],
    ])
    A = np.array([
        [ 1.,  0.],
        [ 0.,  1.],
        [-1.,  0.],
        [ 0., -1.],
    ])
    b = np.array([
        [width/2.],
        [height/2.],
        [width/2.],
        [height/2.],
    ])
    b += A @ center
                
    return A, b 

def vehicle_Hrep(params):
    Gv = np.array([
        [ 1.,  0.],
        [ 0.,  1.],
        [-1.,  0.],
        [ 0., -1.],
    ])
    gv = np.array([
        [params['L1'] / 2],
        [params['W1'] / 2],
        [params['L1'] / 2],
        [params['W1'] / 2],
    ])
    return Gv, gv

def trailer_Hrep(params):
    Gt = np.array([
        [ 1.,  0.],
        [ 0.,  1.],
        [-1.,  0.],
        [ 0., -1.],
    ])
    gt = np.array([
        [params['L2'] / 2],
        [params['W2'] / 2],
        [params['L2'] / 2],
        [params['W2'] / 2],
    ])
    return Gt, gt

def visualize_halfspace(A, b):
    # Sample points
    x = np.linspace(-10, 10, 400)
    y = np.linspace(-10, 10, 400)
    x, y = np.meshgrid(x, y)

    # Prepare mask array
    mask = np.zeros_like(x, dtype=bool)
    
    # Iterate over all points
    for i in range(x.shape[0]):
        for j in range(x.shape[1]):
            point = np.array([x[i, j], y[i, j]])
            
            # Apply inequalities to the point
            ineqs = np.less_equal(A @ point, b.flatten())

            # If all inequalities hold true, mark the point in the mask
            if np.all(ineqs):
                mask[i, j] = True

    # Plot the points that fulfill all inequalities
    plt.figure(figsize=(8,8))
    plt.plot(x[mask], y[mask], 'o')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid(True)
    plt.show()

# Define the obstacle
obstacle = {
    "width": 5,
    "height": 7,
    "center": [2, 3]
}

# Calculate the half-space representation
A, b = obstacle_Hrep(obstacle)

# Visualize obstacle halfspace
visualize_halfspace(A, b)


# Define the vehicle and trailer parameters
params = {
    "L1": 7.05,
    "W1": 3.05,
    "L2": 12.45,
    "W2": 2.95
}

# Calculate the half-space representation for vehicle and trailer
Gv, gv = vehicle_Hrep(params)
Gt, gt = trailer_Hrep(params)

# Visualize
visualize_halfspace(Gv, gv)
visualize_halfspace(Gt, gt)