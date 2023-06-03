# -*- coding: utf-8 -*-
"""
Created on Fri Jun  2 07:24:26 2023

@author: Mitchel
"""
import casadi as ca
import numpy as np

# Define a rectangle class to make it easier to keep track of the rectangles
class Rectangle:
    def __init__(self, corners):
        self.corners = corners

    def __repr__(self):
        return f'Rectangle({self.corners})'

def rotate_points(points, angle):
    rotation_matrix = ca.horzcat(
        ca.vertcat(ca.cos(-angle), -ca.sin(-angle)),
        ca.vertcat(ca.sin(-angle), ca.cos(-angle))
    )
    return ca.mtimes(points, rotation_matrix.T)



def translate_points(points, dx, dy):
    return points + ca.repmat(ca.vertcat(dx, dy).T, points.shape[0], 1)


def rectangle_to_points(rectangle):
    return rectangle.corners

def get_rectangles(x, y, theta, psi): 
    L_truck1 = 5.5 
    L_truck2 = -1.55
    W_truck = 3.05 

    L_hitch = -0.15 

    L_trailer1 = 1.3 
    L_trailer2 = -11.15
    W_trailer = 2.95 

    truck_points = ca.horzcat(
        ca.vertcat(L_truck1, W_truck/2), 
        ca.vertcat(L_truck1, -W_truck/2), 
        ca.vertcat(L_truck2, -W_truck/2), 
        ca.vertcat(L_truck2, W_truck/2)
    ).T
    
    truck_points = rotate_points(truck_points, theta)
    truck_points = translate_points(truck_points, x, y)

    truck = Rectangle(truck_points)

    hitch_point = ca.DM([L_hitch, 0])
    hitch_point = ca.reshape(hitch_point, 1, 2)  # reshape to a row vector
    hitch_point = rotate_points(hitch_point, theta)
    hitch_point = translate_points(hitch_point, x, y)

    trailer_points = ca.horzcat(
        ca.vertcat(L_trailer1, W_trailer/2), 
        ca.vertcat(L_trailer1, -W_trailer/2), 
        ca.vertcat(L_trailer2, -W_trailer/2), 
        ca.vertcat(L_trailer2, W_trailer/2)
    ).T

    totaltrailerangle = theta + psi
    trailer_points = rotate_points(trailer_points, totaltrailerangle)
    trailer_points = translate_points(trailer_points, hitch_point[0], hitch_point[1])

    trailer = Rectangle(trailer_points)
    
    return truck, trailer, hitch_point

def plot_rectangles(truck, trailer, hitch_point):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots()

    truck_points = rectangle_to_points(truck)
    trailer_points = rectangle_to_points(trailer)
    
    truck_np = ca.DM(truck_points).full()
    trailer_np = ca.DM(trailer_points).full()

    truck_polygon = plt.Polygon(truck_np, fill=None, edgecolor='r')
    ax.add_patch(truck_polygon)

    trailer_polygon = plt.Polygon(trailer_np, fill=None, edgecolor='b')
    ax.add_patch(trailer_polygon)

    ax.plot(hitch_point[0], hitch_point[1], 'go')

    ax.set_xlim(-5, 20)
    ax.set_ylim(-10, 10)
    ax.set_aspect('equal', adjustable='datalim')
    plt.grid()
    plt.show()




# Define initial state
x = 10  # initial x position
y = 0  # initial y position
theta = np.radians(45)  # orientation of truck in radians
psi = np.radians(25)  # hitch angle in radians

# Calculate and plot rectangles
x_ca = ca.DM(x)
y_ca = ca.DM(y)
theta_ca = ca.DM(theta)
psi_ca = ca.DM(psi)

# truck, trailer, hitch_point = get_rectangles(x_ca, y_ca, theta_ca, psi_ca)
# plot_rectangles(truck, trailer, hitch_point)
