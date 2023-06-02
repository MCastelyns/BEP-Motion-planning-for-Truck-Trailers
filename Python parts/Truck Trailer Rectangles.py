# -*- coding: utf-8 -*-
"""
Created on Fri Jun  2 07:24:26 2023

@author: Mitchel
"""
import numpy as np


# 1 is the truck 2 is the trailer
# gebaseerd op de MATLAB plot functie
def get_rectangles(L1, W1, L2, W2, M, x1, y1, theta1, beta):
    # Define the transformation function
    def transform_points(points, T):
        points = np.hstack((points, np.ones((points.shape[0], 1))))
        return np.dot(T, points.T).T[:, :2]

    # Define the function to get rectangle vertices
    def get_rectangle_vertices(L, W):
        return np.array([[L/2, W/2], [L/2, -W/2], [-L/2, -W/2], [-L/2, W/2]])

    # Calculate the position and orientation of the trailer's rear axle
    theta2 = theta1 - beta
    x2 = x1 - M * np.cos(theta1) - L2 * np.cos(theta2)
    y2 = y1 - M * np.sin(theta1) - L2 * np.sin(theta2)

    # Get the vertices for the truck and the trailer
    truck_points = get_rectangle_vertices(L1, W1)
    trailer_points = get_rectangle_vertices(L2, W2)

    # Create the transformation matrices for the truck and the trailer
    T_truck = np.array([[np.cos(theta1), -np.sin(theta1), x1],
                        [np.sin(theta1), np.cos(theta1), y1],
                        [0, 0, 1]])
    T_trailer = np.array([[np.cos(theta2), -np.sin(theta2), x2],
                          [np.sin(theta2), np.cos(theta2), y2],
                          [0, 0, 1]])

    # Apply the transformations to the points
    truck_points = transform_points(truck_points, T_truck)
    trailer_points = transform_points(trailer_points, T_trailer)

    return truck_points, trailer_points

