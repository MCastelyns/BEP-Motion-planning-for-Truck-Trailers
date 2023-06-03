# -*- coding: utf-8 -*-
"""
Created on Fri Jun  2 07:24:26 2023

@author: Mitchel
"""
import numpy as np
import matplotlib.pyplot as plt

def rotate_points(points, angle):
    """Rotates a list of points by the given angle."""
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)], 
        [np.sin(angle), np.cos(angle)]
    ])
    return np.dot(points, rotation_matrix.T)

def translate_points(points, dx, dy):
    """Translates a list of points by the given distance."""
    return points + np.array([dx, dy])

def get_rectangles(L1, W1, L2, W2, x1, y1, theta1, beta, truck_hitch_offset, trailer_hitch_offset):
    """Calculate the positions of the truck and trailer rectangles."""
    # Define the corners of the truck rectangle relative to the rear of the truck
    truck_points = np.array([
        [L1, W1/2], 
        [L1, -W1/2], 
        [0, -W1/2], 
        [0, W1/2]
    ])
    # Rotate and translate the truck rectangle
    truck_points = rotate_points(truck_points, theta1)
    truck_points = translate_points(truck_points, x1, y1)

    # Calculate position of the hitch point on the truck
    truck_hitch_point = np.array([truck_hitch_offset, 0])
    truck_hitch_point = rotate_points(truck_hitch_point, theta1)
    truck_hitch_point = translate_points(truck_hitch_point, x1, y1)

    # Define the corners of the trailer rectangle relative to the pivot of the trailer
    trailer_points = np.array([
        [trailer_hitch_offset + 1.3, W2/2], 
        [trailer_hitch_offset + 1.3, -W2/2], 
        [-2.55, -W2/2], 
        [-2.55, W2/2]
    ])
    # Rotate and translate the trailer rectangle
    theta2 = theta1 + beta
    x2 = truck_hitch_point[0] - trailer_hitch_offset*np.cos(theta2)
    y2 = truck_hitch_point[1] - trailer_hitch_offset*np.sin(theta2)
    trailer_points = rotate_points(trailer_points, theta2)
    trailer_points = translate_points(trailer_points, x2, y2)
    
    return truck_points, trailer_points, truck_hitch_point

def plot_rectangles(truck_points, trailer_points, truck_hitch_point):
    """Plot the truck and trailer rectangles."""
    fig, ax = plt.subplots()

    # Plot truck
    truck_polygon = plt.Polygon(truck_points, fill=None, edgecolor='r')
    ax.add_patch(truck_polygon)

    # Plot trailer
    trailer_polygon = plt.Polygon(trailer_points, fill=None, edgecolor='b')
    ax.add_patch(trailer_polygon)

    # Plot hitch point on the truck
    ax.plot(truck_hitch_point[0], truck_hitch_point[1], 'go')

    ax.set_xlim(-5, 20)
    ax.set_ylim(-10, 10)
    ax.set_aspect('equal', adjustable='datalim')
    plt.show()


# Define truck and trailer dimensions
L1 = 7.05  # length of truck
W1 = 3.05  # width of truck
L2 = 12.45  # length of trailer
W2 = 2.95  # width of trailer

# Define initial state
x1 = 10  # initial x position
y1 = 0  # initial y position
theta1 = np.radians(45)  # orientation of truck in radians
beta = np.radians(-10)  # hitch angle in radians

# Hitch offset from the rear of the truck
truck_hitch_offset = 1.4 + 0.15  # from rear to pivot attachment + from pivot to trailer attachment

# Hitch offset from the front of the trailer
trailer_hitch_offset = 8.6  # from pivot (rear wheels) to front wheels (trailer attachment)

# Calculate and plot rectangles
truck_points, trailer_points, hitch_point = get_rectangles(L1, W1, L2, W2, x1, y1, theta1, beta, truck_hitch_offset, trailer_hitch_offset)
plot_rectangles(truck_points, trailer_points, hitch_point)
