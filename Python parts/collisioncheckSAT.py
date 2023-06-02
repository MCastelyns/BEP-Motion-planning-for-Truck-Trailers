# -*- coding: utf-8 -*-
"""
Created on Thu Jun  1 18:40:19 2023

@author: Mitchel
"""
import matplotlib.pyplot as plt
import matplotlib.patches as patches #Dit is nodig voor plotten, op een of andere manier deed normale plot het niet zo lekker
import numpy as np
# https://www.youtube.com/watch?v=Nm1Cgmbg5SQ&ab_channel=GamesWithGabe
# Seperating axes theorem
# This video does a pretty good job explaining the basic concept, and since python already has good tools for
# Calculating dot product and vector subtraction etc. We don't need to do as much work as he does in the video


def perpendicular_vector(v): #Used to find a perpendicular vector to the supplied vector v
    return np.array([-v[1], v[0]])

# Function to project a polygon(in our case a rectangle) onto an axis, returning the min and max dot products.
# These min and max values will then be compared to the other rectangle to see if the other rectangle has a point in between these min and max values
def project_polygon_to_axis(axis, polygon):
    min_projection = np.dot(axis, polygon[0])
    max_projection = min_projection
    for point in polygon[1:]:
        projection = np.dot(axis, point)
        min_projection = min(min_projection, projection)
        max_projection = max(max_projection, projection)
    return min_projection, max_projection

# Function to check if there is overlap between two ranges (min_a to max_a) and (min_b to max_b)
def is_overlap(min_a, max_a, min_b, max_b):
    return min_b <= max_a and min_a <= max_b

# Main function to check if there is a collision between two rectangles
def is_collision(rect1, rect2):
    # Convert input to numpy arrays for vector operations
    rect1 = np.array(rect1)
    rect2 = np.array(rect2)

    # Define edges of the rectangles
    edges = [rect1[1] - rect1[0], rect1[2] - rect1[1], rect2[0] - rect2[3], rect2[3] - rect2[2]]

    # Looping over each edge
    for edge in edges:
        # Finding the axis perpendicular to the current edge
        axis = perpendicular_vector(edge)
        # Projecting both rectangles onto this axis
        min_a, max_a = project_polygon_to_axis(axis, rect1)
        min_b, max_b = project_polygon_to_axis(axis, rect2)
        # Checking if there is overlap on this axis, between the projected points
        if not is_overlap(min_a, max_a, min_b, max_b):
            # If there is no overlap on this axis, rectangles do not collide
            return False
    # If there is overlap on all axes, rectangles collide
    return True

def plot_rectangles(rect1, rect2, collision):
    fig, ax = plt.subplots()
    rect1_patch = patches.Polygon(rect1, fill=False, edgecolor='red')
    rect2_patch = patches.Polygon(rect2, fill=False, edgecolor='blue')
    ax.add_patch(rect1_patch)
    ax.add_patch(rect2_patch)
    ax.set_xlim([-35, 35])
    ax.set_ylim([-35, 35])
    plt.title(f'Collision: {collision}')
    plt.grid(True)
    plt.show()

# test cases
# ([[x,y],[x,y],[x,y],[x,y]],[[x,y],[x,y],[x,y],[x,y]], True/False) waarbij die x,y de hoekpunten van rectangle 1 en 2 zijn
# in de volgorde: [rectangle1punt1, rectangle1punt2, rectangle1punt3, rectangle1punt4],[rectangle2punt1, rectangle2punt2, rectangle2punt3, rectangle2punt4]
# en de volgorde is clockwise of counterclockwise. Kan volgensmij allebei
test_cases = [
    ([[-10, -10], [10, -10], [10, 10], [-10, 10]], [[-20, -20], [0, -20], [0, 0], [-20, 0]], True),
    ([[-10, -10], [10, -10], [10, 10], [-10, 10]], [[-5, -5], [5, -5], [5, 5], [-5, 5]], True),
    ([[-10, -10], [10, -10], [10, 10], [-10, 10]], [[10, 10], [20, 10], [20, 20], [10, 20]], True),
    ([[0, 0], [0, 10], [10, 10], [10, 0]], [[0, 0], [10, 0], [10, -10], [0, -10]], True),
    ([[0, 0], [10, 0], [10*np.cos(np.pi/4), 10*np.sin(np.pi/4)], [0, 10]], [[10*np.cos(np.pi/4), -10*np.sin(np.pi/4)], [20*np.cos(np.pi/4), 0], [20*np.cos(np.pi/4), 10*np.sin(np.pi/4)], [10*np.cos(np.pi/4), 10*np.sin(np.pi/4)]], True),
    ([[0, 0], [10, 0], [10*np.cos(np.pi/4), 10*np.sin(np.pi/4)], [0, 10]], [[20, -20], [30, -20], [30, -10], [20, -10]], False),
    ([[0, 0], [10*np.cos(np.pi/4), 10*np.sin(np.pi/4)], [10*np.cos(np.pi/2), 10*np.sin(np.pi/2)], [0, 10]], [[10*np.cos(np.pi/4), -10*np.sin(np.pi/4)], [20*np.cos(np.pi/4), 0], [20*np.cos(np.pi/4), 10*np.sin(np.pi/4)], [10*np.cos(np.pi/4), 10*np.sin(np.pi/4)]], True),
    ([[0, 0], [10*np.cos(np.pi/4), 10*np.sin(np.pi/4)], [10*np.cos(np.pi/2), 10*np.sin(np.pi/2)], [0, 10]], [[20*np.cos(np.pi/3), -20*np.sin(np.pi/3)], [30*np.cos(np.pi/3), -10*np.sin(np.pi/3)], [30*np.cos(np.pi/3), 10*np.sin(np.pi/3)], [20*np.cos(np.pi/3), 20*np.sin(np.pi/3)]], False),
]

# for rect1, rect2, expected in test_cases:
#     collision = is_collision(rect1, rect2)
#     print(f'Expected: {expected}, Got: {collision}') #Print expected result and actual result from function to see if it's right
#     plot_rectangles(rect1, rect2, collision) #Plot the rectangles with a title showing if the function returned true or false