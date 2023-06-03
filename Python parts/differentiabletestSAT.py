# -*- coding: utf-8 -*-
"""
Created on Sat Jun  3 13:45:36 2023

@author: Mitchel
"""
import matplotlib.pyplot as plt
import matplotlib.patches as patches #Dit is nodig voor plotten, op een of andere manier deed normale plot het niet zo lekker
import numpy as np
import casadi as ca
# https://www.youtube.com/watch?v=Nm1Cgmbg5SQ&ab_channel=GamesWithGabe
# Seperating axes theorem
# This video does a pretty good job explaining the basic concept, and since python already has good tools for
# Calculating dot product and vector subtraction etc. We don't need to do as much work as he does in the video

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import casadi as ca

# Function to create perpendicular vector
def perpendicular_vector(v):
    return ca.vertcat(-v[1], v[0])

# Function to project polygon onto an axis
def project_polygon_to_axis(axis, polygon):
    min_projection = ca.mtimes(axis.T, polygon[:, 0])
    max_projection = min_projection
    for i in range(1, polygon.shape[1]):
        projection = ca.mtimes(axis.T, polygon[:, i])
        min_projection = ca.fmin(min_projection, projection)
        max_projection = ca.fmax(max_projection, projection)
    return min_projection, max_projection

# Function to check overlap between two intervals
def is_overlap(min_a, max_a, min_b, max_b):
    return ca.fmax(0, ca.fmin(max_a, max_b) - ca.fmax(min_a, min_b))

# Define the collision function
def is_collision(rect1, rect2):
    edges = ca.horzcat(rect1[:, 1] - rect1[:, 0], rect1[:, 2] - rect1[:, 1], rect2[:, 0] - rect2[:, 3], rect2[:, 3] - rect2[:, 2])

    min_distance = ca.inf
    for i in range(edges.shape[1]):
        axis = perpendicular_vector(edges[:, i])
        min_a, max_a = project_polygon_to_axis(axis, rect1)
        min_b, max_b = project_polygon_to_axis(axis, rect2)
        overlap = is_overlap(min_a, max_a, min_b, max_b)
        gap = ca.fmax(0, ca.fmin(max_a, max_b) - ca.fmax(min_a, min_b))
        min_distance = ca.fmin(min_distance, gap)
    return min_distance

# Create CasADi symbols for the rectangles
rect1_sym = ca.MX.sym('rect1', 4, 2)
rect2_sym = ca.MX.sym('rect2', 4, 2)

# Create the collision function using CasADi Function
F_is_collision = ca.Function('is_collision', [rect1_sym, rect2_sym], [is_collision(rect1_sym, rect2_sym)])

# Test cases
rect1 = np.array([[0, 0], [1, 0], [1, 1], [0, 1]])
rect2 = np.array([[1, 1], [2, 1], [2, 2], [1, 2]])
rect3 = np.array([[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]])
rect4 = np.array([[0.25, 0.25], [0.75, 0.25], [0.75, 0.75], [0.25, 0.75]])
rect5 = np.array([[0.75, 0.75], [1.25, 0.75], [1.25, 1.25], [0.75, 1.25]])

# Evaluate the function for each test case
result1 = F_is_collision(rect1, rect2)
result2 = F_is_collision(rect1, rect3)
result3 = F_is_collision(rect1, rect4)
result4 = F_is_collision(rect1, rect5)

# Print the results
print("Test 1 (expected: sqrt(2), i.e., ~1.41): ", result1)
print("Test 2 (expected: 1): ", result2)
print("Test 3 (expected: 0): ", result3)
print("Test 4 (expected: 0): ", result4)







