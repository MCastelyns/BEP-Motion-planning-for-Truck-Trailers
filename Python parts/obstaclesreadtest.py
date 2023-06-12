# -*- coding: utf-8 -*-
"""
Created on Thu Jun  1 16:48:16 2023

@author: Mitchel
"""
import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Load JSON data from the file
with open('obstacles.json', 'r') as f:
    data = json.load(f)

# Initialize obstacle_list for storing data
obstacle_list = []

# Iterate over list of obstacles in data
for i, obstacle in enumerate(data):
    # Retrieve corner coordinates
    FL = obstacle['FL']
    FR = obstacle['FR']
    BL = obstacle['BL']
    BR = obstacle['BR']
    
    # Center_x = average of x coordinates of 4 corners, same for center_y
    center_x = (FL['X'] + FR['X'] + BL['X'] + BR['X']) / 4
    center_y = (FL['Y'] + FR['Y'] + BL['Y'] + BR['Y']) / 4
    
    # Width and Height = differences in x and y coordinates (only if rectangle is upright, but this is the case for us)
    width = abs(FR['X'] - FL['X'])
    height = abs(BL['Y'] - FL['Y'])
    
    # Append data to the new data list
    obstacle_list.append({"center": (center_x, center_y), "width": width, "height": height})


# DEBUGGING/TESTING, REMOVE WHEN WE ARE SURE IT'S WORKING AS INTENDED
#####################################################################
""" 
    print(f'Obstacle {i+1}:')
    print(f'Center: ({center_x}, {center_y})')
    print(f'Width: {width}')
    print(f'Height: {height}\n')


# Create a figure and a set of subplots
fig, ax = plt.subplots()

# Plot each obstacle as a rectangleg
for obstacle in obstacle_list:
    center = obstacle["center"]
    width = obstacle["width"]
    height = obstacle["height"]
    
    # The Rectangle function expects the bottom left corner, so we subtract half of width and height from center
    rect = patches.Rectangle((center[0]-width/2, center[1]-height/2), width, height, linewidth=1, edgecolor='r', facecolor='none')
    
    ax.add_patch(rect)

plt.xlim([-1, 120])
plt.ylim([-1, 120])
plt.show() """
