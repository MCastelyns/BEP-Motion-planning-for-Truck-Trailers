# -*- coding: utf-8 -*-
"""
Created on Thu Jun  1 16:48:16 2023

@author: Mitchel
"""
import json
import matplotlib.pyplot as plt

# Load JSON data from the file
with open('obstacles.json', 'r') as f:
    data = json.load(f)

fig, ax = plt.subplots()

ax.set_xlim([0, 120])
ax.set_ylim([0, 120])

# Iterate over list of obstacles in data
for i, obstacle in enumerate(data):
    # Retrieve corner coordinates (this has values still under keys 'X' and 'Y', so can't directly use this have to extract the raw values first)
    FL = obstacle['FL']
    FR = obstacle['FR']
    BL = obstacle['BL']
    BR = obstacle['BR']
    
    print(f'Obstacle {i+1}:')
    print(f'FL: {FL}')
    print(f'FR: {FR}')
    print(f'BL: {BL}')
    print(f'BR: {BR}\n')

    # Get the raw coordinate values in a list for plotting
    x = [FL['X'], FR['X'], BR['X'], BL['X'], FL['X']]
    y = [FL['Y'], FR['Y'], BR['Y'], BL['Y'], FL['Y']]

    # Plot obstacle
    ax.plot(x, y)

# Show plot
plt.show()