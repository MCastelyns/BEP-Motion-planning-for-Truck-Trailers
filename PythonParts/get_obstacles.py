import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def get_obstacles():
    # Load JSON data from the file
    with open('python-files\obstacles.json', 'r') as f:
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
        center_x = round((FL['X'] + FR['X'] + BL['X'] + BR['X']) / 4, 4)
        center_y = round((FL['Y'] + FR['Y'] + BL['Y'] + BR['Y']) / 4, 4)
        
        # Width and Height = differences in x and y coordinates (only if rectangle is upright, but this is the case for us)
        width = round(abs(FR['X'] - FL['X']), 4)
        height = round(abs(BL['Y'] - FL['Y']), 4)
        
        # Append data to the new data list
        obstacle_list.append({"center": (center_x, center_y), "width": width, "height": height})
    
    return obstacle_list

# Call the function and print its return value for testing
#print(get_obstacles())
