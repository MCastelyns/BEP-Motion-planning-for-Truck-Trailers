import json
import numpy as np

# Load data from the JSON file
with open('initialize.json', 'r') as f:
    data = json.load(f)

# Now we can access the data from the file. For example:
positions = np.array(data['Positions'])
headings = np.array(data['Headings'])
hitch_angles = np.array(data['HitchAngles'])

# Each variable is now a numpy array. For example, positions is a 2D array with shape (N, 2), where column 1 is x coordinates and column 2 is y (or in our case z) coordinates

print(positions)
print(headings)
print(hitch_angles)