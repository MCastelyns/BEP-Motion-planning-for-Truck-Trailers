import json
import numpy as np


# Load data from the JSON file
with open('initialize.json', 'r') as f:
    data = json.load(f)

    headings = np.array(data['HitchAngles'])

horizon = len(headings)
print(horizon)