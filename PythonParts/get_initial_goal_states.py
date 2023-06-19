import json
import numpy as np

def get_initial_goal_states():
    # Load data from the JSON file
    with open('python-files\initialize.json', 'r') as f:
        data = json.load(f)

    # Now we can access the data from the file. For example:
    positions = np.array(data['Positions'])
    headings = np.array(data['Headings']) + np.pi/2 # Have to do this, because of differences in coordinate systems
    hitch_angles = np.array(data['HitchAngles'])

    # Reconstruct initial state and goal state, so we can send them back in the right format
    initial_state = []
    initial_state.append([positions[0,0],positions[0,1],headings[0],hitch_angles[0]])
    goal_state = []
    goal_state.append([positions[-1,0],positions[-1,1],headings[-1],hitch_angles[-1]])
    return initial_state[0], goal_state[0]
# Each variable is now a numpy array. For example, positions is a 2D array with shape (N, 2), where column 1 is x coordinates and column 2 is y (or in our case z) coordinates


initial_state, goal_state = get_initial_goal_states()

# print('Initial state:', initial_state)
# print('Goal state:', goal_state)
# print('Trajectory Received:', len(headings) ,'Waypoints') #This includes node 0, which is the initial state
# print('X and Y Positions:', positions)
# print('Headings:', headings)
# print('Hitch Angles:', hitch_angles) 