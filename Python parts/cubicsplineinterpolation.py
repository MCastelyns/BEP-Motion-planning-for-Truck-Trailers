import numpy as np
from scipy.interpolate import CubicSpline
import json
import casadi as ca

def interpolate_waypoints(waypoints, num_output_nodes): #lijst met waypoints/nodes en hoeveelheid gewenste geinterpoleerde nodes.
    # Extract each states components from the waypoints list
    num_waypoints = len(waypoints)

    # Create a list of sampling point values, the spacing between the input waypoints, could be by time or distance. linspace = evenly spaced
    spacing = np.linspace(0, 1, num_waypoints)

    # Create a list of output sampling point values, again could be spaced by time or distance. linspace = evenly spaced
    output_spacing = np.linspace(0, 1, num_output_nodes)

    # Interpolate each state component separately, we have to interpolate over all the waypoints per state seperately
    interpolated_states = []
    # Create a cubic spline for this state component, using the current spacing between the points
    spline = CubicSpline(spacing, waypoints)
        
    # Evaluate the spline at the output sampling point values; the spacing we want
    interpolated_state = spline(output_spacing)
        
    # Append the interpolated state component to the list
    interpolated_states.append(interpolated_state)

    return interpolated_states

# Deze functie maakt een cubic spline interpolation van de states, hiermee kun je interpoleren en daarmee dus de hoeveelheid nodes/waypoints aanpassen, misschien nodig als 
# het blijkt dat we ergens een vaste hoeveelheid nodes moeten hebben terwijl dat variabel is

# Voorbeeld van gebruik : (Hier gaan we er van uit dat de waypoints gelijke spacing ; tijd of afstand, tussen elkaar hebben)

#from interpolate_waypoints import interpolate_waypoints

# List of waypoints 
#waypoints = [(x1, y1, theta1, psi1, phi1), (x2, y2, theta2, psi2, phi2), ...]

# Desired number of output nodes
#num_output_nodes = 100

# Interpolate the waypoints
#interpolated_waypoints = interpolate_waypoints(waypoints, num_output_nodes)
# Load data from the JSON file
with open('initialize.json', 'r') as f:
    data = json.load(f)

# Now we can access the data from the file. For example:
positions = np.array(data['Positions'])
headings = np.array(data['Headings']) + np.pi/2 # Have to do this, because of differences in coordinate systems
hitch_angles = np.array(data['HitchAngles'])

# Interpolate these states using cubic spline interpolation, making as many states as the horizon 
positions_new = interpolate_waypoints(positions, 100)[0]
headings_new = interpolate_waypoints(headings, 100)[0]
hitch_angles_new = interpolate_waypoints(hitch_angles, 100)[0]

# Now we construct the vars_guess using these interpolated values and our guesses for the dual variables
vars_guess = []
for k in range(100):
    state_guess = positions_new[k]
    state_guess = np.append(state_guess, headings_new[k])
    state_guess = np.append(state_guess, hitch_angles_new[k])
    state_guess = np.append(state_guess, 0) #Guess for steering angle, has to be implemented later
    state_guess = np.append(state_guess, 0) #Guess for velocity, has to be implemented later

    input_guess = ca.DM.zeros((2))

    vars_guess = ca.vertcat(vars_guess, 
                            state_guess, 
                            input_guess,
                            100.*ca.DM.ones(8*1),
                            ca.kron(ca.DM.ones(1), ca.DM([100, 105, 110, 115, 100, 105, 110, 115])))
# Goal state, could be done in a different way when we actually implement it, for now it's just the last state of the A* trajectory
state_guess = positions_new[-1]
state_guess = np.append(state_guess, headings_new[-1])
state_guess = np.append(state_guess, hitch_angles_new[-1])
state_guess = np.append(state_guess, 0) #Guess for steering angle, has to be implemented later
state_guess = np.append(state_guess, 0) #Guess for velocity, has to be implemented later

# Final state outside of loop, we don't have input guess for this
vars_guess = ca.vertcat(vars_guess, 
                        state_guess,
                        100.*ca.DM.ones(8*1),
                        ca.kron(ca.DM.ones(1), ca.DM([100, 105, 110, 115, 100, 105, 110, 115])))
print(vars_guess)
