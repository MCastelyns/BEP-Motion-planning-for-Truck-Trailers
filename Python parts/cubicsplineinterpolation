import numpy as np
from scipy.interpolate import CubicSpline

def interpolate_waypoints(waypoints, num_output_nodes): #lijst met waypoints/nodes en hoeveelheid gewenste geinterpoleerde notes
    # Extract each states components from the waypoints list
    states = list(zip(*waypoints))
    num_waypoints = len(waypoints)

    # Create a list of parameter values, the spacing between the waypoints, could be by time or distance, not sure what this would be in our case, probably time. Currently linspace so evenly spaced
    parameter_values = np.linspace(0, 1, num_waypoints)

    # Create a list of output parameter values, again could be spaced by time or distance, not sure what we would do. Currently linspace so evenly spaced
    output_parameter_values = np.linspace(0, 1, num_output_nodes)

    # Interpolate each state component separately, you have to interpolate over all the waypoints per state seperately
    interpolated_states = []
    for state in states:
        # Create a cubic spline for this state component, the current spacing between the points
        spline = CubicSpline(parameter_values, state)
        
        # Evaluate the spline at the output parameter values, the spacing we want
        interpolated_state = spline(output_parameter_values)
        
        # Append the interpolated state component to the list
        interpolated_states.append(interpolated_state)
    
    # Transpose the list of interpolated states to get a list of states
    interpolated_waypoints = list(zip(*interpolated_states))
    
    return interpolated_waypoints

# Deze functie maakt een cubic spline interpolation van de states, hiermee kun je interpoleren en daarmee dus de hoeveelheid nodes/waypoints aanpassen, misschien nodig als 
# het blijkt dat we ergens een vaste hoeveelheid nodes moeten hebben terwijl dat variabel is

# Voorbeeld van gebruik :

#from interpolate_waypoints import interpolate_waypoints

# List of waypoints
#waypoints = [(x1, y1, theta1, psi1, phi1), (x2, y2, theta2, psi2, phi2), ...]

# Desired number of output nodes
#num_output_nodes = 100

# Interpolate the waypoints
#interpolated_waypoints = interpolate_waypoints(waypoints, num_output_nodes)
