import numpy as np
from scipy.interpolate import CubicSpline
import json

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

