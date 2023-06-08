import numpy as np
from scipy.interpolate import CubicSpline

def interpolate_waypoints(waypoints, num_output_nodes): #lijst met waypoints/nodes en hoeveelheid gewenste geinterpoleerde notes
    # Extract each states components from the waypoints list
    states = list(zip(*waypoints))
    num_waypoints = len(waypoints)

    # Create a list of sampling point values, the spacing between the input waypoints, could be by time or distance. linspace = evenly spaced
    spacing = np.linspace(0, 1, num_waypoints)

    # Create a list of output sampling point values, again could be spaced by time or distance. linspace = evenly spaced
    output_spacing = np.linspace(0, 1, num_output_nodes)

    # Interpolate each state component separately, you have to interpolate over all the waypoints per state seperately
    interpolated_states = []
    for state in states:
        # Create a cubic spline for this state component, using the current spacing between the points
        spline = CubicSpline(spacing, state)
        
        # Evaluate the spline at the output sampling point values; the spacing we want
        interpolated_state = spline(output_spacing)
        
        # Append the interpolated state component to the list
        interpolated_states.append(interpolated_state)
    
    # Transpose the list of interpolated states to get a list of states
    interpolated_waypoints = list(zip(*interpolated_states))
    
    return interpolated_waypoints

# Deze functie maakt een cubic spline interpolation van de states, hiermee kun je interpoleren en daarmee dus de hoeveelheid nodes/waypoints aanpassen, misschien nodig als 
# het blijkt dat we ergens een vaste hoeveelheid nodes moeten hebben terwijl dat variabel is

# Voorbeeld van gebruik : (Hier gaan we er van uit dat de waypoints gelijke spacing , tijd of afstand, tussen elkaar hebben)

#from interpolate_waypoints import interpolate_waypoints

# List of waypoints
#waypoints = [(x1, y1, theta1, psi1, phi1), (x2, y2, theta2, psi2, phi2), ...]

# Desired number of output nodes
#num_output_nodes = 100

# Interpolate the waypoints
#interpolated_waypoints = interpolate_waypoints(waypoints, num_output_nodes)
