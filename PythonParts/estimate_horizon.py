import json
import numpy as np

def estimate_horizon():
    # This function estimates the horizon in seconds, using the amount of waypoints and their total length and a low estimate of the average speed
    with open('initialize.json', 'r') as f:
        data = json.load(f)

    headings = np.array(data['Headings'])
    # our max forward speed is 10m/s, our max backwards speed is 5m/s, we also sometimes change direction so an estimate of average speed around 5m/s seems reasonable
    # the distance per waypoint is 2*sqrt(2) this is around 2.83m
    # So we travel 2.83m per node, with an average speed (low estimate) of 5 m/s. 2.83/5 is very close to 0.5 seconds per node
    horizon = len(headings)*0.5 #Our horizon in seconds is then the amount of nodes * 0.5 seconds
    return(horizon)
