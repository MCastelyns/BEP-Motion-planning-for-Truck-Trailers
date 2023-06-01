using UnityEngine;
using System.Collections;
using System;

// Make a new class for storing the 'waypoints/nodes' after Trajectory Optimization
// But since both waypoint and node are already defined I went with TrajectoryPoint
namespace PathfindingForVehicles
{
    public class TrajectoryPoint
    {
        // State variables
        public double X { get; set; }       // X position (m)
        public double Y { get; set; }       // Y position (m)
        public double Theta { get; set; }   // Truck Heading (radians)
        public double Psi { get; set; }     // Hitch Angle (radians)
        public double Phi { get; set; }     // Steering angle (radians)

        // Control variables
        public double V { get; set; }       // Velocity (m/s)
        public double Omega { get; set; }   // Steering speed (radians/s)

        // Default constructor
        public TrajectoryPoint() {}

        // Constructor with all parameters, this is what we want to use. We can set the values when constructing a new point
        public TrajectoryPoint(double x, double y, double theta, double psi, double phi, double v, double omega)
        {
            X = x;
            Y = y;
            Theta = theta;
            Psi = psi;
            Phi = phi;
            V = v;
            Omega = omega;
        }
    }
}
