using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PathfindingForVehicles
{
    //Parameters related to pathfinding so we can change them here
    public static class Parameters
    {
        //Car data
        //Some are not the same for all cars, so is in their own class called CarData
        //The max speed when the car is following the path
        public const float maxPathFollowSpeed = 15f;
        //The speed with which we turn the steering wheel
        public const float steeringWheelSpeed = 5f;

        //Map data
        //The size of all cells in [m]
        public const int mapWidth = 160;
        //The size of one cell in [m]
        public const float cellWidth = 1f;

        //Obstacles
        //Make the car fatter to be on the safe side when checking collisions
        public const float marginOfSafety = 1.0f;
        //Size of each random obstacle
        public const float maxObstacleSize = 15f;
        public const float minObstacleSize = 1f;
        //How many obstacles
        public const int obstaclesToAdd = 30;

        //Smooth path
        //Minimize the distance between the smooth position and the non-smooth position
        public const float alpha = 0.10f; //0.10f;
        //Minimize the distance between this position and the surrounding positions
        public const float beta = 0.40f; //0.40f;
        //Maximize the distance between the position and the closest obstacle
        public const float gamma = 0.05f; //0.05f;
        //Push the path towards low areas in the Voronoi field = areas far away from obstacles
        public const float delta = 0.00f;

        //Hybrid A*
        //Costs to make the car behave in different ways
        //For example, we prefere to drive forward instead of reversing
        //Cost to go
        public static float turningCost = 1.0f;      // [rad]
        public static float turningChangeCost = 5.0f;// [rad]
        public const float obstacleCost = 0.0f;
        public const float reverseCost = 0f;        // [m]
        public static float switchingDirectionOfMovementCost = 20f;
        //Extra cost for trailer because its not good at reversing
        public static float trailerReverseCost = 1f;
        public static float trailerAngleCost = 1.0f * Mathf.Deg2Rad; // Angle truck/trailer [deg]
        //Heuristic Costs scale factors
        public static float carDistance = 0.0f;              // distance to end position of car/truck
        public static float trailerDistance = 1.0f;          // distance to end position of trailer
        public static float trailerSidewaysDistance = 3.0f;  // sideways distance to end position of trailer
        public const float trailerForwardDistance = 10.0f;   // forward distance to end position of trailer
        public static float trailerAngle = 30.0f * Mathf.Deg2Rad; // diff angle trailer current/end
        public const float truckSidewaysDistance = 0.0f;    // sideways distance to end position of truck
        public const float truckForwardDistance = 0.0f;

        //Voronoi field
        //The falloff rate > 0
        public const float voronoi_alpha = 5f;
        //The maximum effective range of the field > 0 
        public const float d_o_max = 30f;
    }
}
