using UnityEngine;
using System.Collections;
using System;
using System.Collections.Generic;
using PathfindingForVehicles.ReedsSheppPaths;
using Newtonsoft.Json;
using System.IO;


public class SerializableObstacle
{
    public SerializableVector2 FL { get; set; }
    public SerializableVector2 FR { get; set; }
    public SerializableVector2 BL { get; set; }
    public SerializableVector2 BR { get; set; }
}

public class SerializableVector2
{
    public float X { get; set; }
    public float Y { get; set; }

    public SerializableVector2(float x, float y)
    {
        X = x;
        Y = y;
    }
}


namespace PathfindingForVehicles
{
    //Hybrid A* pathfinding algorithm
    public static class HybridAStar
    {
        //The distance between each waypoint
        //Should be greater than the hypotenuse of the cell width or node may end up in the same cell
        public static float driveDistance = Mathf.Sqrt((Parameters.cellWidth * Parameters.cellWidth) * 2f) * 2f + 0.01f;
        //Used in the loop to easier include reversing
        private static float[] driveDistances = new float[] { -driveDistance, driveDistance};
        //The steering angles we are going to test
        private static float maxAngle = 30f;
        // Calculate the angle spacing
        private static int steeringAnglesAmount= 5;
        //private static float[] steeringAngles = new float[] { -maxAngle * Mathf.Deg2Rad, 0f, maxAngle * Mathf.Deg2Rad };
        private static float[] steeringAngles = generateSteeringAngles(maxAngle, steeringAnglesAmount);
        //The car will never reach the exact goal position, this is how accurate we want to be
        private const float posAccuracy = 1f;
        private const float headingAccuracy = 15f;
        //The heading resolution (Junior had 5) [degrees]
        private const float headingResolution = 5f;
        private const float headingResolutionTrailer = 5f;
        //To time the different parts of the algorithm 
        private static int timer_selectLowestCostNode;
        private static int timer_addNodeToHeap;
        private static int timer_findChildren;
        private static int timer_isCollidingWithObstacle;
        private static int timer_ReedsSheppNode;
        private static int timer_ReedsSheppHeuristics;
        private static int timer_TrailerCollision;
        //At what distance to should we start expanding Reeds-Shepp nodes
        private static float maxReedsSheppDist = 0f;


        //
        // Generate equally spaced steering angles to explore during child finding
        //
        public static float[] generateSteeringAngles(float maxAngle, int numberOfAngles)
        {
            float[] angles = new float[numberOfAngles];
            maxAngle *= Mathf.Deg2Rad;
            float angleStep = (2 * maxAngle) / (numberOfAngles - 1);

            for (int i = 0; i < numberOfAngles; i++)
            {
                angles[i] = (-maxAngle + (i * angleStep));
                //Debug.Log(angles[i]);
            }
            
            return angles;
        }

        //
        // Generate a path with Hybrid A*
        //
        public static List<Node> GeneratePath(Car startCar, Car endCar, Map map, List<Node> allExpandedNodes, Car startTrailer, Car endTrailer)
        {
            //Reset timers
            timer_selectLowestCostNode = 0;
            timer_addNodeToHeap = 0;
            timer_findChildren = 0;
            timer_isCollidingWithObstacle = 0;
            timer_ReedsSheppNode = 0;
            timer_ReedsSheppHeuristics = 0;
            timer_TrailerCollision = 0;
            //Other data we want to track
            //How many nodes did we prune?
            int prunedNodes = 0;
            //To track max number of nodes in the heap
            int maxNodesInHeap = 0;


            //Init the data structure we need
            int mapWidth = map.MapWidth;
            //Is faster to cache this than using map.cellData
            Cell[,] cellData = map.cellData;

            //Open nodes - the parameter is how many items can fit in the heap
            //If we lower the heap size it will still find a path, which is more drunk
            Heap<Node> openNodes = new Heap<Node>(200000);
            //int in the dictionaries below is the rounded heading used to enter a cell
            HashSet<int>[,] closedCells = new HashSet<int>[mapWidth, mapWidth];
            //The node in the cell with the lowest g-cost at a certain angle
            Dictionary<int, Node>[,] lowestCostNodes = new Dictionary<int, Node>[mapWidth, mapWidth];
            //Trailer
            //int in the dictionaries below is the rounded heading used to enter a cell
            Dictionary<int, HashSet<int>>[,] closedCellsTrailer = new Dictionary<int, HashSet<int>>[mapWidth, mapWidth];
            HashSet<int>[,] lowestCostNodesTrailer = new HashSet<int>[mapWidth, mapWidth];

            for (int x = 0; x < mapWidth; x++)
            {
                for (int z = 0; z < mapWidth; z++)
                {
                    closedCells[x, z] = new HashSet<int>();
                    lowestCostNodes[x, z] = new Dictionary<int, Node>();

                    //Trailer
                    closedCellsTrailer[x, z] = new Dictionary<int, HashSet<int>>();
                    lowestCostNodesTrailer[x, z] = new HashSet<int>();
                }
            }


            //Create the first node
            //Why rear wheel? Because we need that position when simulating the "skeleton" car
            //and then it's easier if everything is done from the rear wheel positions            
            IntVector2 startCellPos = map.ConvertWorldToCell(startCar.rearWheelPos);

            Node node = new Node(
                previousNode: null,
                rearWheelPos: startCar.rearWheelPos,
                heading: startCar.HeadingInRadians,
                isReversing: false);
            node.steeringAngle = 0f;

            Debug.Log($"Start Car info \th {startCar.HeadingInDegrees} \tx {startCar.rearWheelPos.x} \ty {startCar.rearWheelPos.z}");
            Debug.Log($"End Car info \th {endCar.HeadingInDegrees} \tx {endCar.rearWheelPos.x} \ty {endCar.rearWheelPos.z}");

            if (startTrailer != null)
            {
                node.TrailerHeadingInRadians = startTrailer.HeadingInRadians;
                Debug.Log($"Start Trailer \th {startTrailer.HeadingInDegrees} \tx {startTrailer.rearWheelPos.x} \ty {startTrailer.rearWheelPos.z}");
                Debug.Log($"End Trailer \th {endTrailer.HeadingInDegrees} \tx {endTrailer.rearWheelPos.x} \ty {endTrailer.rearWheelPos.z}");
            }

            node.AddCosts(gCost: 0f, hCost: cellData[startCellPos.x, startCellPos.z].heuristics);

            openNodes.Add(node);


            //The end of the path, which we will return
            Node finalNode = null;


            //bools so we can break out of the main loop
            //Set when search is complete
            bool found = false;
            //Set if we can't find a node to expand  
            bool resign = false;

            //To break out of the loop if it takes too long time
            int iterations = 0;

            IntVector2 goalCellPos = map.ConvertWorldToCell(endCar.rearWheelPos);

            while (!found && !resign)
            {
                if (iterations > 20000)
                {
                    //Debug.Log("Stuck in infinite loop");

                    break;
                }

                iterations += 1;

                //If we don't have any nodes to expand
                if (openNodes.Count == 0)
                {
                    resign = true;

                    //Debug.Log("Failed to find a path");
                }
                //We have nodes to expand
                else
                {
                    if (openNodes.Count > maxNodesInHeap)
                    {
                        maxNodesInHeap = openNodes.Count;
                    }
                
                    //Get the node with the lowest f cost
                    int timeBefore = Environment.TickCount;

                    Node nextNode = openNodes.RemoveFirst();

                    timer_selectLowestCostNode += Environment.TickCount - timeBefore;


                    //Close this cell
                    IntVector2 cellPos = map.ConvertWorldToCell(nextNode.rearWheelPos);

                    int roundedHeading = HelpStuff.RoundValue(nextNode.HeadingInDegrees, headingResolution);

                    HashSet<int> closedHeadingsInThisCell = closedCells[cellPos.x, cellPos.z];

                    bool haveAlreadyClosedCell = false;

                    //Close the cell, but we can still drive into this cell from another angle
                    if (!closedHeadingsInThisCell.Contains(roundedHeading))
                    {
                        closedHeadingsInThisCell.Add(roundedHeading);
                    }
                    else if (startTrailer == null)
                    {
                        haveAlreadyClosedCell = true;
                    }
                  

                    if (startTrailer != null)
                    {
                        int roundedHeadingTrailer = HelpStuff.RoundValue(nextNode.TrailerHeadingInDegrees, headingResolutionTrailer);

                        Dictionary<int, HashSet<int>> closedTrailerHeadingsInThisCell = closedCellsTrailer[cellPos.x, cellPos.z];

                        /*if (!closedTrailerHeadingsInThisCell[roundedHeading].Contains(roundedHeadingTrailer))
                        {
                            closedTrailerHeadingsInThisCell[roundedHeading].Add(roundedHeadingTrailer);
                        }*/

                        if (!closedTrailerHeadingsInThisCell.TryGetValue(roundedHeading, out HashSet<int> trailerHeadings))
                        {
                            trailerHeadings = new HashSet<int>();
                            closedTrailerHeadingsInThisCell[roundedHeading] = trailerHeadings;
                        }

                        if (!trailerHeadings.Contains(roundedHeadingTrailer))
                        {
                            trailerHeadings.Add(roundedHeadingTrailer);
                        }
                        else
                        {
                            haveAlreadyClosedCell = true;
                        }
                    }

                    //We have already expanded a better node with the same heading so dont expand this node
                    if (haveAlreadyClosedCell)
                    {
                        iterations -= 1;

                        continue;
                    }


                    //For debugging
                    allExpandedNodes.Add(nextNode);

                    //Check if the vehicle has reached the target
                    float distanceSqrToGoal = (nextNode.rearWheelPos - endCar.rearWheelPos).sqrMagnitude;

                    //But we also need to make sure the vehicle has correct heading
                    float headingDifference = Mathf.Abs(Mathf.DeltaAngle(endCar.HeadingInDegrees, nextNode.HeadingInDegrees));
                    float trailerHeadingDiff = 0.0f;
                    if (startTrailer != null)
                    {
                        trailerHeadingDiff = Mathf.Abs(Mathf.DeltaAngle(endTrailer.HeadingInDegrees, nextNode.TrailerHeadingInDegrees));
                    }


                    //If we end up in the same cell or is within a certain distance from the goal
                    if ((distanceSqrToGoal < posAccuracy * posAccuracy || (cellPos.x == goalCellPos.x && cellPos.z == goalCellPos.z)) &&
                        headingDifference < headingAccuracy && trailerHeadingDiff < headingAccuracy)
                    {
                        found = true;

                        Debug.Log("Found a path");
                        Debug.Log($"Heading Diff Truck: {headingDifference} Trailer: {trailerHeadingDiff} degrees");

                        finalNode = nextNode;

                        //Make sure the end node has the same position as the target
                        finalNode.rearWheelPos.x = endCar.rearWheelPos.x;
                        finalNode.rearWheelPos.z = endCar.rearWheelPos.z;
                    }
                    //If we havent found the goal, then expand this node
                    else
                    {
                        //Get all child nodes
                        timeBefore = Environment.TickCount;
                        //Debug.Log($"Parent Node heading: {nextNode.HeadingInDegrees} \t cost: {nextNode.fCost}");

                        List<Node> children = GetChildrenToNode(nextNode, map, cellData, startCar.carData, endCar, startTrailer, endTrailer);

                        timer_findChildren += Environment.TickCount - timeBefore;


                        //Should we add any of the child nodes to the open nodes?
                        foreach (Node child in children)
                        {
                            IntVector2 childCell = map.ConvertWorldToCell(child.rearWheelPos);
                            //Debug.Log($"\tChild Node reversing: {child.isReversing} \t Heading: {child.HeadingInDegrees} \t cost: {child.fCost}");

                            int roundedChildHeading = HelpStuff.RoundValue(child.HeadingInDegrees, headingResolution);
                            //Has this cell been closed with this heading?
                            //If so, it means we already have a path at this cell with this heading, 
                            //and the existing node is cheaper because we have already expanded it
                            if (closedCells[childCell.x, childCell.z].Contains(roundedChildHeading))
                            {
                                if (startTrailer == null)
                                {
                                    prunedNodes += 1;

                                    //Debug.Log("\t\tpruned node");
                                    continue;
                                }

                                //If we have a trailer
                                else
                                {
                                    int roundedTrailerHeading = HelpStuff.RoundValue(child.TrailerHeadingInDegrees, headingResolutionTrailer) % 360;

                                    if (closedCellsTrailer[childCell.x, childCell.z][roundedChildHeading].Contains(roundedTrailerHeading))
                                    {
                                        prunedNodes += 1;

                                        continue;
                                    }
                                }
                            }

                            //Have we already expanded a node with lower cost in this cell at this heading?
                            float costSoFar = child.gCost;

                            //The dictionary with lowest cost nodes in this cell
                            Dictionary<int, Node> nodesWithLowestCost = lowestCostNodes[childCell.x, childCell.z];

                            //Have we expanded with this angle to the cell before?
                            if (nodesWithLowestCost.ContainsKey(roundedChildHeading) && startTrailer == null)
                            {
                                //If the open node has a large gCost then we need to update that node with data
                                //from the child node
                                if (costSoFar < nodesWithLowestCost[roundedChildHeading].gCost)
                                {
                                    //If this child node is better then we should update the node in the open list
                                    //which is faster than deleting the old node and adding this child node
                                    Node existingNode = nodesWithLowestCost[roundedChildHeading];

                                    child.StealDataFromThisNode(existingNode);

                                    //Modify the heap-position of the node already in the open nodes
                                    openNodes.UpdateItem(existingNode);
                                }
                                //If the open node has a smaller gCost, then we dont need this child node, so do nothing

                                prunedNodes += 1;
                                //Debug.Log("\t\tpruned node");
                                continue;
                            }
                            //We have a trailer
                            else if (nodesWithLowestCost.ContainsKey(roundedChildHeading) && startTrailer != null)
                            {
                                //Have we expanded to this node before with this trailer heading
                                int roundedTrailerHeading = HelpStuff.RoundValue(child.TrailerHeadingInDegrees, headingResolutionTrailer);

                                if (lowestCostNodesTrailer[childCell.x, childCell.z].Contains(roundedTrailerHeading))
                                {
                                    //If the open node has a large gCost then we need to update that node with data
                                    //from the child node
                                    if (costSoFar < nodesWithLowestCost[roundedChildHeading].gCost)
                                    {
                                        //If this child node is better then we should update the node in the open list
                                        //which is faster than deleting the old node and adding this child node
                                        Node existingNode = nodesWithLowestCost[roundedChildHeading];

                                        child.StealDataFromThisNode(existingNode);

                                        //Modify the heap-position of the node already in the open nodes
                                        openNodes.UpdateItem(existingNode);
                                    }
                                    //If the open node has a smaller gCost, then we dont need this child node, so do nothing

                                    prunedNodes += 1;

                                    continue;
                                }
                            }
                            else
                            {
                                //Add the node to the cell with this angle
                                nodesWithLowestCost[roundedChildHeading] = child;

                                if (startTrailer != null)
                                {
                                    int roundedTrailerHeading = HelpStuff.RoundValue(child.TrailerHeadingInDegrees, headingResolutionTrailer);

                                    lowestCostNodesTrailer[childCell.x, childCell.z].Add(roundedTrailerHeading);
                                }
                            }


                            //Dont add the node if its colliding with an obstacle or is outside of map
                            timeBefore = Environment.TickCount;
                            if (ObstaclesDetection.HasCarInvalidPosition(child.rearWheelPos, child.heading, startCar.carData, map))
                            {
                                prunedNodes += 1;
                                //Debug.Log("\t\tpruned node");
                                continue;
                            }
                            timer_isCollidingWithObstacle += Environment.TickCount - timeBefore;


                            //Trailer obstacle calculations
                            int startTrailerTimer = Environment.TickCount;
                           
                            if (startTrailer != null)
                            {
                                //Now we need to check if this new position is valid by checking for collision with obstacles and the drag vehicle
                                //To do that we need the rear wheel pos of the trailer with this heading
                                //We know where the trailer is attached to the drag vehicle
                                Vector3 trailerAttachmentPoint = startCar.carData.GetTrailerAttachmentPoint(child.rearWheelPos, child.HeadingInRadians);

                                //Now we need the trailer's rear-wheel pos based on the new heading
                                Vector3 trailerRearWheelPos = startTrailer.carData.GetTrailerRearWheelPos(trailerAttachmentPoint, child.TrailerHeadingInRadians);

                                //Obstacle detection
                                //With the environment
                                if (ObstaclesDetection.HasCarInvalidPosition(trailerRearWheelPos, child.TrailerHeadingInRadians, startTrailer.carData, map))
                                {
                                    prunedNodes += 1;

                                    //Debug.Log("Semi trailer environment collision");

                                    continue;
                                }
                                //With the drag vehicle
                                if (ObstaclesDetection.IsTrailerCollidingWithDragVehicle(
                                    child.rearWheelPos, child.HeadingInRadians, startCar.carData,
                                    trailerRearWheelPos, child.TrailerHeadingInRadians, startTrailer.carData))
                                {
                                    prunedNodes += 1;

                                    //Debug.Log("Semi trailer collision");

                                    continue;
                                }
                            }
                            timer_TrailerCollision += Environment.TickCount - startTrailerTimer;


                            timeBefore = Environment.TickCount;

                            openNodes.Add(child);

                            timer_addNodeToHeap += Environment.TickCount - timeBefore;
                        }
                    }
                }
            }


            //Generate the final path when Hybrid A* has found the goal
            List<Node> finalPath = GenerateFinalPath(finalNode);

            // List of variables for storing node/waypoints state values to form datastructure we can send via .JSON to python
            var positions = new List<List<double>>();
            var headings = new List<double>();
            var hitch_angles = new List<double>();
            var steer = new List<double>();

            for (int i = 0; i < finalPath.Count; i++)
            {
                // Actually adding the node/waypoint state values to the variables
                positions.Add(new List<double> { finalPath[i].rearWheelPos.x, finalPath[i].rearWheelPos.z }); // has to be x and z because our coordinate system is weird in Unity
                headings.Add(Mathf.DeltaAngle(finalPath[i].heading * Mathf.Rad2Deg, 0) * Mathf.Deg2Rad); // truck heading
                hitch_angles.Add(Mathf.DeltaAngle(finalPath[i].TrailerHeadingInRadians * Mathf.Rad2Deg, finalPath[i].heading * Mathf.Rad2Deg) * Mathf.Deg2Rad);
                // Angles need to be calculated with Mathf.DeltaAngle, also hitch angle could probably be set to 0 as reference
                // Since we want to keep hitch angle as low as possible, using a higher than 0 hitch angle as reference/initial guess might actually make it work worse, have to test this
                // Might just remove the hitch_angles from the JSON data structure and just use np.zeros
                // Steering angle and velocity + steering speed might add something, so could look into adding those. Steering angle is already available under node.nodesteeringangle
                // Velocity and steering speed not sure how to implement, I also wonder if those will even add anything to the TO, as there is already upper and lower bounds on those
                // So not too much freedom is left to the optimization algorithm (IPOPT)
            }
            // Creating the datastructure we want to send
            var pathdata = new
            {
                Positions = positions,
                Headings = headings,
                HitchAngles = hitch_angles
            };
            // Writing to the .JSON named 'initialize' that we can send once to our TO (Trajectory Optimization) as initial guess
            File.WriteAllText("initialize.json", JsonConvert.SerializeObject(pathdata, Formatting.Indented));
            Debug.Log($"Path JSON succesfully made");


            // Test to see what format the obstacles are in exactly, needed for importing to python. 
            // First we take a list of all obstacles, then we make this into the new class serializableobstacle, which just has the info we want (Makes it easier to handle)
            List<Obstacle> obstaclesall = map.allObstacles;
            List<SerializableObstacle> serializableObstacles = new List<SerializableObstacle>();

            // Now we loop through all obstacles and get the Vector2 coordinates of the cornerpoints
            // Then we add a new serializableobstacle with those cornerpoints to the list of serializableobstacles (starts out empty)
            for (int i = 0; i < obstaclesall.Count; i++)
            {
                Vector2 FL = obstaclesall[i].cornerPos.FL.XZ();
                Vector2 FR = obstaclesall[i].cornerPos.FR.XZ();
                Vector2 BL = obstaclesall[i].cornerPos.BL.XZ();
                Vector2 BR = obstaclesall[i].cornerPos.BR.XZ();

                serializableObstacles.Add(new SerializableObstacle
                {
                    FL = new SerializableVector2(FL.x, FL.y),
                    FR = new SerializableVector2(FR.x, FR.y),
                    BL = new SerializableVector2(BL.x, BL.y),
                    BR = new SerializableVector2(BR.x, BR.y),
                });
            }

            // Now we format the string to be put into the JSON in the correct way
            string obstaclejson = JsonConvert.SerializeObject(serializableObstacles, Formatting.Indented);
            // And write the data to a JSON, this data can then be read in a python script to import the obstacle coordinates
            File.WriteAllText("obstacles.json", obstaclejson);
            Debug.Log($"Obstacle JSON succesfully made");

            for (int i = 0; i < finalPath.Count; i++)
            {
                Debug.Log($"Node {i} | fCost {finalPath[i].fCost} | gCost {finalPath[i].gCost} | hCost {finalPath[i].hCost} | " +
                    $"Heading: {finalPath[i].heading * Mathf.Rad2Deg} | Trailer: {finalPath[i].TrailerHeadingInDegrees} | " +
                    $"Relative: {Mathf.DeltaAngle(finalPath[i].TrailerHeadingInDegrees, finalPath[i].heading * Mathf.Rad2Deg)} | " +
                    $"Reversing: {finalPath[i].isReversing} | " +
                    $"Steering Angle: {finalPath[i].SteeringAngleInDegrees}");
            }

            //Display how long time everything took
            string display = DisplayController.GetDisplayTimeText(timer_selectLowestCostNode, "Select lowest cost node");

            display += DisplayController.GetDisplayTimeText(timer_addNodeToHeap, "Add new node to heap");

            display += DisplayController.GetDisplayTimeText(timer_findChildren, "Find children");

            display += DisplayController.GetDisplayTimeText(timer_isCollidingWithObstacle, "Is node colliding");

            display += DisplayController.GetDisplayTimeText(timer_ReedsSheppNode, "Reeds-Shepp Node");

            display += DisplayController.GetDisplayTimeText(timer_ReedsSheppHeuristics, "Reeds-Shepp Heuristics");

            display += DisplayController.GetDisplayTimeText(timer_TrailerCollision, "Trailer collision");

            display += DisplayController.GetDisplayText("Max nodes in heap", maxNodesInHeap, ". ");

            display += DisplayController.GetDisplayText("Expanded nodes", allExpandedNodes.Count, ". ");
            
            display += DisplayController.GetDisplayText("Pruned nodes", prunedNodes, null);

            Debug.Log(display);


            //Display car positions along the final path
            DisplayController.DisplayVehicleAlongPath(finalPath, startCar.carData, startTrailer);


            return finalPath;
        }



        //
        // Get all children to a node
        //
        private static List<Node> GetChildrenToNode(Node currentNode, Map map, Cell[,] cellData, CarData carData, Car endCar, Car startTrailer, Car endTrailer)
        {
            List<Node> childNodes = new List<Node>();
        
            //To be able to expand we need the simulated vehicle's heading and position
            float heading = currentNode.heading;

            //Expand both forward and reverse
            for (int i = 0; i < driveDistances.Length; i++)
            {
                float driveDistance = driveDistances[i];

                //Expand all steering angles
                for (int j = 0; j < steeringAngles.Length; j++)
                {
                    //Steering angle
                    float alpha = steeringAngles[j];

                    //Turning angle
                    float beta = (driveDistance / carData.WheelBase) * Mathf.Tan(alpha);

                    //Simulate the car driving forward by using a mathematical car model
                    Vector3 newRearWheelPos = VehicleSimulationModels.CalculateNewPosition(heading, beta, driveDistance, currentNode.rearWheelPos);

                    float newHeading = VehicleSimulationModels.CalculateNewHeading(heading, beta);

                    //In which cell did we end up?
                    IntVector2 cellPos = map.ConvertWorldToCell(newRearWheelPos);

                    //Because we are doing obstacle detection later, we have to check if this pos is within the map
                    if (!map.IsCellWithinGrid(cellPos))
                    {
                        continue;
                    }

                    //Generate a new child node
                    Node childNode = new Node(
                       previousNode: currentNode,
                       rearWheelPos: newRearWheelPos,
                       heading: newHeading,
                       isReversing: driveDistance < 0f ? true : false);
                    childNode.steeringAngle = alpha;

                    float heuristics = HeuristicsToReachGoal(cellData, cellPos, childNode, endCar, carData);

                    childNode.AddCosts(
                        gCost: CostToReachNode(childNode, map, cellData, alpha, 0f),
                        hCost: heuristics);

                    //Calculate the new heading of the trailer if we have a trailer and add trailer heuristics
                    if (startTrailer != null)
                    {
                        //Whats the new trailer heading at this childNode
                        float thetaOld = currentNode.TrailerHeadingInRadians;
                        float thetaOldDragVehicle = currentNode.HeadingInRadians;
                        float D = driveDistance;
                        float d = startTrailer.carData.WheelBase;
                        float newTrailerHeading = VehicleSimulationModels.CalculateNewTrailerHeading(thetaOld, thetaOldDragVehicle, driveDistance,
                                                                                                     startTrailer.carData.WheelBase, beta);

                        childNode.TrailerHeadingInRadians = newTrailerHeading;

                        //The trailer sux when reversing so add an extra cost
                        if (childNode.isReversing)
                        {
                            childNode.gCost -= (Parameters.trailerReverseCost - Parameters.reverseCost) * driveDistance;
                        }

                        // Add generic cost of trailer angle
                        //Debug.Log($"State info | Heading: {newHeading} | Trailer heading: {newTrailerHeading}");
                        float relTrailerAngle = Math.Abs(Mathf.DeltaAngle(newTrailerHeading * Mathf.Rad2Deg, newHeading * Mathf.Deg2Rad));
                        childNode.gCost += relTrailerAngle * Parameters.trailerAngleCost;

                        //We know where the trailer is attached to the drag vehicle
                        Vector3 trailerAttachmentPoint = carData.GetTrailerAttachmentPoint(newRearWheelPos, childNode.HeadingInRadians);
                        //Now we need the trailer's rear-wheel position based on the new heading and current attachment point
                        Vector3 trailerRearWheelPos = startTrailer.carData.GetTrailerRearWheelPos(trailerAttachmentPoint, newTrailerHeading);
                        Vector3 trailerToTarget = endTrailer.rearWheelPos - trailerRearWheelPos;
                        //Debug.Log(trailerToTarget);
                        float trailerDistance = (trailerToTarget).magnitude;
                        //Debug.Log(trailerDistance);
                        float trailerForwardDistance = (float)Math.Abs((endTrailer.rearWheelPos.x - trailerRearWheelPos.x) * Math.Sin(endTrailer.HeadingInRadians) 
                            + (endTrailer.rearWheelPos.z - trailerRearWheelPos.z) * Math.Cos(endTrailer.HeadingInRadians));
                        float trailerSidewaysDistance = (float)Math.Abs((endTrailer.rearWheelPos.x - trailerRearWheelPos.x) * Math.Cos(endTrailer.HeadingInRadians) 
                            - (endTrailer.rearWheelPos.z - trailerRearWheelPos.z) * Math.Sin(endTrailer.HeadingInRadians));
                        float truckSidewaysDistance = (float)Math.Abs((endCar.rearWheelPos.x - newRearWheelPos.x) * Math.Cos(endCar.HeadingInRadians)
                            - (endCar.rearWheelPos.z - newRearWheelPos.z) * Math.Sin(endCar.HeadingInRadians));
                        float trailerAngleH = 0f;

                        if (trailerDistance < 55)
                        { // Add heuristic costs for trailer position/angle
                            trailerAngleH = Math.Abs(Mathf.DeltaAngle(newTrailerHeading * Mathf.Rad2Deg, endTrailer.HeadingInDegrees));
                            trailerAngleH *= Mathf.Clamp01(1f - (trailerDistance - 25) / (55 - 25));
                        }

                        if (trailerForwardDistance < 25)
                        {
                            trailerForwardDistance = 0;
                        }
                        else
                        {
                            trailerForwardDistance -= 25;
                        }

                        float trailerHeuristics = trailerAngleH * Parameters.trailerAngle + trailerDistance * Parameters.trailerDistance +
                            trailerForwardDistance * Parameters.trailerForwardDistance + trailerSidewaysDistance * Parameters.trailerSidewaysDistance + 
                            truckSidewaysDistance * Parameters.truckSidewaysDistance;

                        childNode.hCost += trailerHeuristics;
                        /*Debug.Log($"fCost: {childNode.fCost} | gCost: {childNode.gCost} | relAngle: {relTrailerAngle} | " +
                            $"hCost: {childNode.hCost} | Trailer: {trailerHeuristics} | Angle: {trailerAngleH} | " +
                            $"Distance: {trailerDistanceH} | Forward: {trailerForwardDistance} | Sideways: {trailerSideaysDistance}");*/

                    }

                    childNodes.Add(childNode);
                }
            }



            //Expand Reeds-Shepp curve and add it as child node if we are "close" to the goal we want to reach
            int timeBefore = Environment.TickCount;

            //Dont do it every node because is expensive
            IntVector2 goalCell = map.ConvertWorldToCell(endCar.rearWheelPos);

            float distanceToEnd = cellData[goalCell.x, goalCell.z].distanceToTarget;

            //The probability should increase the close to the end we are
            float testProbability = Mathf.Clamp01((maxReedsSheppDist - distanceToEnd) / maxReedsSheppDist) * 0.2f;

            float probability = UnityEngine.Random.Range(0f, 1f);

            if (distanceToEnd < maxReedsSheppDist && (probability < testProbability || probability < 0.005f))
            {
                List<RSCar> shortestPath = ReedsShepp.GetShortestPath(
                    currentNode.rearWheelPos, 
                    currentNode.heading, 
                    endCar.rearWheelPos, 
                    endCar.HeadingInRadians, 
                    carData.turningRadius, 
                    driveDistance,
                    generateOneWp: true);

                if (shortestPath != null && shortestPath.Count > 1)
                {
                    //The first node in this list is where we currently are so we will use the second node
                    //But we might need to use several Reeds-Shepp nodes because if the path is going from
                    //forward to reverse, we cant ignore the change in direction, so we add a node before the 
                    //length which should be the driving distance

                    //But the easiest is just to add the second node
                    RSCar carToAdd = shortestPath[1];

                    bool isReversing = carToAdd.gear == RSCar.Gear.Back ? true : false;

                    IntVector2 cellPos = map.ConvertWorldToCell(carToAdd.pos);

                    //Because we are doing obstacle detection later, we have to check if this pos is within the map
                    if (map.IsCellWithinGrid(cellPos))
                    {
                        Node childNode = new Node(
                           previousNode: currentNode,
                           rearWheelPos: carToAdd.pos,
                           heading: carToAdd.HeadingInRad,
                           isReversing: isReversing);

                        float heuristics = HeuristicsToReachGoal(cellData, cellPos, childNode, endCar, carData);

                        childNode.AddCosts(
                            gCost: CostToReachNode(childNode, map, cellData, 0f, 0f),
                            hCost: heuristics);

                        childNodes.Add(childNode);

                        if (childNode.heading < 0)
                        {
                            Debug.Log("RS Heading < 0!");
                            Debug.Log(distanceToEnd < maxReedsSheppDist);
                            Debug.Log(distanceToEnd);
                        }
                        //Debug.Log("Added RS node");
                    }    
                }
            }

            timer_ReedsSheppNode += Environment.TickCount - timeBefore;

            return childNodes;
        }



        //
        // Calculate heuristics
        //
        private static float HeuristicsToReachGoal(Cell[,] cellData, IntVector2 cellPos, Node node, Car endCar, CarData carData)
        {
            float heuristics = cellData[cellPos.x, cellPos.z].heuristics;

            //But if we are close we might want to use the Reeds-Shepp distance as heuristics
            //This distance can be pre-calculated
            if (cellData[cellPos.x, cellPos.z].distanceToTarget < 0f)
            {
                int timeBefore = Environment.TickCount;

                float RS_distance = ReedsShepp.GetShortestDistance(
                    node.rearWheelPos,
                    node.heading,
                    endCar.rearWheelPos,
                    endCar.HeadingInRadians,
                    carData.turningRadius);

                timer_ReedsSheppHeuristics += Environment.TickCount - timeBefore;

                //Should use the max value according to the Junior report
                if (RS_distance > heuristics)
                {
                    heuristics = RS_distance;

                    //Debug.Log("Added Reeds-Shepp heuristics");
                }
            }

            return heuristics * Parameters.carDistance;
        }


        //
        // Calculate costs
        //
        private static float CostToReachNode(Node node, Map map, Cell[,] cellData, float steeringAngle, float oldSteeringAngle)
        {
            Node previousNode = node.previousNode;

            IntVector2 cellPos = map.ConvertWorldToCell(node.rearWheelPos);


            //Cost 0 - how far have we driven so far
            float costSoFar = previousNode.gCost;

            //Cost 1 - driving distance to reach this node
            //Cant use driveDistance because sometimes we take steps smaller than than when generating Reeds-Shepp curves
            float distanceCost = (node.rearWheelPos - previousNode.rearWheelPos).magnitude;

            //Cost 2 - avoid obstacles by using the voronoi field
            float voronoiCost = Parameters.obstacleCost * cellData[cellPos.x, cellPos.z].voronoiFieldCell.voronoiFieldValue;
            voronoiCost = 0;

            //Cost 3 - reversing because its better to drive forward
            float reverseCost = node.isReversing ? Parameters.reverseCost : 0f;

            //Cost 4 - changing direction of motion from forward to reverse or the opposite because its annoying to sit in such a car
            float switchMotionCost = 0f;

            if (node.isReversing != previousNode.isReversing)
            {
                if (previousNode.previousNode != null)
                {
                    switchMotionCost = Parameters.switchingDirectionOfMovementCost;
                }
            }

            //Cost 5 - steering angle
            float steeringCost = Math.Abs(Mathf.DeltaAngle(steeringAngle, 0)) * Parameters.turningCost;

            //Cost 6 - change in steering angle
            float steeringChangeCost = Math.Abs(steeringAngle - node.steeringAngle) * Parameters.turningChangeCost;

            //Calculate the final cost
            float cost = costSoFar + distanceCost * (1f + voronoiCost + reverseCost) + switchMotionCost + steeringCost + steeringChangeCost;


            return cost;
        }



        //
        // Generate the final path when Hybrid A* has found the goal node
        //
        private static List<Node> GenerateFinalPath(Node finalNode)
        {
            List<Node> finalPath = new List<Node>();

            //Generate the path
            Node currentNode = finalNode;

            //Loop from the end of the path until we reach the start node
            while (currentNode != null)
            {
                if (!finalPath.Contains(currentNode))
                {
                    finalPath.Add(currentNode);
                }
                else
                {
                    //Debug.Log("node already in path! Circle detected");
                    break;
                }

                //Get the next node
                currentNode = currentNode.previousNode;
            }

            //If we have found a path 
            if (finalPath.Count > 1)
            {
                //Reverse the list so the finalNode is the last one in the list
                finalPath.Reverse();

                //Make sure the first node has the same driving direction has the second node
                //We dont really need it but it looks better when debugging
                finalPath[0].isReversing = finalPath[1].isReversing;
            }

            return finalPath;
        }
    }
}
