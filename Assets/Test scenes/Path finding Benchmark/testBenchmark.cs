using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PathfindingForVehicles;
using SelfDrivingVehicle;


public class testBenchmark : MonoBehaviour
{
    public static testBenchmark current;

    public Transform truckStart;
    public Transform trailerStart;

    public Transform truckTrailerGoal;
    private Transform truckTrailerMove;

    public enum VehicleTypes { None, Car, Semi, Semi_Trailer }
    private VehicleTypes activeVehicle = VehicleTypes.Semi_Trailer;
    //The vehicles start data
    private Vector3 startPos;
    private Quaternion startRot;

    void Awake()
    {
        current = this;

        GameObject deadTruckTrailerGoal = Instantiate(truckTrailerGoal.gameObject) as GameObject;

        truckTrailerMove = deadTruckTrailerGoal.transform;

        startPos = truckStart.position;
        startRot = truckStart.rotation;
        // Set vehicle to truck trailer
        ActivateSemiWithTrailerBench(startPos, startRot);
        Debug.Log("semi+trailer activated");
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    private void DeActivateAllVehiclesBench()
    {
        truckStart.gameObject.SetActive(false);
        trailerStart.gameObject.SetActive(false);
        truckTrailerGoal.gameObject.SetActive(false);
        truckTrailerMove.gameObject.SetActive(false);
    }


    private void ActivateSemiWithTrailerBench(Vector3 position, Quaternion rotation)
    {
        DeActivateAllVehiclesBench();

        truckStart.position = position;
        truckStart.rotation = rotation;
        //Make the models visible
        truckStart.gameObject.SetActive(true);

        //DisplayController.current.ResetGUI();

        //Move it to the correct position behind the semi
        trailerStart.position = position;
        trailerStart.rotation = rotation;

        //Make the models visible
        trailerStart.gameObject.SetActive(true);
        truckTrailerMove.gameObject.SetActive(true);

        Rigidbody rb = trailerStart.GetComponent<Rigidbody>();

        rb.angularVelocity = Vector3.zero;
        rb.velocity = Vector3.zero;

        activeVehicle = VehicleTypes.Semi_Trailer;
    }

    //The self-driving car moving around
    public Transform GetSelfDrivingCarTrans()
    {
        switch (activeVehicle)
        {
            case VehicleTypes.Semi_Trailer:
                return truckStart;
        }

        return null;
    }

    //Get data such as speed, length, etc belonging to the self-driving car
    public VehicleDataController GetActiveCarData()
    {
        Transform activeCar = truckStart;

        VehicleDataController carData = activeCar.GetComponent<VehicleController>().GetCarData();

        return carData;
    }

    public Transform GetCarMouse()
    {
        switch (activeVehicle)
        {
            case VehicleTypes.Semi_Trailer:
                return truckTrailerMove;
        }

        return null;
    }

    //Get the trailer transform attached to the self-driving semi if its active
    public Transform TryGetTrailerTrans()
    {
        if (activeVehicle == VehicleTypes.Semi_Trailer)
        {
            return trailerStart;
        }

        return null;
    }

    //Get the trailer data if its active
    public VehicleDataController TryGetTrailerData()
    {
        if (activeVehicle == VehicleTypes.Semi_Trailer)
        {
            return trailerStart.GetComponent<VehicleDataController>();
        }

        return null;
    }

    //Get the trailer transform attached to the mouse if its active
    public Transform TryGetTrailerTransMouse()
    {
        if (activeVehicle == VehicleTypes.Semi_Trailer)
        {
            return truckTrailerMove.GetComponent<SemiWithTrailer>().trailerTrans;
        }

        return null;
    }
}
