using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

public class SimpleDroneHoverScript : Agent
{
    public GameObject marker;
    public GameObject target;
    public GameObject frontLeftThruster;
    public GameObject frontRightThruster;
    public GameObject backLeftThruster;
    public GameObject backRightThruster;

    public float startTargetHeightMin = -3f;
    public float startTargetHeightMax = 3f;
    public float upForce = 10f;
    public float rewardScale = 1f;
    public float energyRewardScale = -0.25f;
    public bool visualize = true;
    public float forceMultiplier = 20f;

    Rigidbody markerRBody;
    float previousDistance = 0f;

    Vector3 startPosition;
    Vector3 startLocalPosition;
    BehaviorParameters behaviorParams;
    
    // Start is called before the first frame update
    void Start()
    {
        markerRBody = marker.GetComponent<Rigidbody>();
        startPosition = transform.position;
        startLocalPosition = transform.localPosition;
        behaviorParams = GetComponent<BehaviorParameters>();
    }

    // How to reinitialize when the game is reset. The Start() of an ML Agent
    public override void OnEpisodeBegin()
    {
        // If the Agent fell, zero its momentum
        if (Math.Abs(marker.transform.localPosition.x - startLocalPosition.x) > 10 ||
            Math.Abs(marker.transform.localPosition.y - startLocalPosition.y) > 10 ||
            Math.Abs(marker.transform.localPosition.z - startLocalPosition.z) > 10)
        {
            markerRBody.angularVelocity = Vector3.zero;
            markerRBody.velocity = Vector3.zero;
            markerRBody.rotation = Quaternion.identity;
            marker.transform.localPosition = new Vector3( 0, -0.38f, 0);
        }
        // if (this.markerRBody != null)
        // {
        //     this.markerRBody.velocity = Vector3.zero;
        //     this.markerRBody.rotation = Quaternion.identity;
        // }

        // this.marker.transform.position =
        //     new Vector3(0, Random.Range(this.startingHeightMin, this.startingHeightMax), 0) 
        //     + this.startPosition;
        target.transform.position =
            new Vector3(Random.Range(startTargetHeightMin, startTargetHeightMax), Random.Range(startTargetHeightMin, startTargetHeightMax), Random.Range(startTargetHeightMin, startTargetHeightMax)) 
            + startPosition;
        previousDistance = Vector3.Distance(marker.transform.position, target.transform.position);

        if (visualize)
        {
            for (var y = -2f; y < 2f; y += 0.05f)
            {
                var reward = CalculateReward(y, this.target.transform.position.y, 0f);
                var c = Color.Lerp(Color.black, Color.white, reward * 5);
                Debug.DrawLine(new Vector3(this.marker.transform.position.x, 
                    y, 
                    marker.transform.position.z), 
                    new Vector3(marker.transform.position.x, 
                        y + 0.05f, 
                        marker.transform.position.z), 
                    c, 
                    30);
            }
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(target.transform.position - marker.transform.position); // distance to target
        sensor.AddObservation(markerRBody.velocity); // Current velocity
        sensor.AddObservation(marker.transform.rotation); // Quaternion
    }

    // What to do every step. The Update() of an ML Agent
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // This example only uses continuous space
        /*if (behaviorParams.BrainParameters.VectorActionDescriptions != SpaceType.Continuous)
        {
            Debug.LogError("Must be continuous state type");
            return;
        }*/

        float actionFrontRight = Mathf.Clamp(actionBuffers.ContinuousActions[0], -1f, 1f) * this.upForce; // One of many actions
        float actionFrontLeft = Mathf.Clamp(actionBuffers.ContinuousActions[1], -1f, 1f) * this.upForce;
        float actionBackRight = Mathf.Clamp(actionBuffers.ContinuousActions[2], -1f, 1f) * this.upForce;
        float actionBackLeft = Mathf.Clamp(actionBuffers.ContinuousActions[3], -1f, 1f) * this.upForce;

        // Calculate the absolute value of the energy used
        var energyUsed = (Mathf.Abs(actionFrontRight) 
                          + Mathf.Abs(actionFrontLeft) 
                          + Mathf.Abs(actionBackRight) 
                          + Mathf.Abs(actionBackLeft)) 
                         / this.upForce;

        this.markerRBody.AddForceAtPosition(this.marker.transform.up * actionFrontRight, frontRightThruster.transform.position);
        this.markerRBody.AddForceAtPosition(this.marker.transform.up * actionFrontLeft, frontLeftThruster.transform.position);
        this.markerRBody.AddForceAtPosition(this.marker.transform.up * actionBackRight, backRightThruster.transform.position);
        this.markerRBody.AddForceAtPosition(this.marker.transform.up * actionBackLeft, backLeftThruster.transform.position);
        this.markerRBody.AddForce(this.marker.transform.up * actionFrontRight);
        this.markerRBody.AddForce(this.marker.transform.up * actionFrontLeft);
        this.markerRBody.AddForce(this.marker.transform.up * actionBackRight);
        this.markerRBody.AddForce(this.marker.transform.up * actionBackLeft);

        // Vector3 controlSignal = Vector3.zero;
        // controlSignal.x = actionBuffers.ContinuousActions[0];
        // controlSignal.y = actionBuffers.ContinuousActions[1];
        // controlSignal.z = actionBuffers.ContinuousActions[2];
        // markerRBody.AddForce(controlSignal * forceMultiplier);
        
        // Rewards
        float distanceToTarget = Vector3.Distance(this.transform.localPosition, target.transform.localPosition);
        var reward = MyCalculateReward(distanceToTarget);
        SetReward(reward);

        // Reached target
        if (distanceToTarget < 0.15f)
        {
            Debug.Log("win");
            SetReward(1.0f);
            EndEpisode();
        }
        
        // Fell off platform
        else if (Math.Abs(marker.transform.localPosition.x - startLocalPosition.x) > 10 ||
                 Math.Abs(marker.transform.localPosition.y - startLocalPosition.y) > 10 ||
                 Math.Abs(marker.transform.localPosition.z - startLocalPosition.z) > 10)
        {
            Debug.Log("lose");
            EndEpisode();
        }
        
        // var reward = CalculateReward(this.marker.transform.position.y, this.target.transform.position.y, energyUsed);
        // SetReward(reward);
        // Debug.Log("reward =" + reward);
    }
    
    // Tell the ML algorithm everything you can about the current state
    public float MyCalculateReward(float currentDistance)
    {
        var distance = currentDistance;
        var improvement = this.previousDistance - distance;
        this.previousDistance = distance;

        var improvementPortion = this.rewardScale * improvement;

        return improvementPortion;
    }
    
    // Tell the ML algorithm everything you can about the current state
    public float CalculateReward(float markerY, float targetY, float energyUsed)
    {
        var distance = Vector3.Distance(this.marker.transform.position, this.target.transform.position);
        var improvement = this.previousDistance - distance;
        this.previousDistance = distance;

        var improvementPortion = this.rewardScale * improvement;
        var energyPortion = this.energyRewardScale * energyUsed;

        return improvementPortion + energyPortion;
    }
    
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        if (Input.GetKey(KeyCode.RightArrow))
        {
            print("-> key");
            continuousActionsOut[0] = 1;
        }
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            print("<- key");
            continuousActionsOut[0] = -1;
        }
        if (Input.GetKey(KeyCode.UpArrow))
        {
            print("/\\ key");
            continuousActionsOut[1] = 1;
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
            print("\\/ key");
            continuousActionsOut[1] = -1;
        }
        // if (Input.GetKey(KeyCode.D))
        // {
        //     print("D key");
        //     continuousActionsOut[3] = 1;
        // }
    }
}
