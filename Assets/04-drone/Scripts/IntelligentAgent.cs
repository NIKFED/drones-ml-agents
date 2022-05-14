using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;
using _04_drone.Scripts.Interfaces;
using System.Collections.Generic;
using System.Linq;

namespace _04_drone.Scripts
{
    [RequireComponent(typeof(DroneInputs))]
    public class IntelligentAgent : Agent
    {
        [Header("Main Properties")] 
        public GameObject marker;
        public GameObject target;

        [Header("Engine Properties")] 
        [SerializeField] private float startTargetHeightMin = -3f;
        [SerializeField] private float startTargetHeightMax = 3f;
        [SerializeField] private float rewardScale = 1f;
        [SerializeField] private float energyRewardScale = -0.25f;
        [SerializeField] private bool visualize = true;
        // public float forceMultiplier = 20f;
        
        /*[Header("Control Properties")]
        [SerializeField] private float minMaxPitch = 30f;
        [SerializeField] private float minMaxRoll = 30f;
        [SerializeField] private float yawPower = 4f;
        [SerializeField] private float lerpSpeed = 2f;*/

        [Header("Other Properties")] 
        private Rigidbody _markerRBody;
        private DroneController _droneController;
        float previousDistance = 0f;
    
        Vector3 startPosition;
        Vector3 startLocalPosition;
        BehaviorParameters behaviorParams;
        
        // private List<IEngine> _engines = new List<IEngine>();

        private DroneInputs _input;
        // private float[] _actions;
        private List<float> _actions;

        // private float _finalPitch;
        // private float _finalRoll;
        // private float _finalYaw;
        // private float _yaw;

        // Start is called before the first frame update
        void Start()
        {
            // Debug.Log("Start DroneAgent");
            _input = GetComponent<DroneInputs>();
            // _engines = GetComponentsInChildren<IEngine>().ToList<IEngine>();
            marker.AddComponent<DroneController>();
            
            _markerRBody = marker.GetComponent<Rigidbody>();
            startPosition = transform.position;
            startLocalPosition = transform.localPosition;
            
            _droneController = marker.GetComponent<DroneController>();
            _droneController.Init(_markerRBody);

            behaviorParams = GetComponent<BehaviorParameters>();

            _actions = new List<float>();
            // _actions = new float[4];
        }
    
        // How to reinitialize when the game is reset. The Start() of an ML Agent
        public override void OnEpisodeBegin()
        {
            // If the Agent fell, zero its momentum
            if (Math.Abs(marker.transform.localPosition.x - startLocalPosition.x) > 10 ||
                Math.Abs(marker.transform.localPosition.y - startLocalPosition.y) > 10 ||
                Math.Abs(marker.transform.localPosition.z - startLocalPosition.z) > 10)
            {
                _markerRBody.angularVelocity = Vector3.zero;
                _markerRBody.velocity = Vector3.zero;
                _markerRBody.rotation = Quaternion.identity;
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
                    var reward = CalculateReward(y, target.transform.position.y, 0f);
                    var c = Color.Lerp(Color.black, Color.white, reward * 5);
                    Debug.DrawLine(new Vector3(marker.transform.position.x, 
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
            sensor.AddObservation(_markerRBody.velocity); // Current velocity
            sensor.AddObservation(marker.transform.rotation); // Quaternion
        }
    
        // What to do every step. The Update() of an ML Agent
        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            _actions.Clear();
            // This example only uses continuous space
            // if (behaviorParams.BrainParameters.VectorActionDescriptions != SpaceType.Continuous)
            // {
            //     Debug.LogError("Must be continuous state type");
            //     return;
            // }
            
            // _actions[0] = Mathf.Clamp(actionBuffers.ContinuousActions[0], -1f, 1f) * upForce; // One of many actions
            // _actions[1] = Mathf.Clamp(actionBuffers.ContinuousActions[1], -1f, 1f) * upForce;
            // _actions[2] = Mathf.Clamp(actionBuffers.ContinuousActions[2], -1f, 1f) * upForce;
            // _actions[3] = Mathf.Clamp(actionBuffers.ContinuousActions[3], -1f, 1f) * upForce;

            float throttleAction = Mathf.Clamp(actionBuffers.ContinuousActions[0], -1f, 1f); // One of many actions
            float cyclicxAction = Mathf.Clamp(actionBuffers.ContinuousActions[1], -1f, 1f);
            float cyclicyAction = Mathf.Clamp(actionBuffers.ContinuousActions[2], -1f, 1f);
            float pedalsAction = Mathf.Clamp(actionBuffers.ContinuousActions[3], -1f, 1f);

            _actions.AddRange(new[]
            {
                throttleAction,
                cyclicxAction,
                cyclicyAction,
                pedalsAction
            });
            
            // float[] actions =
            // {
            //     throttleAction,
            //     cyclicxAction,
            //     cyclicyAction,
            //     pedalsAction
            // };
            
            // float[] actions =
            // {
            //     throttleAction
            // };
            _droneController.HandlePhysics(_actions);
            
            // Calculate the absolute value of the energy used
            // var energyUsed = (Mathf.Abs(rfAction) 
            //                   + Mathf.Abs(lfAction) 
            //                   + Mathf.Abs(rbAction) 
            //                   + Mathf.Abs(lbAction)) 
            //                  / upForce;

            var energyUsed = 0f;
            // Vector3 controlSignal = Vector3.zero;
            // controlSignal.x = actionBuffers.ContinuousActions[0];
            // controlSignal.y = actionBuffers.ContinuousActions[1];
            // controlSignal.z = actionBuffers.ContinuousActions[2];
            // markerRBody.AddForce(controlSignal * forceMultiplier);
            
            // Rewards
            float distanceToTarget = Vector3.Distance(transform.localPosition, target.transform.localPosition);
            // var reward = MyCalculateReward(distanceToTarget);
            var reward = NewMyCalculateReward(energyUsed);
            SetReward(reward);
            // Debug.Log("reward = " + reward);
    
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
        
        // private void HandlePhysics()
        // {
        //     // Debug.Log("HandlePhysics");
        //     HandleEngines();
        //     HandleControls();
        // }
        //
        // private void HandleEngines()
        // {
        //     // Debug.Log("HandleEngines");
        //     foreach (var engine in _engines)
        //     {
        //         // switch (engine.GetObjectName())
        //         // {
        //         //     case "RFEngine":
        //         //         engine.UpdateEngine(_markerRBody, actions[0], rfEngine.transform.position);
        //         //         break;
        //         //     case "LFEngine":
        //         //         engine.UpdateEngine(_markerRBody, actions[1], lfEngine.transform.position);
        //         //         break;
        //         //     case "RBEngine":
        //         //         engine.UpdateEngine(_markerRBody, actions[2], rbEngine.transform.position);
        //         //         break;
        //         //     case "LBEngine":
        //         //         engine.UpdateEngine(_markerRBody, actions[3], lbEngine.transform.position);
        //         //         break;
        //         // }
        //         engine.UpdateEngine(_markerRBody, _actions[0]);
        //     }
        // }
        //
        // private void HandleControls()
        // {
        //     // Debug.Log("HandleControls");
        //     float pitch =  _actions[1] * minMaxPitch;
        //     float roll = -_actions[2] * minMaxRoll;
        //     _yaw += _actions[3] * yawPower;
        //
        //     _finalPitch = Mathf.Lerp(_finalPitch, pitch, Time.deltaTime * lerpSpeed);
        //     _finalRoll = Mathf.Lerp(_finalRoll, roll, Time.deltaTime * lerpSpeed);
        //     _finalYaw = Mathf.Lerp(_finalYaw, _yaw, Time.deltaTime * lerpSpeed);
        //     
        //     Quaternion rot = Quaternion.Euler(_finalPitch, _finalRoll, _finalYaw);
        //     _markerRBody.MoveRotation(rot);
        //
        // }
        
        // Tell the ML algorithm everything you can about the current state
        private float MyCalculateReward(float currentDistance)
        {
            var distance = currentDistance;
            var improvement = previousDistance - distance;
            previousDistance = distance;
    
            var improvementPortion = rewardScale * improvement;
    
            return improvementPortion;
        }
        
        // Tell the ML algorithm everything you can about the current state
        private float NewMyCalculateReward(float energyUsed)
        {
            float distance = Vector3.Distance(transform.localPosition, target.transform.localPosition);
            var improvement = previousDistance - distance;
            previousDistance = distance;
    
            var improvementPortion = rewardScale * improvement;
            var energyPortion = energyRewardScale * energyUsed;
    
            return improvementPortion + energyPortion;
        }
        
        // Tell the ML algorithm everything you can about the current state
        private float CalculateReward(float markerY, float targetY, float energyUsed)
        {
            var distance = Vector3.Distance(marker.transform.position, target.transform.position);
            var improvement = previousDistance - distance;
            previousDistance = distance;
    
            var improvementPortion = rewardScale * improvement;
            var energyPortion = energyRewardScale * energyUsed;
    
            return improvementPortion + energyPortion;
        }
        
        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var continuousActionsOut = actionsOut.ContinuousActions;
            // if (Input.GetKey(KeyCode.Q))
            // {
            //     print("Q key");
            //     continuousActionsOut[0] = 1;
            // }
            // if (Input.GetKey(KeyCode.W))
            // {
            //     print("W key");
            //     continuousActionsOut[1] = 1;
            // }
            // if (Input.GetKey(KeyCode.E))
            // {
            //     print("E key");
            //     continuousActionsOut[2] = 1;
            // }
            // if (Input.GetKey(KeyCode.R))
            // {
            //     print("R key");
            //     continuousActionsOut[3] = 1;
            // }
            // if (Input.GetKey(KeyCode.Q))
            // {
            //     print("Q key");
            //     continuousActionsOut[0] = 1;
            //     continuousActionsOut[1] = 1;
            //     continuousActionsOut[2] = 1;
            //     continuousActionsOut[3] = 1;
            // }
            if (_input != null)
            {
                continuousActionsOut[0] = _input.Throttle;
                continuousActionsOut[1] = _input.Cyclic.x;
                continuousActionsOut[2] = _input.Cyclic.y;
                continuousActionsOut[3] = _input.Pedals;
            }
        }
    }
}