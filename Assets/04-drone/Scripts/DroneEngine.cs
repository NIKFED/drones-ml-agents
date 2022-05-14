using System.Collections;
using System.Collections.Generic;
using _04_drone.Scripts.Interfaces;
using UnityEngine;

namespace _04_drone.Scripts
{
    [RequireComponent(typeof(BoxCollider))]
    public class DroneEngine : MonoBehaviour, IEngine
    {
        [Header("Engine Properties")] 
        [SerializeField] private float maxPower = 4f;
        
        public void InitEngine()
        {
            
        }

        public void UpdateEngine()
        {
            // Debug.Log("Running Engine: " + gameObject.name);
            
        }
    }
}
