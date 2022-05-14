using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace _04_drone.Scripts
{
    public class DroneInputs : MonoBehaviour
    {
        private Vector2 _cyclic;
        private float _pedals;
        private float _throttle;
        
        public Vector2 Cyclic { get => _cyclic; }
        public float Pedals { get => _pedals; }
        public float Throttle { get => _throttle; }
        
        void Update()
        {

        }
    }
}
