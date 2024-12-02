using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KeepVelocity : MonoBehaviour
{
    public float force;
    public float torque;
    public Rigidbody rb;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {

        rb.AddForce (Random.onUnitSphere* force);
        rb.AddTorque(Random.onUnitSphere * torque);




    }
}
