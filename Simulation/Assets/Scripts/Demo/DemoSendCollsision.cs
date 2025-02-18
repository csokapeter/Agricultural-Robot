using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DemoSendCollsision : MonoBehaviour
{
    void OnCollisionEnter(Collision other)
    {
        transform.parent.GetComponent<DemoDrive>().CollisionDetected();
    }
}
