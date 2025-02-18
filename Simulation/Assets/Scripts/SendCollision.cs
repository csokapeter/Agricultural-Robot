using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SendCollision : MonoBehaviour
{
    void OnCollisionEnter(Collision other)
    {
        transform.parent.GetComponent<Drive>().CollisionDetected();
    }
}
