using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class LidarSensor : MonoBehaviour
{
    [SerializeField] private float _angleMin;
    [SerializeField] private float _angleMax;
    [SerializeField] private float _angleIncrement;
    [SerializeField] private float _rangeMin;
    [SerializeField] private float _rangeMax;
    [SerializeField] private int _numberOfSections;

    private float[] _measurements;
    private int _numberOfRays;

    void Start()
    {
        _measurements = Enumerable.Repeat(1f, _numberOfSections).ToArray();
        _numberOfRays = (int)((_angleMax - _angleMin) / _angleIncrement);
    }

    public float[] GetLidarSensorMeasurements()
    {
        RaycastHit hit;
        Vector3 lidarStartPosition = transform.position;
        _measurements = Enumerable.Repeat(1f, _numberOfSections).ToArray();
        float sectionSize = (_angleMax - _angleMin) / _numberOfSections;

        for (int i = 0; i < _numberOfSections; i++)
        {
            for (int j = 0; j < (_numberOfRays / _numberOfSections); j++)
            {
                float tempRad = (i * sectionSize) + (j * _angleIncrement);
                Vector3 dir = (transform.forward * Mathf.Cos(tempRad) + transform.right * Mathf.Sin(tempRad)).normalized;
                if (Physics.Raycast(lidarStartPosition, dir, out hit, _rangeMax))
                {
                    float normalizedDistance = hit.distance / _rangeMax;
                    if (normalizedDistance < _measurements[i])
                        _measurements[i] = normalizedDistance;
                }
            }
        }

        return _measurements;
    }
}
