using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RefillStation : MonoBehaviour
{
    [SerializeField] private GameObject _scoutParent;
    [SerializeField] private Transform _scoutTransform;
    [SerializeField] private GameObject _baseStationPoint;
    private DemoDrive _agent; 

    [SerializeField] private float _refillRate = 0.5f;
    [SerializeField] private float _refillRadius = 0.1f;

    private void Start()
    {
        _agent = _scoutParent.GetComponent<DemoDrive>();
        _baseStationPoint.transform.localScale = new Vector3(_refillRadius, 0.001f, _refillRadius);
    }

    private void FixedUpdate()
    {
        float robotToStationDist = Vector2.Distance(new Vector2(_scoutTransform.position.x, _scoutTransform.position.z), 
                                                    new Vector2(_baseStationPoint.transform.position.x, _baseStationPoint.transform.position.z));

        if (robotToStationDist < _refillRadius && (_agent.currentWater < _agent.maxWater || _agent.currentEnergy < _agent.maxEnergy * 0.99f))
        {
            _agent.inRefillZone = true;
            _agent.currentWater += _refillRate * Time.fixedDeltaTime;
            _agent.currentEnergy += _refillRate * Time.fixedDeltaTime;
            _agent.currentWater = Mathf.Clamp(_agent.currentWater, 0, _agent.maxWater);
            _agent.currentEnergy = Mathf.Clamp(_agent.currentEnergy, 0, _agent.maxEnergy);
        }
        else
        {
            _agent.inRefillZone = false;
        }
    }
}
