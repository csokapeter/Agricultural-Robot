using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using System;
using System.IO;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;

public class DemoDrive : Agent
{
    [SerializeField] private List<GameObject> _wheels;
    [SerializeField] private Rigidbody _scoutRb;
    [SerializeField] private Transform _scoutTransform;
    [SerializeField] private GameObject _lidarSensorFront;
    [SerializeField] private GameObject _lidarSensorBack;
    
    private const float _targetThreshold = 0.1f;

    private float _steerInput;
    private float _forwardInput;
    private float _currentDistance;
    private float _maxDistance;
    private float _signedAngle;
    private Vector2 _currentPosition;
    public Vector2 _targetPosition;
    private Vector2 _currentDirection;
    private Vector2 _targetDirection;

    public float maxWater = 40f;
    public float maxEnergy = 100f;
    private float _waterUsagePerPlant = 10f;
    private float _energyUsagePerSecond = 0.005f;
    private float _criticalEnergyLevel = 0.3f;

    public float currentWater;
    public float currentEnergy;
    public bool inRefillZone = false;
    private bool _hasCalledTargetPoint = false;

    public delegate void EnvironmentRestart();
    public event EnvironmentRestart EnvRestart;
    public delegate void TargetReached(bool isBaseStation);
    public event TargetReached TargetPointReached;

    private void Start()
    {
        currentWater = maxWater;
        currentEnergy = maxEnergy;
    }

    public override void OnEpisodeBegin()
    {
        CallEnvRestart();

        _hasCalledTargetPoint = false;
        _scoutRb.velocity = Vector3.zero;
        _scoutRb.angularVelocity = Vector3.zero;

        _currentDirection = new Vector2(-_scoutTransform.up.x, -_scoutTransform.up.z);
        _currentPosition = new Vector2(_scoutTransform.position.x, _scoutTransform.position.z);
        _currentDistance = Vector2.Distance(_currentPosition, _targetPosition);
        _targetDirection = Vector3.Normalize(new Vector2(_targetPosition.x - _currentPosition.x, _targetPosition.y - _currentPosition.y));
        _maxDistance = Vector2.Distance(_currentPosition, _targetPosition);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if ((currentEnergy < (maxEnergy * _criticalEnergyLevel) || currentWater < _waterUsagePerPlant) && !_hasCalledTargetPoint)
        {
            Debug.Log("Low on resources! Returning to refill station.");
            CallTargetPointReached(true);
            _hasCalledTargetPoint = true;
        }

        float[] lidarSensorMeasurementsFront = _lidarSensorFront.GetComponent<LidarSensor>().GetLidarSensorMeasurements();
        float[] lidarSensorMeasurementsBack = _lidarSensorBack.GetComponent<LidarSensor>().GetLidarSensorMeasurements();

        if (lidarSensorMeasurementsFront.Any(value => value < 0.04) || 
            lidarSensorMeasurementsBack.Any(value => value < 0.04))
        {
            Debug.Log("About to hit something! HELP! Exiting.");
            EditorApplication.ExitPlaymode();
        }


        Vector2 velocity = new Vector2(_scoutRb.velocity.x, _scoutRb.velocity.z);
        float speed = velocity.magnitude;
        speed *= Mathf.Sign(Vector2.Dot(velocity.normalized, new Vector2(-_scoutTransform.up.x, -_scoutTransform.up.z)));

        _currentPosition = new Vector2(_scoutTransform.position.x, _scoutTransform.position.z);
        _currentDistance = Vector2.Distance(_currentPosition, _targetPosition);
        _currentDirection.x = _scoutTransform.up.x * -1f;
        _currentDirection.y = _scoutTransform.up.z * -1f;
        _targetDirection = Vector3.Normalize(new Vector2(_targetPosition.x - _currentPosition.x, _targetPosition.y - _currentPosition.y));
        _signedAngle = Vector2.SignedAngle(_currentDirection, _targetDirection) / 180f;
        float dist = _maxDistance == 0 ? 0 : _currentDistance / _maxDistance;

        sensor.AddObservation(_signedAngle); // dim: 1
        sensor.AddObservation(speed); //dim: 1
        sensor.AddObservation(dist); //dim: 1
        sensor.AddObservation(lidarSensorMeasurementsFront); //dim: 4
        sensor.AddObservation(lidarSensorMeasurementsBack); //dim: 4
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (!inRefillZone && currentEnergy > 0f)
        {
            foreach (GameObject wheel in _wheels)
            {
                JointController joint = wheel.GetComponent<JointController>();
                joint.forwardInput = Mathf.Clamp(actions.ContinuousActions[0], -1.0f, 1.0f);
                _forwardInput = joint.forwardInput;
                joint.steerInput = Mathf.Clamp(actions.ContinuousActions[1], -1.0f, 1.0f);
                _steerInput = joint.steerInput;
                joint.UpdateWheel();
            }
            CheckDone();
            currentEnergy -= Time.deltaTime * 0.3f *(_energyUsagePerSecond + (Mathf.Abs(_forwardInput) + Mathf.Abs(_steerInput)));
            currentEnergy = Mathf.Clamp(currentEnergy, 0, maxEnergy);
        }
        else
        {
            Debug.Log($"Charging water: {currentWater}, energy: {currentEnergy}");
        }
        
    }

    private void CheckDone()
    {
        if (_currentDistance < _targetThreshold)
        {
            Debug.Log("TARGET REACHED");
            WaterPlant();
            CallTargetPointReached(false);
            return;
        }
    }

    public void CollisionDetected()
    {
        Debug.Log("COLLISION");
        AddReward(-200f);
        EndEpisode();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var ContinuousActions = actionsOut.ContinuousActions;
        _forwardInput = Input.GetAxis("Vertical");
        _steerInput = Input.GetAxis("Horizontal");

        ContinuousActions[0] = _forwardInput;
        ContinuousActions[1] = _steerInput;
    }

    public void SetTargetPosition(Vector2 targetPos)
    {
        _targetPosition = targetPos;
    }

    public void WaterPlant()
    {
        if (currentWater >= _waterUsagePerPlant)
        {
            currentWater -= _waterUsagePerPlant;
            Debug.Log($"Plant watered! Remaining water: {currentWater} Remaining energy: {currentEnergy}");
        }
    }

    private void CallEnvRestart()
    {
        EnvRestart?.Invoke();
    }

    private void CallTargetPointReached(bool isBaseStation)
    {
        TargetPointReached?.Invoke(isBaseStation);
    }
}
