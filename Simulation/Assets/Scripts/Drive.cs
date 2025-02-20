using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;

public class Drive : Agent
{
    [SerializeField] private RenderTexture _renderTexture;
    private Texture2D _cameraImageTexture;

    [SerializeField] private List<GameObject> _frontWheels;
    [SerializeField] private List<GameObject> _wheels;
    [SerializeField] private Rigidbody _scoutRb;
    [SerializeField] private Transform _scoutTransform;
    [SerializeField] private GameObject _lidarSensorFront;
    [SerializeField] private GameObject _lidarSensorBack;
    
    [SerializeField] private int _maxSteeringAngle = 30;
    [SerializeField] private int _maxMotorTorque = 700;
    private int _steps = 1;
    private const float _targetThreshold = 0.1f;

    private float _steerInput;
    private float _forwardInput;
    private float _oldDistance;
    private float _currentDistance;
    private float _maxDistance;
    private float _signedAngle;
    private Vector2 _currentPosition;
    private Vector2 _targetPosition;
    private Vector2 _currentDirection;
    private Vector2 _targetDirection;

    public delegate void EnvironmentRestart();
    public event EnvironmentRestart EnvRestart;

    public override void OnEpisodeBegin()
    {
        CallEnvRestart();

        foreach (GameObject wheel in _frontWheels)
        {
            var wc = wheel.GetComponent<WheelCollider>();
            wc.motorTorque = 0;
            wc.brakeTorque = 0;
        }
        _scoutRb.velocity = Vector3.zero;
        _scoutRb.angularVelocity = Vector3.zero;

        _steps = 1;
        _currentDirection = new Vector2(-_scoutTransform.up.x, -_scoutTransform.up.z);
        _currentPosition = new Vector2(_scoutTransform.position.x, _scoutTransform.position.z);
        _currentDistance = _oldDistance = Vector2.Distance(_currentPosition, _targetPosition);
        _targetDirection = Vector3.Normalize(new Vector2(_targetPosition.x - _currentPosition.x, _targetPosition.y - _currentPosition.y));
        _signedAngle = Vector2.SignedAngle(_currentDirection, _targetDirection) / 180f;
        _maxDistance = Vector2.Distance(_currentPosition, _targetPosition);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        _steps++;
        float[] lidarSensorMeasurementsFront = _lidarSensorFront.GetComponent<LidarSensor>().GetLidarSensorMeasurements();
        float[] lidarSensorMeasurementsBack = _lidarSensorBack.GetComponent<LidarSensor>().GetLidarSensorMeasurements();

        var velocity = new Vector2(_scoutRb.velocity.x, _scoutRb.velocity.z);
        var speed = velocity.magnitude;
        speed *= Mathf.Sign(Vector2.Dot(velocity.normalized, new Vector2(-_scoutTransform.up.x, -_scoutTransform.up.z)));


        _currentPosition = new Vector2(_scoutTransform.position.x, _scoutTransform.position.z);
        _currentDistance = Vector2.Distance(_currentPosition, _targetPosition);
        _currentDirection.x = _scoutTransform.up.x * -1f;
        _currentDirection.y = _scoutTransform.up.z * -1f;
        _targetDirection = Vector3.Normalize(new Vector2(_targetPosition.x - _currentPosition.x, _targetPosition.y - _currentPosition.y));
        _signedAngle = Vector2.SignedAngle(_currentDirection, _targetDirection) / 180f;
        var dist = _maxDistance == 0 ? 0 : _currentDistance / _maxDistance;

        sensor.AddObservation(_signedAngle); //1
        sensor.AddObservation(speed); //1
        sensor.AddObservation(dist); //1
        sensor.AddObservation(lidarSensorMeasurementsFront); //4 
        sensor.AddObservation(lidarSensorMeasurementsBack); //4 
    }

    public override void OnActionReceived(ActionBuffers actions)
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
        CheckRewards();
    }

    private void CheckRewards()
    {
        if (_steps > 1500)
        {
            print("MAX STEPS REACHED");
            AddReward(-150f);
            EndEpisode();
            return;
        }

        if (_currentDistance < _targetThreshold)
        {
            print("TARGET REACHED");
            AddReward(200f);
            EndEpisode();
            return;
        }

        var deltaTargetDist = _oldDistance - _currentDistance;
        AddReward(deltaTargetDist * 10);
        _oldDistance = _currentDistance;
    }

    public void CollisionDetected()
    {
        print("COLLISION");
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

    private void CallEnvRestart()
    {
        EnvRestart?.Invoke();
    }
}
