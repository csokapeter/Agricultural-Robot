using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class JointController : MonoBehaviour
{
    // Input values for forward movement and turning
    public float forwardInput;
    public float steerInput;
    
    // Maximimum angular velocity of the wheel measured in radians per second
    [SerializeField] private float _maxAngularVelocity = 15f;
    // Relative torque added to the wheel
    [SerializeField] private int _speed = 10000;

    // Configurable joint component used for steering
    private ConfigurableJoint _joint;
    // Rigidbody component of the vehicle
    private Rigidbody _rb;
    // Enum defining the wheel positions
    private enum WheelPos
    {
        FrontRight,
        FrontLeft,
        RearRight,
        RearLeft
    };
    // The selected wheel position
    [SerializeField] private WheelPos _wheelPos;
    // Skid steering parameters
    [SerializeField] private GameObject _pair;
    [SerializeField] private bool _left;
    [SerializeField] private float _turnSpeed = 50f;
    [SerializeField] private float _actualSpeed;
    private bool _isGrounded;

    void Start()
    {
        _joint = this.GetComponent<ConfigurableJoint>();
        _rb = this.GetComponent<Rigidbody>();
        _isGrounded = false;
    }

    // Updates the steering behavior of the vehicle based on user input in fixed intervals.
    public void UpdateWheel()
    {
        transform.rotation = _pair.transform.rotation;
        _rb.maxAngularVelocity = _maxAngularVelocity;
        _joint.angularYMotion = ConfigurableJointMotion.Locked;
        _rb.freezeRotation = false;
        if (forwardInput != 0 && steerInput == 0)
        {
            GoLinear();
        }
        else if (forwardInput == 0 && steerInput != 0)
        {
            StandRotate();
        }
        else if ((steerInput == 0 && forwardInput == 0) || !_isGrounded)
        {
            Stop();
        }
        else
        {
            TurnAndGo();
        }
    }
    
    // Moves the object forward or backward in a straight line.
    private void GoLinear()
    {
        transform.rotation = _pair.transform.rotation;
        _rb.AddRelativeTorque((forwardInput * _speed), 0, 0);
        _actualSpeed = (forwardInput * _speed);
    }

    // Rotates the object in place when there is no forward input but there is turning input.
    private void StandRotate()
    {
        transform.rotation = _pair.transform.rotation;
        if (_left)
        {
            _rb.AddRelativeTorque((steerInput * _speed), 0, 0);
            _actualSpeed = (steerInput * _speed);
        }
        else
        {
            _rb.AddRelativeTorque((steerInput * -_speed), 0, 0);
            _actualSpeed = (steerInput * -_speed);
        }
    }

    // Combines turning and forward movement to achieve curved trajectories.
    private void TurnAndGo()
    {
        if (forwardInput > 0 && steerInput > 0)
        {
            if (_left)
            {
                transform.rotation = _pair.transform.rotation;
                _rb.AddRelativeTorque((forwardInput * _speed), 0, 0);
                _actualSpeed = forwardInput * _speed;
            }
            else
            {
                if (_wheelPos == WheelPos.FrontRight)
                {
                    transform.rotation = _pair.transform.rotation;
                }
                else
                {
                    _rb.AddRelativeTorque((forwardInput - steerInput) * _speed / _turnSpeed, 0, 0);
                    _actualSpeed = (forwardInput - steerInput) * _speed / _turnSpeed;
                }
            }
        }
        else if (forwardInput > 0 && steerInput < 0)
        {
            if (_left)
            {
                if (_wheelPos == WheelPos.FrontLeft)
                {
                    transform.rotation = _pair.transform.rotation;
                }
                else
                {
                    _rb.AddRelativeTorque((forwardInput + steerInput) * _speed / _turnSpeed, 0, 0);
                    _actualSpeed = (forwardInput + steerInput) * _speed / _turnSpeed;
                }
            }
            else
            {
                transform.rotation = _pair.transform.rotation;
                _rb.AddRelativeTorque(forwardInput * _speed, 0, 0);
                _actualSpeed = forwardInput * _speed;
            }
        }
        if (forwardInput < 0 && steerInput > 0)
        {
            if (_left)
            {
                transform.rotation = _pair.transform.rotation;
                _rb.AddRelativeTorque(forwardInput * _speed, 0, 0);
                _actualSpeed = forwardInput * _speed;
            }
            else
            {
                if (_wheelPos == WheelPos.RearRight)
                {
                    transform.rotation = _pair.transform.rotation;
                }
                else
                {
                    _rb.AddRelativeTorque((forwardInput + steerInput) * _speed / _turnSpeed, 0, 0);
                    _actualSpeed = (forwardInput + steerInput) * _speed / _turnSpeed;
                }
            }
        }
        else if (forwardInput < 0 && steerInput < 0)
        {
            if (_left)
            {
                if (_wheelPos == WheelPos.RearLeft)
                {
                    transform.rotation = _pair.transform.rotation;
                }
                else
                {
                    _rb.AddRelativeTorque((forwardInput - steerInput) * _speed / _turnSpeed, 0, 0);
                    _actualSpeed = (forwardInput - steerInput) * _speed / _turnSpeed;
                }
            }
            else
            {
                transform.rotation = _pair.transform.rotation;
                _rb.AddRelativeTorque(forwardInput * _speed, 0, 0);
                _actualSpeed = forwardInput * _speed;
            }
        }
    }

    private void Stop()
    {
        _actualSpeed = 0f;
    }

    private void OnCollisionStay(Collision collision)
    {
        _isGrounded = true;
    }

    private void OnCollisionExit(Collision collision)
    {
        _isGrounded = false;
    }
}
