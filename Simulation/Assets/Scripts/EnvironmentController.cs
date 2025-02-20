using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EnvironmentController : MonoBehaviour
{
    [SerializeField] private int _mapLength = 10;
    [SerializeField] private int _mapWidth = 10;
    [SerializeField] private bool _walls = false;
    [SerializeField] private bool _alternate = false;
    [SerializeField] private float _wallsDistance = 1.5f;
    [SerializeField] private float _maxStartRotationAngle = 360f;
    [SerializeField] private float _minWallLength = 0.1f;
    [SerializeField] private float _maxWallLength = 15f;

    [SerializeField] private Transform _scoutTransform;
    [SerializeField] private Transform _targetTransform;

    private Drive _agent;
    private GameObject _wallLeft;
    private GameObject _wallRight;
    private float _wallHeight = 2f;
    private int _episode = 0;

    void Start()
    {
        _agent = _scoutTransform.GetComponent<Drive>();
        _agent.EnvRestart += InitializeEnv;

        if (_walls)
        {
            _wallLeft = GameObject.CreatePrimitive(PrimitiveType.Cube);
            _wallLeft.transform.parent = transform;
            _wallLeft.transform.localPosition = new Vector3(0, _wallHeight / 2f, _mapLength / 2f);
            _wallRight = GameObject.CreatePrimitive(PrimitiveType.Cube);
            _wallRight.transform.parent = transform;
            _wallRight.transform.localPosition = new Vector3(_mapWidth, _wallHeight / 2f, _mapLength / 2f);
        }

        // makes the raycast ignore the robot
        GameObject scoutGO = _scoutTransform.gameObject;
        scoutGO.layer = 2;
        for (int i = 0; i < _scoutTransform.childCount; i++)
        {
            GameObject child = _scoutTransform.GetChild(i).gameObject;
            child.layer = 2;
        }

        InitializeEnv();
    }

    private void InitializeEnv()
    {
        _episode++;
        var startPosition = new Vector3(Random.Range(0, _mapLength), 0.25f, Random.Range(0, _mapWidth));
        _scoutTransform.localPosition = startPosition;
        
        _targetTransform.localPosition = new Vector3(Random.Range(0, _mapLength), 0f, Random.Range(0, _mapWidth));
        _agent.SetTargetPosition(new Vector2(_targetTransform.position.x, _targetTransform.position.z));

        _scoutTransform.LookAt(_targetTransform.position);
        // TODO: Forward?
        _scoutTransform.Rotate(Vector3.up * ((Random.value - 0.5f) * _maxStartRotationAngle));

        if (_alternate && _episode % 100 == 0)
        {
            _walls ^= true;
            _wallLeft.SetActive(_walls);
            _wallRight.SetActive(_walls);
        }

        if (_walls)
        {
            Vector3 targetDir = Vector3.Normalize(new Vector3(_targetTransform.localPosition.x - startPosition.x, 0f, _targetTransform.localPosition.z - startPosition.z));
            Vector3 wallPos = new Vector3((_targetTransform.localPosition.x + startPosition.x) / 2f, 1f, (_targetTransform.localPosition.z + startPosition.z) / 2f);
            Vector3 perpVectorToTargetDir = Vector3.Cross(targetDir, Vector3.up);

            var shiftWall = Random.Range(-1f, 1f);
            _wallLeft.transform.localScale = new Vector3(0.1f, _wallHeight, Random.Range(_minWallLength, _maxWallLength));
            _wallLeft.transform.rotation = Quaternion.LookRotation(targetDir);
            _wallLeft.transform.localPosition = wallPos - perpVectorToTargetDir * _wallsDistance + perpVectorToTargetDir * shiftWall;
            _wallRight.transform.localScale = new Vector3(0.1f, _wallHeight, Random.Range(_minWallLength, _maxWallLength));
            _wallRight.transform.rotation = Quaternion.LookRotation(targetDir);
            _wallRight.transform.localPosition = wallPos + perpVectorToTargetDir * _wallsDistance + perpVectorToTargetDir * shiftWall;
        }
    }
}
