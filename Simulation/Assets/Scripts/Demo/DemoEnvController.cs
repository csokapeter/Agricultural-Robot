using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEditor;

public class DemoEnvController : MonoBehaviour
{
    [SerializeField] private Transform _scoutParentTransform;
    [SerializeField] private Transform _scoutTransform;
    [SerializeField] private Transform _targetTransform;
    [SerializeField] private GameObject _baseStationPoint;

    private DemoDrive _agent;

    private Vector3 _scoutStartPos;
    private float _scoutStartRotZ;

    private float _radius = 2f;
    private Vector2 _regionSize = new Vector2(20f, 20f);
    private int _rejectionSamples = 1;
    
    [SerializeField] private GameObject _flowerPrefab;
    private int _numberOfFlowers = 10;
    private List<Vector2> _flowerPositions;
    [SerializeField] private GameObject _dogPrefab;
    private int _numberOfDogs = 2;
    private List<Vector2> _dogPositions;
    [SerializeField] private GameObject _rockPrefab;
    private int _numberOfRocks = 2;
    private List<Vector2> _rockPositions;
    private int _currentTargetIdx = -1;

    void Start()
    {
        _agent = _scoutParentTransform.GetComponent<DemoDrive>();
        _agent.EnvRestart += ResetEnv;
        _agent.TargetPointReached += NextTarget;

        // makes the raycast ignore the rover
        GameObject scoutGO = _scoutParentTransform.gameObject;
        scoutGO.layer = 2;
        for (int i = 0; i < _scoutParentTransform.childCount; i++)
        {
            GameObject child = _scoutParentTransform.GetChild(i).gameObject;
            child.layer = 2;
        }

        _scoutStartPos = _scoutTransform.position;
        _scoutStartRotZ = _scoutTransform.rotation.eulerAngles.z;
        print(_scoutTransform.eulerAngles);

        List<Vector2> points = PoissonDiscSampling.GeneratePoints(_radius, _regionSize, _rejectionSamples);
        _flowerPositions = points.Take(_numberOfFlowers).ToList();
        GenerateObject(_flowerPrefab, _flowerPositions);
        _dogPositions = points.Skip(_numberOfFlowers).Take(_numberOfDogs).ToList();
        GenerateObject(_dogPrefab, _dogPositions);
        _rockPositions = points.Skip(_numberOfFlowers + _numberOfDogs).Take(_numberOfRocks).ToList();
        GenerateObject(_rockPrefab, _rockPositions);

        ResetEnv();
        NextTarget(false);
    }

    private void ResetEnv()
    {
        _scoutParentTransform.position = _scoutStartPos;
        _scoutParentTransform.rotation = Quaternion.Euler(0, _scoutStartRotZ, 0);
    }

    private void NextTarget(bool isBaseStation)
    {
        if (!isBaseStation && _currentTargetIdx < _flowerPositions.Count)
        {
            _scoutStartPos = _scoutTransform.position;
            _scoutStartRotZ = _scoutTransform.rotation.eulerAngles.z;
            _currentTargetIdx++;
            _targetTransform.localPosition = new Vector3(_flowerPositions[_currentTargetIdx].x + 0.7f, 0f, _flowerPositions[_currentTargetIdx].y);
            _agent.SetTargetPosition(new Vector2(_targetTransform.position.x, _targetTransform.position.z));
        }
        else if (isBaseStation)
        {
            _currentTargetIdx--;
            _targetTransform.localPosition = _baseStationPoint.transform.localPosition;
            _agent.SetTargetPosition(new Vector2(_targetTransform.position.x, _targetTransform.position.z));
        }
        else
        {
            print("All plants watered! Exiting.");
            EditorApplication.ExitPlaymode();
        }
    }

    private void GenerateObject(GameObject prefab, List<Vector2> points)
    {
        if (prefab != null)
        {
            foreach (Vector2 point in points)
            {
                Instantiate(prefab, new Vector3(point.x, 0, point.y), Quaternion.identity);
            }
        }
        else
        {
            Debug.LogError("FBX Prefab not assigned!");
        }
    }
}
