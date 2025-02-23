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
    [SerializeField] private List<Transform> _flowers;
    [SerializeField] private List<Transform> _dogs;
    [SerializeField] private List<Transform> _rocks;

    private DemoDrive _agent;

    private Vector3 _scoutStartPos;
    private float _scoutStartRotZ;

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

        ResetEnv();
        NextTarget(false);
    }

    void FixedUpdate()
    {
        if (_currentTargetIdx > 3)
        {
            _dogs[0].transform.position = Vector3.MoveTowards(_dogs[0].transform.position, _baseStationPoint.transform.position + new Vector3(1.5f, 0f, 1f), 0.2f * Time.fixedDeltaTime);
        }
    }

    private void ResetEnv()
    {
        _scoutParentTransform.position = _scoutStartPos;
        _scoutParentTransform.rotation = Quaternion.Euler(0, _scoutStartRotZ, 0);
    }

    private void NextTarget(bool isBaseStation)
    {
        if (!isBaseStation && _currentTargetIdx < _flowers.Count - 1)
        {
            _scoutStartPos = _scoutTransform.position;
            _scoutStartRotZ = _scoutTransform.rotation.eulerAngles.z;
            _currentTargetIdx++;
            _targetTransform.position = new Vector3(_flowers[_currentTargetIdx].position.x - 0.8f, 0f, _flowers[_currentTargetIdx].position.z);
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
            Debug.Log("All plants watered! Exiting.");
            EditorApplication.ExitPlaymode();
        }
    }
}
