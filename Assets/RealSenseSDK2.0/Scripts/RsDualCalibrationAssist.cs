using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Live calibration helper for dual RealSense point clouds.
/// - Nudges B->A extrinsics with keyboard while running.
/// - Displays overlap score and centroid distance for fast alignment.
/// </summary>
[DefaultExecutionOrder(920)]
public class RsDualCalibrationAssist : MonoBehaviour
{
    [Header("References")]
    public RsDualCameraPointCloudRig rig;
    public MeshFilter pointCloudMeshA;
    public MeshFilter pointCloudMeshB;
    public RsDualCameraExtrinsicsCalibration calibrationPersistence;

    [Header("Sampling")]
    [Min(1)] public int sampleStride = 6;
    [Min(0.01f)] public float voxelSize = 0.06f;
    [Min(16)] public int minPointsPerCloud = 200;
    [Min(0.01f)] public float minDepthMeters = 0.4f;
    [Min(0.02f)] public float maxDepthMeters = 2.5f;

    [Header("Nudge Base Step")]
    [Min(0.0001f)] public float positionStepMeters = 0.005f;
    [Min(0.01f)] public float rotationStepDegrees = 0.25f;
    public bool applyNudgesInPlayMode = true;

    [Header("Step Modifiers")]
    [Tooltip("Hold this for very fine movements.")]
    public KeyCode fineModifier = KeyCode.LeftShift;
    [Min(0.01f)] public float fineMultiplier = 0.2f;
    [Tooltip("Hold this for larger movements.")]
    public KeyCode coarseModifier = KeyCode.LeftControl;
    [Min(1f)] public float coarseMultiplier = 5f;

    [Header("Controls")]
    public KeyCode movePosX = KeyCode.D;
    public KeyCode moveNegX = KeyCode.A;
    public KeyCode movePosY = KeyCode.E;
    public KeyCode moveNegY = KeyCode.Q;
    public KeyCode movePosZ = KeyCode.W;
    public KeyCode moveNegZ = KeyCode.S;
    public KeyCode yawPos = KeyCode.RightArrow;
    public KeyCode yawNeg = KeyCode.LeftArrow;
    public KeyCode pitchPos = KeyCode.UpArrow;
    public KeyCode pitchNeg = KeyCode.DownArrow;
    public KeyCode rollPos = KeyCode.PageUp;
    public KeyCode rollNeg = KeyCode.PageDown;
    public KeyCode saveKey = KeyCode.F5;

    [Header("Debug")]
    public bool showOverlay = true;

    private readonly List<Vector3> _vertsA = new List<Vector3>(640 * 480);
    private readonly List<Vector3> _vertsB = new List<Vector3>(640 * 480);
    private readonly HashSet<int> _voxA = new HashSet<int>();
    private readonly HashSet<int> _voxB = new HashSet<int>();

    private float _iou;
    private float _centroidDistance;
    private int _countA;
    private int _countB;
    private int _overlapCount;
    private int _unionCount;
    private bool _hasMetrics;

    private void Update()
    {
        if (rig == null)
            return;

        HandleNudges();
        ComputeMetrics();
    }

    private void HandleNudges()
    {
        if (!applyNudgesInPlayMode || !Application.isPlaying)
            return;

        bool fine = Input.GetKey(fineModifier);
        bool coarse = Input.GetKey(coarseModifier);
        float multiplier = 1f;
        if (fine)
            multiplier *= fineMultiplier;
        if (coarse)
            multiplier *= coarseMultiplier;

        float posStep = positionStepMeters * multiplier;
        float rotStep = rotationStepDegrees * multiplier;

        bool changed = false;
        Vector3 pos = rig.positionBInA;
        Vector3 rot = rig.rotationBInAEuler;

        if (Input.GetKeyDown(movePosX)) { pos.x += posStep; changed = true; }
        if (Input.GetKeyDown(moveNegX)) { pos.x -= posStep; changed = true; }
        if (Input.GetKeyDown(movePosY)) { pos.y += posStep; changed = true; }
        if (Input.GetKeyDown(moveNegY)) { pos.y -= posStep; changed = true; }
        if (Input.GetKeyDown(movePosZ)) { pos.z += posStep; changed = true; }
        if (Input.GetKeyDown(moveNegZ)) { pos.z -= posStep; changed = true; }

        if (Input.GetKeyDown(yawPos)) { rot.y += rotStep; changed = true; }
        if (Input.GetKeyDown(yawNeg)) { rot.y -= rotStep; changed = true; }
        if (Input.GetKeyDown(pitchPos)) { rot.x += rotStep; changed = true; }
        if (Input.GetKeyDown(pitchNeg)) { rot.x -= rotStep; changed = true; }
        if (Input.GetKeyDown(rollPos)) { rot.z += rotStep; changed = true; }
        if (Input.GetKeyDown(rollNeg)) { rot.z -= rotStep; changed = true; }

        if (changed)
        {
            rig.positionBInA = pos;
            rig.rotationBInAEuler = rot;
            rig.ApplyTransforms();
        }

        if (Input.GetKeyDown(saveKey) && calibrationPersistence != null)
            calibrationPersistence.SaveToJson();
    }

    private void ComputeMetrics()
    {
        if (pointCloudMeshA == null || pointCloudMeshB == null)
        {
            _hasMetrics = false;
            return;
        }

        var meshA = pointCloudMeshA.sharedMesh;
        var meshB = pointCloudMeshB.sharedMesh;
        if (meshA == null || meshB == null)
        {
            _hasMetrics = false;
            return;
        }

        _vertsA.Clear();
        _vertsB.Clear();
        meshA.GetVertices(_vertsA);
        meshB.GetVertices(_vertsB);

        _voxA.Clear();
        _voxB.Clear();

        int stride = sampleStride < 1 ? 1 : sampleStride;
        float invVoxel = 1f / Mathf.Max(0.01f, voxelSize);
        Vector3 sumA = Vector3.zero;
        Vector3 sumB = Vector3.zero;
        _countA = 0;
        _countB = 0;

        for (int i = 0; i < _vertsA.Count; i += stride)
        {
            Vector3 local = _vertsA[i];
            if (local.z < minDepthMeters || local.z > maxDepthMeters)
                continue;

            Vector3 world = pointCloudMeshA.transform.TransformPoint(local);
            _voxA.Add(PackVoxel(world, invVoxel));
            sumA += world;
            _countA++;
        }

        for (int i = 0; i < _vertsB.Count; i += stride)
        {
            Vector3 local = _vertsB[i];
            if (local.z < minDepthMeters || local.z > maxDepthMeters)
                continue;

            Vector3 world = pointCloudMeshB.transform.TransformPoint(local);
            _voxB.Add(PackVoxel(world, invVoxel));
            sumB += world;
            _countB++;
        }

        if (_countA < minPointsPerCloud || _countB < minPointsPerCloud)
        {
            _hasMetrics = false;
            return;
        }

        _overlapCount = 0;
        foreach (int key in _voxA)
        {
            if (_voxB.Contains(key))
                _overlapCount++;
        }

        _unionCount = _voxA.Count + _voxB.Count - _overlapCount;
        _iou = _unionCount > 0 ? (float)_overlapCount / _unionCount : 0f;

        Vector3 centroidA = sumA / Mathf.Max(1, _countA);
        Vector3 centroidB = sumB / Mathf.Max(1, _countB);
        _centroidDistance = Vector3.Distance(centroidA, centroidB);

        _hasMetrics = true;
    }

    private static int PackVoxel(Vector3 p, float invVoxel)
    {
        int x = Mathf.FloorToInt(p.x * invVoxel);
        int y = Mathf.FloorToInt(p.y * invVoxel);
        int z = Mathf.FloorToInt(p.z * invVoxel);

        unchecked
        {
            int h = 17;
            h = h * 31 + x;
            h = h * 31 + y;
            h = h * 31 + z;
            return h;
        }
    }

    private void OnGUI()
    {
        if (!showOverlay)
            return;

        string msg = "Dual Calibration Assist\n";
        msg += "W/S: +Z/-Z, A/D: -X/+X, Q/E: -Y/+Y\n";
        msg += "Arrows: Pitch/Yaw, PgUp/PgDn: Roll, F5: Save JSON\n";
        msg += "Hold LeftShift=fine, LeftCtrl=coarse\n";

        if (_hasMetrics)
        {
            msg += string.Format(
                "\nIoU: {0:0.000}  CentroidDist: {1:0.000}m\nA:{2} B:{3} overlap:{4} union:{5}",
                _iou,
                _centroidDistance,
                _countA,
                _countB,
                _overlapCount,
                _unionCount
            );
        }
        else
        {
            msg += "\nMetrics: waiting for enough points...";
        }

        msg += string.Format(
            "\nB->A Pos: ({0:0.000}, {1:0.000}, {2:0.000})  Rot: ({3:0.00}, {4:0.00}, {5:0.00})",
            rig != null ? rig.positionBInA.x : 0f,
            rig != null ? rig.positionBInA.y : 0f,
            rig != null ? rig.positionBInA.z : 0f,
            rig != null ? rig.rotationBInAEuler.x : 0f,
            rig != null ? rig.rotationBInAEuler.y : 0f,
            rig != null ? rig.rotationBInAEuler.z : 0f
        );

        GUI.color = new Color(0f, 0f, 0f, 0.75f);
        GUI.Box(new Rect(10f, 10f, 640f, 135f), GUIContent.none);
        GUI.color = Color.white;
        GUI.Label(new Rect(20f, 18f, 620f, 125f), msg);
    }
}
