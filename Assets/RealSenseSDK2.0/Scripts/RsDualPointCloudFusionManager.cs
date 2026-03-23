using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Shared fusion controller for dual RealSense point clouds.
/// - Computes one dominant person cluster center from camera A + B clouds.
/// - Applies the same world ROI center to both filters.
/// - Optionally keeps camera B transform in sync from dual-camera rig.
/// </summary>
[DefaultExecutionOrder(900)]
public class RsDualPointCloudFusionManager : MonoBehaviour
{
    [Header("Cloud Inputs")]
    public MeshFilter pointCloudMeshA;
    public MeshFilter pointCloudMeshB;

    [Header("ROI Targets")]
    public RsPointCloudRoiFilter roiFilterA;
    public RsPointCloudRoiFilter roiFilterB;

    [Header("Optional Rig Sync")]
    public RsDualCameraPointCloudRig rig;
    public bool applyRigTransformEachFrame = true;

    [Header("Enable")]
    public bool fusionEnabled = true;

    [Header("Detection Gate (local camera space)")]
    [Min(0.01f)] public float minDepthMeters = 0.45f;
    [Min(0.02f)] public float maxDepthMeters = 2.5f;
    public Vector2 localYRange = new Vector2(0.1f, 2.2f);
    [Min(1)] public int sampleStride = 4;

    [Header("Cluster Selection")]
    [Min(0.02f)] public float xzCellSize = 0.12f;
    [Min(0.05f)] public float clusterRadiusMeters = 0.45f;
    [Min(1)] public int minClusterPoints = 180;

    [Header("Center Smoothing")]
    public float yOffset = 0f;
    [Min(0.01f)] public float smoothing = 8f;
    [Min(0.01f)] public float maxMoveSpeed = 2.0f;

    [Header("ROI Consistency")]
    [Tooltip("Use ROI-A size for both A and B.")]
    public bool forceSameRoiSize = true;
    [Tooltip("Use ROI-A depth gate for both A and B.")]
    public bool forceSameRoiDepth = true;

    [Header("Debug")]
    public bool drawDetectedCenter = true;

    private readonly List<Vector3> _vertsA = new List<Vector3>(640 * 480);
    private readonly List<Vector3> _vertsB = new List<Vector3>(640 * 480);
    private readonly Dictionary<int, int> _cellCounts = new Dictionary<int, int>(2048);
    private Vector3 _lastDetectedCenter;
    private bool _hasDetectedCenter;

    private void LateUpdate()
    {
        if (!fusionEnabled)
            return;
        if (pointCloudMeshA == null || pointCloudMeshB == null || roiFilterA == null || roiFilterB == null)
            return;

        if (applyRigTransformEachFrame && rig != null)
            rig.ApplyTransforms();

        if (forceSameRoiSize)
            roiFilterB.worldSize = roiFilterA.worldSize;

        if (forceSameRoiDepth)
        {
            roiFilterB.minDepthMeters = roiFilterA.minDepthMeters;
            roiFilterB.maxDepthMeters = roiFilterA.maxDepthMeters;
        }

        var meshA = pointCloudMeshA.sharedMesh;
        var meshB = pointCloudMeshB.sharedMesh;
        if (meshA == null || meshB == null)
            return;

        _vertsA.Clear();
        _vertsB.Clear();
        meshA.GetVertices(_vertsA);
        meshB.GetVertices(_vertsB);
        if (_vertsA.Count == 0 && _vertsB.Count == 0)
            return;

        int stride = sampleStride < 1 ? 1 : sampleStride;
        float cellSize = xzCellSize < 0.02f ? 0.02f : xzCellSize;

        _cellCounts.Clear();
        int bestKey = 0;
        int bestCount = 0;

        AccumulateCells(_vertsA, pointCloudMeshA.transform, stride, cellSize, ref bestKey, ref bestCount);
        AccumulateCells(_vertsB, pointCloudMeshB.transform, stride, cellSize, ref bestKey, ref bestCount);
        if (bestCount == 0)
            return;

        UnpackCell(bestKey, out int bestCx, out int bestCz);
        Vector3 bestCellCenter = new Vector3(
            (bestCx + 0.5f) * cellSize,
            0f,
            (bestCz + 0.5f) * cellSize
        );

        float radiusSqr = clusterRadiusMeters * clusterRadiusMeters;
        Vector3 sum = Vector3.zero;
        int clusterCount = 0;

        AccumulateCluster(_vertsA, pointCloudMeshA.transform, stride, bestCellCenter, radiusSqr, ref sum, ref clusterCount);
        AccumulateCluster(_vertsB, pointCloudMeshB.transform, stride, bestCellCenter, radiusSqr, ref sum, ref clusterCount);

        if (clusterCount < minClusterPoints)
            return;

        Vector3 detectedCenter = sum / clusterCount;
        detectedCenter.y += yOffset;
        _lastDetectedCenter = detectedCenter;
        _hasDetectedCenter = true;

        Vector3 current = roiFilterA.worldCenter;
        float t = 1f - Mathf.Exp(-smoothing * Time.deltaTime);
        Vector3 smoothed = Vector3.Lerp(current, detectedCenter, t);
        Vector3 delta = smoothed - current;
        float maxStep = maxMoveSpeed * Time.deltaTime;
        if (delta.magnitude > maxStep)
            smoothed = current + delta.normalized * maxStep;

        roiFilterA.worldCenter = smoothed;
        roiFilterB.worldCenter = smoothed;
    }

    [ContextMenu("Copy A ROI Params To B")]
    public void CopyRoiASettingsToB()
    {
        if (roiFilterA == null || roiFilterB == null)
            return;

        roiFilterB.worldSize = roiFilterA.worldSize;
        roiFilterB.minDepthMeters = roiFilterA.minDepthMeters;
        roiFilterB.maxDepthMeters = roiFilterA.maxDepthMeters;
    }

    private void AccumulateCells(
        List<Vector3> vertices,
        Transform cloudTransform,
        int stride,
        float cellSize,
        ref int bestKey,
        ref int bestCount)
    {
        for (int i = 0; i < vertices.Count; i += stride)
        {
            Vector3 local = vertices[i];
            if (local.z < minDepthMeters || local.z > maxDepthMeters)
                continue;
            if (local.y < localYRange.x || local.y > localYRange.y)
                continue;

            Vector3 world = cloudTransform.TransformPoint(local);
            int cx = Mathf.FloorToInt(world.x / cellSize);
            int cz = Mathf.FloorToInt(world.z / cellSize);
            int key = PackCell(cx, cz);

            int count;
            if (_cellCounts.TryGetValue(key, out count))
                count++;
            else
                count = 1;

            _cellCounts[key] = count;
            if (count > bestCount)
            {
                bestCount = count;
                bestKey = key;
            }
        }
    }

    private void AccumulateCluster(
        List<Vector3> vertices,
        Transform cloudTransform,
        int stride,
        Vector3 bestCellCenter,
        float radiusSqr,
        ref Vector3 sum,
        ref int clusterCount)
    {
        for (int i = 0; i < vertices.Count; i += stride)
        {
            Vector3 local = vertices[i];
            if (local.z < minDepthMeters || local.z > maxDepthMeters)
                continue;
            if (local.y < localYRange.x || local.y > localYRange.y)
                continue;

            Vector3 world = cloudTransform.TransformPoint(local);
            Vector2 dxz = new Vector2(world.x - bestCellCenter.x, world.z - bestCellCenter.z);
            if (dxz.sqrMagnitude > radiusSqr)
                continue;

            sum += world;
            clusterCount++;
        }
    }

    private static int PackCell(int x, int z)
    {
        return (x << 16) ^ (z & 0xFFFF);
    }

    private static void UnpackCell(int key, out int x, out int z)
    {
        x = key >> 16;
        z = (short)(key & 0xFFFF);
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawDetectedCenter || !_hasDetectedCenter)
            return;

        Gizmos.color = Color.cyan;
        Gizmos.DrawSphere(_lastDetectedCenter, 0.06f);
    }
}
