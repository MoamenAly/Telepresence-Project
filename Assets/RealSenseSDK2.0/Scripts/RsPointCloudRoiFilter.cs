using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Runtime filter for RealSense point cloud meshes.
/// Keeps only points inside a world-space ROI and depth range.
/// Attach this to the same GameObject as RsPointCloudRenderer/MeshFilter.
/// </summary>
[DefaultExecutionOrder(1000)]
[RequireComponent(typeof(MeshFilter))]
public class RsPointCloudRoiFilter : MonoBehaviour
{
    [Header("Enable")]
    public bool filterEnabled = true;

    [Header("World ROI (meters)")]
    [Tooltip("Center of the keep volume in world space.")]
    public Vector3 worldCenter = new Vector3(0f, 1.1f, 1.0f);

    [Tooltip("Size of the keep volume in world space.")]
    public Vector3 worldSize = new Vector3(1.2f, 1.8f, 1.2f);

    [Header("Depth Gate (local camera z, meters)")]
    [Min(0.01f)] public float minDepthMeters = 0.35f;
    [Min(0.02f)] public float maxDepthMeters = 2.5f;

    [Header("Downsample")]
    [Tooltip("Keep every Nth point after filtering. 1 = keep all.")]
    [Min(1)] public int sampleStride = 2;

    [Header("Debug")]
    public bool drawRoiGizmo = true;

    private MeshFilter _meshFilter;
    private readonly List<Vector3> _vertices = new List<Vector3>(640 * 480);
    private readonly List<int> _keptIndices = new List<int>(640 * 480);
    private readonly int[] _empty = new int[0];

    private void Awake()
    {
        _meshFilter = GetComponent<MeshFilter>();
    }

    private void LateUpdate()
    {
        if (!filterEnabled || _meshFilter == null)
            return;

        var mesh = _meshFilter.sharedMesh;
        if (mesh == null)
            return;

        _vertices.Clear();
        mesh.GetVertices(_vertices);
        if (_vertices.Count == 0)
            return;

        _keptIndices.Clear();

        var half = worldSize * 0.5f;
        var min = worldCenter - half;
        var max = worldCenter + half;
        var stride = sampleStride < 1 ? 1 : sampleStride;

        for (int i = 0; i < _vertices.Count; i += stride)
        {
            var local = _vertices[i];
            var z = local.z;
            if (z < minDepthMeters || z > maxDepthMeters)
                continue;

            var world = transform.TransformPoint(local);
            if (world.x < min.x || world.x > max.x)
                continue;
            if (world.y < min.y || world.y > max.y)
                continue;
            if (world.z < min.z || world.z > max.z)
                continue;

            _keptIndices.Add(i);
        }

        if (_keptIndices.Count == 0)
        {
            mesh.SetIndices(_empty, MeshTopology.Points, 0, false);
            return;
        }

        mesh.SetIndices(_keptIndices, MeshTopology.Points, 0, false);
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawRoiGizmo)
            return;

        Gizmos.color = Color.green;
        Gizmos.DrawWireCube(worldCenter, worldSize);
    }
}
