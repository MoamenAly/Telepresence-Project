using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

[DefaultExecutionOrder(200)]
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class RsDualPointCloudFusionManager : MonoBehaviour
{
    [Header("Source Clouds")]
    public MeshFilter meshFilterA;
    public MeshFilter meshFilterB;

    [Header("Extrinsics")]
    public RsDualCameraExtrinsicsCalibration calibration;

    [Header("Fusion Settings")]
    [Tooltip("Hide the individual PointCloud_A / B renderers and show only the fused mesh.")]
    public bool hideSeparateClouds = true;

    [Tooltip("Voxel grid cell size (meters) for merging overlapping points. " +
             "Points from both clouds in the same voxel are averaged. 0 = no merging.")]
    public float voxelSize = 0.005f;

    [Tooltip("Maximum number of points in the merged cloud.")]
    public int maxMergedPoints = 307200;

    private Mesh fusedMesh;
    private Vector3[] mergedVerts;
    private Vector2[] mergedGridUVs;
    private Vector2[] mergedCloudIds;
    private int[] mergedIndices;

    private MeshRenderer rendererA;
    private MeshRenderer rendererB;
    private MeshRenderer fusedRenderer;
    private Material fusedMaterial;

    private struct VoxelAccum
    {
        public float sumX, sumY, sumZ;
        public int totalCount;
        public Vector2 uvA;
        public Vector2 uvB;
        public bool hasA, hasB;
    }

    private Dictionary<long, VoxelAccum> voxelGrid;

    void Start()
    {
        fusedMesh = new Mesh { indexFormat = IndexFormat.UInt32 };
        fusedMesh.MarkDynamic();
        GetComponent<MeshFilter>().sharedMesh = fusedMesh;

        fusedRenderer = GetComponent<MeshRenderer>();
        if (meshFilterA != null) rendererA = meshFilterA.GetComponent<MeshRenderer>();
        if (meshFilterB != null) rendererB = meshFilterB.GetComponent<MeshRenderer>();

        mergedVerts = new Vector3[maxMergedPoints];
        mergedGridUVs = new Vector2[maxMergedPoints];
        mergedCloudIds = new Vector2[maxMergedPoints];
        mergedIndices = new int[maxMergedPoints];
        for (int i = 0; i < maxMergedPoints; i++)
            mergedIndices[i] = i;

        voxelGrid = new Dictionary<long, VoxelAccum>(maxMergedPoints);

        CreateFusedMaterial();
    }

    private void CreateFusedMaterial()
    {
        var shader = Shader.Find("Custom/PointCloudFused");
        if (shader == null)
        {
            Debug.LogError("[Fusion] 'Custom/PointCloudFused' shader not found. " +
                           "Make sure PointCloudFused.shader exists in Assets/RealSenseSDK2.0/Shaders/");
            enabled = false;
            return;
        }

        fusedMaterial = new Material(shader);
        fusedMaterial.SetFloat("_PointSize", 10.0f);
        fusedMaterial.EnableKeyword("USE_DISTANCE");
        fusedMaterial.SetColor("_Color", Color.white);
        fusedRenderer.sharedMaterial = fusedMaterial;
    }

    void LateUpdate()
    {
        if (meshFilterA == null || meshFilterB == null) return;
        if (fusedMaterial == null) return;

        var meshA = meshFilterA.sharedMesh ?? meshFilterA.mesh;
        var meshB = meshFilterB.sharedMesh ?? meshFilterB.mesh;
        if (meshA == null || meshB == null) return;
        if (meshA.vertexCount == 0 && meshB.vertexCount == 0) return;

        UpdateShaderTextures();

        var vertsA = meshA.vertices;
        var vertsB = meshB.vertices;
        var gridUVsA = meshA.uv;
        var gridUVsB = meshB.uv;

        Matrix4x4 transformB = Matrix4x4.TRS(
            calibration != null ? calibration.translationOffset : Vector3.zero,
            Quaternion.Euler(calibration != null ? calibration.rotationEulerOffset : Vector3.zero),
            Vector3.one
        );

        int count = MergeVerticesVoxelAvg(vertsA, gridUVsA, vertsB, gridUVsB, transformB);
        UpdateFusedMesh(count);
        UpdateCloudVisibility();
    }

    private void UpdateShaderTextures()
    {
        if (rendererA != null && rendererA.sharedMaterial != null)
        {
            var matA = rendererA.sharedMaterial;
            fusedMaterial.SetTexture("_UVMapA", matA.GetTexture("_UVMap"));
            fusedMaterial.SetTexture("_MainTexA", matA.GetTexture("_MainTex"));
        }

        if (rendererB != null && rendererB.sharedMaterial != null)
        {
            var matB = rendererB.sharedMaterial;
            fusedMaterial.SetTexture("_UVMapB", matB.GetTexture("_UVMap"));
            fusedMaterial.SetTexture("_MainTexB", matB.GetTexture("_MainTex"));
        }
    }

    private int MergeVerticesVoxelAvg(Vector3[] vertsA, Vector2[] uvsA,
                                      Vector3[] vertsB, Vector2[] uvsB,
                                      Matrix4x4 transformB)
    {
        bool useGrid = voxelSize > 0;

        if (!useGrid)
            return MergeVerticesSimple(vertsA, uvsA, vertsB, uvsB, transformB);

        float invCell = 1f / voxelSize;
        voxelGrid.Clear();

        // Pass 1: Accumulate Cloud A points into voxel grid
        for (int i = 0; i < vertsA.Length; i++)
        {
            var v = vertsA[i];
            if (v.x == 0 && v.y == 0 && v.z == 0) continue;

            long key = SpatialKey(v, invCell);
            Vector2 uv = (uvsA != null && i < uvsA.Length) ? uvsA[i] : Vector2.zero;

            if (voxelGrid.TryGetValue(key, out var acc))
            {
                acc.sumX += v.x;
                acc.sumY += v.y;
                acc.sumZ += v.z;
                acc.totalCount++;
                acc.uvA = uv;
                acc.hasA = true;
                voxelGrid[key] = acc;
            }
            else
            {
                voxelGrid[key] = new VoxelAccum
                {
                    sumX = v.x, sumY = v.y, sumZ = v.z,
                    totalCount = 1,
                    uvA = uv,
                    hasA = true
                };
            }
        }

        // Pass 2: Accumulate Cloud B points (transformed) into the same grid
        for (int i = 0; i < vertsB.Length; i++)
        {
            var v = vertsB[i];
            if (v.x == 0 && v.y == 0 && v.z == 0) continue;

            v = transformB.MultiplyPoint3x4(v);

            long key = SpatialKey(v, invCell);
            Vector2 uv = (uvsB != null && i < uvsB.Length) ? uvsB[i] : Vector2.zero;

            if (voxelGrid.TryGetValue(key, out var acc))
            {
                acc.sumX += v.x;
                acc.sumY += v.y;
                acc.sumZ += v.z;
                acc.totalCount++;
                acc.uvB = uv;
                acc.hasB = true;
                voxelGrid[key] = acc;
            }
            else
            {
                voxelGrid[key] = new VoxelAccum
                {
                    sumX = v.x, sumY = v.y, sumZ = v.z,
                    totalCount = 1,
                    uvB = uv,
                    hasB = true
                };
            }
        }

        // Pass 3: Emit one averaged point per occupied voxel
        int idx = 0;
        foreach (var kvp in voxelGrid)
        {
            if (idx >= maxMergedPoints) break;

            var acc = kvp.Value;
            float inv = 1f / acc.totalCount;
            mergedVerts[idx] = new Vector3(acc.sumX * inv, acc.sumY * inv, acc.sumZ * inv);

            if (acc.hasA && !acc.hasB)
            {
                mergedGridUVs[idx] = acc.uvA;
                mergedCloudIds[idx] = new Vector2(0, 0);
            }
            else if (acc.hasB && !acc.hasA)
            {
                mergedGridUVs[idx] = acc.uvB;
                mergedCloudIds[idx] = new Vector2(1, 0);
            }
            else
            {
                // Overlap: both clouds contributed -- use A as reference
                mergedGridUVs[idx] = acc.uvA;
                mergedCloudIds[idx] = new Vector2(0, 0);
            }

            idx++;
        }

        // Zero remaining slots
        for (int i = idx; i < maxMergedPoints; i++)
            mergedVerts[i] = Vector3.zero;

        return idx;
    }

    /// <summary>
    /// Simple concatenation when voxel merging is disabled (voxelSize = 0).
    /// </summary>
    private int MergeVerticesSimple(Vector3[] vertsA, Vector2[] uvsA,
                                    Vector3[] vertsB, Vector2[] uvsB,
                                    Matrix4x4 transformB)
    {
        int idx = 0;

        for (int i = 0; i < vertsA.Length && idx < maxMergedPoints; i++)
        {
            var v = vertsA[i];
            if (v.x == 0 && v.y == 0 && v.z == 0) continue;
            mergedVerts[idx] = v;
            mergedGridUVs[idx] = (uvsA != null && i < uvsA.Length) ? uvsA[i] : Vector2.zero;
            mergedCloudIds[idx] = new Vector2(0, 0);
            idx++;
        }

        for (int i = 0; i < vertsB.Length && idx < maxMergedPoints; i++)
        {
            var v = vertsB[i];
            if (v.x == 0 && v.y == 0 && v.z == 0) continue;
            v = transformB.MultiplyPoint3x4(v);
            mergedVerts[idx] = v;
            mergedGridUVs[idx] = (uvsB != null && i < uvsB.Length) ? uvsB[i] : Vector2.zero;
            mergedCloudIds[idx] = new Vector2(1, 0);
            idx++;
        }

        for (int i = idx; i < maxMergedPoints; i++)
            mergedVerts[i] = Vector3.zero;

        return idx;
    }

    private static long SpatialKey(Vector3 v, float invCell)
    {
        int x = Mathf.FloorToInt(v.x * invCell);
        int y = Mathf.FloorToInt(v.y * invCell);
        int z = Mathf.FloorToInt(v.z * invCell);
        return ((long)(x & 0x1FFFFF) << 42) | ((long)(y & 0x1FFFFF) << 21) | (long)(z & 0x1FFFFF);
    }

    private void UpdateFusedMesh(int activeCount)
    {
        if (fusedMesh.vertexCount != maxMergedPoints)
        {
            fusedMesh.Clear();
            fusedMesh.vertices = mergedVerts;
            fusedMesh.uv = mergedGridUVs;
            fusedMesh.uv2 = mergedCloudIds;
            fusedMesh.SetIndices(mergedIndices, MeshTopology.Points, 0, false);
            fusedMesh.bounds = new Bounds(Vector3.zero, Vector3.one * 10f);
        }
        else
        {
            fusedMesh.vertices = mergedVerts;
            fusedMesh.uv = mergedGridUVs;
            fusedMesh.uv2 = mergedCloudIds;
        }

        fusedMesh.UploadMeshData(false);
    }

    private void UpdateCloudVisibility()
    {
        if (rendererA != null) rendererA.enabled = !hideSeparateClouds;
        if (rendererB != null) rendererB.enabled = !hideSeparateClouds;
    }

    void OnDestroy()
    {
        if (fusedMesh != null) Destroy(fusedMesh);
        if (fusedMaterial != null) Destroy(fusedMaterial);
    }
}
