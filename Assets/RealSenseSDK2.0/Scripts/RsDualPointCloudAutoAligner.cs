using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Automatic rigid alignment of camera B to camera A (Umeyama / Kabsch, scale = 1).
/// Updates <see cref="RsDualCameraPointCloudRig"/> extrinsics without manual Inspector edits.
/// Press F6 in Play mode, or enable auto-align. Subject must be visible in both depth views.
/// </summary>
[DefaultExecutionOrder(925)]
public class RsDualPointCloudAutoAligner : MonoBehaviour
{
    [Header("References")]
    public RsDualCameraPointCloudRig rig;
    public MeshFilter pointCloudMeshA;
    public MeshFilter pointCloudMeshB;
    public RsDualCameraExtrinsicsCalibration calibrationPersistence;

    [Tooltip("RsDevice_B transform (parent of PointCloud_B). Defaults to rig.cameraB.transform.")]
    public Transform deviceBTransform;

    [Header("Sampling")]
    [Min(1)] public int sampleStride = 8;
    [Min(0.01f)] public float minDepthMeters = 0.4f;
    [Min(0.02f)] public float maxDepthMeters = 2.5f;
    [Min(32)] public int maxPointsPerCloud = 1200;

    [Header("ICP")]
    [Min(1)] public int icpIterationsPerRun = 2;
    [Min(0.01f)] public float maxNeighborDistance = 0.3f;
    [Range(0f, 1f)] public float stepBlend = 0.45f;

    [Header("Auto")]
    public bool autoAlignInPlayMode;
    [Min(0.2f)] public float autoAlignIntervalSeconds = 0.75f;

    [Header("Input")]
    public KeyCode alignKey = KeyCode.F6;

    [Header("Debug")]
    public bool logToConsole = true;

    private readonly List<Vector3> _scratchVerts = new List<Vector3>(640 * 480);
    private readonly List<Vector3> _worldA = new List<Vector3>(2048);
    private readonly List<Vector3> _worldB = new List<Vector3>(2048);
    private readonly List<Vector3> _pairedA = new List<Vector3>(2048);
    private readonly List<Vector3> _pairedB = new List<Vector3>(2048);

    private float _lastAutoTime;

    private void Awake()
    {
        if (deviceBTransform == null && rig != null && rig.cameraB != null)
            deviceBTransform = rig.cameraB.transform;
    }

    private void Update()
    {
        if (!Application.isPlaying || rig == null)
            return;

        if (Input.GetKeyDown(alignKey))
            RunAlignment();

        if (autoAlignInPlayMode && Time.unscaledTime - _lastAutoTime >= autoAlignIntervalSeconds)
        {
            _lastAutoTime = Time.unscaledTime;
            RunAlignment();
        }
    }

    [ContextMenu("Run Auto Alignment Now")]
    public void RunAlignment()
    {
        if (rig == null || pointCloudMeshA == null || pointCloudMeshB == null)
        {
            Debug.LogWarning("RsDualPointCloudAutoAligner: missing rig or mesh filters.");
            return;
        }

        if (deviceBTransform == null && rig.cameraB != null)
            deviceBTransform = rig.cameraB.transform;

        if (deviceBTransform == null)
        {
            Debug.LogWarning("RsDualPointCloudAutoAligner: assign deviceBTransform.");
            return;
        }

        for (int iter = 0; iter < icpIterationsPerRun; iter++)
        {
            rig.ApplyTransforms();

            if (!BuildWorldPoints(pointCloudMeshA, _worldA) || !BuildWorldPoints(pointCloudMeshB, _worldB))
            {
                if (logToConsole)
                    Debug.LogWarning("RsDualPointCloudAutoAligner: not enough depth points.");
                return;
            }

            BuildNearestPairs(_worldA, _worldB, maxNeighborDistance);
            if (_pairedA.Count < 32)
            {
                if (logToConsole)
                    Debug.LogWarning("RsDualPointCloudAutoAligner: too few correspondences.");
                return;
            }

            Vector3 meanA = Mean(_pairedA);
            Vector3 meanB = Mean(_pairedB);

            float[,] sigma = new float[3, 3];
            int n = _pairedA.Count;
            for (int i = 0; i < n; i++)
            {
                Vector3 a = _pairedA[i] - meanA;
                Vector3 b = _pairedB[i] - meanB;
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        sigma[r, c] += Component(a, r) * Component(b, c);
            }
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    sigma[r, c] /= n;

            UmeyamaRigidNoScale(sigma, meanA, meanB, out Quaternion rWorld, out Vector3 tWorld);

            Matrix4x4 tw = Matrix4x4.TRS(tWorld, rWorld, Vector3.one);
            Matrix4x4 tb = deviceBTransform.localToWorldMatrix;
            Matrix4x4 teOld = Matrix4x4.TRS(rig.positionBInA, Quaternion.Euler(rig.rotationBInAEuler), Vector3.one);
            Matrix4x4 teNew = tb.inverse * tw * tb * teOld;

            Vector3 targetPos = new Vector3(teNew.m03, teNew.m13, teNew.m23);
            Quaternion targetRot = RotationFromUpper3x3(teNew);

            rig.positionBInA = Vector3.Lerp(rig.positionBInA, targetPos, stepBlend);
            rig.rotationBInAEuler = Vector3.Lerp(rig.rotationBInAEuler, targetRot.eulerAngles, stepBlend);
            rig.ApplyTransforms();
        }

        if (logToConsole)
            Debug.LogFormat(
                "RsDualPointCloudAutoAligner: pairs={0} B->A pos=({1:F4},{2:F4},{3:F4})",
                _pairedA.Count, rig.positionBInA.x, rig.positionBInA.y, rig.positionBInA.z
            );
    }

    [ContextMenu("Save Extrinsics To JSON")]
    public void SaveExtrinsics()
    {
        if (calibrationPersistence != null)
            calibrationPersistence.SaveToJson();
    }

    /// <summary>
    /// sigma = (1/n) sum (a_i - meanA)(b_i - meanB)^T with a=A world, b=B world.
    /// R,t minimize || R b + t - a ||; t = meanA - R meanB (Umeyama, c=1).
    /// </summary>
    private static void UmeyamaRigidNoScale(float[,] sigma, Vector3 meanA, Vector3 meanB, out Quaternion r, out Vector3 t)
    {
        float[,] ata = new float[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                float s = 0f;
                for (int k = 0; k < 3; k++)
                    s += sigma[k, i] * sigma[k, j];
                ata[i, j] = s;
            }

        JacobiEigenSymmetric3(ata, out float[] evals, out float[,] v);

        float s0 = Mathf.Sqrt(Mathf.Max(evals[0], 1e-14f));
        float s1 = Mathf.Sqrt(Mathf.Max(evals[1], 1e-14f));
        float s2 = Mathf.Sqrt(Mathf.Max(evals[2], 1e-14f));

        float[,] vInvS = new float[3, 3];
        for (int i = 0; i < 3; i++)
        {
            vInvS[i, 0] = v[i, 0] / s0;
            vInvS[i, 1] = v[i, 1] / s1;
            vInvS[i, 2] = v[i, 2] / s2;
        }

        float[,] svT = Mul3(vInvS, Transpose3(v));
        float[,] u = Mul3(sigma, svT);

        float detU = Determinant3(u);
        float detV = Determinant3(v);
        if (detU * detV < 0f)
        {
            for (int i = 0; i < 3; i++)
                v[i, 2] = -v[i, 2];
            for (int i = 0; i < 3; i++)
            {
                vInvS[i, 0] = v[i, 0] / s0;
                vInvS[i, 1] = v[i, 1] / s1;
                vInvS[i, 2] = v[i, 2] / s2;
            }
            svT = Mul3(vInvS, Transpose3(v));
            u = Mul3(sigma, svT);
        }

        float[,] rMat = Mul3(u, Transpose3(v));
        r = QuaternionFromOrthonormal(rMat);
        t = meanA - r * meanB;
    }

    private static float[,] Mul3(float[,] a, float[,] b)
    {
        float[,] o = new float[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                float s = 0f;
                for (int k = 0; k < 3; k++)
                    s += a[i, k] * b[k, j];
                o[i, j] = s;
            }
        return o;
    }

    private static float[,] Transpose3(float[,] m)
    {
        float[,] t = new float[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                t[i, j] = m[j, i];
        return t;
    }

    private static float Determinant3(float[,] m)
    {
        return m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1])
             - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0])
             + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);
    }

    /// <summary>Jacobi eigen-decomposition for symmetric 3x3 (rows/cols 0..2).</summary>
    private static void JacobiEigenSymmetric3(float[,] a, out float[] evals, out float[,] evecs)
    {
        float[,] d = (float[,])a.Clone();
        evecs = new float[3, 3];
        for (int i = 0; i < 3; i++) evecs[i, i] = 1f;

        for (int sweep = 0; sweep < 32; sweep++)
        {
            float o01 = Mathf.Abs(d[0, 1]);
            float o02 = Mathf.Abs(d[0, 2]);
            float o12 = Mathf.Abs(d[1, 2]);
            if (o01 < 1e-10f && o02 < 1e-10f && o12 < 1e-10f)
                break;

            int p = 0, q = 1;
            float w = o01;
            if (o02 > w) { p = 0; q = 2; w = o02; }
            if (o12 > w) { p = 1; q = 2; }

            float app = d[p, p];
            float aqq = d[q, q];
            float apq = d[p, q];
            float phi = 0.5f * Mathf.Atan2(2f * apq, aqq - app);
            float c = Mathf.Cos(phi);
            float s = Mathf.Sin(phi);

            float[,] j = new float[3, 3];
            j[0, 0] = j[1, 1] = j[2, 2] = 1f;
            j[p, p] = c;
            j[q, q] = c;
            j[p, q] = -s;
            j[q, p] = s;

            d = Mul3(Mul3(Transpose3(j), d), j);
            evecs = Mul3(evecs, j);
        }

        evals = new float[3];
        evals[0] = d[0, 0];
        evals[1] = d[1, 1];
        evals[2] = d[2, 2];
    }

    private static Quaternion QuaternionFromOrthonormal(float[,] r)
    {
        float tr = r[0, 0] + r[1, 1] + r[2, 2];
        Quaternion q = Quaternion.identity;
        if (tr > 0f)
        {
            float s = Mathf.Sqrt(tr + 1f) * 2f;
            q.w = 0.25f * s;
            q.x = (r[2, 1] - r[1, 2]) / s;
            q.y = (r[0, 2] - r[2, 0]) / s;
            q.z = (r[1, 0] - r[0, 1]) / s;
        }
        else if (r[0, 0] > r[1, 1] && r[0, 0] > r[2, 2])
        {
            float s = Mathf.Sqrt(1f + r[0, 0] - r[1, 1] - r[2, 2]) * 2f;
            q.w = (r[2, 1] - r[1, 2]) / s;
            q.x = 0.25f * s;
            q.y = (r[0, 1] + r[1, 0]) / s;
            q.z = (r[0, 2] + r[2, 0]) / s;
        }
        else if (r[1, 1] > r[2, 2])
        {
            float s = Mathf.Sqrt(1f + r[1, 1] - r[0, 0] - r[2, 2]) * 2f;
            q.w = (r[0, 2] - r[2, 0]) / s;
            q.x = (r[0, 1] + r[1, 0]) / s;
            q.y = 0.25f * s;
            q.z = (r[1, 2] + r[2, 1]) / s;
        }
        else
        {
            float s = Mathf.Sqrt(1f + r[2, 2] - r[0, 0] - r[1, 1]) * 2f;
            q.w = (r[1, 0] - r[0, 1]) / s;
            q.x = (r[0, 2] + r[2, 0]) / s;
            q.y = (r[1, 2] + r[2, 1]) / s;
            q.z = 0.25f * s;
        }
        return q.normalized;
    }

    private static Quaternion RotationFromUpper3x3(Matrix4x4 m)
    {
        float[,] r =
        {
            { m.m00, m.m01, m.m02 },
            { m.m10, m.m11, m.m12 },
            { m.m20, m.m21, m.m22 }
        };
        Orthonormalize(ref r);
        return QuaternionFromOrthonormal(r);
    }

    private static void Orthonormalize(ref float[,] r)
    {
        Vector3 c0 = new Vector3(r[0, 0], r[1, 0], r[2, 0]);
        Vector3 c1 = new Vector3(r[0, 1], r[1, 1], r[2, 1]);
        Vector3 c2 = new Vector3(r[0, 2], r[1, 2], r[2, 2]);
        c0.Normalize();
        c1 = Vector3.ProjectOnPlane(c1, c0).normalized;
        c2 = Vector3.Cross(c0, c1);
        r[0, 0] = c0.x; r[1, 0] = c0.y; r[2, 0] = c0.z;
        r[0, 1] = c1.x; r[1, 1] = c1.y; r[2, 1] = c1.z;
        r[0, 2] = c2.x; r[1, 2] = c2.y; r[2, 2] = c2.z;
    }

    private bool BuildWorldPoints(MeshFilter mf, List<Vector3> worldOut)
    {
        worldOut.Clear();
        _scratchVerts.Clear();
        if (mf.sharedMesh == null)
            return false;

        mf.sharedMesh.GetVertices(_scratchVerts);
        int stride = sampleStride < 1 ? 1 : sampleStride;
        int added = 0;
        for (int i = 0; i < _scratchVerts.Count && added < maxPointsPerCloud; i += stride)
        {
            Vector3 local = _scratchVerts[i];
            if (local.z < minDepthMeters || local.z > maxDepthMeters)
                continue;
            worldOut.Add(mf.transform.TransformPoint(local));
            added++;
        }
        return worldOut.Count >= 32;
    }

    private static Vector3 Mean(List<Vector3> pts)
    {
        Vector3 s = Vector3.zero;
        for (int i = 0; i < pts.Count; i++)
            s += pts[i];
        return s / Mathf.Max(1, pts.Count);
    }

    private void BuildNearestPairs(List<Vector3> cloudA, List<Vector3> cloudB, float maxDist)
    {
        _pairedA.Clear();
        _pairedB.Clear();
        float maxSq = maxDist * maxDist;

        for (int i = 0; i < cloudB.Count; i++)
        {
            Vector3 pb = cloudB[i];
            int best = -1;
            float bestSq = float.MaxValue;
            for (int j = 0; j < cloudA.Count; j++)
            {
                float d = (cloudA[j] - pb).sqrMagnitude;
                if (d < bestSq)
                {
                    bestSq = d;
                    best = j;
                }
            }
            if (best >= 0 && bestSq <= maxSq)
            {
                _pairedB.Add(pb);
                _pairedA.Add(cloudA[best]);
            }
        }
    }

    private static float Component(Vector3 v, int i)
    {
        if (i == 0) return v.x;
        if (i == 1) return v.y;
        return v.z;
    }
}
