using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using UnityEngine;
using Debug = UnityEngine.Debug;

public class RsDualPointCloudAutoAligner : MonoBehaviour
{
    [Header("ICP Parameters")]
    public int iterations = 30;
    public float maxNeighborDistance = 0.15f;
    [Range(0.1f, 1f)] public float convergenceRate = 0.5f;
    public int targetSampleCount = 1000;

    [Header("Safety")]
    [Tooltip("Only apply ICP result if mean error improves by at least this fraction (e.g. 0.05 = 5%).")]
    public float minImprovementRatio = 0.05f;

    [Header("Persistence")]
    public bool saveExtrinsicsAfterAlign = false;

    private RsDualCameraPointCloudRig rig;
    private RsDualCameraExtrinsicsCalibration calibration;

    private volatile bool isAligning;
    private volatile bool resultReady;

    private Quaternion resultQuaternion;
    private Vector3 resultTranslation;
    private float resultMeanError;
    private float beforeMeanError;
    private int resultPairCount;
    private volatile bool resultAccepted;

    // Backup for F7 revert
    private Vector3 backupTranslation;
    private Vector3 backupRotationEuler;
    private bool hasBackup;

    void Awake()
    {
        rig = GetComponent<RsDualCameraPointCloudRig>();
        calibration = GetComponent<RsDualCameraExtrinsicsCalibration>();

        if (rig == null || calibration == null)
        {
            Debug.LogError("[AutoAlign] Requires RsDualCameraPointCloudRig and RsDualCameraExtrinsicsCalibration.", this);
            enabled = false;
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.F6) && !isAligning)
            StartAlignment();

        // F7 = revert to backup
        if (Input.GetKeyDown(KeyCode.F7) && hasBackup)
        {
            calibration.translationOffset = backupTranslation;
            calibration.rotationEulerOffset = backupRotationEuler;
            calibration.ApplyExtrinsics();
            Debug.Log("[AutoAlign] F7: Reverted to pre-F6 calibration.");
        }

        if (resultReady)
        {
            if (resultAccepted)
            {
                Vector3 euler = resultQuaternion.eulerAngles;
                if (euler.x > 180f) euler.x -= 360f;
                if (euler.y > 180f) euler.y -= 360f;
                if (euler.z > 180f) euler.z -= 360f;

                calibration.translationOffset = resultTranslation;
                calibration.rotationEulerOffset = euler;
                calibration.ApplyExtrinsics();

                Debug.Log($"[AutoAlign] Applied! Before={beforeMeanError:F4}m -> After={resultMeanError:F4}m " +
                          $"({resultPairCount} pairs) | " +
                          $"T=({resultTranslation.x:F4}, {resultTranslation.y:F4}, {resultTranslation.z:F4}) | " +
                          $"R=({euler.x:F2}, {euler.y:F2}, {euler.z:F2}) | Press F7 to undo");

                if (saveExtrinsicsAfterAlign)
                    calibration.SaveExtrinsics();
            }
            else
            {
                Debug.LogWarning($"[AutoAlign] REJECTED: ICP did not improve alignment. " +
                                 $"Before={beforeMeanError:F4}m, After={resultMeanError:F4}m. " +
                                 $"Calibration unchanged. Try manual WASDQE adjustment first.");
            }

            resultReady = false;
            isAligning = false;
        }
    }

    private void StartAlignment()
    {
        if (rig.pointCloudA == null || rig.pointCloudB == null)
        {
            Debug.LogWarning("[AutoAlign] Point cloud references not set.");
            return;
        }

        var mfA = rig.pointCloudA.GetComponent<MeshFilter>();
        var mfB = rig.pointCloudB.GetComponent<MeshFilter>();
        if (mfA == null || mfB == null || mfA.sharedMesh == null || mfB.sharedMesh == null)
        {
            Debug.LogWarning("[AutoAlign] Meshes not ready yet.");
            return;
        }

        var vertsA = mfA.sharedMesh.vertices;
        var vertsB = mfB.sharedMesh.vertices;

        int strideA = Mathf.Max(1, vertsA.Length / (targetSampleCount * 3));
        int strideB = Mathf.Max(1, vertsB.Length / (targetSampleCount * 3));
        var sampledA = SampleValidVertices(vertsA, strideA, targetSampleCount);
        var sampledB = SampleValidVertices(vertsB, strideB, targetSampleCount);

        if (sampledA.Length < 30 || sampledB.Length < 30)
        {
            Debug.LogWarning($"[AutoAlign] Not enough valid vertices: A={sampledA.Length}, B={sampledB.Length}. Need 30+.");
            return;
        }

        // Backup current calibration before ICP
        backupTranslation = calibration.translationOffset;
        backupRotationEuler = calibration.rotationEulerOffset;
        hasBackup = true;

        Quaternion startRot = Quaternion.Euler(calibration.rotationEulerOffset);
        Vector3 startTrans = calibration.translationOffset;
        int iters = iterations;
        float maxDist = maxNeighborDistance;
        float rate = convergenceRate;
        float minImprove = minImprovementRatio;

        isAligning = true;
        resultReady = false;

        Debug.Log($"[AutoAlign] F6: Starting ICP ({sampledA.Length}/{sampledB.Length} pts, {iters} iters, maxDist={maxDist}m)");

        ThreadPool.QueueUserWorkItem(_ =>
        {
            try
            {
                RunICP(sampledA, sampledB, startTrans, startRot, iters, maxDist, rate, minImprove);
            }
            catch (Exception e)
            {
                Debug.LogError("[AutoAlign] ICP error: " + e.Message + "\n" + e.StackTrace);
                isAligning = false;
            }
        });
    }

    private Vector3[] SampleValidVertices(Vector3[] verts, int stride, int maxCount)
    {
        var list = new List<Vector3>(maxCount);
        for (int i = 0; i < verts.Length && list.Count < maxCount; i += stride)
        {
            var v = verts[i];
            if (v.x != 0 || v.y != 0 || v.z != 0)
                list.Add(v);
        }
        return list.ToArray();
    }

    private void RunICP(Vector3[] srcA, Vector3[] srcB, Vector3 startTrans, Quaternion startRot,
                        int iters, float maxDist, float rate, float minImprove)
    {
        var sw = Stopwatch.StartNew();

        // Compute alignment error BEFORE ICP
        float errorBefore = ComputeAlignmentError(srcA, srcB, startTrans, startRot, maxDist);
        Debug.Log($"[AutoAlign] Error before ICP: {errorBefore:F4}m");

        Quaternion currentRot = startRot;
        Vector3 currentTrans = startTrans;
        var transformedB = new Vector3[srcB.Length];
        int[] matchA = new int[srcB.Length];
        int[] matchB = new int[srcB.Length];
        float lastError = errorBefore;
        int lastPairCount = 0;

        for (int iter = 0; iter < iters; iter++)
        {
            float adaptMaxDist = maxDist;
            float adaptMaxDistSq = adaptMaxDist * adaptMaxDist;

            for (int i = 0; i < srcB.Length; i++)
                transformedB[i] = RotatePoint(currentRot, srcB[i]) + currentTrans;

            var grid = BuildSpatialGrid(srcA, adaptMaxDist);
            int matchCount = 0;

            for (int i = 0; i < transformedB.Length; i++)
            {
                int nearest = FindNearest(grid, srcA, transformedB[i], adaptMaxDist, adaptMaxDistSq);
                if (nearest >= 0)
                {
                    matchA[matchCount] = nearest;
                    matchB[matchCount] = i;
                    matchCount++;
                }
            }

            if (matchCount < 10)
            {
                Debug.Log($"[AutoAlign] Iter {iter}: only {matchCount} pairs, stopping.");
                break;
            }

            float cax = 0, cay = 0, caz = 0;
            float cbx = 0, cby = 0, cbz = 0;
            for (int i = 0; i < matchCount; i++)
            {
                cax += srcA[matchA[i]].x; cay += srcA[matchA[i]].y; caz += srcA[matchA[i]].z;
                cbx += srcB[matchB[i]].x; cby += srcB[matchB[i]].y; cbz += srcB[matchB[i]].z;
            }
            float invN = 1f / matchCount;
            cax *= invN; cay *= invN; caz *= invN;
            cbx *= invN; cby *= invN; cbz *= invN;

            float Sxx = 0, Sxy = 0, Sxz = 0;
            float Syx = 0, Syy = 0, Syz = 0;
            float Szx = 0, Szy = 0, Szz = 0;

            for (int i = 0; i < matchCount; i++)
            {
                float bx = srcB[matchB[i]].x - cbx;
                float by = srcB[matchB[i]].y - cby;
                float bz = srcB[matchB[i]].z - cbz;
                float ax = srcA[matchA[i]].x - cax;
                float ay = srcA[matchA[i]].y - cay;
                float az = srcA[matchA[i]].z - caz;

                Sxx += bx * ax; Sxy += bx * ay; Sxz += bx * az;
                Syx += by * ax; Syy += by * ay; Syz += by * az;
                Szx += bz * ax; Szy += bz * ay; Szz += bz * az;
            }

            Quaternion optRot = HornQuaternion(Sxx, Sxy, Sxz, Syx, Syy, Syz, Szx, Szy, Szz, currentRot);
            var rc = RotatePoint(optRot, new Vector3(cbx, cby, cbz));
            Vector3 optTrans = new Vector3(cax - rc.x, cay - rc.y, caz - rc.z);

            currentRot = QuatSlerp(currentRot, optRot, rate);
            currentRot = NormalizeQuat(currentRot);
            currentTrans = new Vector3(
                currentTrans.x + rate * (optTrans.x - currentTrans.x),
                currentTrans.y + rate * (optTrans.y - currentTrans.y),
                currentTrans.z + rate * (optTrans.z - currentTrans.z)
            );

            float error = 0;
            for (int i = 0; i < matchCount; i++)
            {
                var tb = RotatePoint(currentRot, srcB[matchB[i]]) + currentTrans;
                float dx = srcA[matchA[i]].x - tb.x;
                float dy = srcA[matchA[i]].y - tb.y;
                float dz = srcA[matchA[i]].z - tb.z;
                error += Mathf.Sqrt(dx * dx + dy * dy + dz * dz);
            }
            error /= matchCount;
            lastError = error;
            lastPairCount = matchCount;

            if (iter > 3 && error < 0.001f) break;
        }

        sw.Stop();
        Debug.Log($"[AutoAlign] ICP done in {sw.ElapsedMilliseconds}ms. Before={errorBefore:F4}m, After={lastError:F4}m, pairs={lastPairCount}");

        beforeMeanError = errorBefore;
        resultMeanError = lastError;
        resultPairCount = lastPairCount;
        resultQuaternion = currentRot;
        resultTranslation = currentTrans;

        // Safety: only accept if alignment actually improved
        bool improved = lastError < errorBefore * (1f - minImprove);
        resultAccepted = improved;
        resultReady = true;
    }

    private float ComputeAlignmentError(Vector3[] srcA, Vector3[] srcB, Vector3 trans, Quaternion rot, float maxDist)
    {
        float maxDistSq = maxDist * maxDist;
        var grid = BuildSpatialGrid(srcA, maxDist);
        float totalDist = 0;
        int count = 0;

        for (int i = 0; i < srcB.Length; i++)
        {
            var pb = RotatePoint(rot, srcB[i]) + trans;
            int nearest = FindNearest(grid, srcA, pb, maxDist, maxDistSq);
            if (nearest < 0) continue;

            float dx = srcA[nearest].x - pb.x;
            float dy = srcA[nearest].y - pb.y;
            float dz = srcA[nearest].z - pb.z;
            totalDist += Mathf.Sqrt(dx * dx + dy * dy + dz * dz);
            count++;
        }

        return count > 0 ? totalDist / count : float.MaxValue;
    }

    #region Horn's Quaternion Method

    private static Quaternion HornQuaternion(
        float Sxx, float Sxy, float Sxz,
        float Syx, float Syy, float Syz,
        float Szx, float Szy, float Szz,
        Quaternion initialGuess)
    {
        float n00 = Sxx + Syy + Szz;
        float n01 = Syz - Szy;
        float n02 = Szx - Sxz;
        float n03 = Sxy - Syx;
        float n11 = Sxx - Syy - Szz;
        float n12 = Sxy + Syx;
        float n13 = Szx + Sxz;
        float n22 = -Sxx + Syy - Szz;
        float n23 = Syz + Szy;
        float n33 = -Sxx - Syy + Szz;

        float v0 = initialGuess.w;
        float v1 = initialGuess.x;
        float v2 = initialGuess.y;
        float v3 = initialGuess.z;

        for (int i = 0; i < 30; i++)
        {
            float t0 = n00 * v0 + n01 * v1 + n02 * v2 + n03 * v3;
            float t1 = n01 * v0 + n11 * v1 + n12 * v2 + n13 * v3;
            float t2 = n02 * v0 + n12 * v1 + n22 * v2 + n23 * v3;
            float t3 = n03 * v0 + n13 * v1 + n23 * v2 + n33 * v3;

            float len = Mathf.Sqrt(t0 * t0 + t1 * t1 + t2 * t2 + t3 * t3);
            if (len < 1e-10f) break;
            float inv = 1f / len;
            v0 = t0 * inv; v1 = t1 * inv; v2 = t2 * inv; v3 = t3 * inv;
        }

        if (v0 < 0) { v0 = -v0; v1 = -v1; v2 = -v2; v3 = -v3; }
        return new Quaternion(v1, v2, v3, v0);
    }

    #endregion

    #region Spatial Hash Grid

    private const float GRID_CELL_MULTIPLIER = 2f;

    private Dictionary<long, List<int>> BuildSpatialGrid(Vector3[] points, float cellSize)
    {
        float cs = cellSize * GRID_CELL_MULTIPLIER;
        float inv = 1f / cs;
        var grid = new Dictionary<long, List<int>>(points.Length);
        for (int i = 0; i < points.Length; i++)
        {
            long key = CellKey(points[i], inv);
            if (!grid.TryGetValue(key, out var list))
            {
                list = new List<int>(4);
                grid[key] = list;
            }
            list.Add(i);
        }
        return grid;
    }

    private int FindNearest(Dictionary<long, List<int>> grid, Vector3[] points, Vector3 query, float cellSize, float maxDistSq)
    {
        float cs = cellSize * GRID_CELL_MULTIPLIER;
        float inv = 1f / cs;
        int cx = Mathf.FloorToInt(query.x * inv);
        int cy = Mathf.FloorToInt(query.y * inv);
        int cz = Mathf.FloorToInt(query.z * inv);

        float bestDistSq = maxDistSq;
        int bestIdx = -1;

        for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
        for (int dz = -1; dz <= 1; dz++)
        {
            long key = PackKey(cx + dx, cy + dy, cz + dz);
            if (!grid.TryGetValue(key, out var list)) continue;
            for (int li = 0; li < list.Count; li++)
            {
                int idx = list[li];
                float ddx = points[idx].x - query.x;
                float ddy = points[idx].y - query.y;
                float ddz = points[idx].z - query.z;
                float dSq = ddx * ddx + ddy * ddy + ddz * ddz;
                if (dSq < bestDistSq) { bestDistSq = dSq; bestIdx = idx; }
            }
        }
        return bestIdx;
    }

    private static long CellKey(Vector3 p, float inv)
    {
        return PackKey(Mathf.FloorToInt(p.x * inv), Mathf.FloorToInt(p.y * inv), Mathf.FloorToInt(p.z * inv));
    }

    private static long PackKey(int x, int y, int z)
    {
        return ((long)(x & 0x1FFFFF) << 42) | ((long)(y & 0x1FFFFF) << 21) | (long)(z & 0x1FFFFF);
    }

    #endregion

    #region Thread-Safe Math

    private static Vector3 RotatePoint(Quaternion q, Vector3 v)
    {
        float qx2 = q.x + q.x, qy2 = q.y + q.y, qz2 = q.z + q.z;
        float xx = q.x * qx2, yy = q.y * qy2, zz = q.z * qz2;
        float xy = q.x * qy2, xz = q.x * qz2, yz = q.y * qz2;
        float wx = q.w * qx2, wy = q.w * qy2, wz = q.w * qz2;
        return new Vector3(
            (1f - yy - zz) * v.x + (xy - wz) * v.y + (xz + wy) * v.z,
            (xy + wz) * v.x + (1f - xx - zz) * v.y + (yz - wx) * v.z,
            (xz - wy) * v.x + (yz + wx) * v.y + (1f - xx - yy) * v.z
        );
    }

    private static Quaternion QuatSlerp(Quaternion a, Quaternion b, float t)
    {
        float dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
        if (dot < 0f) { b = new Quaternion(-b.x, -b.y, -b.z, -b.w); dot = -dot; }
        if (dot > 0.9999f)
        {
            float rx = a.x + t * (b.x - a.x), ry = a.y + t * (b.y - a.y);
            float rz = a.z + t * (b.z - a.z), rw = a.w + t * (b.w - a.w);
            float len = Mathf.Sqrt(rx * rx + ry * ry + rz * rz + rw * rw);
            float inv = 1f / len;
            return new Quaternion(rx * inv, ry * inv, rz * inv, rw * inv);
        }
        float theta = Mathf.Acos(dot);
        float sinT = Mathf.Sin(theta);
        float wa = Mathf.Sin((1f - t) * theta) / sinT;
        float wb = Mathf.Sin(t * theta) / sinT;
        return new Quaternion(wa * a.x + wb * b.x, wa * a.y + wb * b.y, wa * a.z + wb * b.z, wa * a.w + wb * b.w);
    }

    private static Quaternion NormalizeQuat(Quaternion q)
    {
        float len = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (len < 1e-10f) return new Quaternion(0, 0, 0, 1);
        float inv = 1f / len;
        return new Quaternion(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
    }

    #endregion
}
