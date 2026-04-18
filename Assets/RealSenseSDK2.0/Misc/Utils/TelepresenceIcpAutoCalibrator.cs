using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Automatic B→A extrinsic calibration using point-to-point ICP + Kabsch (quaternion eigenmethod).
/// Requires <b>overlapping geometry</b> in both views (person, furniture, calibration object). Does not work with
/// empty or non-overlapping scenes — that case needs a Charuco board + OpenCV or hardware sync + factory extrinsics.
/// </summary>
public class TelepresenceIcpAutoCalibrator : MonoBehaviour
{
    public RsDualCameraPointCloudRig rig;
    public MeshFilter meshFilterA;
    public MeshFilter meshFilterB;

    [Tooltip("Points sampled per cloud (higher = slower, more stable).")]
    [Range(128, 4096)]
    public int sampleCount = 768;

    [Tooltip("ICP iterations.")]
    [Range(5, 60)]
    public int maxIterations = 22;

    [Tooltip("Ignore correspondences farther than this (meters).")]
    public float maxCorrespondenceDistance = 0.12f;

    [Tooltip("Start from current rig extrinsic (symmetric or previous run) — strongly recommended.")]
    public bool warmStartFromRig = true;

    [Tooltip("Stop when rotation change (deg) and translation (m) are below these.")]
    public float convergenceAngleDeg = 0.15f;
    public float convergenceTransMeters = 0.002f;

    [Header("Last run")]
    [SerializeField] float lastRmse;
    [SerializeField] int lastPairs;
    [SerializeField] bool lastSuccess;

    void Reset()
    {
        rig = GetComponent<RsDualCameraPointCloudRig>();
        if (rig != null)
        {
            if (meshFilterA == null && rig.pointCloudRootA != null)
                meshFilterA = rig.pointCloudRootA.GetComponentInChildren<MeshFilter>();
            if (meshFilterB == null && rig.pointCloudRootB != null)
                meshFilterB = rig.pointCloudRootB.GetComponentInChildren<MeshFilter>();
        }
    }

    [ContextMenu("Run automatic ICP calibration (current meshes)")]
    public void RunAutoCalibration()
    {
        lastSuccess = false;
        if (!Application.isPlaying)
        {
            Debug.LogWarning("TelepresenceIcpAutoCalibrator: enter Play Mode first so point-cloud meshes have depth data.");
            return;
        }

        if (rig == null || meshFilterA == null || meshFilterB == null)
        {
            Debug.LogWarning("TelepresenceIcpAutoCalibrator: assign rig and both MeshFilters.");
            return;
        }

        var meshA = meshFilterA.sharedMesh;
        var meshB = meshFilterB.sharedMesh;
        if (meshA == null || meshB == null)
        {
            Debug.LogWarning("TelepresenceIcpAutoCalibrator: meshes not ready.");
            return;
        }

        var va = meshA.vertices;
        var vb = meshB.vertices;
        if (va == null || vb == null || va.Length == 0 || vb.Length == 0)
            return;

        var setP = SampleValid(va, sampleCount);
        var setQ = SampleValid(vb, sampleCount);
        if (setP.Count < 64 || setQ.Count < 64)
        {
            Debug.LogWarning("TelepresenceIcpAutoCalibrator: not enough valid depth points. Overlap both cameras on the same scene content.");
            return;
        }

        Quaternion R = Quaternion.identity;
        Vector3 t = Vector3.zero;
        if (warmStartFromRig)
        {
            R = Quaternion.Euler(rig.rotationBInAEuler);
            t = rig.positionBInA;
        }

        for (int iter = 0; iter < maxIterations; iter++)
        {
            // Wider gate early, tight gate late — helps escape bad symmetric / stale extrinsics (reduces ghosting).
            float tSched = maxIterations <= 1 ? 1f : (float)iter / (maxIterations - 1);
            float corrMeters = Mathf.Lerp(maxCorrespondenceDistance * 2.35f, maxCorrespondenceDistance, tSched * tSched);
            float corrSq = corrMeters * corrMeters;

            var pairsQ = new List<Vector3>(setQ.Count);
            var pairsP = new List<Vector3>(setQ.Count);
            float distSum = 0f;

            for (int i = 0; i < setQ.Count; i++)
            {
                Vector3 x = R * setQ[i] + t;
                int j = NearestIndex(setP, x, out float d);
                if (d > corrSq)
                    continue;
                pairsQ.Add(setQ[i]);
                pairsP.Add(setP[j]);
                distSum += d;
            }

            lastPairs = pairsQ.Count;
            if (pairsQ.Count < 48)
            {
                Debug.LogWarning($"TelepresenceIcpAutoCalibrator: too few pairs ({pairsQ.Count}). Move cameras or increase overlap / maxCorrespondenceDistance.");
                return;
            }

            lastRmse = Mathf.Sqrt(distSum / pairsQ.Count);

            if (!KabschRotationTranslation(pairsQ, pairsP, out Quaternion Rstep, out Vector3 tstep))
                return;

            Quaternion rPrev = R;
            Vector3 tPrev = t;
            R = Rstep;
            t = tstep;

            float dAng = Quaternion.Angle(R, rPrev);
            float dTrans = (t - tPrev).magnitude;
            if (dAng < convergenceAngleDeg && dTrans < convergenceTransMeters)
                break;
        }

        rig.SetExtrinsicBToA(t, R);
        lastSuccess = true;
        Debug.Log($"TelepresenceIcpAutoCalibrator: done. RMSE≈{lastRmse:F4} m, pairs={lastPairs}. Symmetric layout disabled; extrinsic applied.");
    }

    static List<Vector3> SampleValid(Vector3[] verts, int want)
    {
        var list = new List<Vector3>(want);
        int step = Mathf.Max(1, verts.Length / (want * 2));
        var rnd = new System.Random(42);
        for (int k = 0; k < want * 4 && list.Count < want; k++)
        {
            int i = rnd.Next(0, verts.Length);
            if (i % step != 0 && rnd.NextDouble() > 0.3)
                continue;
            Vector3 v = verts[i];
            if (float.IsNaN(v.x) || float.IsInfinity(v.x))
                continue;
            float s = v.sqrMagnitude;
            if (s < 1e-6f || s > 400f)
                continue;
            list.Add(v);
        }

        if (list.Count < want / 2)
        {
            for (int i = 0; i < verts.Length && list.Count < want; i += step)
            {
                Vector3 v = verts[i];
                float s = v.sqrMagnitude;
                if (s < 1e-6f || s > 400f || float.IsNaN(v.x))
                    continue;
                list.Add(v);
            }
        }

        return list;
    }

    static int NearestIndex(List<Vector3> pts, Vector3 x, out float distSq)
    {
        distSq = float.MaxValue;
        int best = 0;
        for (int i = 0; i < pts.Count; i++)
        {
            float d = (pts[i] - x).sqrMagnitude;
            if (d < distSq)
            {
                distSq = d;
                best = i;
            }
        }

        return best;
    }

    /// <summary>Kabsch: find R,t with p ≈ R q + t (centroid alignment).</summary>
    static bool KabschRotationTranslation(List<Vector3> qList, List<Vector3> pList, out Quaternion rotation, out Vector3 translation)
    {
        rotation = Quaternion.identity;
        translation = Vector3.zero;
        int n = qList.Count;
        if (n != pList.Count || n < 3)
            return false;

        Vector3 muQ = Vector3.zero, muP = Vector3.zero;
        for (int i = 0; i < n; i++)
        {
            muQ += qList[i];
            muP += pList[i];
        }

        muQ /= n;
        muP /= n;

        float sxx = 0, sxy = 0, sxz = 0, syx = 0, syy = 0, syz = 0, szx = 0, szy = 0, szz = 0;
        for (int i = 0; i < n; i++)
        {
            Vector3 q = qList[i] - muQ;
            Vector3 p = pList[i] - muP;
            sxx += p.x * q.x;
            sxy += p.x * q.y;
            sxz += p.x * q.z;
            syx += p.y * q.x;
            syy += p.y * q.y;
            syz += p.y * q.z;
            szx += p.z * q.x;
            szy += p.z * q.y;
            szz += p.z * q.z;
        }

        float[,] N =
        {
            { sxx + syy + szz, syz - szy, szx - sxz, sxy - syx },
            { syz - szy, sxx - syy - szz, sxy + syx, szx + sxz },
            { szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy },
            { sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz }
        };

        Vector4 ev = PowerIterationSymmetric4(N, 64);
        // Horn: eigenvector (w, x, y, z) scalar first; Unity Quaternion is (x,y,z,w)
        rotation = new Quaternion(ev.y, ev.z, ev.w, ev.x).normalized;
        if (float.IsNaN(rotation.x))
            return false;

        translation = muP - rotation * muQ;
        return true;
    }

    static Vector4 PowerIterationSymmetric4(float[,] N, int iters)
    {
        Vector4 v = new Vector4(1f, 0.1f, 0.1f, 0.1f);
        v = Normalize4(v);
        for (int k = 0; k < iters; k++)
        {
            Vector4 nv = Vector4.zero;
            for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                nv[i] += N[i, j] * v[j];
            v = Normalize4(nv);
        }

        return v;
    }

    static Vector4 Normalize4(Vector4 v)
    {
        float s = v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w;
        if (s < 1e-12f)
            return new Vector4(1f, 0f, 0f, 0f);
        s = Mathf.Sqrt(s);
        return new Vector4(v.x / s, v.y / s, v.z / s, v.w / s);
    }
}
