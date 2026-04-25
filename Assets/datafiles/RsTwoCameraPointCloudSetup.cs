using System;
using System.Collections.Generic;
using UnityEngine;
using Intel.RealSense;

/// <summary>
/// Two-camera RealSense setup with separate point clouds and per-camera depth clipping.
/// </summary>
[DefaultExecutionOrder(-1000)]
public class RsTwoCameraPointCloudSetup : MonoBehaviour
{
    private const float DefaultMinDistance = 0.28f;
    private const float DefaultMaxDistance = 1.55f;

    [Header("Devices")]
    public RsDevice cameraA;
    public RsDevice cameraB;
    public RsProcessingPipe cameraAPipe;
    public RsProcessingPipe cameraBPipe;

    [Header("Point Cloud Roots")]
    public Transform pointCloudRootA;
    public Transform pointCloudRootB;

    [Header("Camera Serials")]
    public string cameraASerial = string.Empty;
    public string cameraBSerial = string.Empty;

    [Header("Depth Clipping")]
    [Range(0f, 16f)] public float cameraAMinDistance = DefaultMinDistance;
    [Range(0f, 16f)] public float cameraAMaxDistance = DefaultMaxDistance;
    [Range(0f, 16f)] public float cameraBMinDistance = DefaultMinDistance;
    [Range(0f, 16f)] public float cameraBMaxDistance = DefaultMaxDistance;

    [Header("B to A Alignment")]
    [Tooltip("When enabled, calculate B to A from baseline and toe-in before applying the point cloud transform.")]
    public bool useSymmetricStartingLayout = true;

    [Tooltip("Distance between the physical camera centers in meters.")]
    public float baselineMeters = 0.35f;

    [Tooltip("Each camera yaws inward by this angle in degrees. Use 0 for parallel cameras.")]
    public float toeInYawDegrees = 0f;

    [Tooltip("Enable when Camera A is the physically left camera and Camera B is physically right.")]
    public bool cameraAIsLeft = true;

    [Tooltip("Camera B origin expressed in Camera A's point-cloud frame, in meters.")]
    public Vector3 cameraBPositionInA = new Vector3(0.35f, 0f, 0f);

    [Tooltip("Rotation from Camera B point-cloud frame into Camera A's point-cloud frame.")]
    public Vector3 cameraBRotationInAEuler = Vector3.zero;

    [Header("Automatic Alignment")]
    [Tooltip("Use the baseline / toe-in estimate as the initial guess before automatic registration.")]
    public bool autoCalibrationStartsFromSymmetricGuess = true;

    [Tooltip("Maximum number of points sampled from each camera for automatic registration.")]
    [Range(256, 5000)] public int autoCalibrationMaxPoints = 1500;

    [Tooltip("Voxel size used to downsample point clouds before registration.")]
    [Range(0.005f, 0.05f)] public float autoCalibrationVoxelSize = 0.02f;

    [Tooltip("Maximum correspondence distance in meters during ICP matching.")]
    [Range(0.01f, 0.20f)] public float autoCalibrationMaxCorrespondenceDistance = 0.06f;

    [Tooltip("Number of ICP refinement iterations.")]
    [Range(1, 30)] public int autoCalibrationIterations = 12;

    [SerializeField, TextArea(2, 5)]
    private string autoCalibrationStatus = "Idle";

    private readonly object autoCalibrationLock = new object();
    private bool autoCalibrationRequested;
    private bool autoCalibrationHasFrameA;
    private bool autoCalibrationHasFrameB;
    private bool autoCalibrationReadyToSolve;
    private Vector3[] autoCalibrationPointsA;
    private Vector3[] autoCalibrationPointsB;

#if UNITY_EDITOR
    void OnValidate()
    {
        if (useSymmetricStartingLayout)
            CalculateSymmetricAlignment();

        ApplyPointCloudLayout();
        ConfigureBindings();
    }
#endif

    void OnEnable()
    {
        SubscribeToCalibrationFrames();
    }

    void OnDisable()
    {
        UnsubscribeFromCalibrationFrames();
    }

    void Awake()
    {
        ApplyProcessingProfiles();
        ApplySerials();
        if (useSymmetricStartingLayout)
            CalculateSymmetricAlignment();
        ApplyPointCloudLayout();
        ConfigureBindings();
    }

    void Update()
    {
        TrySolveAutoCalibration();
    }

    [ContextMenu("Apply Full Setup")]
    public void ApplyFullSetup()
    {
        ApplyProcessingProfiles();
        ApplySerials();
        if (useSymmetricStartingLayout)
            CalculateSymmetricAlignment();
        ApplyPointCloudLayout();
        ConfigureBindings();
    }

    [ContextMenu("Calculate Symmetric B to A Alignment")]
    public void CalculateSymmetricAlignment()
    {
        float halfBaseline = Mathf.Max(0f, baselineMeters) * 0.5f;
        float toeInYaw = toeInYawDegrees;

        Vector3 positionA = cameraAIsLeft ? new Vector3(-halfBaseline, 0f, 0f) : new Vector3(halfBaseline, 0f, 0f);
        Vector3 positionB = cameraAIsLeft ? new Vector3(halfBaseline, 0f, 0f) : new Vector3(-halfBaseline, 0f, 0f);

        Quaternion rotationA = Quaternion.Euler(0f, cameraAIsLeft ? toeInYaw : -toeInYaw, 0f);
        Quaternion rotationB = Quaternion.Euler(0f, cameraAIsLeft ? -toeInYaw : toeInYaw, 0f);

        Quaternion rotationBToA = Quaternion.Inverse(rotationA) * rotationB;
        Vector3 translationBToA = Quaternion.Inverse(rotationA) * (positionB - positionA);

        cameraBPositionInA = translationBToA;
        cameraBRotationInAEuler = rotationBToA.eulerAngles;
    }

    [ContextMenu("Apply B to A Point Cloud Transform")]
    public void ApplyPointCloudLayout()
    {
        if (pointCloudRootA != null)
        {
            pointCloudRootA.localPosition = Vector3.zero;
            pointCloudRootA.localRotation = Quaternion.identity;
        }

        if (pointCloudRootB != null)
        {
            pointCloudRootB.localPosition = cameraBPositionInA;
            pointCloudRootB.localRotation = Quaternion.Euler(cameraBRotationInAEuler);
        }
    }

    public void NudgeManualAlignment(Vector3 positionDelta, Vector3 rotationDeltaEuler)
    {
        useSymmetricStartingLayout = false;
        cameraBPositionInA += positionDelta;
        cameraBRotationInAEuler += rotationDeltaEuler;
        ApplyPointCloudLayout();
    }

    [ContextMenu("Begin Automatic Alignment")]
    public void BeginAutomaticAlignment()
    {
        if (autoCalibrationStartsFromSymmetricGuess)
        {
            useSymmetricStartingLayout = true;
            CalculateSymmetricAlignment();
            ApplyPointCloudLayout();
        }

        lock (autoCalibrationLock)
        {
            autoCalibrationRequested = true;
            autoCalibrationHasFrameA = false;
            autoCalibrationHasFrameB = false;
            autoCalibrationReadyToSolve = false;
            autoCalibrationPointsA = null;
            autoCalibrationPointsB = null;
        }

        autoCalibrationStatus = "Waiting for one fresh point cloud from each camera...";
    }

    public string GetAutoCalibrationStatus()
    {
        return autoCalibrationStatus;
    }

    private void ApplySerials()
    {
        ApplySerial(cameraA, cameraASerial);
        ApplySerial(cameraB, cameraBSerial);
    }

    private static void ApplySerial(RsDevice device, string serial)
    {
        if (device == null)
            return;

        var config = device.DeviceConfiguration;
        config.mode = RsConfiguration.Mode.Live;
        config.RequestedSerialNumber = (serial ?? string.Empty).Trim();
        device.DeviceConfiguration = config;
    }

    private void ConfigureBindings()
    {
        ConfigureBinding(pointCloudRootA, cameraA);
        ConfigureBinding(pointCloudRootB, cameraB);
    }

    private void SubscribeToCalibrationFrames()
    {
        if (cameraAPipe != null)
            cameraAPipe.OnNewSample += OnCameraAFrame;

        if (cameraBPipe != null)
            cameraBPipe.OnNewSample += OnCameraBFrame;
    }

    private void UnsubscribeFromCalibrationFrames()
    {
        if (cameraAPipe != null)
            cameraAPipe.OnNewSample -= OnCameraAFrame;

        if (cameraBPipe != null)
            cameraBPipe.OnNewSample -= OnCameraBFrame;
    }

    private void OnCameraAFrame(Frame frame)
    {
        TryCaptureCalibrationFrame(frame, true);
    }

    private void OnCameraBFrame(Frame frame)
    {
        TryCaptureCalibrationFrame(frame, false);
    }

    private void TryCaptureCalibrationFrame(Frame frame, bool forCameraA)
    {
        lock (autoCalibrationLock)
        {
            if (!autoCalibrationRequested)
                return;

            if (forCameraA && autoCalibrationHasFrameA)
                return;

            if (!forCameraA && autoCalibrationHasFrameB)
                return;
        }

        Vector3[] sampledPoints = ExtractSampledPointCloud(frame);
        if (sampledPoints == null || sampledPoints.Length == 0)
            return;

        lock (autoCalibrationLock)
        {
            if (!autoCalibrationRequested)
                return;

            if (forCameraA)
            {
                autoCalibrationPointsA = sampledPoints;
                autoCalibrationHasFrameA = true;
            }
            else
            {
                autoCalibrationPointsB = sampledPoints;
                autoCalibrationHasFrameB = true;
            }

            autoCalibrationReadyToSolve = autoCalibrationHasFrameA && autoCalibrationHasFrameB;
        }
    }

    private Vector3[] ExtractSampledPointCloud(Frame frame)
    {
        if (frame == null)
            return null;

        try
        {
            if (frame.IsComposite)
            {
                using (var frameSet = frame.As<FrameSet>())
                using (var points = frameSet.FirstOrDefault<Points>(Stream.Depth, Format.Xyz32f))
                {
                    if (points == null)
                        return null;

                    return SamplePoints(points);
                }
            }

            if (frame.Is(Extension.Points))
            {
                using (var points = frame.As<Points>())
                {
                    if (points == null)
                        return null;

                    return SamplePoints(points);
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogException(ex);
        }

        return null;
    }

    private Vector3[] SamplePoints(Points points)
    {
        var vertices = new Vector3[points.Count];
        points.CopyVertices(vertices);
        return DownsamplePointCloud(vertices, autoCalibrationVoxelSize, autoCalibrationMaxPoints);
    }

    private static Vector3[] DownsamplePointCloud(Vector3[] points, float voxelSize, int maxPoints)
    {
        if (points == null || points.Length == 0)
            return Array.Empty<Vector3>();

        float safeVoxel = Mathf.Max(0.001f, voxelSize);
        var sampled = new List<Vector3>(Mathf.Min(maxPoints, points.Length));
        var seenVoxels = new HashSet<Vector3Int>();

        for (int i = 0; i < points.Length; i++)
        {
            Vector3 p = points[i];
            if (!IsUsablePoint(p))
                continue;

            Vector3Int key = new Vector3Int(
                Mathf.FloorToInt(p.x / safeVoxel),
                Mathf.FloorToInt(p.y / safeVoxel),
                Mathf.FloorToInt(p.z / safeVoxel));

            if (!seenVoxels.Add(key))
                continue;

            sampled.Add(p);
            if (sampled.Count >= maxPoints)
                break;
        }

        return sampled.ToArray();
    }

    private static bool IsUsablePoint(Vector3 point)
    {
        return
            !float.IsNaN(point.x) &&
            !float.IsNaN(point.y) &&
            !float.IsNaN(point.z) &&
            !float.IsInfinity(point.x) &&
            !float.IsInfinity(point.y) &&
            !float.IsInfinity(point.z) &&
            point.sqrMagnitude > 0.000001f &&
            point.z > 0f;
    }

    private void TrySolveAutoCalibration()
    {
        Vector3[] pointsA = null;
        Vector3[] pointsB = null;

        lock (autoCalibrationLock)
        {
            if (!autoCalibrationReadyToSolve)
                return;

            autoCalibrationReadyToSolve = false;
            autoCalibrationRequested = false;
            pointsA = autoCalibrationPointsA;
            pointsB = autoCalibrationPointsB;
        }

        if (pointsA == null || pointsB == null || pointsA.Length == 0 || pointsB.Length == 0)
        {
            autoCalibrationStatus = "Automatic alignment failed: missing point-cloud data.";
            return;
        }

        if (TryRunIcp(pointsA, pointsB, out Vector3 solvedPosition, out Quaternion solvedRotation, out int matchCount, out float meanError))
        {
            useSymmetricStartingLayout = false;
            cameraBPositionInA = solvedPosition;
            cameraBRotationInAEuler = solvedRotation.eulerAngles;
            ApplyPointCloudLayout();
            autoCalibrationStatus = $"Automatic alignment complete. Matches: {matchCount}, mean error: {meanError:F4} m.";
        }
        else
        {
            autoCalibrationStatus = "Automatic alignment failed: ICP did not converge. Increase overlap or start from a better baseline / toe-in guess.";
        }
    }

    private bool TryRunIcp(
        Vector3[] targetPointsA,
        Vector3[] sourcePointsB,
        out Vector3 solvedPosition,
        out Quaternion solvedRotation,
        out int finalMatchCount,
        out float finalMeanError)
    {
        solvedPosition = cameraBPositionInA;
        solvedRotation = Quaternion.Euler(cameraBRotationInAEuler);
        Matrix4x4 currentTransform = Matrix4x4.TRS(cameraBPositionInA, Quaternion.Euler(cameraBRotationInAEuler), Vector3.one);
        float maxDistanceSqr = autoCalibrationMaxCorrespondenceDistance * autoCalibrationMaxCorrespondenceDistance;
        finalMatchCount = 0;
        finalMeanError = float.MaxValue;

        for (int iteration = 0; iteration < autoCalibrationIterations; iteration++)
        {
            var matchedSource = new List<Vector3>(sourcePointsB.Length);
            var matchedTarget = new List<Vector3>(sourcePointsB.Length);
            float errorSum = 0f;

            for (int i = 0; i < sourcePointsB.Length; i++)
            {
                Vector3 transformed = currentTransform.MultiplyPoint3x4(sourcePointsB[i]);
                if (!TryFindClosestPoint(targetPointsA, transformed, maxDistanceSqr, out Vector3 nearestTarget, out float distanceSqr))
                    continue;

                matchedSource.Add(transformed);
                matchedTarget.Add(nearestTarget);
                errorSum += Mathf.Sqrt(distanceSqr);
            }

            if (matchedSource.Count < 32)
                return false;

            if (!TrySolveBestFitTransform(matchedSource, matchedTarget, out Matrix4x4 deltaTransform))
                return false;

            currentTransform = deltaTransform * currentTransform;
            finalMatchCount = matchedSource.Count;
            finalMeanError = errorSum / matchedSource.Count;

            float translationStep = ExtractTranslation(deltaTransform).magnitude;
            float rotationStep = Quaternion.Angle(Quaternion.identity, ExtractRotation(deltaTransform));

            if (translationStep < 0.001f && rotationStep < 0.2f)
                break;
        }

        solvedPosition = ExtractTranslation(currentTransform);
        solvedRotation = ExtractRotation(currentTransform);
        return finalMatchCount >= 32;
    }

    private static bool TryFindClosestPoint(Vector3[] targetPoints, Vector3 sourcePoint, float maxDistanceSqr, out Vector3 nearestTarget, out float bestDistanceSqr)
    {
        nearestTarget = Vector3.zero;
        bestDistanceSqr = maxDistanceSqr;
        bool found = false;

        for (int i = 0; i < targetPoints.Length; i++)
        {
            float distanceSqr = (targetPoints[i] - sourcePoint).sqrMagnitude;
            if (distanceSqr > bestDistanceSqr)
                continue;

            bestDistanceSqr = distanceSqr;
            nearestTarget = targetPoints[i];
            found = true;
        }

        return found;
    }

    private static bool TrySolveBestFitTransform(IReadOnlyList<Vector3> sourcePoints, IReadOnlyList<Vector3> targetPoints, out Matrix4x4 transform)
    {
        transform = Matrix4x4.identity;
        int count = sourcePoints.Count;
        if (count == 0 || count != targetPoints.Count)
            return false;

        Vector3 sourceCentroid = Vector3.zero;
        Vector3 targetCentroid = Vector3.zero;
        for (int i = 0; i < count; i++)
        {
            sourceCentroid += sourcePoints[i];
            targetCentroid += targetPoints[i];
        }

        sourceCentroid /= count;
        targetCentroid /= count;

        float sxx = 0f, sxy = 0f, sxz = 0f;
        float syx = 0f, syy = 0f, syz = 0f;
        float szx = 0f, szy = 0f, szz = 0f;

        for (int i = 0; i < count; i++)
        {
            Vector3 a = sourcePoints[i] - sourceCentroid;
            Vector3 b = targetPoints[i] - targetCentroid;

            sxx += a.x * b.x;
            sxy += a.x * b.y;
            sxz += a.x * b.z;
            syx += a.y * b.x;
            syy += a.y * b.y;
            syz += a.y * b.z;
            szx += a.z * b.x;
            szy += a.z * b.y;
            szz += a.z * b.z;
        }

        float[,] n = new float[4, 4];
        float trace = sxx + syy + szz;
        n[0, 0] = trace;
        n[0, 1] = syz - szy;
        n[0, 2] = szx - sxz;
        n[0, 3] = sxy - syx;

        n[1, 0] = syz - szy;
        n[1, 1] = sxx - syy - szz;
        n[1, 2] = sxy + syx;
        n[1, 3] = szx + sxz;

        n[2, 0] = szx - sxz;
        n[2, 1] = sxy + syx;
        n[2, 2] = -sxx + syy - szz;
        n[2, 3] = syz + szy;

        n[3, 0] = sxy - syx;
        n[3, 1] = szx + sxz;
        n[3, 2] = syz + szy;
        n[3, 3] = -sxx - syy + szz;

        Vector4 eigen = new Vector4(1f, 0f, 0f, 0f);
        for (int i = 0; i < 20; i++)
        {
            Vector4 next = new Vector4(
                n[0, 0] * eigen.x + n[0, 1] * eigen.y + n[0, 2] * eigen.z + n[0, 3] * eigen.w,
                n[1, 0] * eigen.x + n[1, 1] * eigen.y + n[1, 2] * eigen.z + n[1, 3] * eigen.w,
                n[2, 0] * eigen.x + n[2, 1] * eigen.y + n[2, 2] * eigen.z + n[2, 3] * eigen.w,
                n[3, 0] * eigen.x + n[3, 1] * eigen.y + n[3, 2] * eigen.z + n[3, 3] * eigen.w);

            float magnitude = next.magnitude;
            if (magnitude < 0.000001f)
                return false;

            eigen = next / magnitude;
        }

        Quaternion rotation = new Quaternion(eigen.y, eigen.z, eigen.w, eigen.x).normalized;
        Vector3 translation = targetCentroid - rotation * sourceCentroid;
        transform = Matrix4x4.TRS(translation, rotation, Vector3.one);
        return true;
    }

    private static Vector3 ExtractTranslation(Matrix4x4 matrix)
    {
        return new Vector3(matrix.m03, matrix.m13, matrix.m23);
    }

    private static Quaternion ExtractRotation(Matrix4x4 matrix)
    {
        Vector3 forward = new Vector3(matrix.m02, matrix.m12, matrix.m22);
        Vector3 upward = new Vector3(matrix.m01, matrix.m11, matrix.m21);
        if (forward.sqrMagnitude < 0.000001f || upward.sqrMagnitude < 0.000001f)
            return Quaternion.identity;

        return Quaternion.LookRotation(forward, upward);
    }

    private void ApplyProcessingProfiles()
    {
        ApplyThresholds(cameraAPipe, cameraAMinDistance, cameraAMaxDistance);
        ApplyThresholds(cameraBPipe, cameraBMinDistance, cameraBMaxDistance);
    }

    private static void ApplyThresholds(RsProcessingPipe pipe, float minDistance, float maxDistance)
    {
        if (pipe == null || pipe.profile == null)
            return;

        float clampedMin = Mathf.Max(0f, minDistance);
        float clampedMax = Mathf.Max(clampedMin, maxDistance);

        foreach (var block in pipe.profile)
        {
            var thresholdFilter = block as RsThresholdFilter;
            if (thresholdFilter == null)
                continue;

            thresholdFilter.MinDistance = clampedMin;
            thresholdFilter.MaxDistance = clampedMax;
            break;
        }
    }

    private static void ConfigureBinding(Transform pointCloudRoot, RsDevice device)
    {
        if (pointCloudRoot == null || device == null)
            return;

        var binding = pointCloudRoot.GetComponent<RsPointCloudColorTextureBinding>();
        if (binding == null)
            return;

        binding.device = device;

        if (binding.pointCloudMeshRenderer == null)
            binding.pointCloudMeshRenderer = pointCloudRoot.GetComponentInChildren<MeshRenderer>(true);
    }
}
