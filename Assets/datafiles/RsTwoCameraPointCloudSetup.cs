using UnityEngine;

/// <summary>
/// Two-camera RealSense setup with separate point clouds and per-camera depth clipping.
/// </summary>
[DefaultExecutionOrder(-1000)]
public class RsTwoCameraPointCloudSetup : MonoBehaviour
{
    private static readonly Vector3 PointCloudRootBOffset = new Vector3(0.75f, 0f, 0f);
    private const float DefaultMinDistance = 0.28f;
    private const float DefaultMaxDistance = 1.55f;

    public RsDevice cameraA;
    public RsDevice cameraB;
    public RsProcessingPipe cameraAPipe;
    public RsProcessingPipe cameraBPipe;
    public Transform pointCloudRootA;
    public Transform pointCloudRootB;
    public string cameraASerial = string.Empty;
    public string cameraBSerial = string.Empty;
    [Range(0f, 16f)] public float cameraAMinDistance = DefaultMinDistance;
    [Range(0f, 16f)] public float cameraAMaxDistance = DefaultMaxDistance;
    [Range(0f, 16f)] public float cameraBMinDistance = DefaultMinDistance;
    [Range(0f, 16f)] public float cameraBMaxDistance = DefaultMaxDistance;

    void Awake()
    {
        ApplyProcessingProfiles();
        ApplySerials();
        ApplyPointCloudLayout();
        ConfigureBindings();
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

    private void ApplyPointCloudLayout()
    {
        if (pointCloudRootA != null)
        {
            pointCloudRootA.localPosition = Vector3.zero;
            pointCloudRootA.localRotation = Quaternion.identity;
        }

        if (pointCloudRootB != null)
        {
            pointCloudRootB.localPosition = PointCloudRootBOffset;
            pointCloudRootB.localRotation = Quaternion.identity;
        }
    }

    private void ConfigureBindings()
    {
        ConfigureBinding(pointCloudRootA, cameraA);
        ConfigureBinding(pointCloudRootB, cameraB);
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
