using UnityEngine;

/// <summary>
/// Phase 1 setup for two manually assigned RealSense cameras shown as separate point clouds.
/// </summary>
public class RsTwoCameraPointCloudSetup : MonoBehaviour
{
    private static readonly Vector3 PointCloudRootBOffset = new Vector3(0.75f, 0f, 0f);

    public RsDevice cameraA;
    public RsDevice cameraB;
    public Transform pointCloudRootA;
    public Transform pointCloudRootB;
    public string cameraASerial = string.Empty;
    public string cameraBSerial = string.Empty;

    void Awake()
    {
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
