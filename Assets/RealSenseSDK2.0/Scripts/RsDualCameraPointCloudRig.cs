using UnityEngine;

/// <summary>
/// Minimal helper for two RealSense devices used in one point cloud view.
/// It assigns camera serial numbers and applies camera B pose relative to camera A.
/// </summary>
public class RsDualCameraPointCloudRig : MonoBehaviour
{
    [Header("Devices")]
    public RsDevice cameraA;
    public RsDevice cameraB;

    [Tooltip("Physical serial number for camera A.")]
    public string cameraASerial = string.Empty;

    [Tooltip("Physical serial number for camera B.")]
    public string cameraBSerial = string.Empty;

    [Header("Point Cloud Roots")]
    [Tooltip("Root transform of camera A point cloud. Usually left at identity.")]
    public Transform pointCloudRootA;

    [Tooltip("Root transform of camera B point cloud (this receives B->A transform).")]
    public Transform pointCloudRootB;

    [Header("Extrinsics (B relative to A)")]
    [Tooltip("Translation in meters from camera A to camera B point cloud frame.")]
    public Vector3 positionBInA = Vector3.zero;

    [Tooltip("Rotation (Euler degrees) from camera B to camera A frame.")]
    public Vector3 rotationBInAEuler = Vector3.zero;

    [Header("Startup")]
    [Tooltip("If true, applies serials and transforms automatically on Awake.")]
    public bool applyOnAwake = true;

    private void Awake()
    {
        if (!applyOnAwake)
            return;

        ApplyConfiguration();
    }

    [ContextMenu("Apply Dual Camera Configuration")]
    public void ApplyConfiguration()
    {
        ApplySerials();
        ApplyTransforms();
    }

    [ContextMenu("Apply Camera Serials")]
    public void ApplySerials()
    {
        if (cameraA != null)
            cameraA.DeviceConfiguration.RequestedSerialNumber = cameraASerial == null ? string.Empty : cameraASerial.Trim();

        if (cameraB != null)
            cameraB.DeviceConfiguration.RequestedSerialNumber = cameraBSerial == null ? string.Empty : cameraBSerial.Trim();
    }

    [ContextMenu("Apply Point Cloud Transforms")]
    public void ApplyTransforms()
    {
        if (pointCloudRootA != null)
        {
            pointCloudRootA.localPosition = Vector3.zero;
            pointCloudRootA.localRotation = Quaternion.identity;
        }

        if (pointCloudRootB != null)
        {
            pointCloudRootB.localPosition = positionBInA;
            pointCloudRootB.localRotation = Quaternion.Euler(rotationBInAEuler);
        }
    }
}
