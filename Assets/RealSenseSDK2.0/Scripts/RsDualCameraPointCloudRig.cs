using UnityEngine;

/// <summary>
/// Two RealSense devices: serials + merge B's point cloud into A's frame.
/// For live telepresence, enable symmetric layout (baseline + matching toe-in) or set extrinsics from calibration.
/// </summary>
[DefaultExecutionOrder(-100)]
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

    [Header("Symmetric telepresence (starting guess only)")]
    [Tooltip("Rough B→A extrinsics from baseline + toe-in. Use TelepresenceIcpAutoCalibrator (Play → Run ICP) or Charuco/OpenCV; disabled automatically when ICP / SetExtrinsic applies.")]
    public bool useSymmetricTelepresenceLayout = true;

    [Tooltip("Distance between the two camera centers (meters). Measure your left/right mount spacing.")]
    public float baselineMeters = 0.35f;

    [Tooltip("Each camera yaws inward by this angle (degrees). Match how much you rotated them toward yourself.")]
    public float toeInYawDegrees = 12f;

    [Tooltip("True if Camera A is the physically LEFT unit (B is on the right).")]
    public bool cameraAIsLeft = true;

    [Header("Extrinsics (B relative to A)")]
    [Tooltip("B origin in A's depth frame (meters). Set manually or computed from symmetric layout.")]
    public Vector3 positionBInA = Vector3.zero;

    [Tooltip("Rotation from B depth frame to A (Euler degrees).")]
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
        if (useSymmetricTelepresenceLayout)
            ApplySymmetricTelepresenceExtrinsics();
        ApplyTransforms();
    }

    /// <summary>
    /// Assumes cameras on a line along X at ±baseline/2, same height, each yawed toward center by toeInYawDegrees.
    /// </summary>
    [ContextMenu("Apply symmetric telepresence extrinsics only")]
    public void ApplySymmetricTelepresenceExtrinsics()
    {
        float half = Mathf.Max(0f, baselineMeters) * 0.5f;
        float yaw = toeInYawDegrees;

        Vector3 posA = cameraAIsLeft ? new Vector3(-half, 0f, 0f) : new Vector3(half, 0f, 0f);
        Vector3 posB = cameraAIsLeft ? new Vector3(half, 0f, 0f) : new Vector3(-half, 0f, 0f);

        Quaternion rotA = Quaternion.Euler(0f, cameraAIsLeft ? yaw : -yaw, 0f);
        Quaternion rotB = Quaternion.Euler(0f, cameraAIsLeft ? -yaw : yaw, 0f);

        Quaternion rBtoA = Quaternion.Inverse(rotA) * rotB;
        Vector3 tBtoA = Quaternion.Inverse(rotA) * (posB - posA);

        positionBInA = tBtoA;
        rotationBInAEuler = rBtoA.eulerAngles;
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

    /// <summary>
    /// Sets B→A extrinsic from automatic calibration (e.g. ICP). Disables symmetric layout so values persist.
    /// </summary>
    public void SetExtrinsicBToA(Vector3 translationInAFrame, Quaternion rotationBToA)
    {
        useSymmetricTelepresenceLayout = false;
        positionBInA = translationInAFrame;
        rotationBInAEuler = rotationBToA.eulerAngles;
        ApplyTransforms();
    }
}
