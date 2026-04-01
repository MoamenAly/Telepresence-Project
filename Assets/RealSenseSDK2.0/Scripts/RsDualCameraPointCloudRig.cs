using UnityEngine;

public class RsDualCameraPointCloudRig : MonoBehaviour
{
    [Header("Device References")]
    public RsDevice deviceA;
    public RsDevice deviceB;

    [Header("Point Cloud Renderers")]
    public RsPointCloudRenderer pointCloudA;
    public RsPointCloudRenderer pointCloudB;

    [Header("Calibration Transform")]
    [Tooltip("The transform that extrinsics calibration will adjust (typically RsDevice_B).")]
    public Transform deviceBTransform;

    void Awake()
    {
        AutoDiscover();
        Validate();
    }

    private void AutoDiscover()
    {
        if (deviceA == null || deviceB == null)
        {
            var devices = GetComponentsInChildren<RsDevice>(true);
            if (devices.Length >= 2)
            {
                if (deviceA == null) deviceA = devices[0];
                if (deviceB == null) deviceB = devices[1];
            }
        }

        if (pointCloudA == null || pointCloudB == null)
        {
            var renderers = GetComponentsInChildren<RsPointCloudRenderer>(true);
            if (renderers.Length >= 2)
            {
                if (pointCloudA == null) pointCloudA = renderers[0];
                if (pointCloudB == null) pointCloudB = renderers[1];
            }
        }

        if (deviceBTransform == null && deviceB != null)
            deviceBTransform = deviceB.transform;
    }

    private void Validate()
    {
        if (deviceA == null) Debug.LogError("[DualRig] deviceA reference is missing.", this);
        if (deviceB == null) Debug.LogError("[DualRig] deviceB reference is missing.", this);
        if (pointCloudA == null) Debug.LogError("[DualRig] pointCloudA reference is missing.", this);
        if (pointCloudB == null) Debug.LogError("[DualRig] pointCloudB reference is missing.", this);
        if (deviceBTransform == null) Debug.LogError("[DualRig] deviceBTransform reference is missing.", this);
    }
}
