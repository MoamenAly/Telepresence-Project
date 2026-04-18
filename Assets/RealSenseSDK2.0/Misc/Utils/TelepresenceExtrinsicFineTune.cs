using UnityEngine;

/// <summary>
/// Interactive correction for camera B’s point cloud in the merged view. The symmetric rig is only a rough guess;
/// use Play Mode sliders here to remove ghosting/double surfaces, then <b>Bake</b> into <see cref="RsDualCameraPointCloudRig"/>.
/// For best results, also measure extrinsics with a Charuco board + OpenCV and paste into the rig, then fine-tune here.
/// </summary>
[DefaultExecutionOrder(200)]
public class TelepresenceExtrinsicFineTune : MonoBehaviour
{
    [Tooltip("Rig that owns PointCloudRootB and Position/Rotation B In A.")]
    public RsDualCameraPointCloudRig rig;

    [Header("Live correction (camera B in parent space of the two roots)")]
    [Tooltip("Shift B cloud (meters). Use to collapse double edges when symmetric layout is wrong.")]
    public Vector3 positionDelta;

    [Tooltip("Rotate B cloud (degrees). Y = yaw between the two views is most useful.")]
    public Vector3 rotationDeltaEuler;

    [Tooltip("If off, this component does nothing (only base rig extrinsics apply).")]
    public bool applyFineTune = true;

    void LateUpdate()
    {
        if (!applyFineTune || rig == null || rig.pointCloudRootB == null)
            return;

        var rootB = rig.pointCloudRootB;
        rootB.localPosition = rig.positionBInA + positionDelta;
        rootB.localRotation = Quaternion.Euler(rotationDeltaEuler) * Quaternion.Euler(rig.rotationBInAEuler);
    }

    /// <summary>
    /// Merges deltas into <see cref="RsDualCameraPointCloudRig.positionBInA"/> / <c>rotationBInAEuler</c> and clears deltas.
    /// Turn off <see cref="RsDualCameraPointCloudRig.useSymmetricTelepresenceLayout"/> if you do not want Awake to overwrite baked values.
    /// </summary>
    [ContextMenu("Bake fine tune into rig (clear deltas)")]
    public void BakeIntoRig()
    {
        if (rig == null)
            return;

        Quaternion combined = Quaternion.Euler(rotationDeltaEuler) * Quaternion.Euler(rig.rotationBInAEuler);
        rig.rotationBInAEuler = combined.eulerAngles;
        rig.positionBInA += positionDelta;
        positionDelta = Vector3.zero;
        rotationDeltaEuler = Vector3.zero;

        rig.useSymmetricTelepresenceLayout = false;
        rig.ApplyTransforms();
    }
}
