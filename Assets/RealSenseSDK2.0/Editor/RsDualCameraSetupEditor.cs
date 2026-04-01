using UnityEngine;
using UnityEditor;

public class RsDualCameraSetupEditor
{
    [MenuItem("RealSense/Setup Dual D455 Scene")]
    public static void SetupDualD455()
    {
        var rig = GameObject.Find("TelepresenceRig");
        if (rig == null)
        {
            EditorUtility.DisplayDialog("Dual D455 Setup",
                "Could not find 'TelepresenceRig' in the scene.\n\n" +
                "Make sure Main.unity is open and contains the TelepresenceRig hierarchy.", "OK");
            return;
        }

        Undo.SetCurrentGroupName("Setup Dual D455");
        int undoGroup = Undo.GetCurrentGroup();

        AddComponentIfMissing<RsDualD455Setup>(rig);

        var rigComp = AddComponentIfMissing<RsDualCameraPointCloudRig>(rig);
        WireRigReferences(rigComp);

        var calibComp = AddComponentIfMissing<RsDualCameraExtrinsicsCalibration>(rig);
        AddComponentIfMissing<RsDualCalibrationAssist>(rig);
        AddComponentIfMissing<RsDualPointCloudAutoAligner>(rig);

        SetupRoiFilters(rig);
        SetupFusionManager(rig, rigComp, calibComp);

        Undo.CollapseUndoOperations(undoGroup);
        EditorUtility.SetDirty(rig);

        Debug.Log("[DualD455 Setup] All components wired. Save the scene (Ctrl+S) to persist.");
    }

    private static T AddComponentIfMissing<T>(GameObject go) where T : Component
    {
        var existing = go.GetComponent<T>();
        if (existing != null) return existing;
        return Undo.AddComponent<T>(go);
    }

    private static void WireRigReferences(RsDualCameraPointCloudRig rig)
    {
        if (rig == null) return;

        var devices = rig.GetComponentsInChildren<RsDevice>(true);
        var pcRenderers = rig.GetComponentsInChildren<RsPointCloudRenderer>(true);

        if (devices.Length >= 2)
        {
            if (rig.deviceA == null) { rig.deviceA = devices[0]; EditorUtility.SetDirty(rig); }
            if (rig.deviceB == null) { rig.deviceB = devices[1]; EditorUtility.SetDirty(rig); }
        }

        if (pcRenderers.Length >= 2)
        {
            if (rig.pointCloudA == null) { rig.pointCloudA = pcRenderers[0]; EditorUtility.SetDirty(rig); }
            if (rig.pointCloudB == null) { rig.pointCloudB = pcRenderers[1]; EditorUtility.SetDirty(rig); }
        }

        if (rig.deviceBTransform == null && rig.deviceB != null)
        {
            rig.deviceBTransform = rig.deviceB.transform;
            EditorUtility.SetDirty(rig);
        }
    }

    private static void SetupRoiFilters(GameObject rigGo)
    {
        var pcRenderers = rigGo.GetComponentsInChildren<RsPointCloudRenderer>(true);
        foreach (var pc in pcRenderers)
        {
            if (pc.GetComponent<RsPointCloudRoiFilter>() == null)
                Undo.AddComponent<RsPointCloudRoiFilter>(pc.gameObject);
        }
    }

    private static void SetupFusionManager(GameObject rigGo,
        RsDualCameraPointCloudRig rig, RsDualCameraExtrinsicsCalibration calib)
    {
        var existingFusion = rigGo.GetComponentInChildren<RsDualPointCloudFusionManager>(true);
        if (existingFusion != null)
        {
            WireFusionReferences(existingFusion, rig, calib);
            return;
        }

        var fusionGo = new GameObject("DualFusionManager");
        Undo.RegisterCreatedObjectUndo(fusionGo, "Create DualFusionManager");
        fusionGo.transform.SetParent(rigGo.transform);
        fusionGo.transform.localPosition = Vector3.zero;
        fusionGo.transform.localRotation = Quaternion.identity;

        fusionGo.AddComponent<MeshFilter>();
        fusionGo.AddComponent<MeshRenderer>();
        var fusionComp = fusionGo.AddComponent<RsDualPointCloudFusionManager>();

        WireFusionReferences(fusionComp, rig, calib);
    }

    private static void WireFusionReferences(RsDualPointCloudFusionManager fusion,
        RsDualCameraPointCloudRig rig, RsDualCameraExtrinsicsCalibration calib)
    {
        if (fusion.calibration == null && calib != null)
        {
            fusion.calibration = calib;
            EditorUtility.SetDirty(fusion);
        }

        if (rig == null) return;

        if (fusion.meshFilterA == null && rig.pointCloudA != null)
        {
            fusion.meshFilterA = rig.pointCloudA.GetComponent<MeshFilter>();
            EditorUtility.SetDirty(fusion);
        }

        if (fusion.meshFilterB == null && rig.pointCloudB != null)
        {
            fusion.meshFilterB = rig.pointCloudB.GetComponent<MeshFilter>();
            EditorUtility.SetDirty(fusion);
        }
    }
}
