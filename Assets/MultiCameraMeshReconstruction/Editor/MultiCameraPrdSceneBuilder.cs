#if UNITY_EDITOR
using System.Collections.Generic;
using System.Linq;
using Intel.RealSense;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>
/// Builds <see cref="ScenePath"/> with two RealSense pipelines, merged point clouds, and PRD workflow notes.
/// </summary>
public static class MultiCameraPrdSceneBuilder
{
    public const string ScenePath = "Assets/MultiCameraMeshReconstruction/Scenes/MultiCameraTelepresencePRD.unity";

    const string RsDevicePrefab = "Assets/RealSenseSDK2.0/Prefabs/RsDevice.prefab";
    const string RsPipePrefab = "Assets/RealSenseSDK2.0/Prefabs/RsProcessingPipe.prefab";
    const string PointCloudPrefab = "Assets/RealSenseSDK2.0/Prefabs/PointCloud.prefab";
    const string DepthColorClippedProfile = "Assets/RealSenseSDK2.0/ProcessingPipe/PointCloudRgbDepthClipped.asset";

    [MenuItem("Telepresence/Create Multi-Camera PRD Scene")]
    public static void CreateSceneFromMenu()
    {
        CreateSceneInternal();
    }

    /// <summary>
    /// For an open telepresence scene: depth+color streams, RGB + depth threshold profile, RGB binding, and depth-range focus.
    /// </summary>
    [MenuItem("Telepresence/Upgrade Open Scene — RGB Point Clouds")]
    public static void UpgradeOpenSceneRgbPointClouds()
    {
        var rig = Object.FindFirstObjectByType<RsDualCameraPointCloudRig>();
        if (rig == null)
        {
            Debug.LogWarning("Telepresence upgrade: No RsDualCameraPointCloudRig in the open scene.");
            return;
        }

        var profile = AssetDatabase.LoadAssetAtPath<RsProcessingProfile>(DepthColorClippedProfile);
        if (profile == null)
        {
            Debug.LogError("Telepresence upgrade: Could not load PointCloudRgbDepthClipped.asset");
            return;
        }

        void UpgradeBranch(RsDevice dev, Transform cloudRoot)
        {
            if (dev == null || cloudRoot == null)
                return;

            var list = new List<RsVideoStreamRequest>();
            if (dev.DeviceConfiguration.Profiles != null)
                list.AddRange(dev.DeviceConfiguration.Profiles);
            bool hasDepth = list.Any(p => p.Stream == Intel.RealSense.Stream.Depth);
            bool hasColor = list.Any(p => p.Stream == Intel.RealSense.Stream.Color);
            if (!hasDepth)
            {
                list.Add(new RsVideoStreamRequest
                {
                    Stream = Intel.RealSense.Stream.Depth,
                    StreamIndex = -1,
                    Width = 640,
                    Height = 480,
                    Format = Format.Z16,
                    Framerate = 30
                });
            }

            if (!hasColor)
            {
                list.Add(new RsVideoStreamRequest
                {
                    Stream = Intel.RealSense.Stream.Color,
                    StreamIndex = -1,
                    Width = 640,
                    Height = 480,
                    Format = Format.Rgb8,
                    Framerate = 30
                });
            }

            var cfg = dev.DeviceConfiguration;
            cfg.Profiles = list.ToArray();
            dev.DeviceConfiguration = cfg;
            EditorUtility.SetDirty(dev);

            foreach (var pc in cloudRoot.GetComponentsInChildren<RsPointCloudRenderer>(true))
            {
                if (pc.Source is RsProcessingPipe pipe)
                {
                    pipe.profile = profile;
                    EditorUtility.SetDirty(pipe);
                }

                var bind = pc.GetComponent<RsPointCloudColorTextureBinding>();
                if (bind == null)
                    bind = pc.gameObject.AddComponent<RsPointCloudColorTextureBinding>();
                bind.device = dev;
                bind.pointCloudMeshRenderer = pc.GetComponent<MeshRenderer>();
                EditorUtility.SetDirty(bind);
            }
        }

        UpgradeBranch(rig.cameraA, rig.pointCloudRootA);
        UpgradeBranch(rig.cameraB, rig.pointCloudRootB);

        var focus = rig.GetComponent<TelepresenceDepthRangeFocus>();
        if (focus == null)
            focus = rig.gameObject.AddComponent<TelepresenceDepthRangeFocus>();
        foreach (var p in rig.GetComponentsInChildren<RsProcessingPipe>(true))
        {
            if (p.name.IndexOf("CameraA", System.StringComparison.OrdinalIgnoreCase) >= 0)
                focus.pipeA = p;
            if (p.name.IndexOf("CameraB", System.StringComparison.OrdinalIgnoreCase) >= 0)
                focus.pipeB = p;
        }

        EditorUtility.SetDirty(focus);

        var icp = rig.GetComponent<TelepresenceIcpAutoCalibrator>();
        if (icp == null)
            icp = rig.gameObject.AddComponent<TelepresenceIcpAutoCalibrator>();
        icp.rig = rig;
        if (rig.pointCloudRootA != null)
        {
            foreach (var mf in rig.pointCloudRootA.GetComponentsInChildren<MeshFilter>(true))
            {
                icp.meshFilterA = mf;
                break;
            }
        }

        if (rig.pointCloudRootB != null)
        {
            foreach (var mf in rig.pointCloudRootB.GetComponentsInChildren<MeshFilter>(true))
            {
                icp.meshFilterB = mf;
                break;
            }
        }

        EditorUtility.SetDirty(icp);

        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Debug.Log("Telepresence upgrade: RGB + depth clip + binding + ICP auto-calibrator applied. Save the scene.");
    }

    /// <summary>
    /// Entry point for batch mode: <c>-executeMethod MultiCameraPrdSceneBuilder.CreateSceneForBatchMode</c>
    /// </summary>
    public static void CreateSceneForBatchMode()
    {
        CreateSceneInternal();
        if (Application.isBatchMode)
            EditorApplication.Exit(0);
    }

    static void CreateSceneInternal()
    {
        EnsureDirectory("Assets/MultiCameraMeshReconstruction");
        EnsureDirectory("Assets/MultiCameraMeshReconstruction/Scenes");
        EnsureDirectory("Assets/MultiCameraMeshReconstruction/Editor");

        var devicePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(RsDevicePrefab);
        var pipePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(RsPipePrefab);
        var pcPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(PointCloudPrefab);
        var profile = AssetDatabase.LoadAssetAtPath<RsProcessingProfile>(DepthColorClippedProfile);

        if (devicePrefab == null || pipePrefab == null || pcPrefab == null || profile == null)
        {
            Debug.LogError("MultiCameraPrdSceneBuilder: Missing prefab or PointCloudRgbDepthClipped profile. Check paths.");
            return;
        }

        var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
        scene.name = "MultiCameraTelepresencePRD";

        var root = new GameObject("PRD_MultiCameraTelepresence");
        var rig = root.AddComponent<RsDualCameraPointCloudRig>();
        var desc = root.AddComponent<CalibrationSceneDescription>();
        desc.Notes = PrdSceneNotes;

        var rootA = new GameObject("PointCloudRoot_CameraA");
        rootA.transform.SetParent(root.transform, false);
        var rootB = new GameObject("PointCloudRoot_CameraB");
        rootB.transform.SetParent(root.transform, false);

        var devA = InstantiateDevice(devicePrefab, root.transform, "RsDevice_CameraA");
        var devB = InstantiateDevice(devicePrefab, root.transform, "RsDevice_CameraB");

        var pipeGoA = InstantiatePipe(pipePrefab, root.transform, "RsProcessingPipe_CameraA", devA.GetComponent<RsDevice>(), profile);
        var pipeGoB = InstantiatePipe(pipePrefab, root.transform, "RsProcessingPipe_CameraB", devB.GetComponent<RsDevice>(), profile);

        var pipeA = pipeGoA.GetComponent<RsProcessingPipe>();
        var pipeB = pipeGoB.GetComponent<RsProcessingPipe>();

        var pcA = InstantiatePointCloud(pcPrefab, rootA.transform, "PointCloud_CameraA", pipeA, devA.GetComponent<RsDevice>());
        var pcB = InstantiatePointCloud(pcPrefab, rootB.transform, "PointCloud_CameraB", pipeB, devB.GetComponent<RsDevice>());

        rig.cameraA = devA.GetComponent<RsDevice>();
        rig.cameraB = devB.GetComponent<RsDevice>();
        rig.pointCloudRootA = rootA.transform;
        rig.pointCloudRootB = rootB.transform;
        rig.cameraASerial = string.Empty;
        rig.cameraBSerial = string.Empty;
        rig.applyOnAwake = true;
        rig.useSymmetricTelepresenceLayout = true;
        rig.baselineMeters = 0.35f;
        rig.toeInYawDegrees = 12f;
        rig.cameraAIsLeft = true;
        rig.ApplyConfiguration();

        var focus = root.AddComponent<TelepresenceDepthRangeFocus>();
        focus.pipeA = pipeA;
        focus.pipeB = pipeB;
        EditorUtility.SetDirty(focus);

        var icp = root.AddComponent<TelepresenceIcpAutoCalibrator>();
        icp.rig = rig;
        icp.meshFilterA = pcA.GetComponent<MeshFilter>();
        icp.meshFilterB = pcB.GetComponent<MeshFilter>();
        EditorUtility.SetDirty(icp);

        SetupMainCameraForPointCloudView();

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene, ScenePath);
        AssetDatabase.Refresh();
        Debug.Log($"MultiCameraPrdSceneBuilder: Saved scene to {ScenePath}");
    }

    static void EnsureDirectory(string path)
    {
        path = path.Replace("\\", "/");
        if (AssetDatabase.IsValidFolder(path))
            return;
        var parent = System.IO.Path.GetDirectoryName(path)?.Replace("\\", "/");
        var leaf = System.IO.Path.GetFileName(path);
        if (string.IsNullOrEmpty(leaf))
            return;
        if (!string.IsNullOrEmpty(parent) && !AssetDatabase.IsValidFolder(parent))
            EnsureDirectory(parent);
        AssetDatabase.CreateFolder(string.IsNullOrEmpty(parent) ? "Assets" : parent, leaf);
    }

    static GameObject InstantiateDevice(GameObject prefab, Transform parent, string name)
    {
        var go = PrefabUtility.InstantiatePrefab(prefab, parent) as GameObject;
        if (go == null)
            return null;
        go.name = name;
        var rs = go.GetComponent<RsDevice>();
        if (rs != null)
        {
            var cfg = rs.DeviceConfiguration;
            cfg.Profiles = new[]
            {
                new RsVideoStreamRequest
                {
                    Stream = Intel.RealSense.Stream.Depth,
                    StreamIndex = -1,
                    Width = 640,
                    Height = 480,
                    Format = Format.Z16,
                    Framerate = 30
                },
                new RsVideoStreamRequest
                {
                    Stream = Intel.RealSense.Stream.Color,
                    StreamIndex = -1,
                    Width = 640,
                    Height = 480,
                    Format = Format.Rgb8,
                    Framerate = 30
                }
            };
            rs.DeviceConfiguration = cfg;
            EditorUtility.SetDirty(rs);
        }

        return go;
    }

    static GameObject InstantiatePipe(GameObject prefab, Transform parent, string name, RsDevice source, RsProcessingProfile processingProfile)
    {
        var go = PrefabUtility.InstantiatePrefab(prefab, parent) as GameObject;
        if (go == null)
            return null;
        go.name = name;
        var pipe = go.GetComponent<RsProcessingPipe>();
        if (pipe != null)
        {
            pipe.Source = source;
            pipe.profile = processingProfile;
            EditorUtility.SetDirty(pipe);
        }

        return go;
    }

    static GameObject InstantiatePointCloud(GameObject prefab, Transform parent, string name, RsProcessingPipe source, RsDevice deviceForRgb)
    {
        var go = PrefabUtility.InstantiatePrefab(prefab, parent) as GameObject;
        if (go == null)
            return null;
        go.name = name;
        var renderer = go.GetComponent<RsPointCloudRenderer>();
        if (renderer != null)
        {
            renderer.Source = source;
            EditorUtility.SetDirty(renderer);
        }

        if (deviceForRgb != null)
        {
            var bind = go.GetComponent<RsPointCloudColorTextureBinding>();
            if (bind == null)
                bind = go.AddComponent<RsPointCloudColorTextureBinding>();
            bind.device = deviceForRgb;
            bind.pointCloudMeshRenderer = go.GetComponent<MeshRenderer>();
            EditorUtility.SetDirty(bind);
        }

        return go;
    }

    static void SetupMainCameraForPointCloudView()
    {
        var cam = Camera.main;
        if (cam == null)
            return;
        cam.transform.position = new Vector3(0.35f, 0.2f, -0.45f);
        cam.transform.rotation = Quaternion.Euler(12f, -35f, 0f);
        cam.nearClipPlane = 0.01f;
        cam.farClipPlane = 20f;

        if (cam.GetComponent<OrbitCameraControl>() == null)
        {
            var orbit = cam.gameObject.AddComponent<OrbitCameraControl>();
            orbit.OrbitCursor = AssetDatabase.LoadAssetAtPath<Texture2D>(
                "Assets/RealSenseSDK2.0/Misc/Textures/Icons/look.png");
            orbit.PanCursor = AssetDatabase.LoadAssetAtPath<Texture2D>(
                "Assets/RealSenseSDK2.0/Misc/Textures/Icons/pan.png");
            orbit.ZoomCursor = AssetDatabase.LoadAssetAtPath<Texture2D>(
                "Assets/RealSenseSDK2.0/Misc/Textures/Icons/magnify.png");
            EditorUtility.SetDirty(orbit);
        }
    }

    const string PrdSceneNotes =
        "PRD: Multi-Camera 3D Mesh / Telepresence (Phase 1–2)\r\n\r\n" +
        "1) Connect two D455f on separate USB3 controllers. Optional: hardware sync (master/slave).\r\n" +
        "2) Set serial numbers: PRD_MultiCameraTelepresence → RsDualCameraPointCloudRig → Camera A/B Serial (from RealSense Viewer).\r\n" +
        "3) Symmetric telepresence: enable Use Symmetric Layout, set Baseline (meters), Toe-In Yaw (same on both cameras), Camera A Is Left.\r\n" +
        "   Or disable and set Position/Rotation B In A from Charuco calibration.\r\n" +
        "4) Background: depth band only (TelepresenceDepthRangeFocus). For person-only cutout use ML segmentation / green screen later.\r\n" +
        "5) Auto merge: Play → TelepresenceIcpAutoCalibrator → ⋮ Run automatic ICP (overlap both views on same scene). Warm start + symmetric layout helps.\r\n" +
        "6) Play: RGB point clouds (PointCloudRgbDepthClipped).\r\n\r\n" +
        "Next PRD phases (not in this scene yet): TSDF fusion, marching cubes mesh, .obj/.ply export.";
}
#endif
