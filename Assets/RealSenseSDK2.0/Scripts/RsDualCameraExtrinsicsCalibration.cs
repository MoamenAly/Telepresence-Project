using System;
using System.IO;
using UnityEngine;

public class RsDualCameraExtrinsicsCalibration : MonoBehaviour
{
    [Header("Extrinsics (Camera B relative to Camera A)")]
    public Vector3 translationOffset;
    public Vector3 rotationEulerOffset;

    [Header("Persistence")]
    [Tooltip("Automatically load DualD455Extrinsics.json on Awake if it exists.")]
    public bool loadExtrinsicsOnAwake = true;

    private RsDualCameraPointCloudRig rig;
    private string JsonPath => Path.Combine(Application.dataPath, "..", "DualD455Extrinsics.json");

    void Awake()
    {
        rig = GetComponent<RsDualCameraPointCloudRig>();
        if (rig == null)
        {
            Debug.LogError("[Extrinsics] RsDualCameraPointCloudRig not found on this GameObject.", this);
            enabled = false;
            return;
        }

        if (loadExtrinsicsOnAwake)
            LoadExtrinsics();
    }

    void LateUpdate()
    {
        ApplyExtrinsics();

        if (Input.GetKeyDown(KeyCode.F5))
        {
            SaveExtrinsics();
            Debug.Log("[Extrinsics] Saved to " + JsonPath);
        }
    }

    public void ApplyExtrinsics()
    {
        if (rig == null || rig.deviceBTransform == null) return;

        rig.deviceBTransform.localPosition = translationOffset;
        rig.deviceBTransform.localRotation = Quaternion.Euler(rotationEulerOffset);
    }

    public void SaveExtrinsics()
    {
        var data = new ExtrinsicsData
        {
            tx = translationOffset.x,
            ty = translationOffset.y,
            tz = translationOffset.z,
            rx = rotationEulerOffset.x,
            ry = rotationEulerOffset.y,
            rz = rotationEulerOffset.z
        };
        string json = JsonUtility.ToJson(data, true);
        File.WriteAllText(JsonPath, json);
    }

    public void LoadExtrinsics()
    {
        if (!File.Exists(JsonPath))
        {
            Debug.Log("[Extrinsics] No saved extrinsics file found at " + JsonPath + ", using scene values.");
            return;
        }

        try
        {
            string json = File.ReadAllText(JsonPath);

            // Try the flat format first (from Python tool or new F5 save)
            var flat = JsonUtility.FromJson<ExtrinsicsData>(json);
            if (flat.tx != 0 || flat.ty != 0 || flat.tz != 0 ||
                flat.rx != 0 || flat.ry != 0 || flat.rz != 0)
            {
                translationOffset = new Vector3(flat.tx, flat.ty, flat.tz);
                rotationEulerOffset = new Vector3(flat.rx, flat.ry, flat.rz);
                Debug.Log($"[Extrinsics] Loaded from {JsonPath} " +
                          $"T=({flat.tx:F3},{flat.ty:F3},{flat.tz:F3}) " +
                          $"R=({flat.rx:F1},{flat.ry:F1},{flat.rz:F1})");
                return;
            }

            // Fall back to legacy nested format (positionBInA / rotationBInAEuler)
            var legacy = JsonUtility.FromJson<LegacyExtrinsicsData>(json);
            if (legacy.positionBInA.x != 0 || legacy.positionBInA.y != 0 || legacy.positionBInA.z != 0 ||
                legacy.rotationBInAEuler.x != 0 || legacy.rotationBInAEuler.y != 0 || legacy.rotationBInAEuler.z != 0)
            {
                translationOffset = legacy.positionBInA;
                rotationEulerOffset = legacy.rotationBInAEuler;
                Debug.Log($"[Extrinsics] Loaded legacy format from {JsonPath} " +
                          $"T=({legacy.positionBInA.x:F3},{legacy.positionBInA.y:F3},{legacy.positionBInA.z:F3}) " +
                          $"R=({legacy.rotationBInAEuler.x:F1},{legacy.rotationBInAEuler.y:F1},{legacy.rotationBInAEuler.z:F1})");
                return;
            }

            Debug.LogWarning("[Extrinsics] JSON loaded but all values are zero: " + JsonPath);
        }
        catch (Exception e)
        {
            Debug.LogError("[Extrinsics] Failed to load JSON: " + e.Message);
        }
    }

    [Serializable]
    private struct ExtrinsicsData
    {
        public float tx, ty, tz;
        public float rx, ry, rz;
    }

    [Serializable]
    private struct LegacyExtrinsicsData
    {
        public Vector3 positionBInA;
        public Vector3 rotationBInAEuler;
    }
}
