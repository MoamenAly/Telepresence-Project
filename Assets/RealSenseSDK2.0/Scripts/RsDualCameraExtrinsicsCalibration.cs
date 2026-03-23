using System;
using System.IO;
using UnityEngine;

/// <summary>
/// Lightweight calibration helper for RsDualCameraPointCloudRig extrinsics.
/// Uses context menu actions to nudge camera B pose in camera A frame and persist values.
/// </summary>
public class RsDualCameraExtrinsicsCalibration : MonoBehaviour
{
    [Serializable]
    private class CalibrationData
    {
        public Vector3 positionBInA;
        public Vector3 rotationBInAEuler;
        public string cameraASerial;
        public string cameraBSerial;
        public string savedAtUtc;
    }

    [Header("Rig")]
    public RsDualCameraPointCloudRig rig;

    [Header("Nudge Steps")]
    [Min(0.0001f)] public float positionStepMeters = 0.005f;
    [Min(0.01f)] public float rotationStepDegrees = 0.25f;

    [Header("Persistence")]
    [Tooltip("Optional filename relative to project root. Example: DualD455Extrinsics.json")]
    public string fileName = "DualD455Extrinsics.json";

    private string FilePath
    {
        get
        {
            var projectRoot = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
            return Path.Combine(projectRoot, fileName);
        }
    }

    [ContextMenu("Sync From Rig")]
    public void SyncFromRig()
    {
        if (!EnsureRig())
            return;

        rig.ApplyTransforms();
        Debug.Log("RsDualCameraExtrinsicsCalibration: synced and applied current rig transform.");
    }

    [ContextMenu("Save Extrinsics To JSON")]
    public void SaveToJson()
    {
        if (!EnsureRig())
            return;

        var data = new CalibrationData
        {
            positionBInA = rig.positionBInA,
            rotationBInAEuler = rig.rotationBInAEuler,
            cameraASerial = rig.cameraASerial,
            cameraBSerial = rig.cameraBSerial,
            savedAtUtc = DateTime.UtcNow.ToString("o")
        };

        var json = JsonUtility.ToJson(data, true);
        File.WriteAllText(FilePath, json);
        Debug.Log("RsDualCameraExtrinsicsCalibration: saved " + FilePath);
    }

    [ContextMenu("Load Extrinsics From JSON")]
    public void LoadFromJson()
    {
        if (!EnsureRig())
            return;

        if (!File.Exists(FilePath))
        {
            Debug.LogWarning("RsDualCameraExtrinsicsCalibration: file not found: " + FilePath);
            return;
        }

        var json = File.ReadAllText(FilePath);
        var data = JsonUtility.FromJson<CalibrationData>(json);
        if (data == null)
        {
            Debug.LogError("RsDualCameraExtrinsicsCalibration: failed to parse calibration json.");
            return;
        }

        rig.positionBInA = data.positionBInA;
        rig.rotationBInAEuler = data.rotationBInAEuler;
        rig.ApplyTransforms();
        Debug.Log("RsDualCameraExtrinsicsCalibration: loaded and applied " + FilePath);
    }

    [ContextMenu("Position +X")]
    public void PosPlusX() { NudgePosition(Vector3.right * positionStepMeters); }
    [ContextMenu("Position -X")]
    public void PosMinusX() { NudgePosition(Vector3.left * positionStepMeters); }
    [ContextMenu("Position +Y")]
    public void PosPlusY() { NudgePosition(Vector3.up * positionStepMeters); }
    [ContextMenu("Position -Y")]
    public void PosMinusY() { NudgePosition(Vector3.down * positionStepMeters); }
    [ContextMenu("Position +Z")]
    public void PosPlusZ() { NudgePosition(Vector3.forward * positionStepMeters); }
    [ContextMenu("Position -Z")]
    public void PosMinusZ() { NudgePosition(Vector3.back * positionStepMeters); }

    [ContextMenu("Rotate +Pitch(X)")]
    public void RotPlusPitch() { NudgeRotation(new Vector3(rotationStepDegrees, 0f, 0f)); }
    [ContextMenu("Rotate -Pitch(X)")]
    public void RotMinusPitch() { NudgeRotation(new Vector3(-rotationStepDegrees, 0f, 0f)); }
    [ContextMenu("Rotate +Yaw(Y)")]
    public void RotPlusYaw() { NudgeRotation(new Vector3(0f, rotationStepDegrees, 0f)); }
    [ContextMenu("Rotate -Yaw(Y)")]
    public void RotMinusYaw() { NudgeRotation(new Vector3(0f, -rotationStepDegrees, 0f)); }
    [ContextMenu("Rotate +Roll(Z)")]
    public void RotPlusRoll() { NudgeRotation(new Vector3(0f, 0f, rotationStepDegrees)); }
    [ContextMenu("Rotate -Roll(Z)")]
    public void RotMinusRoll() { NudgeRotation(new Vector3(0f, 0f, -rotationStepDegrees)); }

    private bool EnsureRig()
    {
        if (rig == null)
            rig = GetComponent<RsDualCameraPointCloudRig>();

        if (rig == null)
        {
            Debug.LogError("RsDualCameraExtrinsicsCalibration: missing RsDualCameraPointCloudRig reference.");
            return false;
        }

        return true;
    }

    private void NudgePosition(Vector3 delta)
    {
        if (!EnsureRig())
            return;

        rig.positionBInA += delta;
        rig.ApplyTransforms();
        LogCurrent();
    }

    private void NudgeRotation(Vector3 deltaEuler)
    {
        if (!EnsureRig())
            return;

        rig.rotationBInAEuler += deltaEuler;
        rig.ApplyTransforms();
        LogCurrent();
    }

    private void LogCurrent()
    {
        Debug.Log(string.Format(
            "RsDualCameraExtrinsicsCalibration: positionBInA={0}, rotationBInAEuler={1}",
            rig.positionBInA,
            rig.rotationBInAEuler
        ));
    }
}
