using UnityEngine;

/// <summary>
/// Shown on the root object in calibration / telepresence scenes for workflow notes.
/// </summary>
public class CalibrationSceneDescription : MonoBehaviour
{
    [TextArea(6, 16)]
    public string Notes;
}
