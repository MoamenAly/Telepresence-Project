using UnityEngine;

public class RsDualCalibrationAssist : MonoBehaviour
{
    [Header("Step Sizes")]
    public float fineTranslation = 0.001f;
    public float mediumTranslation = 0.005f;
    public float coarseTranslation = 0.01f;

    public float fineRotation = 0.1f;
    public float mediumRotation = 0.5f;
    public float coarseRotation = 1.0f;

    [Header("On-Screen Display")]
    public bool showOverlay = true;

    private RsDualCameraExtrinsicsCalibration calibration;

    void Awake()
    {
        calibration = GetComponent<RsDualCameraExtrinsicsCalibration>();
        if (calibration == null)
        {
            Debug.LogError("[CalibAssist] RsDualCameraExtrinsicsCalibration not found.", this);
            enabled = false;
        }
    }

    void Update()
    {
        if (calibration == null) return;

        float tStep = mediumTranslation;
        float rStep = mediumRotation;

        if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
        {
            tStep = fineTranslation;
            rStep = fineRotation;
        }
        else if (Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl))
        {
            tStep = coarseTranslation;
            rStep = coarseRotation;
        }

        HandleTranslation(tStep);
        HandleRotation(rStep);
    }

    private void HandleTranslation(float step)
    {
        Vector3 delta = Vector3.zero;

        if (Input.GetKeyDown(KeyCode.D)) delta.x += step;
        if (Input.GetKeyDown(KeyCode.A)) delta.x -= step;
        if (Input.GetKeyDown(KeyCode.E)) delta.y += step;
        if (Input.GetKeyDown(KeyCode.Q)) delta.y -= step;
        if (Input.GetKeyDown(KeyCode.W)) delta.z += step;
        if (Input.GetKeyDown(KeyCode.S)) delta.z -= step;

        if (delta != Vector3.zero)
            calibration.translationOffset += delta;
    }

    private void HandleRotation(float step)
    {
        Vector3 delta = Vector3.zero;

        if (Input.GetKeyDown(KeyCode.Keypad8)) delta.x += step;  // pitch up
        if (Input.GetKeyDown(KeyCode.Keypad2)) delta.x -= step;  // pitch down
        if (Input.GetKeyDown(KeyCode.Keypad6)) delta.y += step;  // yaw right
        if (Input.GetKeyDown(KeyCode.Keypad4)) delta.y -= step;  // yaw left
        if (Input.GetKeyDown(KeyCode.Keypad9)) delta.z += step;  // roll CW
        if (Input.GetKeyDown(KeyCode.Keypad7)) delta.z -= step;  // roll CCW

        if (delta != Vector3.zero)
            calibration.rotationEulerOffset += delta;
    }

    void OnGUI()
    {
        if (!showOverlay || calibration == null) return;

        var t = calibration.translationOffset;
        var r = calibration.rotationEulerOffset;

        string mode = "Medium";
        if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift)) mode = "Fine";
        else if (Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl)) mode = "Coarse";

        string text = $"[Calibration] Mode: {mode}\n" +
                      $"Pos: ({t.x:F4}, {t.y:F4}, {t.z:F4})\n" +
                      $"Rot: ({r.x:F2}, {r.y:F2}, {r.z:F2})\n" +
                      "WASDQE=Translate  Numpad=Rotate  F5=Save  F6=AutoAlign  F7=Undo";

        GUI.Label(new Rect(10, 10, 500, 80), text);
    }
}
