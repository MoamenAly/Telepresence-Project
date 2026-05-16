using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Intel.RealSense;
using UnityEngine;

/// <summary>
/// Runtime health check for the dual D455 pipeline. Drop on the TelepresenceRig
/// (or any GameObject in the Main scene) and watch the Console + on-screen HUD
/// to confirm every link in the chain is alive before starting a session.
///
/// Press F12 (configurable) to toggle the HUD.
/// </summary>
public class RsDualDiagnostics : MonoBehaviour
{
    [Header("References (auto-discovered if empty)")]
    public RsDualCameraPointCloudRig rig;
    public RsDualCameraExtrinsicsCalibration calibration;
    public RsDualPointCloudFusionManager fusion;
    public RsDualD455Setup setup;

    [Header("Display")]
    public bool showHud = true;
    public KeyCode toggleKey = KeyCode.F12;
    public Vector2 hudPosition = new Vector2(10, 110);
    public int hudWidth = 520;
    public int hudFontSize = 12;

    [Header("Polling")]
    [Tooltip("How often to re-enumerate connected cameras (seconds). " +
             "0 = every frame (slower, native call).")]
    public float deviceQueryInterval = 2f;

    private string dllStatus = "?";
    private string dllError;
    private string connectedSerials = "?";
    private List<string> connectedList = new List<string>();
    private float nextQueryTime;

    private string jsonPath;
    private bool jsonExists;
    private string jsonError;

    private GUIStyle hudStyle;

    void Awake()
    {
        AutoDiscover();
        jsonPath = Path.Combine(Application.dataPath, "..", "DualD455Extrinsics.json");

        ProbeDll();
        QueryConnectedCameras();
        ProbeJson();
        LogStartupSummary();
    }

    void Update()
    {
        if (Input.GetKeyDown(toggleKey))
            showHud = !showHud;

        if (deviceQueryInterval <= 0f || Time.realtimeSinceStartup >= nextQueryTime)
        {
            nextQueryTime = Time.realtimeSinceStartup + Mathf.Max(0.01f, deviceQueryInterval);
            QueryConnectedCameras();
        }
    }

    // -----------------------------------------------------------------
    // Probes
    // -----------------------------------------------------------------

    private void AutoDiscover()
    {
        if (rig == null)
            rig = GetComponent<RsDualCameraPointCloudRig>() ??
                  FindFirstObjectByType<RsDualCameraPointCloudRig>();

        if (calibration == null)
            calibration = GetComponent<RsDualCameraExtrinsicsCalibration>() ??
                          FindFirstObjectByType<RsDualCameraExtrinsicsCalibration>();

        if (fusion == null)
            fusion = FindFirstObjectByType<RsDualPointCloudFusionManager>();

        if (setup == null)
            setup = GetComponent<RsDualD455Setup>() ??
                    FindFirstObjectByType<RsDualD455Setup>();
    }

    /// <summary>
    /// Touch the native DLL by allocating a Context. If realsense2.dll or
    /// Intel.RealSense.dll are missing this will throw and we report it.
    /// </summary>
    private void ProbeDll()
    {
        try
        {
            using (var _ = new Context())
            {
                dllStatus = "OK";
                dllError = null;
            }
        }
        catch (DllNotFoundException e)
        {
            dllStatus = "MISSING";
            dllError = e.Message + "  -> run CopyRealSenseDlls.bat";
        }
        catch (Exception e)
        {
            dllStatus = "ERROR";
            dllError = e.GetType().Name + ": " + e.Message;
        }
    }

    private void QueryConnectedCameras()
    {
        connectedList.Clear();

        if (dllStatus != "OK")
        {
            connectedSerials = "(DLL not loaded)";
            return;
        }

        try
        {
            using (var ctx = new Context())
            {
                var list = ctx.QueryDevices();
                int count = list.Count;
                for (int i = 0; i < count; i++)
                {
                    try
                    {
                        var dev = list[i];
                        string serial = dev.Info.GetInfo(CameraInfo.SerialNumber);
                        connectedList.Add(serial);
                    }
                    catch
                    {
                        // ignore single-device failures
                    }
                }
            }

            connectedSerials = connectedList.Count == 0
                ? "(none)"
                : string.Join(", ", connectedList);
        }
        catch (Exception e)
        {
            connectedSerials = "ERROR: " + e.Message;
        }
    }

    private void ProbeJson()
    {
        if (!File.Exists(jsonPath))
        {
            jsonExists = false;
            jsonError = "file not found";
            return;
        }

        try
        {
            string json = File.ReadAllText(jsonPath);
            if (string.IsNullOrWhiteSpace(json))
            {
                jsonExists = false;
                jsonError = "file is empty";
                return;
            }
            jsonExists = true;
            jsonError = null;
        }
        catch (Exception e)
        {
            jsonExists = false;
            jsonError = e.Message;
        }
    }

    // -----------------------------------------------------------------
    // Derived state
    // -----------------------------------------------------------------

    private string RequestedSerial(int index)
    {
        if (rig == null) return "(no rig)";
        var dev = index == 0 ? rig.deviceA : rig.deviceB;
        if (dev == null) return "(unassigned)";
        var s = dev.DeviceConfiguration.RequestedSerialNumber;
        return string.IsNullOrEmpty(s) ? "(empty)" : s;
    }

    private bool RequestedSerialIsConnected(int index)
    {
        var s = RequestedSerial(index);
        return !string.IsNullOrEmpty(s) && connectedList.Contains(s);
    }

    private bool ProfilesMatch()
    {
        if (rig == null || rig.deviceA == null || rig.deviceB == null) return false;
        var pa = rig.deviceA.DeviceConfiguration.Profiles;
        var pb = rig.deviceB.DeviceConfiguration.Profiles;
        if (pa == null || pb == null) return false;
        if (pa.Length != pb.Length) return false;

        for (int i = 0; i < pa.Length; i++)
        {
            var match = pb.FirstOrDefault(p => p.Stream == pa[i].Stream &&
                                               p.StreamIndex == pa[i].StreamIndex);
            if (match.Width == 0) return false;
            if (match.Width != pa[i].Width ||
                match.Height != pa[i].Height ||
                match.Framerate != pa[i].Framerate)
                return false;
        }
        return true;
    }

    private int FusedVertexCount()
    {
        if (fusion == null) return -1;
        var mf = fusion.GetComponent<MeshFilter>();
        if (mf == null || mf.sharedMesh == null) return -1;
        return mf.sharedMesh.vertexCount;
    }

    // -----------------------------------------------------------------
    // Output
    // -----------------------------------------------------------------

    private void LogStartupSummary()
    {
        Debug.Log("[Diag] ===== RsDualDiagnostics =====");
        Debug.Log($"[Diag] Native DLL: {dllStatus}{(dllError == null ? "" : " (" + dllError + ")")}");
        Debug.Log($"[Diag] Connected cameras: {connectedSerials}");
        Debug.Log($"[Diag] Requested A={RequestedSerial(0)} (connected={RequestedSerialIsConnected(0)})");
        Debug.Log($"[Diag] Requested B={RequestedSerial(1)} (connected={RequestedSerialIsConnected(1)})");
        Debug.Log($"[Diag] Profiles match: {ProfilesMatch()}");
        Debug.Log($"[Diag] Calibration JSON: {jsonPath}");
        Debug.Log($"[Diag] Calibration JSON status: {(jsonExists ? "OK" : "MISSING - " + jsonError)}");

        if (calibration != null)
        {
            Debug.Log($"[Diag] Loaded extrinsics: T=({calibration.translationOffset.x:F3}, " +
                      $"{calibration.translationOffset.y:F3}, {calibration.translationOffset.z:F3}) " +
                      $"R=({calibration.rotationEulerOffset.x:F2}, " +
                      $"{calibration.rotationEulerOffset.y:F2}, " +
                      $"{calibration.rotationEulerOffset.z:F2})");
        }

        if (setup != null)
            Debug.Log($"[Diag] Setup startupTimeout={setup.startupTimeout}s");
    }

    void OnGUI()
    {
        if (!showHud) return;

        if (hudStyle == null)
        {
            hudStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = hudFontSize,
                richText = true,
                alignment = TextAnchor.UpperLeft,
                wordWrap = true,
            };
            hudStyle.normal.background = MakeBackground(new Color(0, 0, 0, 0.55f));
            hudStyle.padding = new RectOffset(8, 8, 6, 6);
        }

        var sb = new System.Text.StringBuilder();
        sb.Append("<b>[Dual D455 Diagnostics]</b>   F12 to toggle\n");

        AppendStatus(sb, "Native DLL", dllStatus == "OK", dllStatus, dllError);

        bool deviceCountOk = connectedList.Count >= 2;
        AppendStatus(sb, "Cameras connected",
            deviceCountOk,
            connectedList.Count + (connectedList.Count > 0 ? " [" + connectedSerials + "]" : ""),
            connectedList.Count == 0 ? "no D455 detected on USB" :
            connectedList.Count == 1 ? "only 1 D455 - check second USB cable / port" : null);

        string serialA = RequestedSerial(0);
        string serialB = RequestedSerial(1);
        AppendStatus(sb, "Serial A",
            RequestedSerialIsConnected(0),
            serialA,
            !RequestedSerialIsConnected(0) && deviceCountOk
                ? "scene requests a serial that isn't on the bus" : null);
        AppendStatus(sb, "Serial B",
            RequestedSerialIsConnected(1),
            serialB,
            !RequestedSerialIsConnected(1) && deviceCountOk
                ? "scene requests a serial that isn't on the bus" : null);

        AppendStatus(sb, "Profiles match", ProfilesMatch(),
            ProfilesMatch() ? "yes" : "A vs B differ",
            null);

        bool streaming = setup != null && setup.BothStreaming;
        AppendStatus(sb, "Both streaming", streaming,
            streaming ? "yes" : "waiting...",
            null);

        AppendStatus(sb, "Calibration JSON", jsonExists,
            jsonExists ? jsonPath : "(missing)",
            jsonError);

        if (calibration != null)
        {
            sb.AppendFormat("  loaded T=({0:F3}, {1:F3}, {2:F3})  R=({3:F2}, {4:F2}, {5:F2})\n",
                calibration.translationOffset.x, calibration.translationOffset.y, calibration.translationOffset.z,
                calibration.rotationEulerOffset.x, calibration.rotationEulerOffset.y, calibration.rotationEulerOffset.z);
        }

        if (rig != null && rig.deviceBTransform != null)
        {
            var p = rig.deviceBTransform.localPosition;
            var r = rig.deviceBTransform.localEulerAngles;
            sb.AppendFormat("  RsDevice_B live   T=({0:F3}, {1:F3}, {2:F3})  R=({3:F1}, {4:F1}, {5:F1})\n",
                p.x, p.y, p.z, r.x, r.y, r.z);
        }

        int fused = FusedVertexCount();
        AppendStatus(sb, "Fused mesh",
            fused > 0,
            fused < 0 ? "no MeshFilter" : fused.ToString() + " verts",
            null);

        var rect = new Rect(hudPosition.x, hudPosition.y, hudWidth, 1);
        rect.height = hudStyle.CalcHeight(new GUIContent(sb.ToString()), hudWidth);
        GUI.Label(rect, sb.ToString(), hudStyle);
    }

    private static void AppendStatus(System.Text.StringBuilder sb, string label,
                                     bool ok, string value, string hint)
    {
        string tag = ok ? "<color=#7CFC7C>[OK]  </color>" : "<color=#FF6464>[FAIL]</color>";
        sb.Append(tag).Append(' ').Append(label).Append(": ").Append(value);
        if (!string.IsNullOrEmpty(hint))
            sb.Append("  <color=#FFC864>(").Append(hint).Append(")</color>");
        sb.Append('\n');
    }

    private static Texture2D MakeBackground(Color c)
    {
        var t = new Texture2D(1, 1);
        t.SetPixel(0, 0, c);
        t.Apply();
        return t;
    }
}
