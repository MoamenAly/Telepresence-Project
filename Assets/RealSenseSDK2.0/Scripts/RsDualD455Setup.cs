using System.Collections;
using System.Linq;
using Intel.RealSense;
using UnityEngine;

public class RsDualD455Setup : MonoBehaviour
{
    [Tooltip("Seconds to wait for both cameras before logging an error.")]
    public float startupTimeout = 10f;

    public bool BothStreaming { get; private set; }

    private RsDevice[] devices;
    private bool[] started;
    private float startTime;

    void Awake()
    {
        devices = GetComponentsInChildren<RsDevice>(true);

        if (devices.Length < 2)
        {
            Debug.LogError("[DualD455] Expected 2 RsDevice children but found " + devices.Length);
            enabled = false;
            return;
        }

        ValidateSerials();
        ValidateProfiles();

        started = new bool[devices.Length];
        for (int i = 0; i < devices.Length; i++)
        {
            int idx = i;
            devices[i].OnStart += profile => OnDeviceStarted(idx, profile);
        }

        startTime = Time.realtimeSinceStartup;
        StartCoroutine(CheckTimeout());
    }

    private void ValidateSerials()
    {
        string serialA = devices[0].DeviceConfiguration.RequestedSerialNumber;
        string serialB = devices[1].DeviceConfiguration.RequestedSerialNumber;

        if (string.IsNullOrEmpty(serialA) || string.IsNullOrEmpty(serialB))
            Debug.LogWarning("[DualD455] One or both RsDevice serial numbers are empty. " +
                             "Each device should target a specific serial to avoid conflicts.");

        if (serialA == serialB)
            Debug.LogError("[DualD455] Both RsDevice components target the same serial '" + serialA +
                           "'. Assign distinct serial numbers.");
    }

    private void ValidateProfiles()
    {
        var profA = devices[0].DeviceConfiguration.Profiles;
        var profB = devices[1].DeviceConfiguration.Profiles;

        if (profA == null || profB == null) return;

        foreach (var pa in profA)
        {
            var match = profB.FirstOrDefault(pb => pb.Stream == pa.Stream);
            if (match.Width == 0) continue;

            if (pa.Width != match.Width || pa.Height != match.Height)
                Debug.LogWarning($"[DualD455] Stream {pa.Stream} resolution mismatch: " +
                                 $"A={pa.Width}x{pa.Height}, B={match.Width}x{match.Height}");

            if (pa.Framerate != match.Framerate)
                Debug.LogWarning($"[DualD455] Stream {pa.Stream} FPS mismatch: " +
                                 $"A={pa.Framerate}, B={match.Framerate}");
        }
    }

    private void OnDeviceStarted(int index, PipelineProfile profile)
    {
        started[index] = true;
        string serial = "unknown";
        try { serial = profile.Device.Info.GetInfo(CameraInfo.SerialNumber); } catch { }
        Debug.Log($"[DualD455] Device {index} streaming (serial {serial})");

        if (started.All(s => s))
        {
            BothStreaming = true;
            float elapsed = Time.realtimeSinceStartup - startTime;
            Debug.Log($"[DualD455] Both D455 cameras streaming. Startup took {elapsed:F1}s");
        }
    }

    private IEnumerator CheckTimeout()
    {
        yield return new WaitForSeconds(startupTimeout);

        if (!BothStreaming)
        {
            for (int i = 0; i < devices.Length; i++)
            {
                if (!started[i])
                    Debug.LogError($"[DualD455] Device {i} (serial '{devices[i].DeviceConfiguration.RequestedSerialNumber}') " +
                                   "did not start within timeout. Check USB connection and serial number.");
            }
        }
    }
}
