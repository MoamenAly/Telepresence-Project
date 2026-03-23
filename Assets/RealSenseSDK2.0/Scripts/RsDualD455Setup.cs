using System;
using System.Collections.Generic;
using Intel.RealSense;
using UnityEngine;

/// <summary>
/// Utility component for fast setup of two connected D455 cameras.
/// Attach it to the same GameObject as RsDualCameraPointCloudRig.
/// </summary>
public class RsDualD455Setup : MonoBehaviour
{
    [Header("Rig")]
    public RsDualCameraPointCloudRig rig;

    [Header("Recommended D455 Streams")]
    [Min(1)] public int depthWidth = 640;
    [Min(1)] public int depthHeight = 480;
    [Min(1)] public int colorWidth = 640;
    [Min(1)] public int colorHeight = 480;
    [Min(1)] public int fps = 30;

    [Header("Startup")]
    [Tooltip("If true, configure serials and stream profiles on Awake.")]
    public bool applyOnAwake;

    private void Awake()
    {
        if (applyOnAwake)
            AutoConfigure();
    }

    [ContextMenu("Auto Configure Dual D455")]
    public void AutoConfigure()
    {
        if (rig == null)
            rig = GetComponent<RsDualCameraPointCloudRig>();

        if (rig == null)
        {
            Debug.LogError("RsDualD455Setup: missing RsDualCameraPointCloudRig reference.");
            return;
        }

        var serials = GetConnectedSerials();
        if (serials.Count < 2)
        {
            Debug.LogError("RsDualD455Setup: fewer than 2 RealSense devices detected.");
            return;
        }

        rig.cameraASerial = serials[0];
        rig.cameraBSerial = serials[1];
        rig.ApplySerials();

        ApplyRecommendedProfiles(rig.cameraA);
        ApplyRecommendedProfiles(rig.cameraB);

        Debug.Log(string.Format(
            "RsDualD455Setup: configured A={0}, B={1}, profile Depth({2}x{3}@{4}) + Color({5}x{6}@{4}).",
            rig.cameraASerial,
            rig.cameraBSerial,
            depthWidth,
            depthHeight,
            fps,
            colorWidth,
            colorHeight
        ));
    }

    [ContextMenu("Apply Recommended Profiles Only")]
    public void ApplyProfilesOnly()
    {
        if (rig == null)
            rig = GetComponent<RsDualCameraPointCloudRig>();

        if (rig == null)
        {
            Debug.LogError("RsDualD455Setup: missing RsDualCameraPointCloudRig reference.");
            return;
        }

        ApplyRecommendedProfiles(rig.cameraA);
        ApplyRecommendedProfiles(rig.cameraB);
        Debug.Log("RsDualD455Setup: applied recommended profiles.");
    }

    [ContextMenu("Print Connected RealSense Serials")]
    public void PrintConnectedSerials()
    {
        var serials = GetConnectedSerials();
        if (serials.Count == 0)
        {
            Debug.LogWarning("RsDualD455Setup: no connected RealSense devices found.");
            return;
        }

        Debug.Log("RsDualD455Setup: connected serials = " + string.Join(", ", serials.ToArray()));
    }

    private List<string> GetConnectedSerials()
    {
        var serials = new List<string>();

        try
        {
            using (var ctx = new Context())
            using (var devices = ctx.QueryDevices())
            {
                foreach (var dev in devices)
                {
                    try
                    {
                        var serial = dev.Info[CameraInfo.SerialNumber];
                        if (!string.IsNullOrEmpty(serial))
                            serials.Add(serial);
                    }
                    finally
                    {
                        if (dev != null)
                            dev.Dispose();
                    }
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogException(e);
        }

        return serials;
    }

    private void ApplyRecommendedProfiles(RsDevice device)
    {
        if (device == null)
            return;

        device.DeviceConfiguration.mode = RsConfiguration.Mode.Live;
        device.DeviceConfiguration.Profiles = new[]
        {
            new RsVideoStreamRequest
            {
                Stream = Stream.Depth,
                StreamIndex = -1,
                Width = depthWidth,
                Height = depthHeight,
                Format = Format.Z16,
                Framerate = fps
            },
            new RsVideoStreamRequest
            {
                Stream = Stream.Color,
                StreamIndex = -1,
                Width = colorWidth,
                Height = colorHeight,
                Format = Format.Rgb8,
                Framerate = fps
            }
        };
    }
}
