using UnityEngine;

/// <summary>
/// Removes background by depth: only points between <see cref="minDistanceMeters"/> and
/// <see cref="maxDistanceMeters"/> are kept (via <see cref="RsThresholdFilter"/> on
/// <c>PointCloudRgbDepthClipped</c>). Lower max = walls/far room disappear; raise min = drop clutter near cameras.
/// </summary>
public class TelepresenceDepthRangeFocus : MonoBehaviour
{
    [Header("Pipes (usually RsProcessingPipe Camera A / B)")]
    public RsProcessingPipe pipeA;
    public RsProcessingPipe pipeB;

    [Header("Background removal — depth band (meters)")]
    [Tooltip("Hide anything closer than this (e.g. stand, fingers on lens).")]
    [Range(0.1f, 3f)]
    public float minDistanceMeters = 0.28f;

    [Tooltip("Hide anything farther than this — walls and room are beyond this distance. Try 1.2–2.0 for desk use.")]
    [Range(0.4f, 10f)]
    public float maxDistanceMeters = 1.55f;

    void OnValidate()
    {
        if (maxDistanceMeters < minDistanceMeters + 0.05f)
            maxDistanceMeters = minDistanceMeters + 0.05f;
    }

    void LateUpdate()
    {
        Apply(pipeA, minDistanceMeters, maxDistanceMeters);
        Apply(pipeB, minDistanceMeters, maxDistanceMeters);
    }

    static void Apply(RsProcessingPipe pipe, float minMeters, float maxMeters)
    {
        if (pipe == null || pipe.profile == null || pipe.profile._processingBlocks == null)
            return;

        foreach (var block in pipe.profile._processingBlocks)
        {
            if (block is RsThresholdFilter tf)
            {
                tf.MinDistance = minMeters;
                tf.MaxDistance = maxMeters;
            }
        }
    }
}
