using Intel.RealSense;
using UnityEngine;

/// <summary>
/// Binds the RealSense color stream to the point cloud material's main texture so points show camera RGB instead of white.
/// Expects an <see cref="RsDevice"/> prefab with an active child named "Color" that has <see cref="RsStreamTextureRenderer"/>.
/// </summary>
[DefaultExecutionOrder(50)]
public class RsPointCloudColorTextureBinding : MonoBehaviour
{
    [Tooltip("Device that streams depth + color for this point cloud.")]
    public RsDevice device;

    [Tooltip("Renderer using Custom/PointCloud material. Defaults to this GameObject.")]
    public MeshRenderer pointCloudMeshRenderer;

    void Reset()
    {
        pointCloudMeshRenderer = GetComponent<MeshRenderer>();
    }

    void Start()
    {
        if (device == null || pointCloudMeshRenderer == null)
            return;

        var colorTf = device.transform.Find("Color");
        if (colorTf == null)
        {
            Debug.LogWarning($"{name}: RsDevice has no child 'Color'. Point cloud stays white without RGB binding.");
            return;
        }

        colorTf.gameObject.SetActive(true);
        var streamR = colorTf.GetComponent<RsStreamTextureRenderer>();
        if (streamR == null)
            return;

        streamR.Source = device;
        streamR._stream = Stream.Color;
        streamR._format = Format.Rgb8;
        streamR._streamIndex = -1;
        streamR.textureBinding.RemoveAllListeners();
        var mat = pointCloudMeshRenderer.material;
        streamR.textureBinding.AddListener(t => mat.mainTexture = t);
    }
}
