using UnityEngine;

[DefaultExecutionOrder(100)]
public class RsPointCloudRoiFilter : MonoBehaviour
{
    [Header("Depth Range (meters, in camera local Z)")]
    public float minDepth = 0.3f;
    public float maxDepth = 2.5f;

    [Header("World-Space Bounding Box")]
    [Tooltip("Enable to also filter by a world-space bounding box.")]
    public bool useBoundsFilter = false;
    public Bounds roiBounds = new Bounds(Vector3.zero, new Vector3(3f, 3f, 3f));

    private MeshFilter meshFilter;
    private Vector3[] vertices;

    void Awake()
    {
        meshFilter = GetComponent<MeshFilter>();
        if (meshFilter == null)
        {
            Debug.LogError("[ROI] MeshFilter not found on " + gameObject.name, this);
            enabled = false;
        }
    }

    void LateUpdate()
    {
        var mesh = meshFilter != null ? meshFilter.sharedMesh : null;
        if (mesh == null || mesh.vertexCount == 0) return;

        vertices = mesh.vertices;
        bool modified = false;

        Matrix4x4 localToWorld = useBoundsFilter ? transform.localToWorldMatrix : Matrix4x4.identity;

        for (int i = 0; i < vertices.Length; i++)
        {
            var v = vertices[i];
            if (v.x == 0 && v.y == 0 && v.z == 0)
                continue;

            float depth = v.z;

            if (depth < minDepth || depth > maxDepth)
            {
                vertices[i] = Vector3.zero;
                modified = true;
                continue;
            }

            if (useBoundsFilter)
            {
                Vector3 worldPos = localToWorld.MultiplyPoint3x4(v);
                if (!roiBounds.Contains(worldPos))
                {
                    vertices[i] = Vector3.zero;
                    modified = true;
                }
            }
        }

        if (modified)
        {
            mesh.vertices = vertices;
            mesh.UploadMeshData(false);
        }
    }
}
