using UnityEngine;

/// <summary>
/// Binds incoming textures (from RsStreamTextureRenderer events) to a renderer material.
/// </summary>
public class RsMaterialTextureBinder : MonoBehaviour
{
    public Renderer targetRenderer;
    public string textureProperty = "_MainTex";

    public void SetTexture(Texture texture)
    {
        if (targetRenderer == null || texture == null)
            return;

        targetRenderer.material.SetTexture(textureProperty, texture);
    }
}
