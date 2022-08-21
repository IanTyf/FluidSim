using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rendering : MonoBehaviour
{
    private MeshRenderer mr;
    private Texture2D tex;

    private FluidSim fs;

    // Start is called before the first frame update
    void Start()
    {
        mr = GetComponent<MeshRenderer>();
        tex = new Texture2D(128, 128);
        drawTexture(1f, 0.2f, 0.3f);
        mr.material.mainTexture = tex;

        fs = GetComponent<FluidSim>();
    }

    // Update is called once per frame
    void LateUpdate()
    {
        //drawTexture(Random.Range(0f, 1f), 0.2f, 0.3f);
        drawTexture(fs.colors);
    }

    private void drawTexture(float r, float g, float b)
    {
        for (int i=0; i<tex.height; i++)
        {
            for (int j=0; j<tex.width; j++)
            {
                tex.SetPixel(j, i, new Color(r, g, b));
            }
        }
        tex.Apply();
    }

    private void drawTexture(Color[] colors)
    {
        tex.SetPixels(colors);
        tex.Apply();
    }
}
