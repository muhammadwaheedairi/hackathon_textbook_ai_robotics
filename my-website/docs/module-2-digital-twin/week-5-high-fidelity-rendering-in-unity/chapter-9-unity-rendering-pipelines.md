---
title: "Unity Rendering Pipelines"
sidebar_label: "Chapter 9: Unity Rendering"
sidebar_position: 9
---

# Chapter 9: Unity Rendering Pipelines

## Overview

This chapter introduces Unity's rendering pipelines for creating photorealistic robot simulation environments. You'll learn about physically-based rendering, lighting systems, and how to configure Unity for high-fidelity visual simulation.

## Learning Objectives

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Understand Unity's rendering pipeline options (URP, HDRP)
- Implement physically-based rendering (PBR) materials
- Configure advanced lighting systems for realistic environments
- Optimize rendering performance for real-time applications
:::

## Unity Rendering Pipelines

Unity offers three main rendering pipelines optimized for different use cases: Built-in Render Pipeline, Universal Render Pipeline (URP), and High Definition Render Pipeline (HDRP).

### Built-in Render Pipeline

The legacy rendering pipeline that offers basic rendering capabilities and is suitable for simple applications. While still functional, it lacks many of the advanced features available in the newer pipelines.

### Universal Render Pipeline (URP)

A lightweight, flexible rendering pipeline designed for performance across a wide range of platforms. URP provides a good balance between visual quality and performance, making it suitable for mobile robotics applications and applications requiring high frame rates.

Key features of URP:
- Lightweight and efficient
- Supports 2D and 3D rendering
- Customizable render passes
- Built-in post-processing effects
- Shader Graph integration

### High Definition Render Pipeline (HDRP)

A state-of-the-art rendering pipeline designed for high-fidelity visuals on powerful hardware. HDRP is ideal for applications requiring photorealistic rendering, such as digital twin creation and high-quality sensor simulation.

Key features of HDRP:
- Physically-based rendering
- Real-time ray tracing
- Advanced lighting models
- Global illumination (Baked and realtime)
- Volumetric effects
- Advanced post-processing stack

## Physically-Based Rendering (PBR)

Physically-Based Rendering is a methodology that simulates light-object interactions using real-world physics principles. PBR materials in Unity are defined by several key properties.

### Albedo (Base Color)

The base color of the material without lighting considerations. This represents the color of the surface when illuminated by white light.

### Metallic

Controls whether a surface behaves like a metal or a non-metal. Metallic surfaces reflect light as colored reflections, while non-metallic surfaces reflect light as white highlights.

### Smoothness/Roughness

Controls the microsurface detail of the material, affecting how light scatters. Smooth surfaces create sharp reflections, while rough surfaces create diffuse reflections.

### Normal Maps

Provide detailed surface geometry information without increasing polygon count. Normal maps create the illusion of complex surface details like scratches, bumps, and grooves.

### Occlusion Maps

Simulate the shadowing effect of small surface details, enhancing the perception of depth and contact between surfaces.

## Lighting Systems in Unity

Unity provides several lighting systems that can be used to create realistic illumination.

### Real-time Lighting

Lights that affect objects dynamically during gameplay. Real-time lighting provides interactive illumination but can be computationally expensive.

Types of real-time lights:
- Directional lights: Simulate distant light sources like the sun
- Point lights: Emit light in all directions from a point
- Spot lights: Emit light in a cone shape
- Area lights: Emit light from a surface area (baked only in some pipelines)

### Baked Lighting

Lighting that is precomputed and stored in lightmaps. Baked lighting provides high-quality global illumination but cannot change at runtime.

### Mixed Lighting

Combines real-time and baked lighting, allowing for static lighting to be baked while enabling dynamic shadows from moving objects.

## Code Examples

### Unity C# Script for Camera Sensor Simulation

```csharp
using System;
using UnityEngine;
using System.Collections;
using System.IO;

[RequireComponent(typeof(Camera))]
public class CameraSensor : MonoBehaviour
{
    [Header("Camera Settings")]
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float fieldOfView = 60f;
    public string outputDirectory = "CameraOutput";

    [Header("Sensor Simulation")]
    public bool simulateDepth = false;
    public float minDepth = 0.1f;
    public float maxDepth = 100f;
    public bool simulateSemanticSegmentation = false;

    private Camera cam;
    private RenderTexture renderTexture;
    private Texture2D outputTexture;

    void Start()
    {
        cam = GetComponent<Camera>();
        SetupCamera();
        CreateRenderTexture();
    }

    void SetupCamera()
    {
        cam.fieldOfView = fieldOfView;
        cam.enabled = false;
    }

    void CreateRenderTexture()
    {
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        renderTexture.format = RenderTextureFormat.ARGB32;
        renderTexture.antiAliasing = 1;
        renderTexture.Create();

        outputTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
    }

    public void CaptureImage()
    {
        cam.targetTexture = renderTexture;
        cam.Render();

        RenderTexture.active = renderTexture;
        outputTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        outputTexture.Apply();

        RenderTexture.active = null;
        cam.targetTexture = null;

        SaveImage(outputTexture, "rgb_image");
    }

    public void CaptureDepthImage()
    {
        if (!simulateDepth) return;

        RenderTexture depthRT = RenderTexture.GetTemporary(
            imageWidth, imageHeight, 24, RenderTextureFormat.RFloat);

        cam.SetTargetBuffers(depthRT.colorBuffer, depthRT.depthBuffer);
        cam.RenderWithShader(Shader.Find("Hidden/DepthOnly"), "");

        RenderTexture.active = depthRT;
        Texture2D depthTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RFloat, false);
        depthTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        depthTexture.Apply();

        RenderTexture.active = null;
        cam.ResetReplacementShader();

        SaveImage(depthTexture, "depth_image");

        RenderTexture.ReleaseTemporary(depthRT);
        DestroyImmediate(depthTexture);
    }

    private void SaveImage(Texture2D texture, string prefix)
    {
        byte[] bytes = texture.EncodeToPNG();

        string fullDir = Path.Combine(Application.dataPath, outputDirectory);
        if (!Directory.Exists(fullDir))
        {
            Directory.CreateDirectory(fullDir);
        }

        string filename = $"{prefix}_{DateTime.Now:yyyyMMdd_HHmmss}.png";
        string filepath = Path.Combine(fullDir, filename);

        File.WriteAllBytes(filepath, bytes);
        Debug.Log($"Image saved to: {filepath}");
    }

    void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
        }
        if (outputTexture != null)
        {
            DestroyImmediate(outputTexture);
        }
    }
}
```

### Unity C# Script for Material Configuration

```csharp
using UnityEngine;

public class PBRMaterialController : MonoBehaviour
{
    [Header("PBR Properties")]
    [Range(0f, 1f)]
    public float metallic = 0f;

    [Range(0f, 1f)]
    public float smoothness = 0.5f;

    public Color albedoColor = Color.white;

    public Texture2D albedoTexture;
    public Texture2D normalMap;
    public Texture2D metallicMap;
    public Texture2D occlusionMap;

    private Material material;
    private Renderer objectRenderer;

    void Start()
    {
        objectRenderer = GetComponent<Renderer>();
        if (objectRenderer != null)
        {
            material = objectRenderer.material;
            UpdateMaterial();
        }
    }

    void Update()
    {
        if (material != null)
        {
            UpdateMaterial();
        }
    }

    void UpdateMaterial()
    {
        material.SetFloat("_Metallic", metallic);
        material.SetFloat("_Glossiness", smoothness);
        material.SetColor("_Color", albedoColor);

        if (albedoTexture != null)
            material.SetTexture("_MainTex", albedoTexture);

        if (normalMap != null)
            material.SetTexture("_BumpMap", normalMap);

        if (metallicMap != null)
            material.SetTexture("_MetallicGlossMap", metallicMap);

        if (occlusionMap != null)
            material.SetTexture("_OcclusionMap", occlusionMap);
    }

    public void SetMetallic(float value)
    {
        metallic = Mathf.Clamp01(value);
        UpdateMaterial();
    }

    public void SetSmoothness(float value)
    {
        smoothness = Mathf.Clamp01(value);
        UpdateMaterial();
    }

    public void SetAlbedoColor(Color color)
    {
        albedoColor = color;
        UpdateMaterial();
    }
}
```

## Summary

Unity's rendering pipelines provide powerful tools for creating photorealistic robot simulation environments. Understanding PBR materials and lighting systems enables the creation of high-fidelity digital twins that closely match real-world conditions for effective sim-to-real transfer.

## Key Takeaways

:::tip Key Takeaways
- HDRP provides photorealistic rendering for high-fidelity simulations
- PBR materials simulate realistic light-surface interactions
- Multiple lighting modes enable flexible environment creation
- Camera sensors can capture RGB, depth, and segmentation data
:::

## What's Next

In the next chapter, we'll explore sensor simulation in Unity, learning how to implement LIDAR, IMU, and other sensors for comprehensive robot perception testing.
