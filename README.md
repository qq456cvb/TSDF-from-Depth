# TSDF-from-Depth

A self-contained study of **TSDF (Truncated Signed Distance Function) volume reconstruction from RGB-D frames** — the KinectFusion-style volumetric representation — implemented twice: once on the **CPU** with OpenCV, and once on the **GPU as pure GLSL fragment shaders**, where the volume lives in a 2D texture and both fusion and ray-cast rendering happen in render-to-texture passes (no CUDA required).

This prototype later grew into the CUDA semantic-fusion pipeline of [SLAM-MaskRCNN](https://github.com/qq456cvb/SLAM-MaskRCNN).

## What's Inside

- **CPU path** (`main.cpp`) — reads a TUM-format depth/RGB pair (16-bit depth, `1/5000 m` units), integrates a 64³ TSDF over a fixed volume, then renders it by **ray casting** with trilinear interpolation of the signed distances (`interpTSDF`) and nearest-voxel color lookup; the result is shown as a colored point cloud in a PCL visualizer.
- **GPU path** (`gpu.cpp` + `shaders/`) — the same algorithm as a GLFW/GLEW shader pipeline:
  - the 64³ volume is flattened into a `√(64³) × √(64³)` 2D texture (`texToPos` recovers the voxel position from a texel);
  - `tsdf_update.frag` projects every voxel into the camera, samples the depth/RGB textures, and writes the truncated signed distance plus color in one full-screen pass;
  - `tsdf_render.frag` ray-marches the volume texture for display.
- **Support code** — `FileReader` (depth PNG loading), `ShaderProgram`/`ColorShader`/`TextureShader`/`ViewShader`/`TSDFShader` (small GL wrappers).

## Build & Run

Requires OpenCV, PCL (with VTK, for the CPU viewer), GLFW + GLEW, and OpenGL 4.0. Frame paths, camera intrinsics, and the volume bounds (`vol_start`/`vol_end`, `dim = 64`) are constants at the top of `main.cpp` and mirrored in the shaders — point them at a depth/RGB pair from a TUM-style RGB-D dataset and build the sources as one executable (the GPU path is entered via `main_gpu`).

This is a single-frame research prototype: it fuses one frame into the volume and visualizes it, with no camera tracking or multi-frame loop — see [SLAM-MaskRCNN](https://github.com/qq456cvb/SLAM-MaskRCNN) for the multi-frame CUDA successor.

## License

AGPL-3.0 — see `LICENSE`.
