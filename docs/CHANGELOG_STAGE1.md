# Changelog: Stage 1 - Native OSV Playback & Vulkan Compute Engine

## Summary of Changes

In this development milestone, we implemented full native support for **DJI Osmo 360 (`.OSV`)** files in **Shotcut** and **MLT (Media Lovin' Toolkit)**.

---

## Detailed File-by-File Explanation

### 1. Build & Project Configuration
- **`CMakeLists.txt`**:
  - Configures C++17 build with `-fPIC` (`CMAKE_POSITION_INDEPENDENT_CODE ON`) for shared module linking.
  - Locates `libavformat`, `libavcodec`, `libavutil`, `libswscale`, `mlt-framework-7`, and `libvulkan.so`.
  - Vendors `Vulkan-Headers` from `third_party/Vulkan-Headers/include`.
  - Sets up an automatic build step invoking `glslc` to compile `osv_stitch.comp` into SPIR-V bytecode (`osv_stitch.spv`).
  - Builds three artifacts: `osv_core` (static library), `osv_stitch` (CLI tool), and `libmltosv.so` (MLT plugin).

- **`.gitignore`**:
  - Ignores build artifacts (`build/`, `*.o`, `*.so`), temporary test renders (`*.ppm`, `*.mp4`), and editor directories.

---

### 2. Shaders
- **`shaders/osv_stitch.comp` (GLSL Vulkan Compute Shader)**:
  - 16×16 workgroup compute shader implementing the full reverse-projection model.
  - Takes two 3840×3840 sampled textures (`lensFront`, `lensRear`) and outputs to an RGBA8 storage image (`outEquirect`).
  - Accepts push constants for lens intrinsics, Kannala-Brandt distortion polynomials, orientation quaternions, and inverse gyro quaternion.
  - Implements smooth hermite blending across the 195° overlapping boundary.
- **`shaders/osv_stitch.cu` (CUDA Compute Kernel)**:
  - NVIDIA counterpart providing identical numerical projection logic using CUDA surface and texture memory for high-performance workstation workflows.

---

### 3. Metadata & Telemetry Demuxer
- **`include/osv_metadata.h` & `src/osv_metadata.cpp`**:
  - Implements `OsvMetadataParser` to extract private metadata from Stream 3 (`djmd`).
  - Decodes Google Protocol Buffers (`dvtm_oq101.proto`) wire format:
    - Extracts camera metadata (model, firmware, serial number).
    - Extracts per-lens calibration matrices ($f_x, f_y, c_x, c_y$).
    - Extracts 4-term Kannala-Brandt radial distortion polynomials ($k_1 \dots k_4$).
    - Extracts relative 3D orientation unit quaternions for both optical sensors.

---

### 4. Vulkan Compute Engine
- **`include/osv_vulkan.h` & `src/osv_vulkan.cpp`**:
  - Implements `OsvVulkanStitcher` providing headless Vulkan compute execution.
  - Selects the primary GPU (e.g., Intel Arc Graphics via Mesa on this system).
  - Manages device memory, staging upload/download buffers, descriptor sets, and pipeline execution.
  - Dispatches compute workgroups `(out_w + 15)/16, (out_h + 15)/16, 1` and reads back the stitched equirectangular frame.

---

### 5. CLI Verification Tool
- **`src/main.cpp` (`osv_stitch`)**:
  - Standalone verification program that loads any `.OSV` file, parses its calibration, decodes one frame from each lens using `libavcodec`, runs `OsvVulkanStitcher`, and writes the stitched 360 equirectangular preview.

---

### 6. Native MLT Plugin & Shotcut Integration
- **`mlt-plugin/mlt_osv.cpp`**:
  - Exports the standard MLT module entry point `mlt_register`.
  - Registers the `osv` producer with the MLT framework (`MLT_REGISTER(mlt_service_producer_type, "osv", producer_osv_init)`).
- **`mlt-plugin/producer_osv.cpp`**:
  - Implements `mlt_producer` "osv":
    - On initialization, reads file metadata and instantiates two internal synchronized `avformat` sub-producers (`video_index=0` and `video_index=1`) plus audio (`audio_index=2`).
    - On `get_frame` and `get_image`, retrieves the decoded frames from both lenses and dispatches the Vulkan compute stitcher.
    - Delivers a unified equirectangular RGBA frame directly into MLT's filter graph.
- **`build/share/mlt-7/core/loader.dict`**:
  - Adds `*.osv=osv` and `*.OSV=osv` mappings so that Shotcut and MLT automatically route any dropped `.OSV` file directly to our native producer.
- **`bin/shotcut-osv` & `bin/melt-osv`**:
  - Launcher scripts configuring `MLT_REPOSITORY` and `MLT_DATA` to load the native plugin and launch Shotcut seamlessly.
