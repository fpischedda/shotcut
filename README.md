# Native DJI Osmo 360 (.OSV) Support for Shotcut / MLT

This project implements native support for DJI Osmo 360 panoramic video files (`.OSV`) in **Shotcut** and the **MLT (Media Lovin' Toolkit)** framework.

---

## Features

- **Direct Ingestion of `.OSV`**: Drop raw `.OSV` files directly into Shotcut without pre-rendering or third-party conversion software.
- **Dual-Sensor Demuxing**: Simultaneously decodes both 3840×3840 10-bit HEVC fisheye streams (Stream 0 = Front Lens, Stream 1 = Rear Lens).
- **Protobuf Telemetry & Calibration Parsing**: Reads the `dvtm_oq101.proto` metadata track (`djmd`) to extract factory lens intrinsics ($f_x, f_y, c_x, c_y$), Kannala-Brandt distortion polynomials ($k_1 \dots k_4$), and relative sensor orientation quaternions.
- **Hardware-Accelerated Stitching**:
  - **Vulkan 1.4 Compute Shader**: Cross-vendor (Intel Arc, AMD, NVIDIA) real-time fisheye-to-equirectangular projection with seamless overlap blending.
  - **CUDA Kernel**: Dedicated NVIDIA implementation for CUDA workstations.
- **Interactive 360 Reframing**: Seamlessly integrates with Shotcut's built-in `bigsh0t` 360 filter suite (`360: Equirectangular to Rectilinear`) for keyframeable pan, tilt, roll, and FOV adjustments.

---

## Architecture & Codebase Layout

```
osmo-360/
├── CMakeLists.txt              # Unified build configuration
├── README.md                   # Documentation
├── include/
│   ├── osv_metadata.h          # Protobuf & timed metadata parser
│   └── osv_vulkan.h            # Vulkan compute pipeline engine
├── src/
│   ├── main.cpp                # Standalone CLI validation stitcher (osv_stitch)
│   ├── osv_metadata.cpp        # libavformat + Protobuf decoder
│   └── osv_vulkan.cpp          # Headless Vulkan compute runner
├── shaders/
│   ├── osv_stitch.comp         # GLSL Vulkan compute shader
│   ├── osv_stitch.spv          # Compiled SPIR-V compute bytecode
│   └── osv_stitch.cu           # CUDA compute kernel
├── mlt-plugin/
│   ├── mlt_osv.cpp             # MLT repository registration
│   └── producer_osv.cpp        # Native mlt_producer "osv" service
└── bin/
    ├── shotcut-osv             # Shotcut launcher with native OSV support
    └── melt-osv                # melt CLI launcher with native OSV support
```

---

## Building

```bash
cd /home/fra/personal/osmo-360
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

The build produces:
1. `osv_stitch`: Standalone verification CLI tool.
2. `libmltosv.so`: The native MLT producer service plugin.
3. `shaders/osv_stitch.spv`: Compiled Vulkan compute shader.

---

## Usage

### 1. Launching Shotcut with Native OSV Support

To launch Shotcut with the native OSV plugin loaded:

```bash
/home/fra/personal/osmo-360/bin/shotcut-osv
```

1. Open any `.OSV` file via **File -> Open File** or drag & drop onto the timeline.
2. The file is recognized by the `osv` MLT producer and stitched in real time on the GPU.
3. Add the **360: Equirectangular to Rectilinear** filter in Shotcut to pan, tilt, and animate the camera perspective.

### 2. Standalone CLI Stitcher

To extract metadata and render a stitched equirectangular frame directly to disk:

```bash
/home/fra/personal/osmo-360/build/osv_stitch /path/to/CAM_..._D.OSV
```

Outputs:
- Camera model, firmware, serial number, and lens calibration matrices.
- `stitched_vulkan_preview.ppm` (and converted `.jpg`).

### 3. Rendering via `melt` CLI

```bash
/home/fra/personal/osmo-360/bin/melt-osv /path/to/CAM_..._D.OSV -consumer avformat:output_360.mp4 vcodec=libx264
```
