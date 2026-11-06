# Research & Technical Analysis: DJI Osmo 360 (.OSV) Format and NLE Ecosystem

## 1. Executive Summary

This research document analyzes the DJI Osmo 360 panoramic video file format (`.OSV`), investigates reverse-engineering libraries and community tooling, evaluates open-source Non-Linear Editors (NLEs), and documents the empirical findings from analyzing real footage captured by the camera.

---

## 2. Evaluation of Open Source Video Editors

We surveyed the leading open-source non-linear video editors to determine which platform is the most adaptable for adding native `.OSV` support:

### A. Shotcut (Selected Platform)
- **Engine**: **MLT (Media Lovin' Toolkit)**.
- **UI Architecture**: C++ / Qt6 / QML.
- **360° Readiness**: **Best-in-class out of the box**. Shotcut bundles and exposes the `bigsh0t` Frei0r plugin suite (`360: Equirectangular to Rectilinear`, `360: Hemispherical to Equirectangular`, `360: Transform`). It includes an interactive 360 viewport where editors can drag to reframe the camera view, with full keyframing on Yaw, Pitch, Roll, and FOV.
- **Extensibility**:
  - Modular MLT service architecture. Media decoding and processing are separated into plugins (`libmltosv.so`).
  - File extension routing is configurable via MLT's `loader.dict`.
  - QML frontend allows rapid UI customizations and filter chaining.

### B. Kdenlive
- **Engine**: MLT.
- **UI Architecture**: C++ / Qt / KDE Frameworks.
- **Assessment**: Shares MLT with Shotcut and can use Frei0r plugins, but its codebase requires the entire KDE ecosystem (KIO, ECM, KF6), making custom development and distribution significantly heavier. Its 360 preview mode is also less integrated than Shotcut's.

### C. OpenShot
- **Engine**: `libopenshot` (C++).
- **UI Architecture**: Python (PyQt5/6).
- **Assessment**: Simplest UI to modify, making it ideal for background proxy scripts. However, `libopenshot` has **zero native 360° pipeline**—no spherical projection shaders, no 360 preview viewport, and no keyframable rectilinear reframing tools.

### D. Olive Video Editor (0.2)
- **Engine**: Node-based compositing with OpenFX (OFX) support.
- **Assessment**: Architecturally attractive because community tools (`OpenOSV`) already offer an OFX plugin. However, Olive 0.2 remains in pre-alpha/unstable development.

---

## 3. Analysis of Existing Tooling & Libraries

| Project | Capabilities & Role |
| :--- | :--- |
| **`Kemerd/OpenOSV`** | Open-source C++/CUDA toolkit designed for DJI Osmo 360 footage. Includes `osvtool` (batch CLI) and an OpenFX plugin for Premiere/DaVinci. Extracts calibration and gyro data; performs CUDA stitching. |
| **`Gyroflow` (`gyroflow-core`)** | Rust-based telemetry and stabilization library. Added support for parsing DJI Osmo 360 motion telemetry in version 1.7.0. |
| **FFmpeg `libavfilter` (`v360`)** | Built-in `input=dfisheye:output=equirect` projection filter. Handles dual-fisheye unwarping but lacks automated telemetry-driven dynamic horizon leveling and optical-flow seam blending. |
| **`bigsh0t`** (Leo Šutić) | Standard Frei0r plugin collection for 360° video manipulation. Bundled natively into Shotcut. |

---

## 4. Empirical Deconstruction of the `.OSV` Container

We analyzed sample footage (`CAM_20261103144622_0006_D.OSV` and companion `CAM_20261103144622_0006_D.LRF`) recorded by a DJI Osmo 360:

### Container Layout
- **Container**: ISO Base Media File Format (MP4 / QuickTime).
- **Stream 0:0 (`hvc1`)**: Front fisheye lens, 3840×3840 (1:1 aspect ratio), 10-bit HEVC (Main 10, BT.709), 25 fps, ~60 Mbps.
- **Stream 0:1 (`hvc1`)**: Rear fisheye lens, 3840×3840 (1:1 aspect ratio), 10-bit HEVC (Main 10, BT.709), 25 fps, ~63 Mbps.
- **Stream 0:2 (`mp4a`)**: Primary audio, AAC 48 kHz stereo, 317 kbps.
- **Stream 0:3 & 0:4 (`djmd`)**: Timed metadata track (~217 kbps) utilizing **Google Protocol Buffers** (`dvtm_oq101.proto`).
- **Stream 0:5 & 0:6 (`dbgi`)**: Debug / high-rate sensor logs (~1.8 Mbps).
- **Stream 0:7 (`mjpeg`)**: 688×344 embedded thumbnail snapshot.

### Protobuf Telemetry Schema (`dvtm_oq101.proto`)
Decoding the binary payload of Stream 3 revealed the exact structure used by DJI:
- **Header Block**: Contains camera model (`Osmo 360`), firmware version (`10.00.25.29`), serial number, sensor resolution (3840×3840), and sample rate (1000 Hz).
- **Calibration Submessage (Message 6)**:
  - **Front Lens**:
    - Focal length: $f_x = 1056.18, f_y = 1055.69$
    - Center of projection: $c_x = 1913.99, c_y = 1914.39$
    - Kannala-Brandt polynomial distortion: $k = [0.044899, 0.014099, -0.007223, -0.001206]$
    - Relative orientation quaternion: $\mathbf{q}_1 = [0.001001, 0.000116, 0.711365, -0.702822]$
  - **Rear Lens**:
    - Focal length: $f_x = 1060.92, f_y = 1060.83$
    - Center of projection: $c_x = 1911.55, c_y = 1914.53$
    - Kannala-Brandt polynomial distortion: $k = [0.047606, 0.009114, -0.003252, -0.002756]$
    - Relative orientation quaternion: $\mathbf{q}_2 = [0.712021, 0.702100, 0.005861, 0.006801]$
- Notice that $\mathbf{q}_1$ and $\mathbf{q}_2$ are normalized unit quaternions separated by a $180^\circ$ rotation ($0.707 \approx \sqrt{2}/2$).

### In-Camera Proxy Discovery (`.LRF`)
Each `.OSV` file is paired with an `.LRF` (Low Resolution File):
- Stored as a single 2048×1024 (2:1 aspect ratio) H.264 video at 25 fps.
- Pre-stitched in-camera directly to equirectangular format.
- Provides an immediate zero-wait proxy editing option for resource-constrained systems.
