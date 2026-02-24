# DM-VIO Codebase Onboarding Guide (for LLM/Coding Agents)

This guide is a fast path to understand and extend this repository without brute-force exploration.

## 1. What this repo builds

- Static library: `libdmvio.a`
- Dataset executable: `dmvio_dataset`
- Live executable (RealSense T265): `dmvio_t265` (if librealsense is found)

Key build definitions:
- `CMakeLists.txt`:
  - `add_library(dmvio ...)`
  - `add_executable(dmvio_dataset ...)`
  - `add_executable(dmvio_t265 ...)`

## 2. High-level architecture

- Core VO/VIO engine: `src/dso/FullSystem/*`
- IMU integration + scale/gravity/extrinsics optimization: `src/IMU/*`, `src/IMUInitialization/*`, `src/GTSAMIntegration/*`
- Input adapters:
  - Dataset: `src/main_dmvio_dataset.cpp` + `src/dso/util/DatasetReader.h`
  - Live camera: `src/main_dmvio_t265.cpp` + `src/live/*`
- Output surface (critical extension point): `src/dso/IOWrapper/Output3DWrapper.h`

## 3. Fast reading order (recommended)

Read in this exact order:

1. `CMakeLists.txt`
- Confirms targets and linkage.

2. `src/main_dmvio_dataset.cpp`
- Dataset execution lifecycle.
- Shows where settings are loaded, data is read, and frames are pushed.

3. `src/main_dmvio_t265.cpp`
- Best reference for incremental/real-time feed pattern.
- Uses one `addActiveFrame(...)` call per frame.

4. `src/dso/FullSystem/FullSystem.h`
- Public API of the engine (`addActiveFrame`, `printResult`, `outputWrapper`).

5. `src/dso/FullSystem/FullSystem.cpp`
- Ground truth for runtime behavior and callback timing.

6. `src/dso/IOWrapper/Output3DWrapper.h`
- Contract for all real-time outputs (pose, keyframes, depth, scale transform, status).

7. `src/dso/IOWrapper/OutputWrapper/SampleOutputWrapper.h`
- Minimal practical example of consuming callbacks.

8. `src/dso/util/DatasetReader.h`
- Dataset loading semantics and strict IMU synchronization assumptions.

9. `src/IMU/IMUTypes.h`
- Exact IMU input type expected by engine.

10. `src/GTSAMIntegration/PoseTransformationIMU.h` and `src/GTSAMIntegration/PoseTransformationIMU.cpp`
- Metric conversion logic (`TransformDSOToIMU`) and scale handling.

## 4. Runtime call graph (core)

Dataset path:
- `main_dmvio_dataset.cpp`
  - init settings/calib
  - create `ImageFolderReader`
  - loop frames
  - `fullSystem->addActiveFrame(image, id, imuData, gtData)`
  - finalize: `blockUntilMappingIsFinished()`, `printResult(...)`

Live/incremental path:
- `main_dmvio_t265.cpp`
  - create `FullSystem`
  - fetch `(image, imuChunk)` from `FrameContainer`
  - `fullSystem->addActiveFrame(...)`

Inside `addActiveFrame(...)` (`FullSystem.cpp`):
- visual init/coarse tracking
- optional IMU integration hooks
- callback emission via `outputWrapper`
- keyframe mapping / optimization / marginalization pipeline

## 5. Input contract (important)

### Image
Type: `dso::ImageAndExposure`
- `float* image`
- `double timestamp`
- `float exposure_time`

### IMU
Type: `dmvio::IMUData` = `std::vector<IMUMeasurement>`
Each measurement:
- accelerometer (3)
- gyroscope (3)
- `integrationTime` = delta from previous IMU sample

Semantics:
- For frame `k`, pass IMU covering interval `(k-1 -> k)`.
- DM-VIO dataset path assumes an IMU sample exists exactly at each image timestamp.
- If sensor does not provide this, interpolate first.

## 6. Output contract (where to extract data)

Implement custom subclass of `Output3DWrapper`:
- `publishCamPose(FrameShell*, CalibHessian*)`
  - per-tracked-frame camera pose (low latency)
- `publishKeyframes(std::vector<FrameHessian*>&, bool final, CalibHessian*)`
  - keyframe states and sparse points (`pointHessians`, etc.)
- `pushDepthImageFloat(MinimalImageF*, FrameHessian*)`
  - per-keyframe inverse-depth map
- `publishTransformDSOToIMU(const TransformDSOToIMU&)`
  - transform to metric IMU/world frame (includes scale)
- `publishSystemStatus(SystemStatus)`
  - `VISUAL_INIT`, `VISUAL_ONLY`, `VISUAL_INERTIAL`

Most practical point cloud extraction:
- In `publishKeyframes`, iterate `FrameHessian::pointHessians` and convert `(u,v,idepth)` to 3D using keyframe intrinsics/pose.

## 7. Key files by goal

### Goal A: Incremental API (C++)
- `src/main_dmvio_t265.cpp`
- `src/dso/FullSystem/FullSystem.h`
- `src/dso/IOWrapper/Output3DWrapper.h`
- `src/live/FrameContainer.h` and `src/live/FrameContainer.cpp`

### Goal B: Python binding
- Wrap a small C++ facade around `FullSystem` + custom `Output3DWrapper`
- Expose only stable surface:
  - `start(config)`
  - `step(image, imu)`
  - `poll_outputs()`
  - `stop()`

Avoid binding internal DSO/DM-VIO classes directly.

### Goal C: Pose scale / metric conversion
- `src/GTSAMIntegration/PoseTransformationIMU.h`
- `src/GTSAMIntegration/PoseTransformationIMU.cpp`
- `publishTransformDSOToIMU(...)` callback output

## 8. Existing file outputs (batch mode)

`main_dmvio_dataset.cpp` writes:
- `result.txt`
- `resultKFs.txt`
- `resultScaled.txt`
- `timings.txt`

Use this for offline validation; for app integration prefer callback stream.

## 9. Practical grep shortcuts

From repo root:

```powershell
rg -n "addActiveFrame\(|outputWrapper|publishKeyframes|publishCamPose|publishTransformDSOToIMU|printResult\(" src
rg -n "class Output3DWrapper|SystemStatus" src/dso/IOWrapper/Output3DWrapper.h
rg -n "TransformDSOToIMU|getScale|transformPose" src/GTSAMIntegration
rg -n "IMUData|IMUMeasurement|integrationTime" src/IMU src/live src/dso/util
```

## 10. Common pitfalls

- Forgetting global calibration initialization (`setGlobalCalib(...)`) before engine use.
- Passing IMU chunk with wrong interval semantics.
- Heavy computation inside output callbacks (can stall tracking/mapping threads).
- Mixing DSO-frame poses with metric-frame poses without applying `TransformDSOToIMU`.

## 11. Licensing note

This repository is GPLv3. Any integration/distribution strategy must respect GPL obligations.

---

If you are another agent: start from section 3 and section 6, then implement a minimal custom `Output3DWrapper` before changing core solver code.
