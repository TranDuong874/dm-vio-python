# DM-VIO Real-time API Tutorial (RGB + IMU)

This tutorial explains how to use the minimal real-time API implemented in:

- `src/main_dmvio_dataset_ply.cpp`
- class: `DmvioRealtimePipeline`

The goal is simple:

- Input: camera frames (`RGB` or `grayscale`) + IMU measurements
- Output: latest DM-VIO state (pose, status, sparse points)


## 1. Build

From repo root (`dm-vio-python`):

```bash
cmake -S . -B build
cmake --build build -j --target dmvio_dataset_ply
```

Binary:

- `build/bin/dmvio_dataset_ply`


## 2. Know the API location

The API is currently in:

- `src/main_dmvio_dataset_ply.cpp`

Main objects:

- `RealtimeOutput` (output struct)
- `RealtimeOutputWrapper` (collects outputs from DM-VIO callbacks)
- `DmvioRealtimePipeline` (the API you feed with image + IMU)


## 3. Prepare calibration + settings files

You need:

- camera calibration (`calib`)
- IMU-camera extrinsic calibration (`imuCalib`, usually `camchain.yaml`)
- optional photometric files (`gamma`, `vignette`) when using photometric mode
- timestamps in seconds for frames you feed

Example files in this repo:

- `configs/tumvi_calib/camera02.txt`
- `configs/tumvi_calib/camchain.yaml`
- `configs/tumvi_calib/pcalib.txt`


## 4. Full initialization + argument parsing example

This is a complete example you can copy and run (dataset-based input):

```cpp
#include "util/MainSettings.h"
#include "util/SettingsUtil.h"
#include "dso/util/DatasetReader.h"
#include "dso/util/settings.h"
#include <fstream>
#include <iostream>
#include <locale.h>
#include <memory>

// Reuse the API class from src/main_dmvio_dataset_ply.cpp
// (recommended next step: move it into src/live/DmvioRealtimePipeline.h/.cpp).
// This snippet compiles if placed in the same translation unit that contains
// RealtimeOutput + DmvioRealtimePipeline, or after moving them to a header/cpp.

using namespace dso;

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "C");

    // -------- Parameters you expose on command line --------
    std::string source = "";
    std::string imuFile = "";
    std::string tsFile = "";
    bool reverse = false;
    int start = 0;
    int end = 100000;
    bool use16Bit = false;

    // -------- Core DM-VIO settings objects --------
    dmvio::MainSettings mainSettings;
    dmvio::IMUCalibration imuCalibration;
    dmvio::IMUSettings imuSettings;

    // -------- Register arguments --------
    auto settingsUtil = std::make_shared<dmvio::SettingsUtil>();
    imuSettings.registerArgs(*settingsUtil);
    imuCalibration.registerArgs(*settingsUtil);
    mainSettings.registerArgs(*settingsUtil);

    settingsUtil->registerArg("files", source);
    settingsUtil->registerArg("start", start);
    settingsUtil->registerArg("end", end);
    settingsUtil->registerArg("imuFile", imuFile);
    settingsUtil->registerArg("tsFile", tsFile);
    settingsUtil->registerArg("reverse", reverse);
    settingsUtil->registerArg("use16Bit", use16Bit);

    // Parse command line + optional settings yaml (settingsFile=...).
    mainSettings.parseArguments(argc, argv, *settingsUtil);

    // Optional IMU extrinsic calibration file.
    if(mainSettings.imuCalibFile != "")
    {
        imuCalibration.loadFromFile(mainSettings.imuCalibFile);
    }

    // Print/save the exact settings used for reproducibility.
    std::cout << "Settings:\n";
    settingsUtil->printAllSettings(std::cout);
    {
        std::ofstream settingsStream(imuSettings.resultsPrefix + "usedSettingsdso.txt");
        settingsUtil->printAllSettings(settingsStream);
    }

    // The current minimal API path is for 8-bit image input.
    if(use16Bit)
    {
        std::cerr << "ERROR: use16Bit=1 is not supported in this minimal API example." << std::endl;
        return 1;
    }

    // -------- Build a dataset reader (only for this example) --------
    auto reader = std::make_unique<ImageFolderReader>(
            source,
            mainSettings.calib,
            mainSettings.gammaCalib,
            mainSettings.vignette,
            use16Bit,
            tsFile);
    reader->loadIMUData(imuFile);

    // -------- Create the realtime pipeline --------
    const bool linearizeOperation = (mainSettings.playbackSpeed == 0);
    DmvioRealtimePipeline::InitConfig cfg;
    cfg.calibFile = mainSettings.calib;
    cfg.gammaCalibFile = mainSettings.gammaCalib;
    cfg.vignetteFile = mainSettings.vignette;

    DmvioRealtimePipeline pipeline(cfg, imuCalibration, imuSettings, linearizeOperation);

    // -------- Feed frames --------
    int lstart = start;
    int lend = end;
    int linc = 1;
    if(reverse)
    {
        lstart = end - 1;
        if(lstart >= reader->getNumImages()) lstart = reader->getNumImages() - 1;
        lend = start;
        linc = -1;
    }

    for(int imageId = lstart;
        imageId >= 0 && imageId < reader->getNumImages() && linc * imageId < linc * lend;
        imageId += linc)
    {
        std::unique_ptr<dso::MinimalImageB> raw(reader->getImageRaw(imageId));
        if(!raw) continue;

        dmvio::IMUData imuForFrame;
        if(dso::setting_useIMU && imageId > 0)
        {
            // IMU between previous and current frame.
            imuForFrame = reader->getIMUData(imageId);
        }

        RealtimeOutput out = pipeline.addGrayFrame(
                raw->data,
                raw->w,
                raw->h,
                reader->getTimestamp(imageId), // seconds
                1.0f,                          // exposure (ms), dataset here is fixed
                imuForFrame,
                false);

        std::cout << "frame=" << imageId
                  << " poseValid=" << out.poseValid
                  << " metricPose=" << out.metricPoseValid
                  << " points=" << out.sparsePointCount
                  << std::endl;

        if(out.isLost) break;
    }

    pipeline.finish();
    return 0;
}
```


## 5. Feed data every frame

For each frame `k`, pass:

- image bytes
- `width`, `height`
- `timestampSec` (seconds)
- `exposureMs` (milliseconds)
- `imuForThisFrame`: IMU measurements between frame `k-1` and frame `k`

```cpp
// Build IMUData from your own sensor queue (between previous and current frame).
dmvio::IMUData buildImuForFrame(const std::vector<MyImuSample>& samples)
{
    dmvio::IMUData out;
    out.reserve(samples.size());
    for(const auto& s : samples)
    {
        Eigen::Vector3d acc(s.ax, s.ay, s.az);
        Eigen::Vector3d gyr(s.gx, s.gy, s.gz);
        out.emplace_back(acc, gyr, s.dt_sec); // dt_sec must be in seconds
    }
    return out;
}

dmvio::IMUData imuForThisFrame = buildImuForFrame(samples_between_prev_and_curr);

RealtimeOutput out = pipeline.addGrayFrame(
    gray_ptr,        // uint8_t*
    width,
    height,
    timestampSec,    // seconds
    exposureMs,      // milliseconds
    imuForThisFrame,
    false            // set true to also return sparsePoints vector
);
```

Or RGB:

```cpp
RealtimeOutput out = pipeline.addRgbFrame(
    rgb_ptr, width, height, timestampSec, exposureMs, imuForThisFrame, false
);
```

Important:

- `addRgbFrame` expects `RGB` byte order.
- OpenCV camera frames are usually `BGR`.
- If you use OpenCV directly, either:
  - convert `BGR -> RGB`, or
  - convert to grayscale and call `addGrayFrame`.


## 6. Read outputs

`RealtimeOutput` gives you:

- `poseValid`: current DSO pose validity
- `camToWorldDso`: current camera pose in DSO frame
- `metricPoseValid`: true after VI scale/transform is initialized
- `imuToWorldMetric`: metric IMU pose (when available)
- `systemStatus`: `VISUAL_INIT`, `VISUAL_ONLY`, or `VISUAL_INERTIAL`
- `sparsePointCount`: number of sparse points in active keyframes
- `sparsePoints`: optional point array (if requested in API call)
- `isLost`, `initFailed`


## 7. Shutdown cleanly

At application end:

```cpp
pipeline.finish();
```

This flushes mapping and writes:

- `result.txt`
- `resultKFs.txt`
- `resultScaled.txt`
- `timings.txt`

under `imuSettings.resultsPrefix`.


## 8. Minimal OpenCV bridge example

```cpp
cv::Mat bgr = ...;                   // CV_8UC3
cv::Mat gray;
cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

dmvio::IMUData imuForFrame = ...;

RealtimeOutput out = pipeline.addGrayFrame(
    gray.data,
    gray.cols,
    gray.rows,
    frameTimestampSec,
    exposureMs,
    imuForFrame,
    false
);
```


## 9. Input contract checklist (most common pitfalls)

1. Image size must match calibration file expected original size.
2. Timestamps must be monotonic and in **seconds**.
3. IMU `integrationTime` must be in **seconds**.
4. IMU vector for frame `k` must represent interval `(k-1, k]`.
5. First frame can use empty IMU data.
6. If photometric mode is enabled, provide valid `gamma` and `vignette`.


## 10. Python binding direction

The current class is inside `main_dmvio_dataset_ply.cpp`. For cleaner binding:

1. Move `RealtimeOutput` and `DmvioRealtimePipeline` into dedicated files:
   - `src/live/DmvioRealtimePipeline.h`
   - `src/live/DmvioRealtimePipeline.cpp`
2. Add them to `CMakeLists.txt` (library target).
3. Wrap methods with `pybind11`:
   - constructor
   - `addGrayFrame`
   - `addRgbFrame`
   - `finish`
   - expose `RealtimeOutput` fields
