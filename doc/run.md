# DM-VIO Run Notes

## What was added

A new dataset executable was added to export sparse point clouds incrementally to `.ply`:

- New file: `dm-vio-python/src/main_dmvio_dataset_ply.cpp`
- New CMake target: `dmvio_dataset_ply` in `dm-vio-python/CMakeLists.txt`

This executable follows the same dataset flow as `main_dmvio_dataset.cpp`, but registers a custom `Output3DWrapper` that:

- Extracts sparse 3D points from keyframes
- Maintains an accumulated cloud
- Rewrites a PLY file incrementally
- Flushes/saves on shutdown (`join()`), so Ctrl+C still leaves a cloud file


## Build

```bash
cd /home/tranthaiduong/visual_odometry/dm-vio-python
cmake -S . -B build
cmake --build build -j --target dmvio_dataset_ply
```


## Important runtime note (GTSAM ABI)

On this machine, `dmvio_dataset*` needs the locally built GTSAM libs to avoid symbol mismatch at runtime.

Use:

```bash
export LD_LIBRARY_PATH=/home/tranthaiduong/visual_odometry/gtsam/build/gtsam:/home/tranthaiduong/visual_odometry/gtsam/build/gtsam/3rdparty/metis/libmetis:$LD_LIBRARY_PATH
```


## Run baseline (trajectory files)

Known working baseline setup here is `mode=1` (non-photometric), headless:

```bash
cd /home/tranthaiduong/visual_odometry/dm-vio-python
./build/bin/dmvio_dataset \
files=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/cam0/images \
imuFile=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/imu.txt \
tsFile=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/cam0/times.txt \
calib=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/cam0/camera.txt \
imuCalib=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/camchain.yaml \
mode=1 use16Bit=1 preset=1 nogui=1 preload=0 start=2 end=220 \
resultsPrefix=/home/tranthaiduong/visual_odometry/build/baseline_smoke/
```

Outputs:

- `/home/tranthaiduong/visual_odometry/build/baseline_smoke/result.txt`
- `/home/tranthaiduong/visual_odometry/build/baseline_smoke/resultKFs.txt`
- `/home/tranthaiduong/visual_odometry/build/baseline_smoke/resultScaled.txt`
- `/home/tranthaiduong/visual_odometry/build/baseline_smoke/timings.txt`


## Run incremental PLY exporter

```bash
cd /home/tranthaiduong/visual_odometry/dm-vio-python
./build/bin/dmvio_dataset_ply \
files=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/cam0/images \
imuFile=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/imu.txt \
tsFile=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/cam0/times.txt \
calib=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/cam0/camera.txt \
imuCalib=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/camchain.yaml \
mode=1 use16Bit=1 preset=1 nogui=1 preload=0 start=2 end=220 \
plyFile=/home/tranthaiduong/visual_odometry/build/ply_test_end220/cloud_incremental.ply \
plySaveEveryN=1 \
resultsPrefix=/home/tranthaiduong/visual_odometry/build/ply_test_end220/
```

Arguments:

- `plyFile`: output path of the incremental PLY
- `plySaveEveryN`: write every N keyframe callback (`1` = every callback)

Default PLY path if `plyFile` is not set:

- `resultsPrefix + "pointcloud_incremental.ply"`


## Ctrl+C behavior

The new executable installs signal handlers and saves point cloud snapshots incrementally.
In testing, interrupting with SIGINT still left a valid PLY:

- `/home/tranthaiduong/visual_odometry/build/ply_test_int/cloud_incremental.ply`

Interrupt test command used:

```bash
timeout -s INT 10s ./build/bin/dmvio_dataset_ply ...
```

## Stability notes

### Why `Assertion 'std::isfinite(p->data->step)' failed` happens

This failure comes from optimizer numeric instability (non-finite step), not from out-of-memory.
The assert is in:

- `dm-vio-python/src/dso/OptimizationBackend/EnergyFunctional.cpp:319`

If it were RAM exhaustion, you would usually see the process killed (SIGKILL / `Killed`), not this finite-check assert.

### Recommended robust profile (offline extraction)

For long offline runs, use:

- `preset=0` (more robust, not real-time constrained)
- `plySaveEveryN=20` (less I/O pressure than saving every callback)
- `mode=1` (on this setup it is much more stable than `mode=0`)

Example:

```bash
cd /home/tranthaiduong/visual_odometry/dm-vio-python
./build/bin/dmvio_dataset_ply \
files=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/cam0/images \
imuFile=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/imu.txt \
tsFile=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/cam0/times.txt \
calib=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/cam0/camera.txt \
imuCalib=/home/tranthaiduong/visual_odometry/dataset/dataset-corridor1_512_16/dso/camchain.yaml \
mode=1 use16Bit=1 preset=0 nogui=1 preload=0 start=2 end=1500 \
plyFile=/home/tranthaiduong/visual_odometry/build/ply_test_robust_end1500/cloud_incremental.ply \
plySaveEveryN=20 \
resultsPrefix=/home/tranthaiduong/visual_odometry/build/ply_test_robust_end1500/
```

Validated run:

- Exit code: `0`
- PLY created: `/home/tranthaiduong/visual_odometry/build/ply_test_robust_end1500/cloud_incremental.ply`
