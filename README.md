# Skydio ROS2 Image Streamer

This script streams Skydio JPG images over ROS 2 as if they were recorded live.
Images are read from a directory, EXIF metadata (GPS + yaw) is extracted, and frames are published as ROS 2 messages. The output can be recorded to a ROS 2 bag.

---

## Features

* Stream JPG images as live camera frames via ROS 2
* Extract GPS position and yaw from EXIF metadata using `exiftool`
* Publish at a fixed Hz **or attempt real-time replay based on EXIF timestamps**
* Optional multiprocessing-based preloading for maximum throughput
* Automatic resizing to control memory and bandwidth usage

---

## Preloading vs. On-Demand (IMPORTANT BEHAVIOR DIFFERENCE)

You may choose **preloading** or **on-demand streaming**, but they behave differently:

---

### 1. Preload All Images (Recommended for Real-Time Replay)

* Images are decoded, resized, converted to ROS messages, and EXIF-parsed **before streaming begins**.
* Multiprocessing speeds this up significantly.
* Streaming is extremely smooth and can sustain very high rates (60–120 Hz).
* **Real-time playback (`--real_time`) ONLY works in preload mode** because:

  * EXIF timestamps are scanned during preload,
  * The actual capture playback rate (Hz) is computed from them.

### ⚠️ Very Important: Real-time accuracy depends on your CPU

Even in preload mode:

* Real-time Hz may fluctuate slightly,
* Variations in system load and ROS scheduling can affect timing,
* Very large images may introduce publishing latency.

This is normal—true frame-accurate playback requires realtime system scheduling and pinned threads, which are outside scope.

### 2. On-Demand Loading (Low RAM, No Real-Time Support)

* Images are loaded and processed **at publish time**.
* Only one image is in memory at once.
* **EXIF timestamps are NOT fully scanned**, so:

  * `--real_time` is **not supported** here,
  * A fixed Hz must be provided (`--hz`).
* High-resolution images (e.g., 8K) may decode too slowly to maintain high publish rates unless resizing is applied.

Use on-demand when:

* RAM is limited,
* You don't need real-time history replay,
* You are dealing with very large datasets.

---

## Requirements

* Python 3
* ROS 2
* `exiftool`
* OpenCV (`cv2`)
* `rclpy`
* `cv_bridge`

---

## Usage

Basic bag recording:

```
python3 main.py \
    --folder /path/to/images \
    --bag_name output_bag
```

---

## Command-Line Arguments

### --folder   (required)

Directory containing JPG images to stream.

### --bag_name (required)

Name of the ROS 2 bag output file.

### --max_width

Resize images proportionally before publishing.
Default: 640 pixels.

### --hz

Fixed publish rate in Hz.
Used unless `--real_time` is enabled.

### --real_time  or  -rt

Replay using real capture timestamps.
**Requires `--preload`.**
If preloading is not used, this flag has no effect.

The computed real-time playback rate may vary depending on CPU load, system performance, and image sizes.

### --max_workers

Number of processes for preloading.
Defaults to number of CPU cores.

### --preload

Force preloading for faster throughput and support for real-time replay.

---

## Examples

Stream at 30 Hz:

```
python3 main.py \
    --folder /data/run01 \
    --bag_name run01
```

Replay with real capture timing (requires preload):

```
python3 main.py \
    --folder /data/run01 \
    --real_time \
    --preload \
    --bag_name realtime_bag
```

High-resolution preload example:

```
python3 main.py \
    --folder imgs \
    --max_width 3840 \
    --hz 60 \
    --max_workers 8 \
    --preload \
    --bag_name highres_test
```

---

## Performance Notes

* **Real-time mode only works with preloading**, since timestamps are extracted in batch during that stage
  (on-demand loading does not have timing data available ahead of time).

* Even in real-time mode:

  * Actual publishing Hz may vary with system load,
  * Very large images may still introduce scheduling lag,
  * Consider resizing (`--max_width`) for smoother playback.

* For highest performance:

  * Use preloading,
  * Resize images,
  * Increase worker count (`--max_workers`) on multi-core systems.

On-demand streaming is still fully functional—just understand that the fixed `--hz` rate depends on how quickly your system can load and decode each frame.
