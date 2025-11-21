# Skydio ROS2 Image Streamer

This script streams Skydio JPG images over ROS 2 as if they were recorded live.
Images are read from a directory, EXIF metadata (GPS + yaw) is extracted, and
the frames are published as ROS 2 messages. The output can be recorded to a
ros2 bag in real time.

---

## Features

* Stream JPG images as live camera frames via ROS 2
* Extract GPS position and yaw from EXIF metadata (using exiftool)
* Publish at a fixed Hz or replay using real capture timing
* Optional multiprocessing-based preloading for maximum throughput
* Automatic image resizing to prevent large bandwidth usage

---

## Preloading vs. On-Demand (IMPORTANT TRADEOFF)

You can either:

### 1. Preload All Images (Fastest Playback, More RAM)

* All images are decoded, resized, converted to ROS messages, and EXIF-parsed
  before streaming begins.
* Multiprocessing accelerates this stage.
* Playback is extremely smooth, even at high publish rates (60–120 Hz).
* **BUT large image sets (e.g., hundreds of 8K frames) can consume many gigabytes
  of RAM.**

### 2. On-Demand Loading (Minimal RAM, But Slower)

* Frames are decoded and resized at publish time.
* Memory footprint is small because only one frame is held at once.
* **High resolution images (8K+) might not load fast enough to sustain high Hz**
  unless resizing is applied.
* Best when working on limited hardware, mobile platforms, or long image sequences.

Use `--preload` when:

* You need high FPS replay.
* You have enough system memory.
* You want smooth streaming without frame drops.

Use on-demand streaming when:

* You are memory-limited.
* Frame timing isn’t critical.
* Your images are extremely large and preloading is impractical.

---

## Requirements

* Python 3
* ROS 2
* exiftool
* OpenCV (cv2)
* rclpy
* cv_bridge

---

## Usage

Basic streaming and bag recording:

```
python3 stream_as_ros2.py \
    --folder /path/to/images \
    --bag_name output_bag
```

---

## Command-Line Arguments

--folder     (required)
Directory containing JPG images to stream.

--bag_name   (required)
Name of the ros2 bag file to record.

--max_width
Images are resized proportionally before publishing.
Default: 640 pixels. Helps reduce processing time and memory.

--hz
Fixed publish rate in Hz. Default: 30.
Used unless --real_time is enabled.

--real_time   or   -rt
Replay images according to their original EXIF timestamps.
Useful for realistic flight playback.

--max_workers
Number of processes used when preloading.
Defaults to number of CPU cores.

--preload
Enable preloading and multiprocessing for fast replay.
Higher RAM usage but significantly faster streaming.

---

## Examples

Stream at 30 Hz and record:

```
python3 stream_as_ros2.py \
    --folder /data/skydio_run01 \
    --bag_name skydio_bag01
```

Replay using real capture timing:

```
python3 stream_as_ros2.py \
    --folder /data/skydio_run01 \
    --real_time \
    --bag_name realtime_bag
```

Use 8 processes and large images:

```
python3 stream_as_ros2.py \
    --folder imgs \
    --max_width 3840 \
    --hz 60 \
    --max_workers 8 \
    --preload \
    --bag_name highres_test
```

---

## Performance Notes

* Preloading is strongly recommended if:

  * Images are large, AND
  * You want high publish rates (>30 Hz).
* On-demand loading reduces RAM use, but publish rate is limited by
  CPU decode + resize time per frame.
* Large 8K images may require resizing (`--max_width`) to avoid CPU
  bottlenecks during real-time streaming.

