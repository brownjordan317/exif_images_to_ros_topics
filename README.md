Skydio ROS2 Image Streamer
==========================

This script streams Skydio JPG images over ROS 2 as if they were recorded live. 
Images are loaded from a folder, EXIF GPS and yaw data are extracted, and the 
images are published as ROS 2 messages at a chosen rate. The output can be 
recorded into a ros2 bag.

------------------------------------------------------------
Features
------------------------------------------------------------
- Stream images from a directory over ROS 2
- Extract GPS and yaw from EXIF metadata
- Publish at fixed Hz or based on original timestamp spacing
- Resize images before publishing to reduce bandwidth
- Multiprocessing for preloading image data

------------------------------------------------------------
Requirements
------------------------------------------------------------
- Python 3
- ROS 2
- OpenCV (cv2)
- rclpy
- cv_bridge
- JPG images with EXIF metadata

------------------------------------------------------------
Usage
------------------------------------------------------------
Run the script:

    python3 stream_as_ros2.py --folder /path/to/images --bag_name output_bag

------------------------------------------------------------
Command Line Arguments
------------------------------------------------------------

--folder        (required)
    Directory containing JPG images to stream.

--bag_name      (required)
    Name of the ros2 bag file that will record streamed data.

--max_width
    Maximum image width when publishing. Images are resized 
    proportionally. Default: 640.

--hz
    Fixed publish rate in Hz. Used unless --real_time is enabled. 
    Default: 30.

--real_time  or  -rt
    Enable real-time playback based on the difference in EXIF 
    timestamps between images. Useful for replaying motion-based 
    image sequences in realistic timing.

--max_workers
    Maximum number of processes used for preloading and resizing 
    images. Default: number of CPU cores.

------------------------------------------------------------
Examples
------------------------------------------------------------

Stream at 30 Hz and record:

    python3 stream_as_ros2.py \
        --folder /data/skydio_run01 \
        --bag_name skydio_bag01

Replay using actual capture spacing:

    python3 stream_as_ros2.py \
        --folder /data/skydio_run01 \
        --real_time \
        --bag_name realtime_bag

Use 8 workers and larger images:

    python3 stream_as_ros2.py \
        --folder imgs \
        --max_width 3840 \
        --hz 60 \
        --max_workers 8 \
        --bag_name highres_test
