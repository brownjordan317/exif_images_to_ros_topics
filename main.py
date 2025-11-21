import os
import glob
import subprocess
import logging
import multiprocessing as mp

from halo import Halo

import rclpy

from utils.skydio_streamer import StreamSkydio
from utils.loading import preload_all

import argparse

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def read_args():
    p = argparse.ArgumentParser(
        description=(
            "Stream Skydio JPG images over ROS 2 as if recorded live."
        )
    )
    p.add_argument("--folder", required=True)
    p.add_argument("--max_width", type=int, default=640)
    p.add_argument("--hz", type=int, default=30)
    p.add_argument("--real_time", "-rt", action='store_true')
    p.add_argument("--bag_name", required=True)
    p.add_argument("--max_workers", type=int, default=None)
    p.add_argument("--preload", action='store_true')
    return p


def main():
    args = read_args().parse_args()

    if args.preload:
        spinner = Halo(
            text=f"Preloading using {args.max_workers or mp.cpu_count()} workers...",
            spinner='dots'
        )
        spinner.start()
        ros_images, exif_list, hz_rt = preload_all(
            args.folder, args.max_width, args.real_time, args.max_workers
        )
        hz = hz_rt if args.real_time and hz_rt > 0 else args.hz
        spinner.succeed(f"Finished preloading. Streaming at {hz:.2f} Hz.")
        image_paths = None
    else:
        ros_images = None
        exif_list = None
        hz = args.hz
        image_paths = sorted(glob.glob(os.path.join(args.folder, "*.JPG")))

    rclpy.init()

    record_proc = subprocess.Popen([
        "ros2", "bag", "record",
        "skydio/global_position/fix",
        "skydio/camera/rgb/image",
        "skydio/gimbal/heading",
        "-o", args.bag_name
    ])

    node = StreamSkydio(
        ros_images=ros_images,
        exif_data=exif_list,
        hz=hz,
        preload=args.preload,
        image_paths=image_paths,
        max_width=args.max_width
    )
    node.record_proc = record_proc

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if record_proc.poll() is None:
        record_proc.terminate()
        record_proc.wait()

    try:
        rclpy.shutdown()
    except RuntimeError:
        pass


if __name__ == "__main__":
    main()
