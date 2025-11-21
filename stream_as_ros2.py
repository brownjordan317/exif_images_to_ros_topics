import argparse
import subprocess
import os
import glob
import logging
from datetime import datetime
import multiprocessing as mp

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Image
from std_msgs.msg import Float64

from halo import Halo

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
logger = logging.getLogger(__name__)

def exif_data(image_path):
    """
    Extracts EXIF metadata from an image using exiftool.

    Args:
        image_path (str): Path to the image file.

    Returns:
        dict: A dictionary containing the extracted EXIF metadata.

    Raises:
        None
    """
    result = subprocess.run(
        ['exiftool', image_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    if result.returncode != 0:
        print("EXIF ERROR:", result.stderr)
        return None

    exif = {}
    for line in result.stdout.split("\n"):
        if " : " in line:
            key, value = line.split(" : ", 1)
            exif[key.strip()] = value.strip()
    return exif

def parse_exif_struct(ex):
    """
    Parse an EXIF struct into a dict containing relevant metadata.

    Args:
        ex (dict): An EXIF struct as returned by exiftool.

    Returns:
        dict: A dictionary containing the parsed EXIF metadata.

    Raises:
        None
    """
    return {
        "timestamp": datetime.strptime(
            str(ex["Date/Time Original"]),
            "%Y:%m:%d %H:%M:%S.%f"
        ),
        "latitude": float(ex["GPS Latitude Raw"]),
        "longitude": float(ex["GPS Longitude Raw"]),
        "altitude": float(ex["GPS Altitude Raw"]),
        "vehicle_yaw": float(ex["Vehicle Orientation NED Yaw"])
    }


def load_single_image(args):
    """
    Loads an image + EXIF from disk.

    Args:
        args (tuple): (path, max_width)

    Returns:
        tuple: (ROS Image, EXIF dict)
    """
    path, max_width = args
    img = cv2.imread(path)
    if img is None:
        return None, None

    # Resize if necessary
    if img.shape[1] > max_width:
        scale = max_width / img.shape[1]
        img = cv2.resize(img, None, fx=scale, fy=scale)

    bridge = CvBridge()
    ros_img = bridge.cv2_to_imgmsg(img, encoding="bgr8")

    ex_raw = exif_data(path)
    ex = parse_exif_struct(ex_raw)

    del img
    return ros_img, ex


def calculate_rt_hz(timestamps):
    """
    Calculate the average Hz given a list of timestamps.

    Args:
        timestamps (list): A list of datetime objects representing the 
        timestamps.

    Returns:
        float: The average Hz calculated from the timestamps.

    Note:
        If the average delta is <= 0, the function returns 30.0 Hz as a 
        default value.
    """
    deltas = [
        (t2 - t1).total_seconds()
        for t1, t2 in zip(timestamps[:-1], timestamps[1:])
    ]
    avg_delta = sum(deltas) / len(deltas)
    return 1.0 / avg_delta if avg_delta > 0 else 30.0


def preload_all(folder, max_width, real_time=False, workers=None):
    """
    Preload all images in a folder and their corresponding EXIF metadata.

    Args:
        folder (str): Folder containing the images to preload.
        max_width (int): Maximum width of the images to resize to.
        real_time (bool, optional): Calculate the average Hz from the EXIF 
        timestamps. workers (int, optional): Number of worker processes to 
        use for preloading.

    Returns:
        tuple: A tuple containing the preloaded ROS images, EXIF metadata, 
        and the average Hz.
    """
    image_paths = sorted(glob.glob(os.path.join(folder, "*.JPG")))
    if not image_paths:
        logger.error("No JPG files found.")
        return [], [], 0.0

    args_list = [(p, max_width) for p in image_paths]
    workers = workers or mp.cpu_count()

    ros_images, exif_list = [], []
    with mp.Pool(processes=workers) as pool:
        for ros_img, exif in pool.imap(load_single_image, args_list):
            if ros_img is not None:
                ros_images.append(ros_img)
                exif_list.append(exif)

    hz = (calculate_rt_hz([d["timestamp"] for d in exif_list])
          if real_time else 0.0)

    return ros_images, exif_list, hz

class StreamSkydio(Node):
    def __init__(self, *, ros_images, exif_data,
                 hz, preload, image_paths, max_width):

        super().__init__('stream_skydio_over_ros')
        self.bridge = CvBridge()

        self.preloaded = preload
        self.ros_images = ros_images
        self.exif_data = exif_data
        self.image_paths = image_paths
        self.max_width = max_width
        self.index = 0

        self.record_proc = None

        self.create_publishers()
        self.timer = self.create_timer(1.0 / hz, self.tick)

        total = len(ros_images) if preload else len(image_paths)
        self.get_logger().info(
            f"Starting stream of {total} frames "
            f"({'preloaded' if preload else 'on-demand'})..."
        )

    def create_publishers(self):
        self.pub_gps = self.create_publisher(NavSatFix,
                                             'skydio/global_position/fix', 1)
        self.pub_img = self.create_publisher(Image,
                                             'skydio/camera/rgb/image', 1)
        self.pub_yaw = self.create_publisher(Float64,
                                             'skydio/gimbal/heading', 1)

    def load_on_demand(self):
        path = self.image_paths[self.index]
        ros_img, ex = load_single_image((path, self.max_width))
        return ros_img, ex

    def tick(self):
        """
        Callback for the ROS timer.

        Checks if the end of the image sequence has been reached. If so,
        shuts down the node. Otherwise, fetches the next image and EXIF
        metadata, and publishes them on the respective topics.
        """
        done = (
            self.index >= len(self.ros_images)
            if self.preloaded
            else self.index >= len(self.image_paths)
        )
        if done:
            self.shutdown()
            return

        # Fetch data
        if self.preloaded:
            ros_img = self.ros_images[self.index]
            ex = self.exif_data[self.index]
        else:
            ros_img, ex = self.load_on_demand()

        # Publish
        self.pub_img.publish(ros_img)

        gps = NavSatFix()
        gps.latitude = ex["latitude"]
        gps.longitude = ex["longitude"]
        gps.altitude = ex["altitude"]
        self.pub_gps.publish(gps)

        yaw = Float64()
        yaw.data = ex["vehicle_yaw"]
        self.pub_yaw.publish(yaw)

        self.index += 1

    def shutdown(self):
        self.get_logger().info("Finished streaming all images.")
        if self.record_proc:
            self.record_proc.terminate()
            self.record_proc.wait()
            self.get_logger().info("Closed ros2 bag recorder.")
        rclpy.shutdown()

def read_args():
    """
    Parse command line arguments and return parser.
    """
    p = argparse.ArgumentParser(
        description=(
            "Stream Skydio JPG images over ROS 2 as if recorded live. "
            "Images are read from a folder, EXIF GPS + yaw is extracted, "
            "and messages are published at a chosen rate while ros2 bag "
            "records the streams."
        )
    )
    p.add_argument("--folder", required=True,
                   help="Folder containing Skydio JPG images.")
    p.add_argument("--max_width", type=int, default=640,
                   help="Resize width for streaming (default: 640).")
    p.add_argument("--hz", type=int, default=30,
                   help="Publish rate in Hz (default: 30).")
    p.add_argument("--real_time", "-rt", action='store_true',
                   help="Stream based on EXIF timestamps.")
    p.add_argument("--bag_name", required=True,
                   help="Name of ros2 bag to record into.")
    p.add_argument("--max_workers", type=int, default=None,
                   help="Worker processes for preload.")
    p.add_argument("--preload", action='store_true',
                   help="Preload images before streaming.")
    return p


# -----------------------------------------------------------------------------
# MAIN
# -----------------------------------------------------------------------------
def main():
    args = read_args().parse_args()

    if not os.path.exists(args.folder):
        logger.error("Folder does not exist.")
        return

    logger.info(f"Folder: {args.folder}")
    logger.info(f"Bag name: {args.bag_name}")

    # Preload?
    if args.preload:
        spinner = Halo(
            text=f"Preloading images using {args.max_workers or mp.cpu_count()} workers...",
            spinner='dots'
        )
        spinner.start()
        ros_images, exif_list, hz_rt = preload_all(
            args.folder,
            args.max_width,
            args.real_time,
            args.max_workers
        )
        hz = hz_rt if args.real_time and hz_rt > 0 else args.hz
        spinner.succeed(f"Finished preloading. Streaming at {hz:.2f} Hz.")
        image_paths = None
    else:
        logger.info("On-demand loading enabled.")
        ros_images = None
        exif_list = None
        hz = args.hz
        image_paths = sorted(glob.glob(os.path.join(args.folder, "*.JPG")))

    # Start ROS
    rclpy.init()

    # Start rosbag record
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
