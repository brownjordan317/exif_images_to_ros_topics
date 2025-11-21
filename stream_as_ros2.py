import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Image
from std_msgs.msg import Float64
import subprocess
import argparse
import os
import cv2
from cv_bridge import CvBridge
import glob
import multiprocessing as mp
from cv_bridge import CvBridge
import cv2
from halo import Halo
import logging
from datetime import datetime

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
logger = logging.getLogger(__name__)

class StreamSkydioOverROS(Node):
    def __init__(self, ros_images, exif_data, hz=30):
        super().__init__('stream_skydio_over_ros')

        self.bridge = CvBridge()
        self.ros_images = ros_images     # list of prebuilt ROS Image messages
        self.exif_data = exif_data       # dict keyed by index
        self.current_index = 0

        # will be injected after construction
        self.record_proc = None

        self.create_publishers()

        # Now timer does almost nothing = TRUE 30 Hz
        self.timer = self.create_timer(1.0 / hz, self.timer_callback)

        self.get_logger().info(
            f"Starting stream of {len(self.ros_images)} frames..."
        )

    def create_publishers(self):
        self.gps_pub = self.create_publisher(
            NavSatFix, 
            'skydio/global_position/fix', 
            1
        )
        self.image_pub = self.create_publisher(
            Image, 
            'skydio/camera/rgb/image', 
            1
        )
        self.yaw_pub = self.create_publisher(
            Float64, 
            'skydio/gimbal/heading', 
            1
        )

    def timer_callback(self):
        """
        Timer callback that publishes preloaded images and telemetry at 
        a rate of {hz} Hz. If all images have been published, the node will 
        shutdown.
        """
        if self.current_index >= len(self.ros_images):
            self.get_logger().info("Finished streaming all images.")

            # Stop the ros2 bag record process
            if self.record_proc:
                self.record_proc.terminate()
                self.record_proc.wait()
                self.get_logger().info("Closed ros2 bag recorder.")

            # Cleanly shut down ROS so rclpy.spin() exits
            rclpy.shutdown()
            return

        # Publish preloaded image
        self.image_pub.publish(self.ros_images[self.current_index])

        # Publish preloaded telemetry
        exif = self.exif_data[self.current_index]
        gps_msg = NavSatFix()
        gps_msg.latitude = exif["latitude"]
        gps_msg.longitude = exif["longitude"]
        gps_msg.altitude = exif["altitude"]
        self.gps_pub.publish(gps_msg)

        yaw_msg = Float64()
        yaw_msg.data = exif["vehicle_yaw"]
        self.yaw_pub.publish(yaw_msg)

        self.current_index += 1


def exif_data(image_path):
    """
    Extracts EXIF metadata from an image file.

    Args:
        image_path (str): Path to the image file.

    Returns:
        dict: A dictionary containing the extracted EXIF metadata.
            If exiftool returns an error, None is returned instead.
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

    output = {}
    for line in result.stdout.split("\n"):
        if " : " in line:
            key, value = line.split(" : ", 1)
            output[key.strip()] = value.strip()
    return output


def get_image_paths(folder):
    return sorted(glob.glob(os.path.join(folder, "*.JPG")))

def load_image_exif(args):
    """
    Loads an image and associated EXIF metadata.

    Args:
        args (tuple): A tuple containing the path to the image file and
            the maximum width to resize the image to.

    Returns:
        tuple: A tuple containing the ROS Image and a dictionary 
            containing the extracted EXIF metadata.
    """
    path, max_width = args
    bridge = CvBridge()
    
    # Load image
    img = cv2.imread(path)
    if img is None:
        return None, None

    # Resize if needed
    if img.shape[1] > max_width:
        scale = max_width / img.shape[1]
        img = cv2.resize(img, None, fx=scale, fy=scale)

    # Convert to ROS Image
    ros_img = bridge.cv2_to_imgmsg(img, encoding="bgr8")

    # Load EXIF
    ex = exif_data(path)
    exif_dict = {
        "timestamp": datetime.strptime(
            str(ex["Date/Time Original"]),
            "%Y:%m:%d %H:%M:%S.%f"
        ),
        "latitude": float(ex["GPS Latitude Raw"]),
        "longitude": float(ex["GPS Longitude Raw"]),
        "altitude": float(ex["GPS Altitude Raw"]),
        "vehicle_yaw": float(ex["Vehicle Orientation NED Yaw"])
    }

    return ros_img, exif_dict

def calculate_rt_hz(timestamps):
    """
    Calculate the rate in Hertz of a given list of timestamps.

    Args:
        timestamps (list): A list of timestamps.

    Returns:
        float: The rate in Hertz of the given list of timestamps.
    """
    deltas = [
        (t2 - t1).total_seconds()
        for t1, t2 in zip(timestamps[:-1], timestamps[1:])
    ]
    avg_delta = sum(deltas) / len(deltas)
    hz_rt = 1.0 / avg_delta if avg_delta > 0 else 30.0
    return hz_rt

def preload_all(folder, max_width=640, rt=False, num_workers=None):
    """
    Preload all images in a folder and associated EXIF metadata.

    Args:
        folder (str): Path to the folder containing the images.
        max_width (int, optional): Maximum width to resize the images to. 
        Defaults to 640. rt (bool, optional): If True, calculate the rate in 
        Hertz of the preloaded images. Defaults to False. num_workers 
        (int, optional): Number of workers to use in the pool. If None, use 
        the number of available CPU cores. Defaults to None.

    Returns:
        tuple: A tuple containing a list of ROS Image messages, a list of 
        dictionaries containing the associated EXIF metadata, and the rate in 
        Hertz of the preloaded images if rt is True.

    """
    image_paths = get_image_paths(folder)
    if not image_paths:
        print("No JPG files found.")
        return [], []

    # Prepare arguments for pool
    args_list = [(p, max_width) for p in image_paths]

    num_workers = num_workers or mp.cpu_count()
    ros_images = []
    exif_list = []

    with mp.Pool(processes=num_workers) as pool:
        for ros_img, exif_dict in pool.imap(load_image_exif, args_list):
            if ros_img is None:
                continue
            ros_images.append(ros_img)
            exif_list.append(exif_dict)

    if rt:
        hz_rt = calculate_rt_hz([exif["timestamp"] for exif in exif_list])
    else:
        hz_rt = 0.0

    return ros_images, exif_list, hz_rt

def read_args():
    """
    Parse command line arguments and return a parser object.

    Returns:
        argparse.ArgumentParser: A parser object containing the parsed
            command line arguments.
    """
    parser = argparse.ArgumentParser(
        description=(
            "Stream Skydio JPG images over ROS 2 as if recorded live. "
            "Images are read from a folder, EXIF GPS + yaw is extracted, "
            "and messages are published at a chosen rate while ros2 bag "
            "records the streams."
        )
    )

    parser.add_argument(
        "--folder",
        required=True,
        help="Folder containing Skydio JPG images."
    )
    parser.add_argument(
        "--max_width",
        type=int,
        default=640,
        help="Resize width for streaming (default: 640)."
    )
    parser.add_argument(
        "--hz",
        type=int,
        default=30,
        help="Publish rate in Hz (default: 30)."
    )
    parser.add_argument(
        "-rt",
        "--real_time",
        action='store_true',
        help="Stream images in real-time based on EXIF timestamps."
    )
    parser.add_argument(
        "--bag_name",
        required=True,
        help="Name of ros2 bag to record output into."
    )
    parser.add_argument(
        "--max_workers",
        type=int,
        default=None,
        help="Maximum number of worker processes for preloading images."
    )

    return parser


def main(args=None):
    
    args = read_args().parse_args()

    if not os.path.exists(args.folder):
        print("Folder does not exist.")
        return
    
    logger.info(f"Preloading images from {args.folder}...")
    logger.info(f"Max width: {args.max_width}")
    if args.real_time:
        logger.info(f"Will stream in real-time based on EXIF timestamps.")
    else:
        logger.info(f"Will stream at fixed rate of {args.hz} Hz.")
    logger.info(f"Ros2 bag will be saved as: {args.bag_name}")

    spinner = Halo(
        text=('Preloading images with '
        f'{args.max_workers or mp.cpu_count()} workers...'), 
        spinner='dots'
    )
    spinner.start()
    ros_images, exif_data_list, hz_rt = preload_all(
        args.folder, 
        args.max_width,
        args.real_time
    )
    if args.real_time and hz_rt > 0:
        hz = hz_rt
    else:
        hz = args.hz
    spinner.succeed(f"Finished preloading images. Streaming at {hz:.2f} Hz.")

    rclpy.init()

    # Start ros2 bag record
    record_proc = subprocess.Popen([
        "ros2", "bag", "record",
        "skydio/global_position/fix",
        "skydio/camera/rgb/image",
        "skydio/gimbal/heading",
        "-o", args.bag_name
    ])

    node = StreamSkydioOverROS(
        ros_images,
        exif_data_list,
        hz=hz
    )

    # Inject rosbag process so the node can stop it when done
    node.record_proc = record_proc

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # If spin exits due to interrupt, ensure bag recorder is stopped
    if record_proc.poll() is None:
        record_proc.terminate()
        record_proc.wait()

    # Safely shut down without crashing if already shut down
    try:
        rclpy.shutdown()
    except RuntimeError:
        pass

if __name__ == '__main__':
    main()
