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
from tqdm import tqdm


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
        Timer callback that publishes preloaded images and telemetry at a rate of {hz} Hz.
        If all images have been published, the node will shutdown.
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


def preload_all(folder, max_width=640):
    """
    Preload all images in a folder and extract their EXIF metadata.

    Args:
        folder (str): Path to the folder containing the images.
        max_width (int, optional): Maximum width of the images in pixels.
            Defaults to 640.

    Returns:
        tuple: A tuple containing two lists. The first list contains ROS Image messages,
            and the second list contains dictionaries containing the extracted EXIF metadata.
    """
    bridge = CvBridge()

    image_paths = get_image_paths(folder)
    if not image_paths:
        print("No JPG files found.")
        return [], []

    ros_images = []
    exif_list = []

    for i, p in tqdm(enumerate(image_paths), 
                     total=len(image_paths), 
                     desc="Preloading images"
                    ):

        # Load image once
        img = cv2.imread(p)
        if img is None:
            print("Failed to load:", p)
            continue

        # Resize once
        if img.shape[1] > max_width:
            scale = max_width / img.shape[1]
            img = cv2.resize(img, None, fx=scale, fy=scale)

        # Convert to ROS Image once
        ros_img = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        ros_images.append(ros_img)

        # Load EXIF once
        ex = exif_data(p)
        exif_list.append({
            "latitude": float(ex["GPS Latitude Raw"]),
            "longitude": float(ex["GPS Longitude Raw"]),
            "altitude": float(ex["GPS Altitude Raw"]),
            "vehicle_yaw": float(ex["Vehicle Orientation NED Yaw"])
        })

    return ros_images, exif_list


def main(args=None):
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
        "--bag_name",
        required=True,
        help="Name of ros2 bag to record output into."
    )

    args = parser.parse_args()

    ros_images, exif_data_list = preload_all(args.folder, args.max_width)

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
        hz=args.hz
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
