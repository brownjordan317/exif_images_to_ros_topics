import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Image
from std_msgs.msg import Float64

from utils.loading import load_single_image


class StreamSkydio(Node):
    def __init__(self, *, ros_images, exif_data,
                 hz, preload, image_paths, max_width):

        super().__init__('stream_skydio_over_ros')

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
        done = (
            self.index >= len(self.ros_images)
            if self.preloaded
            else self.index >= len(self.image_paths)
        )
        if done:
            self.shutdown()
            return

        if self.preloaded:
            ros_img = self.ros_images[self.index]
            ex = self.exif_data[self.index]
        else:
            ros_img, ex = self.load_on_demand()

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
