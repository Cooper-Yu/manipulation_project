import rclpy
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("object_detection_node")

        self.point_cloud2_subscription = self.create_subscription(
            PointCloud2,
            "/wrist_rgbd_depth_sensor/points",
            self.pcl_callback,
            10,
        )

        self.pcl2_ready = False

    def pcl_callback(self, data: PointCloud2):
        self.pcl2_ready = True

        points = point_cloud2.read_points(
            data,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )

        points_array = np.array(list(points), dtype=np.float32)

        if points_array.size == 0:
            self.get_logger().warning("Received empty point cloud")
            return

        filtered_points = points_array[
            np.isfinite(points_array).all(axis=1)
        ]

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
