import rclpy
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node

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


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
