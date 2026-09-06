import rclpy
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
import numpy as np
import pcl



def summarize_clusters(clusters):
    centroids = []
    dimensions = []

    for cluster in clusters:
        points = cluster.to_array()

        if points.size == 0:
            continue

        centroids.append(np.mean(points, axis=0))
        dimensions.append(np.max(points, axis=0) - np.min(points, axis=0))

    return centroids, dimensions

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

        cloud = pcl.PointCloud()
        cloud.from_array(
            np.asarray(filtered_points, dtype=np.float32)
        )

        segmenter = cloud.make_segmenter()
        segmenter.set_model_type(pcl.SACMODEL_PLANE)
        segmenter.set_method_type(pcl.SAC_RANSAC)
        segmenter.set_distance_threshold(0.01)

        indices, coefficients = segmenter.segment()
        extractor = cloud.make_ExtractIndices()

        extractor.set_Indices(indices)
        surface_cloud = extractor.filter()

        extractor.set_Negative(True)
        object_cloud = extractor.filter()

        cluster_extractor = object_cloud.make_EuclideanClusterExtraction()
        cluster_extractor.set_SearchMethod(tree)
        cluster_extractor.set_ClusterTolerance(0.02)
        cluster_extractor.set_MinClusterSize(100)
        cluster_extractor.set_MaxClusterSize(25000)

        cluster_indices = cluster_extractor.Extract()

        cluster_clouds = []
        for cluster_index_group in cluster_indices:
            cluster_clouds.append(object_cloud.extract(cluster_index_group))

        centroids, dimensions = summarize_clusters(cluster_clouds)

        

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
