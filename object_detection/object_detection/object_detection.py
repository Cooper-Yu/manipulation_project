#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from object_detection.msg import DetectedObjects, DetectedSurfaces
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
import numpy as np
import pcl




def quaternion_to_rotation_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ], dtype=np.float64)


def transform_point(point, rotation_matrix, translation):
    return rotation_matrix @ point + translation
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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.point_cloud2_subscription = self.create_subscription(
            PointCloud2,
            "/wrist_rgbd_depth_sensor/points",
            self.pcl_callback,
            10,
        )

        self.pcl2_ready = False

        self.object_detected_pub = self.create_publisher(
            DetectedObjects,
            "object_detected",
            10,
        )
        self.surface_detected_pub = self.create_publisher(
            DetectedSurfaces,
            "surface_detected",
            10,
        )
        self.surface_marker_pub = self.create_publisher(MarkerArray, "surface_markers", 10)
        self.object_marker_pub = self.create_publisher(MarkerArray, "object_markers", 10)

    def pcl_callback(self, data: PointCloud2):
        self.pcl2_ready = True

        points = point_cloud2.read_points(
            data,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )

        points_array = np.column_stack((
            points["x"],
            points["y"],
            points["z"],
        )).astype(np.float32)

        if points_array.size == 0:
            self.get_logger().warning("Received empty point cloud")
            return

        filtered_points = points_array[
            np.isfinite(points_array).all(axis=1)
        ]

        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", data.header.frame_id, rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"Cannot transform {data.header.frame_id} to base_link: {exc}"
            )
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rotation_matrix = quaternion_to_rotation_matrix(
            (rotation.x, rotation.y, rotation.z, rotation.w)
        )
        translation_vector = np.array(
            [translation.x, translation.y, translation.z], dtype=np.float64
        )
        filtered_points = np.array(
            [transform_point(point, rotation_matrix, translation_vector)
             for point in filtered_points],
            dtype=np.float32,
        )
        filtered_points = filtered_points.astype(np.float32)

        cloud = pcl.PointCloud()
        cloud.from_array(
            np.asarray(filtered_points, dtype=np.float32)
        )

        segmenter = cloud.make_segmenter()
        segmenter.set_model_type(pcl.SACMODEL_PLANE)
        segmenter.set_method_type(pcl.SAC_RANSAC)
        segmenter.set_distance_threshold(0.01)

        indices, coefficients = segmenter.segment()
        surface_cloud = cloud.extract(indices)

        surface_index_set = set(indices)
        object_indices = [index for index in range(len(filtered_points)) if index not in surface_index_set]
        object_cloud = pcl.PointCloud()
        object_cloud.from_array(filtered_points[object_indices])

        tree = object_cloud.make_kdtree()
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

        plane_a, plane_b, plane_c, plane_d = coefficients
        valid_centroids = []
        valid_dimensions = []
        valid_cluster_clouds = []
        for centroid, size, cluster_cloud in zip(centroids, dimensions, cluster_clouds):
            if max(size) > 0.20:
                continue
            if abs(plane_c) > 1e-6:
                plane_z = -(plane_a * centroid[0] + plane_b * centroid[1] + plane_d) / plane_c
                if centroid[2] - plane_z < 0.005:
                    continue
            valid_centroids.append(centroid)
            valid_dimensions.append(size)
            valid_cluster_clouds.append(cluster_cloud)
        centroids, dimensions = valid_centroids, valid_dimensions

        surface_centroids, surface_dimensions = summarize_clusters([surface_cloud])
        surface_markers = MarkerArray()
        if surface_centroids:
            surface_msg = DetectedSurfaces()
            surface_msg.surface_id = 0
            surface_msg.position.x = float(surface_centroids[0][0])
            surface_msg.position.y = float(surface_centroids[0][1])
            surface_msg.position.z = float(surface_centroids[0][2])
            surface_msg.width = float(surface_dimensions[0][1])
            surface_msg.height = float(surface_dimensions[0][2])
            self.surface_detected_pub.publish(surface_msg)
            marker = Marker()
            marker.header = data.header
            marker.header.frame_id = "base_link"
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = surface_msg.position.x
            marker.pose.position.y = surface_msg.position.y
            marker.pose.position.z = surface_msg.position.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(surface_dimensions[0][0])
            marker.scale.y = float(surface_dimensions[0][1])
            marker.scale.z = max(float(surface_dimensions[0][2]), 0.01)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 0.35
            surface_markers.markers.append(marker)
        self.surface_marker_pub.publish(surface_markers)

        object_markers = MarkerArray()
        for object_id, (centroid, size) in enumerate(zip(centroids, dimensions)):
            object_msg = DetectedObjects()
            object_msg.object_id = object_id
            object_msg.position.x = float(centroid[0])
            object_msg.position.y = float(centroid[1])
            object_msg.position.z = float(centroid[2])
            object_msg.thickness = float(size[0])
            object_msg.width = float(size[1])
            object_msg.height = float(size[2])
            self.object_detected_pub.publish(object_msg)
            marker = Marker()
            marker.header = data.header
            marker.header.frame_id = "base_link"
            marker.id = object_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = object_msg.position.x
            marker.pose.position.y = object_msg.position.y
            marker.pose.position.z = object_msg.position.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(object_msg.thickness, 0.01)
            marker.scale.y = max(object_msg.width, 0.01)
            marker.scale.z = max(object_msg.height, 0.01)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 0.5
            object_markers.markers.append(marker)
        self.object_marker_pub.publish(object_markers)

        


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()





