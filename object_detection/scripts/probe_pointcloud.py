#!/usr/bin/env python3
"""Read-only, bounded check for an actual nonempty real point-cloud message."""
import sys
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = rclpy.create_node("real_pointcloud_readiness_probe")
received = []

def on_cloud(msg):
    if msg.width * msg.height > 0 and msg.data and {"x", "y", "z"}.issubset(
        {field.name for field in msg.fields}
    ):
        received.append(msg.header.frame_id)

subscription = node.create_subscription(
    PointCloud2, sys.argv[1], on_cloud, qos_profile_sensor_data
)
try:
    deadline = time.monotonic() + 10.0
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    if received:
        print("PointCloud2 received; frame:", received[0])
    else:
        print("No PointCloud2 received within 10 seconds.")
finally:
    node.destroy_node()
    rclpy.shutdown()
sys.exit(0 if received else 1)
