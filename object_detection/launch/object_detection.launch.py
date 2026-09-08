from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory("object_detection"), "rviz", "object_detection.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=rviz_config),
        DeclareLaunchArgument("point_cloud_topic", default_value="/wrist_rgbd_depth_sensor/points"),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="object_detection", executable="object_detection",
            name="object_detection_node", output="screen",
            parameters=[{
                "point_cloud_topic": LaunchConfiguration("point_cloud_topic"),
                "target_frame": LaunchConfiguration("target_frame"),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            }],
        ),
        Node(package="rviz2", condition=IfCondition(LaunchConfiguration("use_rviz")), executable="rviz2", name="rviz2", output="screen", arguments=["-d", LaunchConfiguration("rviz_config")]),
    ])
