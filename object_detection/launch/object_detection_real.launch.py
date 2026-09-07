from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("object_detection"), "launch", "object_detection.launch.py"])
            ),
            launch_arguments={
                "rviz_config": PathJoinSubstitution([FindPackageShare("object_detection"), "rviz", "object_detection_real.rviz"]),
                "point_cloud_topic": "/camera/depth/color/points",
                "target_frame": "base_link",
                "use_sim_time": "false",
            }.items(),
        )
    ])
