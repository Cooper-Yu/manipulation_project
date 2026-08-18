from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="real_moveit_config")
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            Node(
                package="moveit2_scripts",
                executable="real_pregrasp_lin_probe",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {"use_sim_time": False},
                ],
            )
        ]
    )
