from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    candidate_x = LaunchConfiguration("candidate_x")
    candidate_y = LaunchConfiguration("candidate_y")
    execute = LaunchConfiguration("execute")
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="real_moveit_config")
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("candidate_x", default_value="0.343"),
            DeclareLaunchArgument("candidate_y", default_value="0.132"),
            DeclareLaunchArgument(
                "execute",
                default_value="false",
                description="Explicit real-motion gate; false keeps Plan-only behavior.",
            ),
            Node(
                package="moveit2_scripts",
                executable="real_hover_calibration",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {"use_sim_time": False},
                    {
                        "candidate_x": candidate_x,
                        "candidate_y": candidate_y,
                        "execute": execute,
                    },
                ],
            ),
        ]
    )
