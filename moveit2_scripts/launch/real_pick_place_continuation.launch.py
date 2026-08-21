from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="real_moveit_config")
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "execute",
                default_value="false",
                description="Execute the retained real pick-and-place sequence.",
            ),
            Node(
                package="moveit2_scripts",
                executable="real_pick_place_continuation",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {"use_sim_time": False},
                    {
                        "execute": ParameterValue(
                            LaunchConfiguration("execute"), value_type=bool
                        )
                    },
                ],
            )
        ]
    )
