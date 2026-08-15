from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    execute = LaunchConfiguration("execute")
    lift_distance = LaunchConfiguration("lift_distance")
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="real_moveit_config")
        .to_moveit_configs()
    )

    lift_test = Node(
        package="moveit2_scripts",
        executable="real_arm_lift_test",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
            {"execute": execute},
            {"lift_distance": lift_distance},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "execute",
                default_value="false",
                description=(
                    "Explicit real-motion gate. Keep false for the first RViz "
                    "Plan-only inspection."
                ),
            ),
            DeclareLaunchArgument(
                "lift_distance",
                default_value="0.020",
                description="Upward tool0 displacement in metres; maximum 0.050.",
            ),
            lift_test,
        ]
    )
