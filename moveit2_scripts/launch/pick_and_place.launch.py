from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    approach_plan_only = LaunchConfiguration("approach_plan_only")
    stop_after_approach = LaunchConfiguration("stop_after_approach")
    skip_pre_grasp = LaunchConfiguration("skip_pre_grasp")
    stop_after_close = LaunchConfiguration("stop_after_close")
    stop_after_transfer = LaunchConfiguration("stop_after_transfer")
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="my_moveit_config")
        .to_moveit_configs()
    )

    plan_only_probe = Node(
        package="moveit2_scripts",
        executable="pick_and_place",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"approach_plan_only": approach_plan_only},
            {"stop_after_approach": stop_after_approach},
            {"skip_pre_grasp": skip_pre_grasp},
            {"stop_after_close": stop_after_close},
            {"stop_after_transfer": stop_after_transfer},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "approach_plan_only",
                default_value="false",
                description="Plan the Pilz LIN approach without executing it.",
            ),
            DeclareLaunchArgument(
                "stop_after_approach",
                default_value="false",
                description="Execute LIN approach, then stop before gripper close.",
            ),
            DeclareLaunchArgument(
                "skip_pre_grasp",
                default_value="false",
                description="Diagnostic only: start from an already reached pre-grasp state.",
            ),
            DeclareLaunchArgument(
                "stop_after_close",
                default_value="false",
                description="Stop after close and dwell, before retreat.",
            ),
            DeclareLaunchArgument(
                "stop_after_transfer",
                default_value="true",
                description="Stop after shoulder transfer with the gripper still closed.",
            ),
            plan_only_probe,
        ]
    )
