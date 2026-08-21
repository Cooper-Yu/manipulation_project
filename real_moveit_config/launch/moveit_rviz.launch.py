from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "name", package_name="real_moveit_config"
    ).to_moveit_configs()
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(moveit_config.package_path / "config/moveit.rviz"),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                arguments=["-d", rviz_config],
                parameters=[
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    {"use_sim_time": False},
                ],
            ),
            Node(
                package="moveit2_scripts",
                executable="real_robot_state_display",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
        ]
    )
