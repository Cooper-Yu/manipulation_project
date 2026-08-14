from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="my_moveit_config")
        .to_moveit_configs()
    )

    plan_only_probe = Node(
        package="moveit2_scripts",
        executable="pick_and_place",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([plan_only_probe])
