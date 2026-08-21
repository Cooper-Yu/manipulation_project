from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="real_moveit_config")
        .to_moveit_configs()
    )

    state_display_node = Node(
        package="moveit2_scripts",
        executable="real_robot_state_display",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )
    pick_place_node = Node(
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "execute",
                default_value="false",
                description="Execute the retained real pick-and-place sequence.",
            ),
            state_display_node,
            pick_place_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=pick_place_node,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="pick-and-place process finished"
                            )
                        )
                    ],
                )
            ),
        ]
    )
