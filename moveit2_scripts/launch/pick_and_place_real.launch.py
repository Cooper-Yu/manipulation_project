from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # This is the single official real-robot entry point. With no arguments it
    # runs the complete sequence; the two optional modes expose bounded
    # calibration/recovery slices without changing the default workflow.
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="real_moveit_config")
        .to_moveit_configs()
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
                ),
                "stop_at_grasp": ParameterValue(
                    LaunchConfiguration("stop_at_grasp"), value_type=bool
                ),
                "continue_from_pregrasp": ParameterValue(
                    LaunchConfiguration("continue_from_pregrasp"), value_type=bool
                ),
                "use_perception": ParameterValue(use_perception, value_type=bool),
                "detection_timeout": ParameterValue(detection_timeout, value_type=float),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "execute",
                default_value="true",
                description=(
                    "Run the selected real-robot sequence; set false to publish "
                    "the retained plans for RViz review without sending commands."
                ),
            ),
            DeclareLaunchArgument(
                "stop_at_grasp",
                default_value="false",
                description=(
                    "Stop at the open-gripper grasp pose after the 60 mm descent; "
                    "skip close, lift, transfer, release, and final home."
                ),
            ),
            DeclareLaunchArgument(
                "use_perception",
                default_value="true",
                description="Use the real /object_detected target while retaining the Checkpoint 13 pose reference.",
            ),
            DeclareLaunchArgument(
                "detection_timeout",
                default_value="15.0",
                description="Seconds to wait for a valid /object_detected message.",
            ),
            DeclareLaunchArgument(
                "continue_from_pregrasp",
                default_value="false",
                description=(
                    "Start at the already-loaded pregrasp pose and run only "
                    "shoulder +pi transfer, release, settling, and final home."
                ),
            ),
            pick_place_node,
        ]
    )
