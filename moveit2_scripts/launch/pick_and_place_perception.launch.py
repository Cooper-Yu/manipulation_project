from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    gripper_only = LaunchConfiguration("gripper_only")
    gripper_close_position = LaunchConfiguration("gripper_close_position")
    approach_plan_only = LaunchConfiguration("approach_plan_only")
    stop_after_approach = LaunchConfiguration("stop_after_approach")
    skip_pre_grasp = LaunchConfiguration("skip_pre_grasp")
    stop_after_close = LaunchConfiguration("stop_after_close")
    stop_after_transfer = LaunchConfiguration("stop_after_transfer")

    moveit_config = (
        MoveItConfigsBuilder("name", package_name="my_moveit_config")
        .to_moveit_configs()
    )

    perception_node = Node(
        package="object_detection",
        executable="object_detection",
        name="object_detection_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    pick_and_place_node = Node(
        package="moveit2_scripts",
        executable="pick_and_place_perception",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"gripper_only": ParameterValue(gripper_only, value_type=bool)},
            {"gripper_close_position": ParameterValue(gripper_close_position, value_type=float)},
            {"approach_plan_only": ParameterValue(approach_plan_only, value_type=bool)},
            {"stop_after_approach": ParameterValue(stop_after_approach, value_type=bool)},
            {"skip_pre_grasp": ParameterValue(skip_pre_grasp, value_type=bool)},
            {"stop_after_close": ParameterValue(stop_after_close, value_type=bool)},
            {"stop_after_transfer": ParameterValue(stop_after_transfer, value_type=bool)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gripper_only", default_value="false",
            description="Only test the gripper from the current arm pose; skip perception and arm motion.",
        ),
        DeclareLaunchArgument(
            "gripper_close_position", default_value="0.643",
            description="Robotiq left knuckle close target in radians.",
        ),
        DeclareLaunchArgument(
            "approach_plan_only", default_value="false",
            description="Plan the Pilz approach without executing it.",
        ),
        DeclareLaunchArgument(
            "stop_after_approach", default_value="false",
            description="Execute approach then stop before gripper close.",
        ),
        DeclareLaunchArgument(
            "skip_pre_grasp", default_value="false",
            description="Diagnostic only: start from the current robot state.",
        ),
        DeclareLaunchArgument(
            "stop_after_close", default_value="false",
            description="Stop after gripper close and dwell.",
        ),
        DeclareLaunchArgument(
            "stop_after_transfer", default_value="true",
            description="Stop after shoulder transfer before release.",
        ),
        perception_node,
        pick_and_place_node,
    ])
