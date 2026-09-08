from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    execute = LaunchConfiguration("execute")
    use_perception = LaunchConfiguration("use_perception")
    detection_timeout = LaunchConfiguration("detection_timeout")
    stop_at_grasp = LaunchConfiguration("stop_at_grasp")
    moveit_config = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("object_detection"), "launch", "object_detection_real.launch.py"])
        )
    )
    pick_place = Node(
        package="moveit2_scripts",
        executable="real_pick_place_continuation",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
            {"use_detected_z_plan_only": ParameterValue(LaunchConfiguration("use_detected_z_plan_only"), value_type=bool)},
            {"execute": ParameterValue(execute, value_type=bool)},
            {"use_perception": ParameterValue(use_perception, value_type=bool)},
            {"detection_timeout": ParameterValue(detection_timeout, value_type=float)},
            {"stop_at_grasp": ParameterValue(stop_at_grasp, value_type=bool)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="false", description="Keep false for plan-only review."),
        DeclareLaunchArgument("use_perception", default_value="true"),
        DeclareLaunchArgument("use_detected_z_plan_only", default_value="false", description="Compare detected Z as tool0 Z; requires execute=false and stop_at_grasp=true."),
        DeclareLaunchArgument("detection_timeout", default_value="15.0"),
        DeclareLaunchArgument("stop_at_grasp", default_value="false"),
        perception_launch,
        pick_place,
    ])
