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
        ),
        launch_arguments={"use_rviz": "false"}.items(),
    )
    pick_place = Node(
        package="moveit2_scripts",
        executable="real_pick_place_continuation",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
            {"use_detected_z_plan_only": ParameterValue(LaunchConfiguration("use_detected_z_plan_only"), value_type=bool)},
            {"detected_z_offset": ParameterValue(LaunchConfiguration("detected_z_offset"), value_type=float)},
            {"use_detected_y_plan_only": ParameterValue(LaunchConfiguration("use_detected_y_plan_only"), value_type=bool)},
            {"use_detected_x_plan_only": ParameterValue(LaunchConfiguration("use_detected_x_plan_only"), value_type=bool)},
            {"reviewed_grasp_test": ParameterValue(LaunchConfiguration("reviewed_grasp_test"), value_type=bool)},
            {"prefer_cp13_branch": ParameterValue(LaunchConfiguration("prefer_cp13_branch"), value_type=bool)},
            {"cp13_reference_joints": ParameterValue(LaunchConfiguration("cp13_reference_joints"), value_type=str)},
            {"reviewed_grasp_center_y": ParameterValue(LaunchConfiguration("reviewed_grasp_center_y"), value_type=bool)},
            {"execute": ParameterValue(execute, value_type=bool)},
            {"use_perception": ParameterValue(use_perception, value_type=bool)},
            {"detection_timeout": ParameterValue(detection_timeout, value_type=float)},
            {"stop_at_grasp": ParameterValue(stop_at_grasp, value_type=bool)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("reviewed_grasp_test", default_value="false", description="Reviewed X centroid/Y half-width/Z +0.155 candidate; requires stop_at_grasp=true and no diagnostic overrides."),
        DeclareLaunchArgument("prefer_cp13_branch", default_value="false"),
        DeclareLaunchArgument("cp13_reference_joints", default_value="[]", description="Verified real pregrasp joints: pan, lift, elbow, wrist1, wrist2, wrist3 (rad)."),
        DeclareLaunchArgument("reviewed_grasp_center_y", default_value="false", description="Remove Y half-width correction only in bounded reviewed_grasp_test mode."),
        DeclareLaunchArgument("execute", default_value="false", description="Keep false for plan-only review."),
        DeclareLaunchArgument("use_perception", default_value="true"),
        DeclareLaunchArgument("use_detected_z_plan_only", default_value="false", description="Compare detected Z plus offset as tool0 Z; requires execute=false and stop_at_grasp=true."),
        DeclareLaunchArgument("detected_z_offset", default_value="0.0", description="World-Z offset in metres; only allowed in detected-Z plan-only mode."),
        DeclareLaunchArgument("use_detected_y_plan_only", default_value="false", description="Use detected Y without half-width shift; requires execute=false and stop_at_grasp=true."),
        DeclareLaunchArgument("use_detected_x_plan_only", default_value="false", description="Use detected X without half-thickness shift; requires execute=false and stop_at_grasp=true."),
        DeclareLaunchArgument("detection_timeout", default_value="15.0"),
        DeclareLaunchArgument("stop_at_grasp", default_value="false"),
        perception_launch,
        pick_place,
    ])
