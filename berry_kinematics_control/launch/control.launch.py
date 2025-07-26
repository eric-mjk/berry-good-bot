# berry_kinematics_control/launch/control.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = FindPackageShare("berry_kinematics_control")

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution(
            # [FindPackageShare("strawberry_description"),
            #  "urdf/berry_good_bot_sampleURDF.xacro"]
            ["/home/swpants05/Desktop/berryGoodBot_ws/src/berry-good-bot/robot/strawberry_description/urdf/berry_good_bot_sampleURDF.xacro"]
             ),
        description="URDF or processed .urdf file path"
    ))

    ld.add_action(DeclareLaunchArgument(
        "pose_yaml",
        default_value=PathJoinSubstitution(
            [pkg_share, "config/sample_robot/named_poses.yaml"]),
        description="Named poses YAML"
    ))

    ld.add_action(Node(
        package="berry_kinematics_control",
        executable="control_node",
        parameters=[{
            "urdf_path": LaunchConfiguration("urdf_path"),
            "pose_yaml": LaunchConfiguration("pose_yaml"),
        }],
        output="screen"))

    return ld
