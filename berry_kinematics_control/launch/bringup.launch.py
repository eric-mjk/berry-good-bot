# berry_kinematics_control/launch/bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, Command)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ───────── 패키지 경로 ─────────
    pkg_desc = FindPackageShare("strawberry_description")
    pkg_self = FindPackageShare("berry_kinematics_control")

    # ───────── 기본 경로 / 인수 선언 ─────────
    default_xacro = PathJoinSubstitution(
        [pkg_desc, "urdf", "berry_good_bot_sampleURDF.xacro"])
    default_yaml  = PathJoinSubstitution(
        [pkg_self, "config/sample_robot/named_poses.yaml"])
    default_rviz  = PathJoinSubstitution(
        [pkg_desc, "rviz", "model_visualization.rviz"])

    ld = LaunchDescription()

    # URDF(xacro) 경로
    ld.add_action(DeclareLaunchArgument(
        "urdf_path", default_value=default_xacro,
        description="URDF or Xacro path for the robot"))
    # Named-pose YAML
    ld.add_action(DeclareLaunchArgument(
        "pose_yaml", default_value=default_yaml,
        description="Named-poses YAML file"))
    # RViz 설정 파일
    ld.add_action(DeclareLaunchArgument(
        "rviz_config", default_value=default_rviz,
        description="RViz config file"))
    # (옵션) GUI 슬라이더를 켤지
    ld.add_action(DeclareLaunchArgument(
        "use_gui", default_value="false",
        description="[true/false] start joint_state_publisher_gui"))

    # ───────── 노드들 ─────────
    # robot_state_publisher  ➜  /tf
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description":
                Command(["xacro ", LaunchConfiguration("urdf_path")])
        }])

    # 실제 관절각 퍼블리셔 (BerryControl)
    control_node = Node(
        package="berry_kinematics_control",
        executable="control_node",
        parameters=[{
            "urdf_path": LaunchConfiguration("urdf_path"),
            "pose_yaml": LaunchConfiguration("pose_yaml"),
        }],
        output="screen")

    # (선택) joint_state_publisher_gui  ➜  /dummy_joint_states
    #   - RViz 시각 디버깅용 슬라이더이며 기본값 off
    gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        remappings=[("/joint_states", "/dummy_joint_states")],
        condition=IfCondition(LaunchConfiguration("use_gui")))

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="log")

    # 런치 설명서에 추가
    for n in (rsp_node, control_node, gui_node, rviz_node):
        ld.add_action(n)

    return ld
