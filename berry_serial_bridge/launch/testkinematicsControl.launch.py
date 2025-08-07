from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_kin = get_package_share_directory('berry_kinematics_control')
    pkg_bridge = get_package_share_directory('berry_serial_bridge')

    kinControl_launch = PathJoinSubstitution(
        [pkg_kin, 'launch', 'berry_bringup.launch.py'])

    return LaunchDescription([
        # 기존 모델 + GUI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kinControl_launch)),
        # 시리얼 브리지
        Node(package='berry_serial_bridge',
             executable='bridge_node',
             parameters=[PathJoinSubstitution([pkg_bridge,'config','joint_map.yaml'])],
             output='screen')
    ])
