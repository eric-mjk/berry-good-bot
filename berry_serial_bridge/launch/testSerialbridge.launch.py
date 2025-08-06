from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_desc = get_package_share_directory('berrybot_description')
    pkg_bridge = get_package_share_directory('berry_serial_bridge')

    view_model_launch = PathJoinSubstitution(
        [pkg_desc, 'launch', 'view_model.launch.py'])

    return LaunchDescription([
        # 기존 모델 + GUI
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(view_model_launch)),
        # 시리얼 브리지
        Node(package='berry_serial_bridge',
             executable='bridge_node',
             parameters=[PathJoinSubstitution([pkg_bridge,'config','joint_map.yaml'])],
             output='screen')
    ])
