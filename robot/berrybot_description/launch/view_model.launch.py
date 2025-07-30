
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue      # 👈 새로 import

def generate_launch_description():
    pkg = get_package_share_directory('berrybot_description')


    # ▶️ RViz config 기본 경로
    default_rviz = PathJoinSubstitution([pkg, 'rviz', 'model_visualization.rviz'])
    
    # 순수 URDF → 문자열로 읽어 robot_state_publisher 에 전달
    urdf_file = PathJoinSubstitution([pkg, 'urdf', 'berrybot.urdf'])
#     robot_description = Command(['cat ', urdf_file])   # ★ xacro 대신 cat 사용

    # ★ cat 뒤에 공백 포함!
    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),   # ← 'cat '  (띄어쓰기 포함)
        value_type=str)

    return LaunchDescription([
        # ▣ 사용자가 다른 rviz 파일을 넘길 수도 있게 인수 선언
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='Full path to the RViz config file'
        ),

        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui'),
        Node(package='rviz2',
             executable='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             output='log')
    ])