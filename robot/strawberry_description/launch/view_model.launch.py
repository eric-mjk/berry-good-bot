
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('strawberry_description')


    # ▶️ RViz config 기본 경로
    default_rviz = PathJoinSubstitution([pkg, 'rviz', 'model_visualization.rviz'])
    
    # ▶️ Xacro → URDF
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'berry_good_bot_sampleURDF.xacro'])
    robot_desc = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # ▣ 사용자가 다른 rviz 파일을 넘길 수도 있게 인수 선언
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='Full path to the RViz config file'
        ),

        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc}]),
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui'),
        Node(package='rviz2',
             executable='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             output='log')
    ])