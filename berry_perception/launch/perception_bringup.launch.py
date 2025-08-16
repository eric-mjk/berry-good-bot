from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_percep = get_package_share_directory('berry_perception')

    rs_launch_path = PathJoinSubstitution([
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    ])

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
         launch_arguments={
             'depth_module.depth_profile': '848x480x30',
             'rgb_camera.color_profile' : '848x480x30',
             'align_depth.enable'       : 'true',
             'enable_sync'              : 'true',
         }.items()
     )


    perception = Node(
        package='berry_perception', executable='perception_node',
        parameters=[PathJoinSubstitution([pkg_percep,'config','camera_to_eef.yaml'])],
        output='screen')
    
    visualServoing = Node(
        package='berry_perception', executable='visualServoing',
        output='screen')

    return LaunchDescription([realsense_launch, perception, visualServoing])
    # return LaunchDescription([realsense_launch, perception])

