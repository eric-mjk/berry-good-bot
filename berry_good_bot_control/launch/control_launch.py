from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="moveit_servo", executable="servo_node_main",
             parameters=["/home/wossas/Desktop/strawberry_robotArm/strawberry_ws/src/berry-good-bot/robot/sample_moveit_config/config/berry_servo_config.yaml"]),
        Node(package="berry_good_bot_control", executable="control_node",
             output="screen"),
    ])
