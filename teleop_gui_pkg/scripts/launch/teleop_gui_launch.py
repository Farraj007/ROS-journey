from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_gui_pkg',
            executable='teleop_gui_node.py',
            output='screen',
            parameters=[
                {'current_speed_topic': '/lexus3/pacmod/vehicle_speed_rpt'},
                {'reference_speed_topic': '/your_reference_speed_topic'},
                {'current_wheel_angle_topic': '/your_current_wheel_angle_topic'},
                {'reference_wheel_angle_topic': '/your_reference_wheel_angle_topic'},
                {'autonomous_manual_drive_topic': '/lexus3/pacmod/enabled'},
                {'gain_current_wheel_angle': 1.0},
                {'gain_reference_wheel_angle': 1.0},
                {'gain_current_speed': 3.6},
                {'gain_reference_speed': 3.6},
                {'max_speed_limit': 30.0}
            ],
        ),
    ])
