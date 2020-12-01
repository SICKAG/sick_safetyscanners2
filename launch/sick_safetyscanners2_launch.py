from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_node",
            name="sick_safetyscanners2_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "scan"}
            ]
        )
    ])
