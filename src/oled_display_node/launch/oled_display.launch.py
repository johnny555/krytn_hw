from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generates the launch description for starting the OLED display nodes."""
    return LaunchDescription([
        # Start the C++ OLED display driver node
        Node(
            package='oled_display_node',
            executable='oled_display_node_exec',
            name='oled_display', # Node name in the ROS graph
            output='screen', # Show node output in the terminal
            # You can add parameters here if needed, matching those in the C++ node
            # parameters=[
            #     {'i2c_device': '/dev/i2c-1'},
            #     {'i2c_address': 0x3C},
            #     {'display_type': 2}, # 1 for SSD1306, 2 for SH1106
            #     {'update_rate_hz': 0.5}
            # ]
        ),

        # Start the Python script that sends example commands
        # Note: This script runs once and exits. For continuous operation,
        # you might want a different script or integrate its logic elsewhere.
        Node(
            package='oled_display_node',
            executable='display_writer.py',
            name='display_writer',
            output='screen'
        )
    ])
