from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():
    # Path to the controller configuration
    controller_config = PathJoinSubstitution([
        FindPackageShare('krytn'),
        'config',
        'diffdrive_control.yaml'
    ])

    camera_info_url = 'file://' + join(
        get_package_share_directory('krytn'),
        'config',
        'rpiv2.yaml'
    )

    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'frame_id': 'camera_link',
            'fps': 1,
            'width': 1280,
            'height': 800,
            'format': "RGB888",
            'camera_info_url': camera_info_url
        }]
        ,
        remappings=[
            ( '/camera_node/image_raw/compressed', '/camera/image_raw/compressed')
        ]
    )



    return LaunchDescription([
        camera_node
    ])