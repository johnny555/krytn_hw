from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the URDF file
    urdf_file = PathJoinSubstitution([
        FindPackageShare('krytn'),'robot_description',
        'krytn.urdf.xacro'
    ])

    # Convert xacro to URDF
    robot_description = Command(['xacro ', urdf_file])

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        robot_state_publisher
    ])