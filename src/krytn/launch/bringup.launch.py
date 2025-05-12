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

    # Include the sllidar_ros2 launch file
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch',
                'sllidar_a1_launch.py'
            ])
        ])
        ,
        launch_arguments=[('frame_id', 'lidar_2d_link')]
    )

    # Include the robot description launch file
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('krytn'),
                'launch',
                'description.launch.py'
            ])
        ])
    )

    # Serial port configuration
    serial_config = ExecuteProcess(
        cmd=['stty', '-F', '/dev/ttyS0', 'sane'],
        shell=True
    )

    # Controller Manager node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[controller_config],
        remappings=[
            ('/diff_drive_base_controller/cmd_vel', '/cmd_vel'),
            ('/diff_drive_base_controller/odom', '/odom')
        ]
    )

    # Spawn the velocity controller
    spawn_controller = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'diff_drive_base_controller', 'joint_state_broadcaster'],
        output='screen'
    )

    # Joystick node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
           # 'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    # Teleop twist joy node to convert joystick input to cmd_vel
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[{
            'axis_linear.x': 1,  # Left stick vertical axis
            'axis_angular.yaw': 0,  # Left stick horizontal axis
            'scale_linear.x': -0.5,  # Inverted forward speed in m/s
            'scale_angular.yaw': 1.0,  # Inverted rotational speed in rad/s
            'enable_button': 4,  # LB 
        }],
        remappings=[('/cmd_vel', '/cmd_vel_unstamped')]  # Remap to the correct topic
    )

    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper.py",
        remappings=[("/cmd_vel_in", "/cmd_vel_unstamped"),
                       ("/cmd_vel_out",  "/cmd_vel")],
        parameters=[{"use_sim_time","False"}],
        output="screen"
    )  

    slam_toolbox = IncludeLaunchDescription(
            PythonLaunchDescriptionSource( 
                    join(get_package_share_directory('slam_toolbox'), 
                          'launch','online_async_launch.py')),
            launch_arguments=[
                    ('slam_params_file', join(get_package_share_directory('krytn'),'config','mapping.yaml'))
                         ]
    )

  
    oled = Node(
            package='oled_display_node',
            executable='oled_display_node_exec',
            name='oled_display',
            output='screen'
        )

    camera_node = Node(
        package='image_publisher',
        executable='image_publisher_node',
        name='camera_node',
        output='screen',
        arguments=['rtsp://localhost:8553/cam']
    )

    return LaunchDescription([
        description_launch,
        sllidar_launch,
        serial_config,
        controller_manager,
        spawn_controller,
        joy_node,  # Add the joystick node
        twist_stamper,
        teleop_node,  # Add the teleop node
        camera_node,  # Add the camera node
        slam_toolbox,
        oled
    ])