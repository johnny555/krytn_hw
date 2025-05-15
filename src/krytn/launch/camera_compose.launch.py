import os
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch Arguments
    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value='file://' + os.path.join(
            get_package_share_directory('krytn'),
            'config',
            'rpiv2.yaml'
        )
    )
    
    fiducial_len_arg = DeclareLaunchArgument(
        'fiducial_len',
        default_value='0.136'
    )
    
    dictionary_arg = DeclareLaunchArgument(
        'dictionary',
        default_value='7'
    )
    
    ignore_fiducials_arg = DeclareLaunchArgument(
        'ignore_fiducials',
        default_value=''
    )
    
    fiducial_len_override_arg = DeclareLaunchArgument(
        'fiducial_len_override',
        default_value=''
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_link'
    )

    # Create the container
    container = ComposableNodeContainer(
        name='camera_aruco_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Camera node
            ComposableNode(
                package='camera_ros',
                plugin='camera_ros::Camera',
                name='camera_node',
                parameters=[{
                    'frame_id': LaunchConfiguration('frame_id'),
                    'fps': 15,
                    'width': 1280,
                    'height': 800,
                    'format': "RGB888", 
                    'camera_info_url': LaunchConfiguration('camera_info_url')
                }],
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # ArUco detect node
            ComposableNode(
                package='aruco_detect',
                plugin='aruco_detect::FiducialsNode',
                name='aruco_detect',
                parameters=[{
                    'image_transport': 'raw',
                    'publish_images': True,
                    'fiducial_len': LaunchConfiguration('fiducial_len'),
                    'dictionary': LaunchConfiguration('dictionary'),
                    'do_pose_estimation': True,
                    'vis_msgs': False,
                    'ignore_fiducials': LaunchConfiguration('ignore_fiducials'),
                    'fiducial_len_override': LaunchConfiguration('fiducial_len_override'),
                    'verbose': LaunchConfiguration('verbose')
                }],
                remappings=[
                    ('camera/image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        camera_info_url_arg,
        fiducial_len_arg,
        dictionary_arg,
        ignore_fiducials_arg,
        fiducial_len_override_arg,
        verbose_arg,
        frame_id_arg,
        container
    ])