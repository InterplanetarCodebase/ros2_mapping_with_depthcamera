import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    qos = LaunchConfiguration('qos', default='1')
    localization = LaunchConfiguration('localization', default='false')
    
    # RTAB-Map node
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,  # Set to False if using LiDAR
            'subscribe_rgb': True,
            'approx_sync': True,
            'queue_size': 100,
            'qos': qos,
            'use_sim_time': use_sim_time,
            # RTAB-Map's parameters
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'true' if localization else 'false',
            'grid_map': 'true',  # Enable 2D grid map
            'Grid/FromDepth': 'true',  # Or 'false' if using LiDAR
            'Grid/MaxGroundHeight': '0.1',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/CellSize': '0.05',
            'Grid/RangeMax': '5.0',
            'Grid/3D': 'false',  # Force 2D mode
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('scan', '/scan'),  # Only if using LiDAR
            ('odom', '/odometry/filtered'),
            ('grid_map', '/map'),  # Remap to standard /map topic
        ]
    )
    
    # RViz2 for visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('depth_camera_mapping'), 'rviz', 'rtabmap.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('qos', default_value='1'),
        DeclareLaunchArgument('localization', default_value='false'),
        rtabmap,
        rviz2
    ])