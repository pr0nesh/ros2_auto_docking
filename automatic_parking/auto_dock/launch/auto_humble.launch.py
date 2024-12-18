import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

def generate_launch_description():
    # Declare launch arguments
    declare_open_rviz_arg = DeclareLaunchArgument(
        'open_rviz',
        default_value='true',
        description='Open RViz automatically'
    )
    
    open_rviz = LaunchConfiguration('open_rviz')

    def launch_setup(context, *args, **kwargs):
        # Conditional RViz node launch
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(get_package_share_directory('auto_dock'), 'rviz', 'rviz')],
            output='screen'
        ) if open_rviz.perform(context) == 'true' else None
        
        return [
            # Uncomment this block if you need to use static_transform_publisher
            # ExecuteProcess(
            #     cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '0', '0', '0', '0', '0', 'map', 'laser'],
            #     output='screen'
            # ),

            # Node for laser line extraction
            Node(
                package='laser_line_extraction',
                executable='laser_line_extraction_node',
                name='laser_line_extraction',
                output='screen',
                parameters=[{
                    "frequency": 50.0,
                    "frame_id": "laser_frame",
                    "scan_topic": "scan",
                    "publish_markers": True,
                    "bearing_std_dev": 0.0015,
                    "range_std_dev": 0.01,
                    "least_sq_angle_thresh": 0.0001,
                    "least_sq_radius_thresh": 0.0001,
                    "max_line_gap": 0.5,
                    "min_line_length": 0.03,
                    "min_range": 0.2,
                    "min_split_dist": 0.05,
                    "outlier_dist": 0.05,
                    "min_line_points": 5,
                }]
            ),

            # Node for auto docking pattern detection
            Node(
                package='auto_dock',
                executable='pattern',
                name='pattern_node',
                output='screen',
                parameters=[{
                    "detect_angle_tolerance": 0.2,
                    "group_dist_tolerance": 0.15,
                    "laser_frame_id": "laser_frame",
                }]
            ),

            # Node for auto docking controller
            Node(
                package='auto_dock',
                executable='controller',
                name='controller_node',
                output='screen',
                parameters=[{
                    "dist_to_dock": 0.22,
                    "dist_to_center": 0.03,
                }]
            ),   # Node for TurtleBot3 navigation
            
           
            # Conditional RViz node
            rviz_node
        ]

    # Launch description
    ld = LaunchDescription([
        declare_open_rviz_arg,
        OpaqueFunction(function=launch_setup)
    ])

    return ld
