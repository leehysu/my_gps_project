from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
import os

def launch_setup(context, *args, **kwargs):
    csv_path = LaunchConfiguration('csv_path').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)
    publish_rate = float(LaunchConfiguration('publish_rate').perform(context))
    z_offset = float(LaunchConfiguration('z_offset').perform(context))
    downsample_step = int(LaunchConfiguration('downsample_step').perform(context))
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() in ['1','true','yes']

    nodes = []

    nodes.append(
        Node(
            package='csv_viewer',
            executable='csv_rviz_viewer',
            name='csv_rviz_viewer',
            output='screen',
            parameters=[{
                'csv_path': csv_path,
                'frame_id': frame_id,
                'publish_rate': publish_rate,
                'z_offset': z_offset,
                'downsample_step': downsample_step,
                'marker_scale': 0.5,
                'line_width': 0.15,
                'publish_points': True,
                'publish_line': True,
                'publish_path': True,
            }]
        )
    )

    if use_rviz:
        rviz_cfg = os.path.join(
            os.getenv('COLCON_CURRENT_PREFIX', ''),  # runtime may be empty
            'share', 'csv_viewer', 'rviz', 'view_csv.rviz'
        )
        # Fallback to package share path via ament index (simpler: use relative install path)
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_cfg],
                output='screen'
            )
        )

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('csv_path', default_value='data/processed/이전대회 GPS 데이터.csv'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('publish_rate', default_value='1.0'),
        DeclareLaunchArgument('z_offset', default_value='0.0'),
        DeclareLaunchArgument('downsample_step', default_value='1'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=launch_setup)
    ])
