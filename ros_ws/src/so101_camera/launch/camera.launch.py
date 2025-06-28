from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def launch_setup(context, *args, **kwargs):
    camera_type = LaunchConfiguration('camera_type').perform(context)
    
    # Common remapping to unify topic names
    remappings = [
        ('/image_raw', '/camera/color/image_raw'),
    ]

    nodes_to_launch = []

    if camera_type == 'usb':
        nodes_to_launch.append(
            Node(
                package='so101_camera',
                executable='usb_camera_publisher',
                name='camera_node',
                output='screen',
                remappings=remappings,
            )
        )
    elif camera_type == 'realsense':
        # This assumes the realsense-ros package is installed
        # The launch file also provides depth and other streams
        nodes_to_launch.append(
            Node(
                package='realsense2_camera',
                executable='realsense2_camera_node',
                name='camera_node',
                output='screen',
                parameters=[{'enable_depth': True, 'enable_color': True, 'point_cloud.enable':True}],
                remappings=remappings, # Only color is remapped here for consistency
            )
        )
    else:
        # You can add more camera types here
        raise RuntimeError(f"Unsupported camera_type: {camera_type}")

    return nodes_to_launch

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_type',
            default_value='usb',
            description='Type of camera to launch (e.g., "usb", "realsense").'
        ),
        OpaqueFunction(function=launch_setup)
    ]) 