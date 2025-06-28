import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('so101_bringup')
    
    # RViz Configuration
    rviz_config_file = os.path.join(
        get_package_share_directory('so101_sim'), 'config', 'so101.rviz')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to start RViz')

    return LaunchDescription([
        use_rviz_arg,

        # This node would be replaced by your robot's official driver
        Node(
            package='so101_bringup',
            executable='hardware_interface',
            name='so101_hardware_interface',
            output='screen',
        ),

        # Publishes the robot's state from joint states to TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[os.path.join(get_package_share_directory('so101_sim'), 'urdf', 'so101_dual.urdf.xacro')]
        ),
        
        # Launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=LaunchConfiguration('rviz')
        ),
    ]) 