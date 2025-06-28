#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory('so101_sim')
    
    # Gazebo Launch
    gazebo_launch_path = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    # RViz Launch Argument
    use_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to start RViz')

    # RViz Configuration
    rviz_config_file = os.path.join(pkg_share, 'config', 'so101.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Whether to launch the Gazebo GUI'),
        
        use_rviz_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path)
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[os.path.join(pkg_share, 'urdf', 'so101_dual.urdf.xacro')]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=LaunchConfiguration('rviz')
        ),
    ])

    # Declare the launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("so101_sim"), "worlds", "so101_world.world"]
            ),
            description="Path to world file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Flag to enable gazebo gui",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Flag to enable rviz",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Flag to enable rviz",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_robot_state_pub",
            default_value="true",
            description="Flag to enable robot_state_publisher",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_joint_state_pub",
            default_value="true",
            description="Flag to enable joint_state_publisher",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rsp",
            default_value="true",
            description="Flag to enable robot_state_publisher",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_planning",
            default_value="true",
            description="Flag to enable moveit planning",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    use_rviz = LaunchConfiguration("use_rviz")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_joint_state_pub = LaunchConfiguration("use_joint_state_pub")
    use_rsp = LaunchConfiguration("use_rsp")
    use_planning = LaunchConfiguration("use_planning")

    # Get the launch directory
    pkg_share = FindPackageShare("so101_sim")
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py")]),
        launch_arguments={
            "world": world,
            "gui": gui,
        }.items(),
    )

    # Launch RViz
    rviz_config_file = PathJoinSubstitution(
        [pkg_share, "config", "so101.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # Launch robot state publisher
    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution([pkg_share, "urdf", "so101_dual.urdf.xacro"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(use_robot_state_pub),
    )

    # Launch joint state publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(use_joint_state_pub),
    )

    # Launch spawn robot
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "so101_dual_arm"],
        output="screen",
    )

    # Launch controller manager
    controller_config_file = PathJoinSubstitution(
        [pkg_share, "config", "controllers.yaml"]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_file, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Launch controllers
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller"],
        output="screen",
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller"],
        output="screen",
    )

    left_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller"],
        output="screen",
    )

    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gripper_controller"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Launch MoveIt if planning is enabled
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            FindPackageShare("so101_sim"), "launch", "moveit_launch.py")]),
        condition=IfCondition(use_planning),
    )

    nodes = [
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
        controller_manager_node,
        left_arm_controller_spawner,
        right_arm_controller_spawner,
        left_gripper_controller_spawner,
        right_gripper_controller_spawner,
        joint_state_broadcaster_spawner,
        rviz_node,
        moveit_launch,
    ]

    return LaunchDescription(declared_arguments + nodes) 