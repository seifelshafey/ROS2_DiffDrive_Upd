import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command

def generate_launch_description():
    package_name = 'teleop_robot'
    pkg_share = get_package_share_directory(package_name)

    # Paths
    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')
    controller_params_file = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'my_robot.rviz')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # Use the Ignition-compatible URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot_ign.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # 1. Robot State Publisher (with Ignition URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # 2. Launch Ignition Gazebo Fortress
    # -r: run immediately, -s: server only (no GUI process that can crash),
    # --headless-rendering: allows sensor rendering without a GUI window
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': f'-r -s --headless-rendering {world_path}',
        }.items(),
    )

    # Launch Ignition GUI separately (not required — won't kill the sim if it crashes)
    ignition_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g'],
        output='screen',
    )

    # 3. Spawn robot into Ignition from robot_description topic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-z', '0.2',
        ],
        output='screen',
    )

    # 4. ros_gz_bridge: bridge Ignition topics to ROS2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen',
    )

    # 5. Controller spawners (delayed until robot is spawned)
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        parameters=[controller_params_file],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        parameters=[controller_params_file],
    )

    delayed_diff_drive = TimerAction(
        period=8.0,
        actions=[diff_drive_spawner],
    )

    delayed_joint_broad = TimerAction(
        period=8.0,
        actions=[joint_broad_spawner],
    )

    # 6. EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
    )

    # 7. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    # 8. Teleop
    my_teleop = Node(
        package='teleop_robot',
        executable='teleop',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -geometry 80x20 -e',
    )

    return LaunchDescription([
        robot_state_publisher,
        ignition_gazebo,
        ignition_gui,
        bridge,
        spawn_entity,
        rviz_node,
        ekf_node,
        delayed_diff_drive,
        delayed_joint_broad,
        my_teleop,
    ])
