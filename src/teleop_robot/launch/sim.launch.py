import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'teleop_robot'
    pkg_share = get_package_share_directory(package_name)

    world_path = PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'),
        'worlds',
        'house.world'
    ])

    # 1. Include the Robot State Publisher (RSP)
    # We force 'use_sim_time' to be true since this is a simulation
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # COMMENT OUT the old pkg_share line
    # pkg_share = get_package_share_directory(package_name)
    controller_params_file = os.path.join(pkg_share, 'config', 'my_controllers.yaml')

    rviz_config_path = PathJoinSubstitution([
        pkg_share, 'rviz', 'my_robot.rviz'
    ])

    # FORCE ABSOLUTE PATH to your source file
    # (Replace 'seifelshafey' with your actual username if different)
    # ... (path definition remains the same)
    # Use the standard path resolution again
    controller_params_file = os.path.join(pkg_share, 'config', 'my_controllers.yaml')

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={
                    'world': world_path,
                    'use_sim_time': 'true',
                }.items()
         )
    
    # ... ensure you remove the manual print statement you added for clarity.
    # 3. Spawn Entity
    # Puts the robot model into the Gazebo world
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # 4. Spawner: Diff Drive Controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        parameters=[controller_params_file],
    )

    # # 5. Spawner: Joint Broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        parameters=[controller_params_file],
    )

    # 6. Delays (To ensure controllers start AFTER the robot exists)
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner],
        )
    )
          
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
    # Optional: Load a specific config file if you have one
        arguments=['-d', rviz_config_path]
    )
    
    return LaunchDescription([
        rsp,
        gazebo,
        rviz_node,
        spawn_entity,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])