import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    package_name = 'teleop_robot'
    pkg_share = get_package_share_directory(package_name)
    
    # 1. Get TurtleBot3 Paths
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    world_path = os.path.join(turtlebot3_gazebo_share, 'worlds', 'house.world')
    
    # 2. CRITICAL: Add TurtleBot3 models to GAZEBO_MODEL_PATH
    # Without this, Gazebo cannot find the walls/furniture and reverts to empty.
    models_path = os.path.join(turtlebot3_gazebo_share, 'models')
    set_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path
    )

    # 3. Controller Params
    controller_params_file = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'my_robot.rviz')


    # 4. Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_path,
            'use_sim_time': 'true',
            'verbose': 'true',  # Enable verbose logging to see if world loading fails
        }.items()
    )
    
    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    
    # Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Spawner: Diff Drive (Pass params directly)
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        parameters=[controller_params_file],
    )

    # Spawner: Joint Broadcaster (Pass params directly)
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        parameters=[controller_params_file],
    )

    # Lidar Topic Relay (Fixes the topic name issue)
    topic_relay = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        output='screen',
        arguments=['/gazebo_ros_ray_sensor/out', '/scan'], 
    )

    # Teleop Node (In a popup window)
    # my_teleop = Node(
    #     package='teleop_robot',      
    #     executable='teleop_node',   
    #     name='teleop_keyboard',
    #     output='screen',
    #     prefix='xterm -geometry 80x20 -e' 
    # )

    # Delays to ensure controllers load after robot
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
        arguments=['-d', rviz_config_path]
    )
    
    return LaunchDescription([
        set_model_path, # <--- MUST BE FIRST
        rsp,
        gazebo,
        rviz_node,
        topic_relay,
        spawn_entity,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])